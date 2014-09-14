function [utraj, xtraj] = runDircolFlapping

options.floating = true;
p = RigidBodyManipulator('pigeon.URDF', options);

p = p.weldJoint('tail_roll');
p = p.weldJoint('tail_yaw');
p = p.weldJoint('left_hip_roll');
p = p.weldJoint('left_hip_pitch');
p = p.weldJoint('left_knee_pitch');
p = p.weldJoint('left_ankle_pitch');
p = p.weldJoint('left_thumb_pitch');
p = p.weldJoint('left_fingers_pitch');
p = p.weldJoint('right_hip_roll');
p = p.weldJoint('right_hip_pitch');
p = p.weldJoint('right_knee_pitch');
p = p.weldJoint('right_ankle_pitch');
p = p.weldJoint('right_thumb_pitch');
p = p.weldJoint('right_fingers_pitch');

p = p.compile();
numstates = getNumStates(p);

N = 31;
minimum_duration = .1;
maximum_duration = 5;
prog = DircolTrajectoryOptimization(p,N,[minimum_duration maximum_duration]); 
prog = prog.setSolverOptions('snopt','superbasicslimit',2000);
% prog = prog.setCheckGrad(true);
v = constructVisualizer(p);
prog = prog.addDisplayFunction(@(x)displayfun(x,N,p.getNumStates(),p.getNumInputs(),v));

% periodicity constraint
per_lb = -.1*ones(numstates,1);
per_ub = .1*ones(numstates,1);
per_lb(findCoordinateIndex(getStateFrame(p),'base_x')) = -Inf; 
per_ub(findCoordinateIndex(getStateFrame(p),'base_x')) = Inf;
per_lb(findCoordinateIndex(getStateFrame(p),'base_xdot')) = -Inf; 
per_ub(findCoordinateIndex(getStateFrame(p),'base_xdot')) = Inf;
per_lb(findCoordinateIndex(getStateFrame(p),'base_zdot')) = -Inf; 
per_ub(findCoordinateIndex(getStateFrame(p),'base_zdot')) = Inf;
per_lb(findCoordinateIndex(getStateFrame(p),'base_pitch')) = -Inf; 
per_ub(findCoordinateIndex(getStateFrame(p),'base_pitch')) = Inf;
per_A = [speye(numstates),-speye(numstates)];
periodicity_constraint = LinearConstraint(per_lb,per_ub,per_A);

sym_A = [];
 function addSymmetricConstraint(jointA,jointB)
    pt = zeros(1,numstates);
    pt(findCoordinateIndex(getStateFrame(p),jointA)) = 1;
    pt(findCoordinateIndex(getStateFrame(p),jointB)) = -1;
    sym_A = [sym_A; double(pt)];
  end
  function addAntiSymmetricConstraint(jointA,jointB)
    pt = zeros(1,numstates);
    pt(findCoordinateIndex(getStateFrame(p),jointA)) = 1;
    pt(findCoordinateIndex(getStateFrame(p),jointB)) = 1;
    sym_A = [sym_A; double(pt)];
  end
addSymmetricConstraint('left_shoulder_roll','right_shoulder_roll');
addSymmetricConstraint('left_shoulder_pitch','right_shoulder_pitch');
addSymmetricConstraint('left_shoulder_yaw','right_shoulder_yaw');
addSymmetricConstraint('left_elbow_yaw','right_elbow_yaw');
addSymmetricConstraint('left_wrist_roll','right_wrist_roll');
addSymmetricConstraint('left_wrist_pitch','right_wrist_pitch');
addSymmetricConstraint('left_wrist_yaw','right_wrist_yaw');
symmetry_constraint = LinearConstraint(zeros(7,1),zeros(7,1),sym_A);

prog = prog.addStateConstraint(periodicity_constraint,{[1 N]});
prog = prog.addStateConstraint(symmetry_constraint,[1:N]);
prog = prog.addRunningCost(@(t,x,u)cost(t,x,u,getStateFrame(p),getInputFrame(p)));
prog = prog.addFinalCost(@finalcost);

display('Finding trim conditions for initial guess...')
[xstar,ustar] = findTrim(p);

display('Computing the initial guess...')
tf0 = minimum_duration;
%xstar(findCoordinateIndex(getStateFrame(p),'left_shoulder_pitch')) = xstar(findCoordinateIndex(getStateFrame(p),'left_shoulder_pitch')) + 0.2;
%xstar(findCoordinateIndex(getStateFrame(p),'right_shoulder_pitch')) = xstar(findCoordinateIndex(getStateFrame(p),'right_shoulder_pitch')) + 0.2;
%xstar(findCoordinateIndex(getStateFrame(p),'left_wrist_roll')) = xstar(findCoordinateIndex(getStateFrame(p),'left_wrist_roll')) + 0.1;
%xstar(findCoordinateIndex(getStateFrame(p),'right_wrist_roll')) = xstar(findCoordinateIndex(getStateFrame(p),'right_wrist_roll')) + 0.1;
utraj0 = ConstantTrajectory(ustar);
%utraj0 = repmat(ustar,1,N);
%left_shoulder_index = findCoordinateIndex(getInputFrame(p),'left_shoulder_roll_servo');
%right_shoulder_index = findCoordinateIndex(getInputFrame(p),'right_shoulder_roll_servo');
%utraj0(left_shoulder_index,:) = sin(linspace(0,tf0,N))*ustar(left_shoulder_index);
%utraj0(right_shoulder_index,:) = sin(linspace(0,tf0,N))*ustar(right_shoulder_index);
%utraj0 = DTTrajectory(linspace(0,tf0,N),utraj0);
utraj0 = setOutputFrame(utraj0,getInputFrame(p));
sys0 = cascade(utraj0,p);
xtraj0 = simulate(sys0,[0 tf0],xstar);
traj_init.x = xtraj0;
traj_init.u = utraj0;

display('Starting trajectory optimization...')
tic
[xtraj,utraj,z,F,info] = prog.solveTraj(tf0,traj_init);
toc

if (nargout<1)
  v = constructVisualizer(p);
  v.playback_speed = .2;
  v.playback(xtraj,struct('slider',true));
end

end

function [g,dg] = cost(t,x,u,state_frame,input_frame)
  Q = zeros(numel(x));
  roll_index = findCoordinateIndex(state_frame,'base_roll');
  yaw_index = findCoordinateIndex(state_frame,'base_yaw');
  Q(roll_index,roll_index) = 10;
  Q(yaw_index,yaw_index) = 10;
  zdot_index = findCoordinateIndex(state_frame,'base_zdot');
  Q(zdot_index,zdot_index) = 10000;
  R = 100*eye(numel(u));
  g = x'*Q*x + u'*R*u;
  if (nargout>1)
    dg = [0,2*x'*Q,2*u'*R]; % assumes Q and R symmetric
  end
end

function [h,dh] = finalcost(t,x)
  % h = t;
  % if (nargout>1)
  %   dh = [1,zeros(1,numel(x))];
  % end
  h = 0;
  if (nargout>1)
    dh = zeros(1,1+numel(x));
  end
end

function displayfun(X,num_t,num_x,num_u,v)
  h = X(1:num_t-1);
  x = X(num_t-1+[1:num_x*num_t]);
  u = X(num_t-1+num_x*num_t+[1:num_u*num_t]);
  x = reshape(x,num_x,num_t);
  u = reshape(u,num_u,num_t);
  tt = [0;cumsum(h)]';

  figure(5);
  plot(tt,x(1,:));
  title('x position');
  
  figure(6);
  plot(tt,x(3,:));
  title('altitude');
  
  xtraj = DTTrajectory(linspace(0,2,size(x,2)),x);
  v = v.setInputFrame(xtraj.getOutputFrame());
  v.playback(xtraj);

end