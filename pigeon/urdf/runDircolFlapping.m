function [utraj, xtraj] = runDircolFlapping(outputfile, seedtraj)

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

display('Finding trim conditions...')
% [xstar,ustar] = findTrim(p);
trim = load('trimConditions.mat');
xstar = trim.xstar;
ustar = trim.ustar;

N = 10;
minimum_duration = .1;
maximum_duration = 5;
prog = DircolTrajectoryOptimization(p,N,[minimum_duration maximum_duration]); 
prog = prog.setSolverOptions('snopt','superbasicslimit',2000);
prog = prog.setSolverOptions('snopt','iterationslimit',50000);
prog = prog.setSolverOptions('snopt','majoroptimalitytolerance',1e-3);

% prog = prog.setCheckGrad(true);
v = constructVisualizer(p);
prog = prog.addDisplayFunction(@(x)displayfun(x,N,p.getNumStates(),p.getNumInputs(),v));

% limits on the states
frame = getStateFrame(p);
lb = Point(frame);
ub = Point(frame);

lb.base_x = 0; ub.base_x = Inf;
lb.base_z = 1; ub.base_z = Inf;
lb.base_pitch = -1.5; ub.base_pitch = 1.5;
lb.tail_pitch = -1.5; ub.tail_pitch = 1.5;
lb.left_shoulder_roll = -1.5; ub.left_shoulder_roll = 1.5;
lb.left_shoulder_pitch = -1.5; ub.left_shoulder_pitch = 1.5;
lb.left_shoulder_yaw = -1.5; ub.left_shoulder_yaw = 1.5;
lb.left_elbow_yaw = -1.5; ub.left_elbow_yaw = 1.5;
lb.left_wrist_roll = -1.5; ub.left_wrist_roll = 1.5;
lb.left_wrist_pitch = -1.5; ub.left_wrist_pitch = 1.5;
lb.left_wrist_yaw = -1.5; ub.left_wrist_yaw = 1.5;
lb.right_shoulder_roll = -1.5; ub.right_shoulder_roll = 1.5;
lb.right_shoulder_pitch = -1.5; ub.right_shoulder_pitch = 1.5;
lb.right_shoulder_yaw = -1.5; ub.right_shoulder_yaw = 1.5;
lb.right_elbow_yaw = -1.5; ub.right_elbow_yaw = 1.5;
lb.right_wrist_roll = -1.5; ub.right_wrist_roll = 1.5;
lb.right_wrist_pitch = -1.5; ub.right_wrist_pitch = 1.5;
lb.right_wrist_yaw = -1.5; ub.right_wrist_yaw = 1.5;

lb.base_xdot = 0.1; ub.base_xdot = 100;
lb.base_zdot = -Inf; ub.base_zdot = Inf;
lb.base_pitchdot = -Inf; ub.base_pitchdot = Inf;
lb.tail_pitchdot = -Inf; ub.tail_pitchdot = Inf;
lb.left_shoulder_rolldot = -Inf;  ub.left_shoulder_rolldot = Inf;
lb.left_shoulder_pitchdot = -Inf;  ub.left_shoulder_pitchdot = Inf;
lb.left_shoulder_yawdot = -Inf;  ub.left_shoulder_yawdot = Inf;
lb.left_elbow_yawdot = -Inf;  ub.left_elbow_yawdot = Inf;
lb.left_wrist_rolldot = -Inf;  ub.left_wrist_rolldot = Inf;
lb.left_wrist_pitchdot = -Inf;  ub.left_wrist_pitchdot = Inf;
lb.left_wrist_yawdot = -Inf;  ub.left_wrist_yawdot = Inf;
lb.right_shoulder_rolldot = -Inf;  ub.right_shoulder_rolldot = Inf;
lb.right_shoulder_pitchdot = -Inf;  ub.right_shoulder_pitchdot = Inf;
lb.right_shoulder_yawdot = -Inf;  ub.right_shoulder_yawdot = Inf;
lb.right_elbow_yawdot = -Inf;  ub.right_elbow_yawdot = Inf;
lb.right_wrist_rolldot = -Inf;  ub.right_wrist_rolldot = Inf;
lb.right_wrist_pitchdot = -Inf;  ub.right_wrist_pitchdot = Inf;
lb.right_wrist_yawdot = -Inf;  ub.right_wrist_yawdot = Inf;

prog = addStateConstraint(prog,BoundingBoxConstraint(double(lb),double(ub)),1:N);

% symmetry of some of the states
A = [];
  function addSymmetricConstraint(jointA,jointB)
    pt = Point(frame);
    pt.(jointA) = 1;
    pt.(jointB) = -1;
    A = [A; double(pt)'];
  end
  function addAntiSymmetricConstraint(jointA,jointB)
    pt = Point(frame);
    pt.(jointA) = 1;
    pt.(jointB) = 1;
    A = [A; double(pt)'];
  end
addAntiSymmetricConstraint('left_shoulder_roll','right_shoulder_roll');
addSymmetricConstraint('left_shoulder_pitch','right_shoulder_pitch');
addSymmetricConstraint('left_shoulder_yaw','right_shoulder_yaw');
addSymmetricConstraint('left_elbow_yaw','right_elbow_yaw');
addAntiSymmetricConstraint('left_wrist_roll','right_wrist_roll');
addAntiSymmetricConstraint('left_wrist_pitch','right_wrist_pitch');
addSymmetricConstraint('left_wrist_yaw','right_wrist_yaw');
prog = addStateConstraint(prog,LinearConstraint(zeros(7,1),zeros(7,1),A),1:N);

% periodicity constraint
% frame = getStateFrame(p);
% lb = Point(frame);
% ub = Point(frame);
% lb.base_x = -Inf; ub.base_x = Inf;
% lb = Point(frame,-Inf);
% ub = Point(frame,Inf);
% lb.base_z = 0; ub.base_z = 0;
% A = [speye(size(lb,1)),-speye(size(lb,1))];
% prog = addStateConstraint(prog,LinearConstraint(double(lb),double(ub),A),{[1 N]});

% initial state constraint
frame = getStateFrame(p);
lb = Point(frame,-Inf);
ub = Point(frame,Inf);
lb.base_x = 0; ub.base_x = 0;
lb.base_z = 2; ub.base_z = 2;
prog = addStateConstraint(prog,BoundingBoxConstraint(double(lb),double(ub)),1);
xstar(1) = 0; 
xstar(3) = 2;

% the cost functions
prog = prog.addRunningCost(@(t,x,u)cost(t,x,u,xstar,ustar,getStateFrame(p),getInputFrame(p)));
prog = prog.addFinalCost(@finalcost);

if (nargin<2)
  display('Computing the initial guess...')
  tf0 = min(2*minimum_duration,maximum_duration);
  utraj0 = ConstantTrajectory(ustar);
  utraj0 = setOutputFrame(utraj0,getInputFrame(p));
  sys0 = cascade(utraj0,p);
  xtraj0 = simulate(sys0,[0 tf0],xstar);
else
  display('Loading intitial guess...')
  seed = load(seedtraj);
  xtraj0 = seed.xtraj;
  utraj0 = seed.utraj;
  tf0 = seed.xtraj.tspan(2);
end
traj_init.x = xtraj0;
traj_init.u = utraj0;

display('Starting trajectory optimization...')
tic
prog = prog.setSolverOptions('snopt','print','runDircolFlapping.out');
[xtraj,utraj,~,~,~] = prog.solveTraj(tf0,traj_init);
toc

if (nargin>0)
  save(outputfile,'xtraj','utraj');
end

if (nargout<1)
  v = constructVisualizer(p);
  v.playback_speed = .2;
  v.playback(xtraj,struct('slider',true));
end

end

function C = addCost(C,frame,name,cost)
    position = findCoordinateIndex(frame,name);
    C(position,position) = cost;
end

function [g,dg] = cost(t,x,u,xstar,ustar,stateframe,inputframe)

%     Q = 10*eye(numel(x));
%     Q = addCost(Q,stateframe,'base_zdot',1000);
%     Q = addCost(Q,stateframe,'left_shoulder_pitch',1000);
%     Q = addCost(Q,stateframe,'right_shoulder_pitch',1000);
%     R = 100*eye(numel(u));
%     g = (x-xstar)'*Q*(x-xstar) + (u-ustar)'*R*(u-ustar);
%     if (nargout>1)
%       dg = [0,2*(x-xstar)'*Q,2*(u-ustar)'*R];
%     end

%     Q = zeros(numel(x));
%
%     Q = addCost(Q,stateframe,'base_pitch',5);
%     Q = addCost(Q,stateframe,'tail_pitch',5);
%     Q = addCost(Q,stateframe,'left_elbow_yaw',10);
%     Q = addCost(Q,stateframe,'right_elbow_yaw',10);
%     Q = addCost(Q,stateframe,'left_shoulder_yaw',10);
%     Q = addCost(Q,stateframe,'right_shoulder_yaw',10);   
%     Q = addCost(Q,stateframe,'left_shoulder_pitch',10);
%     Q = addCost(Q,stateframe,'right_shoulder_pitch',10);
%     Q = addCost(Q,stateframe,'left_wrist_yaw',10);
%     Q = addCost(Q,stateframe,'right_wrist_yaw',10);
%     Q = addCost(Q,stateframe,'left_wrist_pitch',50);
%     Q = addCost(Q,stateframe,'right_wrist_pitch',50);
%     
%     Q = addCost(Q,stateframe,'base_xdot',10000);
%     Q = addCost(Q,stateframe,'base_zdot',500);
%     Q = addCost(Q,stateframe,'base_pitchdot',5);
%     Q = addCost(Q,stateframe,'tail_pitchdot',10);
%     
%     Q = addCost(Q,stateframe,'left_shoulder_rolldot',5);
%     Q = addCost(Q,stateframe,'left_shoulder_pitchdot',5);
%     Q = addCost(Q,stateframe,'left_shoulder_yawdot',5);
%     Q = addCost(Q,stateframe,'left_elbow_yawdot',5);
%     Q = addCost(Q,stateframe,'left_wrist_rolldot',5);
%     Q = addCost(Q,stateframe,'left_wrist_pitchdot',5);
%     Q = addCost(Q,stateframe,'left_wrist_yawdot',5);
%     Q = addCost(Q,stateframe,'right_shoulder_rolldot',5);
%     Q = addCost(Q,stateframe,'right_shoulder_pitchdot',5);
%     Q = addCost(Q,stateframe,'right_shoulder_yawdot',5);
%     Q = addCost(Q,stateframe,'right_elbow_yawdot',5);
%     Q = addCost(Q,stateframe,'right_wrist_rolldot',5);
%     Q = addCost(Q,stateframe,'right_wrist_pitchdot',5);
%     Q = addCost(Q,stateframe,'right_wrist_yawdot',5);
%     
%     R = eye(numel(u));
%     
%     g = x'*Q*x + u'*R*u;
%     if (nargout>1)
%       dg = [0,2*x'*Q,2*u'*R];
%     end
%     
%     % this needs to have an x0 constraint to work
%     g = g + 500*(x(1)-15)^2;
%     if (nargout>1)
%       dg = dg + [0,500*2*(x(1)-15),zeros(1,numel(x)-1+numel(u))];
%     end
%     
%     g = g + 10000*(x(3)-2)^2;
%     if (nargout>1)
%       dg = dg + [0,0,0,10000*2*(x(3)-2),zeros(1,numel(x)-3+numel(u))];
%     end

    R = 100*eye(numel(u));
    g = (x(1)-15)^2+(x(3)-2)^2+u'*R*u;
    if (nargout>1)
        dg = [0,2*(x(1)-15),0,2*(x(3)-2),zeros(1,numel(x)-3),2*u'*R];
    end

end

function [h,dh] = finalcost(t,x)
  h = 0.01*t;
  if (nargout>1)
    dh = [0.01,zeros(1,numel(x))];
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
  plot(x(1,:),x(3,:));
  title('z vs x');
  
  xtraj = PPTrajectory(spline(tt,x));
  v = v.setInputFrame(xtraj.getOutputFrame());
  v.playback_speed = .2;
  v.playback(xtraj);
end