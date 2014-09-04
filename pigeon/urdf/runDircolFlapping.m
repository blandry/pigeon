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

N = 21;
minimum_duration = .5;
maximum_duration = 3;
prog = DircolTrajectoryOptimization(p,N,[minimum_duration maximum_duration]); 
prog = prog.setSolverOptions('snopt','superbasicslimit',2000);
% prog = prog.setCheckGrad(true);
v = constructVisualizer(p);
prog = prog.addDisplayFunction(@(x)displayfun(x,N,p.getNumStates(),p.getNumInputs(),v));

% periodicity constraint
per_lb = -.001*ones(numstates,1);
per_ub = .001*ones(numstates,1);
per_lb(findCoordinateIndex(getStateFrame(p),'base_x')) = -Inf; 
per_ub(findCoordinateIndex(getStateFrame(p),'base_x')) = Inf;
per_A = [speye(numstates),-speye(numstates)];
periodicity_constraint = LinearConstraint(per_lb,per_ub,per_A);

roll_index = findCoordinateIndex(getStateFrame(p),'base_roll');
pitch_index = findCoordinateIndex(getStateFrame(p),'base_pitch');
yaw_index = findCoordinateIndex(getStateFrame(p),'base_yaw');

prog = prog.addStateConstraint(periodicity_constraint,{[1 N]});
prog = prog.addRunningCost(@(t,x,u)cost(t,x,u,roll_index,pitch_index,yaw_index));
prog = prog.addFinalCost(@finalcost);

display('Finding trim conditions for initial guess...')
[xstar,ustar] = findTrim(p);

tf0 = .5*(minimum_duration+maximum_duration);
utraj0 = ConstantTrajectory(ustar);
xtraj0 = ConstantTrajectory(xstar);
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

function [g,dg] = cost(t,x,u,roll_index,pitch_index,yaw_index)
  Q = zeros(numel(x));
  Q(roll_index,roll_index) = 100;
  Q(pitch_index,pitch_index) = 10;
  Q(yaw_index,yaw_index) = 100;
  R = ones(numel(u));
  g = x'*Q*x + u'*R*u;
  if (nargout>1)
    dg = [0,2*x'*Q,2*u'*R];
  end
end

function [h,dh] = finalcost(t,x)
  h = t;
  if (nargout>1)
    dh = [1,zeros(1,numel(x))];
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
  xtraj = DTTrajectory(tt,x);
  v = v.setInputFrame(xtraj.getOutputFrame());
  v.playback_speed = .2;
  display('Running vis...');
  v.playback(xtraj);
end