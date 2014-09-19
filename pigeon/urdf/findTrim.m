function [xstar, ustar] = findTrim(p)

if (nargin<1)
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
  
end

prog = FixedPointProgram(p,[1;3]);  % ignores xdot, zdot
prog = prog.setSolverOptions('snopt','majoriterationslimit',2000);

frame = MultiCoordinateFrame({getStateFrame(p); getInputFrame(p)});

lb = Point(frame);
ub = Point(frame);
lb.base_z = .5;  ub.base_z = .5;
lb.base_pitch = -1.5;  ub.base_pitch = 0;

lb.tail_pitch = -1.5;  ub.tail_pitch = 1.5;

lb.left_shoulder_roll = -1.5;  ub.left_shoulder_roll = 1.5;
lb.left_shoulder_pitch = -1.5;  ub.left_shoulder_pitch = 1.5;
lb.left_shoulder_yaw = -1.5;  ub.left_shoulder_yaw = 1.5;
lb.left_elbow_yaw = -1.5;  ub.left_elbow_yaw = 1.5;
lb.left_wrist_roll = -1.5;  ub.left_wrist_roll = 1.5;
lb.left_wrist_pitch = -1.5;  ub.left_wrist_pitch = 1.5;
lb.left_wrist_yaw = -1.5;  ub.left_wrist_yaw = 1.5;

lb.right_shoulder_roll = -1.5;  ub.right_shoulder_roll = 1.5;
lb.right_shoulder_pitch = -1.5;  ub.right_shoulder_pitch = 1.5;
lb.right_shoulder_yaw = -1.5;  ub.right_shoulder_yaw = 1.5;
lb.right_elbow_yaw = -1.5;  ub.right_elbow_yaw = 1.5;
lb.right_wrist_roll = -1.5;  ub.right_wrist_roll = 1.5;
lb.right_wrist_pitch = -1.5;  ub.right_wrist_pitch = 1.5;
lb.right_wrist_yaw = -1.5;  ub.right_wrist_yaw = 1.5;

lb.base_xdot = 0;  ub.base_xdot = 50;
lb.base_zdot = -Inf; ub.base_zdot = Inf;

lb.tail_pitch_servo = -Inf; ub.tail_pitch_servo = Inf;
lb.left_shoulder_roll_servo = -Inf;  ub.left_shoulder_roll_servo = Inf;
lb.left_shoulder_pitch_servo = -Inf;  ub.left_shoulder_pitch_servo = Inf;
lb.left_shoulder_yaw_servo = -Inf;  ub.left_shoulder_yaw_servo = Inf;
lb.left_elbow_yaw_servo = -Inf;  ub.left_elbow_yaw_servo = Inf;
lb.left_wrist_roll_servo = -Inf;  ub.left_wrist_roll_servo = Inf;
lb.left_wrist_pitch_servo = -Inf;  ub.left_wrist_pitch_servo = Inf;
lb.left_wrist_yaw_servo = -Inf;  ub.left_wrist_yaw_servo = Inf;
lb.right_shoulder_roll_servo = -Inf;  ub.right_shoulder_roll_servo = Inf;
lb.right_shoulder_pitch_servo = -Inf;  ub.right_shoulder_pitch_servo = Inf;
lb.right_shoulder_yaw_servo = -Inf;  ub.right_shoulder_yaw_servo = Inf;
lb.right_elbow_yaw_servo = -Inf;  ub.right_elbow_yaw_servo = Inf;
lb.right_wrist_roll_servo = -Inf;  ub.right_wrist_roll_servo = Inf;
lb.right_wrist_pitch_servo = -Inf;  ub.right_wrist_pitch_servo = Inf;
lb.right_wrist_yaw_servo = -Inf;  ub.right_wrist_yaw_servo = Inf;

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

xdot_ind = findCoordinateIndex(getStateFrame(p),'base_xdot');
zdot_ind = findCoordinateIndex(getStateFrame(p),'base_zdot');

prog = addConstraint(prog,BoundingBoxConstraint(double(lb),double(ub)));
prog = addConstraint(prog,FunctionHandleObjective(prog.num_vars,@(x)cost(x,xdot_ind,zdot_ind)));
prog = addConstraint(prog,LinearConstraint(zeros(7,1),zeros(7,1),A));

x0 = Point(getStateFrame(p));
x0.base_x = .5;
x0.base_pitch = -0.126;
x0.left_shoulder_roll = -0.251;
x0.left_shoulder_pitch = 0.126;
x0.left_wrist_roll = 0.251;
x0.right_shoulder_roll = 0.251;
x0.right_shoulder_pitch = 0.126;
x0.right_wrist_roll = -0.251;
x0.base_xdot = 20;
num_inputs = getNumInputs(p);
u0 = zeros(num_inputs,1);

%prog = prog.setSolverOptions('snopt','print','findTrim.out');
w = prog.solve([double(x0);u0]); 
xstar = w(1:getNumStates(p));
ustar = w(getNumStates(p)+1:end);

if (nargout<1)
  utraj = ConstantTrajectory(ustar);
  utraj = setOutputFrame(utraj,getInputFrame(p));
  sys = cascade(utraj,p);
  xtraj = simulate(sys,[0 .1],xstar);
  v = p.constructVisualizer(); 
  v.playback(xtraj, struct('slider',true));
end

end

function [c,dc] = cost(x,xdot_ind,zdot_ind)
  xdot = x(xdot_ind);
  zdot = x(zdot_ind);
  c = (zdot/xdot)^2;
  dc = 0*x';
  dc(xdot_ind) = 2*(zdot/xdot)*(-zdot/xdot^2);
  dc(zdot_ind) = 2*(zdot/xdot)*(1/xdot);
end