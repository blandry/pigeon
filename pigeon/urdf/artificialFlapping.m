function [xtraj,utraj] = artificialFlapping

trim = load('trimConditions.mat');
xstar = trim.xstar;
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
trimstateframe = getStateFrame(p);

dt = .001;
options.floating = true;
p = TimeSteppingRigidBodyManipulator('pigeon.URDF', dt, options);
p = enableIdealizedPositionControl(p,true);
p = p.compile();
inputframe = getInputFrame(p);
stateframe = getStateFrame(p);
u = Point(inputframe);
for i=1:size(u,1)
    state = u.frame.coordinates{i};
    if findCoordinateIndex(trimstateframe,state)
        u.(state) = xstar(findCoordinateIndex(trimstateframe,state));
    end
end
x0 = Point(stateframe);
for i=1:size(x0,1)
    state = x0.frame.coordinates{i};
    if findCoordinateIndex(trimstateframe,state)
        x0.(state) = xstar(findCoordinateIndex(trimstateframe,state));
    end
end

tf = .2;
N = tf/dt;
uu = repmat(double(u),1,N);

left_shoulder_roll = findCoordinateIndex(inputframe,'left_shoulder_roll');
right_shoulder_roll = findCoordinateIndex(inputframe,'right_shoulder_roll');
left_shoulder_pitch = findCoordinateIndex(inputframe,'left_shoulder_pitch');
right_shoulder_pitch = findCoordinateIndex(inputframe,'right_shoulder_pitch');
left_wrist_roll = findCoordinateIndex(inputframe,'left_wrist_roll');
right_wrist_roll = findCoordinateIndex(inputframe,'right_wrist_roll');
sinus = sin(4*2*pi*(1:N)/N);
for j=1:N
    uu(left_shoulder_roll,j)=uu(left_shoulder_roll,j)-sinus(j);
    uu(right_shoulder_roll,j)=uu(right_shoulder_roll,j)+sinus(j);
    uu(left_shoulder_pitch,j)=uu(left_shoulder_pitch,j)+.2*sinus(j);
    uu(right_shoulder_pitch,j)=uu(right_shoulder_pitch,j)+.2*sinus(j);
    uu(left_wrist_roll,j)=uu(left_wrist_roll,j)-.1*sinus(j);
    uu(right_wrist_roll,j)=uu(right_wrist_roll,j)+.1*sinus(j);
end

utraj = PPTrajectory(spline(linspace(0,tf,N),uu));
utraj = setOutputFrame(utraj,getInputFrame(p));
sys = cascade(utraj,p);
xtraj = simulate(sys,[0 tf],x0);

if (nargout<1)
  v = p.constructVisualizer();
  v.playback_speed = .2;
  v.playback(xtraj,struct('slider',true));
end

end

