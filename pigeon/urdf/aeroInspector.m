function aeroInspector(urdf)
    if (nargin<1)
        urdf = 'pigeon.URDF';
    end
    megaclear;
    options.floating = true;
    p = RigidBodyManipulator(urdf,options);
    v = p.constructVisualizer();
    v.inspector(zeros(getNumStates(p),1),1:getNumStates(p));
end