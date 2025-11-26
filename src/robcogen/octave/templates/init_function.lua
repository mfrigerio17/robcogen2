local template = [[
function robot = «meta.func_init.name»()

robot.mic = «ns_qualifier»«meta.class_inertia_constants.name»();
robot.mip = «ns_qualifier»«meta.class_inertia_parameters.name»();
robot.mgc = «ns_qualifier»«ctgenMeta.constants_class.class_name(transforms)»();
robot.mgp = «ns_qualifier»«meta.class_geom_parameters.name»();

robot.ip  = «ns_qualifier»«meta.class_inertia_properties.name»(robot.mic, robot.mip);
robot.xm  = «ns_qualifier»«meta.class_transforms_container.name»(robot.mgc, robot.mgp);

@if robot.isFloatingBase then
robot.ID  = @(v_base, gravity, qd, qdd, fext) «ns_qualifier»«meta.func_inverse_dynamics.name»(robot.ip, robot.xm, v_base, gravity, qd, qdd, fext);
@else
robot.ID  = @(qd, qdd, fext) «ns_qualifier»«meta.func_inverse_dynamics.name»(robot.ip, robot.xm, qd, qdd, fext);
@end

% addpath( <robcogen_root>/testing/src );
% addpath( genpath( <spatial_V2 root> ) );
% robot.roy = «ns_qualifier»«meta.func_roys_model.name(robot)»(«robot.name».mgc, «robot.name».mgp, «robot.name».mic, «robot.name».mip);
]]


return template
