local template = [[
${heading}

classdef «here.name» < handle

properties
    «here.members.constants»
@ for _, link in sorted_links(robot.isFloatingBase) do
    «here.members.linkip(link)»
@ end
end

methods
    function obj = «here.name»(«here.farguments.model_constants», «here.farguments.model_parameters»)
        obj.«here.members.constants» = «here.farguments.model_constants»;
@ for name, link in sorted_links(robot.isFloatingBase) do
@   local ip = inertial_data.byLinkName(name)
        obj.«here.members.linkip(link)» = «ns_qualifier»RigidBodyInertia(...
            «valueof(ip.mass)»,...
            [«valueof(ip.com.x)»; «valueof(ip.com.y)»; «valueof(ip.com.z)»],...
            «ns_qualifier»RigidBodyInertia.makeTensor3D(«valueof(ip.moments.ixx)»,«valueof(ip.moments.iyy)»,«valueof(ip.moments.izz)»,«valueof(ip.moments.ixy)»,«valueof(ip.moments.ixz)»,«valueof(ip.moments.iyz)»)...
        );
@ end
    end
@ if robot.inertia.isParametric then
@   local valueof = function(x) return valueof(x, 'obj.'..here.members.constants) end

    function updateParameters(obj, «here.farguments.model_parameters»)
@  for link, flags in pairs(robot.inertia.parametric_flags) do
@    local ip = inertial_data.byLink(link)
@    if flags.allParametric() then
        % the inertial properties of «link.name» are all parametric... recreate the tensor
        obj.«here.members.linkip(link)» = «ns_qualifier»RigidBodyInertia(...
            «valueof(ip.mass)»,...
            ,...
            «ns_qualifier»RigidBodyInertia.makeTensor3D(«valueof(ip.moments.ixx)»,«valueof(ip.moments.iyy)»,«valueof(ip.moments.izz)»,«valueof(ip.moments.ixy)»,«valueof(ip.moments.ixz)»,«valueof(ip.moments.iyz)»)...
        );
@    else
@      if flags.parametricMass() then
            obj.«here.members.linkip(link)».changeMass(«valueof(ip.mass)»);
@      end
@      if flags.parametricCoM() then
            obj.«here.members.linkip(link)».changeCoM([«valueof(ip.com.x)»; «valueof(ip.com.y)»; «valueof(ip.com.z)»]);
@      end
@      if flags.parametricTensor() then
            obj.«here.members.linkip(link)».changeRotationalInertia(«ns_qualifier»RigidBodyInertia.makeTensor3D(«valueof(ip.moments.ixx)»,«valueof(ip.moments.iyy)»,«valueof(ip.moments.izz)»,«valueof(ip.moments.ixy)»,«valueof(ip.moments.ixz)»,«valueof(ip.moments.iyz)»));
@      end
@    end
@  end
    end
@ end
end

end

]]

return template
