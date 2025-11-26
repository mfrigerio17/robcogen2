
local inertia_parameters = [[
${heading}

classdef «classname» < handle

properties
@ for _, parameter in ipairs(robot.inertia.parameters) do
    «membername(parameter)» = «num2str(parameter.defaultValue)»;
@ end
end

end

]]

local geometry_parameters = [[
${heading}

classdef «classname» < handle

properties
@ for _, parameter in ipairs(robot.kinematics.parameters) do
    «membername(parameter)» = «num2str(parameter.defaultValue)»;
@ end
end

end

]]


return {
    inertia  = inertia_parameters,
    geometry = geometry_parameters,
}
