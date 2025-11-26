

local template = [[
${heading}

classdef «thisclass.name» < handle

properties (Constant = true)
@ for _, constant in ipairs(robot.inertia.constants) do
    «thisclass.members.constant(constant)» = «num2str(constant.value)»
@ end
end

end

]]


return template
