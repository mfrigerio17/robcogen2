local template = [[
${heading}

classdef «thisclass.name» < handle
properties
    «thisclass.members.parameters»
@for ct_name, mx_meta in pairs(matrices_metadata) do
    «thisclass.members.individual_tf(mx_meta)»
@end
end

methods
    function obj = «thisclass.name»(«thisclass.farguments.constants», «thisclass.farguments.parameters»)
        obj.«thisclass.members.parameters» = «thisclass.farguments.parameters»;
@for ct_name, mx_meta in pairs(matrices_metadata) do
        obj.«thisclass.members.individual_tf(mx_meta)» = «ns_qualifier»«tf_class_name(mx_meta)»(«thisclass.farguments.constants»);
@end
    end

    function «thisclass.methods.update_all»(obj, «thisclass.farguments.vars_state»)
@for ct_name, mx_meta in pairs(matrices_metadata) do
        obj.«thisclass.members.individual_tf(mx_meta)».«ctgenMeta.tf_class.methods.update»(«thisclass.farguments.vars_state»);
@end
    end

    function «thisclass.methods.update_parameters»(obj, «thisclass.farguments.parameters»)
        obj.«thisclass.members.parameters» = «thisclass.farguments.parameters»;
@for ct_name, mx_meta in pairs(matrices_metadata) do
@  if mx_meta.ctMetadata.is_parametric then
@    local member = thisclass.members.individual_tf(mx_meta)
@    local method = ctgenMeta.tf_class.methods.update_parameters
        obj.«member».«method»(«tf_params_as_func_arguments(mx_meta, thisclass.farguments.parameters)»);
@  end
@end
    end
end
end
]]


return template
