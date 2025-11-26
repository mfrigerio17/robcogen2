
local mx_class_name = function(matrixMetadata)
    ctr  = matrixMetadata.ctMetadata.ct
    kind = matrixMetadata.representationKind.name
    key = 'X' -- default
    if    (kind == "homogeneous") then key = 'xh'
    elseif(kind == "spatial_motion") then key = 'xm'
    elseif(kind == "spatial_force") then key = 'xf'
    elseif(kind == "pure_rotation") then key = 'rot'
    end

    return ctr.leftFrame.name .. '_' .. key .. '_' .. ctr.rightFrame.name
end

--- The configuration table for the Octave generator of CtGen, which is invoked
-- by RobCoGen.
-- Override here only the default values from CtGen which we need to change, or
-- that we need to refer to.
--
local config = {
    meta = {
        constants_class = {
            class_name = function(ctModel) return 'ModelGeometryConstants' end,
        },
        tf_class = {
            class_name = mx_class_name,
            methods = {
                update_parameters = 'updateParameters',
                update = 'update',
            },
        },
    },
    variables = {
        status_formal_parameter = 'q',
        --value_expression -- we have to patch this at runtime
    },
}

return config
