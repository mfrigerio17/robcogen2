local config = {
    meta = {
        class_inertia_constants = {
            name = "ModelInertiaConstants",
            members = {
                constant = function(constant) return constant.name end,
            },
        },
        class_inertia_parameters = {
            name = "ModelInertiaParameters",
            members = {
                parameter = function(param) return param.name end
            }
        },
        class_geom_parameters = {
            name = "ModelGeometryParameters",
            members = {
                parameter = function(param) return param.name end,
            },
        },
        class_inertia_properties = {
            name = "InertiaProperties",
            members = {
                linkip = function(link) return link.name end,
                constants = "constants",
            },
            farguments = {
                model_constants = "mc",
                model_parameters= "mp",
            },
        },
        class_transforms_container = {
            name = "MotionTransforms",
            farguments = {
                constants = "mc",
                parameters = "mp",
                vars_state = "q",
            },
            members = {
                parameters = "parameters",
                individual_tf = function(mxMetadata) return mxMetadata.ctMetadata.name end,
            },
            methods = {
                update_parameters = "updateParameters",
                update_all = "updateAll",
            }
        },
        func_inverse_dynamics = {
            name = "inverseDynamics",
            args = {
                ip = "ip",
                transforms = "xm",
                q="q", qd="qd", qdd="qdd", fext="fext",
            },
        },
        class_jsim = {
            name = "CompositeInertia",
            ctor = {
                inertia = "ip",
                transforms = "xm",
            },
            members = {
                inertia = "ip",
                transforms = "xm",
                H = "H",
                F = "F", -- only for floating bases
            },
            methods = {
                update_ci = "update_composite_inertia",
            },
        },
        func_roys_model = {
            name = function(robot) return "RoyModel" end,
            args = {
                inertia_constants   = "mic",
                inertia_parameters  = "mip",
                geometry_constants  = "mgc",
                geometry_parameters = "mgp",
            },
            ret_val = "roy",
        },
        func_init = {
            name = "init",
        },
    },
    ids = {
        dyn_vars = {
            I     = function(link) return 'I_' .. link.name end,
            Ic    = function(link) return 'Ic_' .. link.name end,
            IA    = function(link) return 'IA_' .. link.name end,
            biasF = function(link) return 'p_' .. link.name end,
            acc   = function(link) return 'a_' .. link.name end,
            vel   = function(link) return 'v_' .. link.name end,
            force = function(link) return 'f_' .. link.name end,
            biasA = function(link) return 'c_' .. link.name end,
            T     = function(link) return 'T_' .. link.name end,
        },
    },

    --- A list of names to be used as the namespace for the generated code.
    -- More than one item will result in nested namespaces.
    -- Due to the way Octave/Matlab namespaces work, this option will also
    -- affect the output folders where the code is generated.
    namespaces = function(robot)
        --return {robot.name, 'rcg2'}
        return {'rcg2'}
    end,

}


return config
