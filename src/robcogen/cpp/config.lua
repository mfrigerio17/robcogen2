

local scalar_type_name = 'Scalar'

local config =
{
    namespaces = function(robot)
        return { robot.name:lower(), 'rcg2' }
    end,
    types = {
        jointState = 'JointState',
        scalar = scalar_type_name,
        scalarTraits = 'ScalarTraits',
        vec3 = 'Vector3',
        jointIDs = 'JointIDs',
        linkIDs = 'LinkIDs',
        externalForces = 'ExtForces',
    },
    ids = {
        joint = function(joint) return joint.name end,
        link  = function(link)  return link.name  end,
        jointStateFormalParameter = 'q'
    },
    classes = {
        coreTypes = 'CoreTypes', -- used only when templating-everything
        transforms= 'Transforms'
    },
    vars = {
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
    mxops = {
        zeroMx = 'Zero',
        setzero = 'setZero',
        row = 'row',
        col = 'col',
        T = 'transpose'
    },

    opts = {
        template_all = false,
    },

    make = {
        libName = function(robot) return robot.name:lower() .. 'rcg2' end,
        incPath = function(robot) return robot.name:lower() .. '/rcg2/' end
    },

    meta = {
        constants = {
            class = 'ModelConstants',
            avoid_constexpr = false,
        },
        inertia_properties = {
            class = 'InertiaProperties',
            members = {
                tensorGetter = function(link) return 'getTensor_'..link.name end,
                comGetter    = function(link) return 'getCOM_'..link.name end,
                massGetter   = function(link) return 'getMass_'..link.name end,
                paramsUpdate = 'updateParameters',
                parameters   = 'parameters'
            }
        },
        inertia_parameters = {
            class = 'RuntimeInertiaParams',
            members = {
                pvalue = function(param) return param.name end
            }
        },
    },

    internal = {
        typesMacro = 'RCG_COMMON_TYPES_ALIASES',
        includeGuard = function(robot)
            local prefix = 'RCG2_' .. robot.name:upper() .. '_'  -- do not use .upper() but :upper()
            return function(header)
                return prefix .. header:upper() .. '_H'
            end
        end
    },

}

rcg__cpp_text_config = config

return config

