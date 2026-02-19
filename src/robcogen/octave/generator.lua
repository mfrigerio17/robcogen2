local genutils  = RCG.utils.templates
local iterutils = RCG.utils.iters
local templates = RCG.octave.templates
local commonsFactory = RCG.octave.commons



local function getAllGenerators(robot, transforms, configurator)
    local _,heading = genutils.tpl_eval(templates.heading,
        {date=os.date(),model_name=robot.name},
        {returnTable=true})

    local txtConfig = configurator.getTextTemplatesConfig()
    local commons = commonsFactory(robot, transforms, configurator)

    --
    -- Some common utility functions first

    local function property_value_accessor(
        expression,
        inertia_constants_container_in_scope,
        inertia_params_container_in_scope,
        geom_constants_container_in_scope,
        geom_params_container_in_scope
    )
        if type(expression) == "number" then
            return expression
        end
        local arg = expression.arg
        local replacement = ""
        if robot.inertia.isParameter(arg) then
            replacement = inertia_params_container_in_scope..'.'..txtConfig.meta.class_inertia_parameters.members.parameter(arg)
        elseif robot.kinematics.isParameter(arg) then
            replacement = geom_params_container_in_scope..'.'..txtConfig.meta.class_geom_parameters.members.parameter(arg)
        elseif expression.arg.name == "pi" then
            replacement = "pi"
        else -- it is a constant, and it is not PI
            -- Check whether it is an inertia or geometric constant
            if robot.inertia.isConstant(arg) then
                replacement = inertia_constants_container_in_scope..'.'..txtConfig.meta.class_inertia_constants.members.constant(arg)
            elseif robot.kinematics.isConstant(arg) then
                -- CtGen is not configurable about the members of the constants
                --  class, so we need to hardcode the same policy here
                replacement = geom_constants_container_in_scope..'.'..arg.name
            else
                error("did not recognize expression argument " .. arg.name, 2)
            end
        end
        return configurator.symbolicExpressionToCode(expression.expr, {[arg.symbol]=replacement} )
    end

    local function link_XM_parent(link)
        local mxMeta = commons.link_X_parent__matrixMetadata(link, RCG.enums.MatrixRepresentation.spatial_motion)
        if not mxMeta then error("No tree transform for link "..link.name,2) end
        return mxMeta
    end

    local function link_XF_parent(link)
        local mxMeta = commons.link_X_parent__matrixMetadata(link, RCG.enums.MatrixRepresentation.spatial_force)
        if not mxMeta then error("No tree transform for link "..link.name,2) end
        return mxMeta
    end

    local ns_qualifier = table.concat(txtConfig.namespaces(robot), ".")
    if ns_qualifier ~= "" then
        ns_qualifier = ns_qualifier .. "."
    end

    --
    -- The common, shared environment for template evaluation
    local env = {
        robot     = robot,
        inertial_data = robot.inertia.actual_data, -- for the inertia properties, and for Roy's model
        transforms= transforms,
        commons   = commons,
        constants = robot.allConstantsIter(),
        parameters= robot.inertia.parameters,
        meta      = txtConfig.meta,
        ctgenMeta = txtConfig.ctgen.meta,
        heading   = heading,
        num2str   = configurator.floatsFormatter.float2str,
        pairs     = genutils.poly_pairs,
        ipairs    = genutils.poly_ipairs,
        sorted_links = iterutils.get_sorted_links_iter_factory(robot),
        sorted_links_reversed  = iterutils.get_sorted_links_iter_factory(robot, true),
        sorted_joints = iterutils.get_sorted_joints_iter_factory(robot),
        link_XM_parent = link_XM_parent,
        link_XF_parent = link_XF_parent,
        ns_qualifier =  ns_qualifier,
        enums = RCG.enums,
    }



    -- the generation functions :

    local function generator_inertia_constants()
        env.thisclass = txtConfig.meta.class_inertia_constants
        local ok, text = genutils.tpl_eval(templates.inertia_constants, env)
        return ok, text
    end

    local function generator_inertia_parameters()
        env.classname = txtConfig.meta.class_inertia_parameters.name
        env.membername= txtConfig.meta.class_inertia_parameters.members.parameter
        local ok,text = genutils.tpl_eval(templates.parameters.inertia, env)
        return ok,text
    end

    local function generator_geometry_parameters()
        env.classname = txtConfig.meta.class_geom_parameters.name
        env.membername= txtConfig.meta.class_geom_parameters.members.parameter
        local ok,text = genutils.tpl_eval(templates.parameters.geometry, env)
        return ok,text
    end

    local function generator_inertia_properties()
        local function value_access_code(expression)
            return property_value_accessor(
                expression,
                txtConfig.meta.class_inertia_properties.farguments.model_constants,
                txtConfig.meta.class_inertia_properties.farguments.model_parameters, nil, nil)
        end
        env.valueof       = value_access_code
        env.here          = txtConfig.meta.class_inertia_properties
        local ok,text = genutils.tpl_eval(templates.inertia_properties, env)
        return ok,text
    end



    local function generator_transforms_container(allMatricesMetadata)
        env.thisclass     = txtConfig.meta.class_transforms_container
        env.tf_class_name = txtConfig.ctgen.meta.tf_class.class_name
        env.matrices_metadata = allMatricesMetadata
        local f = function(mx_meta)
            local args = {}
            for _,param in genutils.poly_ipairs(mx_meta.ctMetadata.parameters) do
               table.insert(args, txtConfig.meta.class_transforms_container.farguments.parameters..
                    '.' .. txtConfig.meta.class_inertia_parameters.members.parameter(param))
            end
            return table.concat(args, ",")
        end
        env.tf_params_as_func_arguments = f
        local ok,text = genutils.tpl_eval(templates.transforms_container, env)
        return ok,text
    end

    env.spatialVelDueToJointOnly = function(joint, qd_var_name, jid)
        local i = commons.spatialVectorIndex(joint)
        local ret = {"0","0","0","0","0","0"}
        ret[i] = qd_var_name .. "(" .. tostring(jid) .. ")"
        return "[" .. table.concat(ret, ";") .. "]"
    end

    local function generator_inverse_dynamics(allMatricesMetadata)
        env.here = txtConfig.meta.func_inverse_dynamics
        env.ids  = txtConfig.ids.dyn_vars
        env.inertia = function(link)
            return env.here.args.ip..'.'..env.meta.class_inertia_properties.members.linkip(link)..'.tensor6D'
        end
        env.child_mx_parent = function(link)
            return env.here.args.transforms..'.'..env.meta.class_transforms_container.members.individual_tf( link_XM_parent(link) )..'.mx'
        end
        local ok,text = genutils.tpl_eval(templates.inverse_dynamics, env)
        return ok,text
    end


    local function generator_jsim()
        local myenv = {}
        for k,v in pairs(env) do myenv[k] = v  end

        myenv.thisclass = env.meta.class_jsim
        myenv.link_XM_parent = function(link)
            return env.meta.class_jsim.members.transforms .. '.' ..
                     env.meta.class_transforms_container.members.individual_tf(
                                                  link_XM_parent(link) )
        end
        local ok,text = genutils.tpl_eval(templates.jsim, myenv)
        return ok,text
    end


    local function generator_forward_dynamics()
        env.here = txtConfig.meta.func_forward_dynamics
        env.ids  = txtConfig.ids.dyn_vars
        env.UTermName = function(link) return link.name..'_U' end
        env.uTermName = function(link) return link.name..'_u' end
        env.DTermName = function(link) return link.name..'_D' end
        env.child_mx_parent = function(link)
            return env.here.args.transforms..'.'..env.meta.class_transforms_container.members.individual_tf( link_XM_parent(link) )..'.mx'
        end
        local ok,text = genutils.tpl_eval(templates.forward_dynamics, env)
        return ok,text
    end


    local function generator_roys_model(joint_transform_map)
        local lookup = { prismatic="'Pz'", revolute="'Rz'" }
        local jointType = function(joint)
            local ret = lookup[joint.kind.name]
            if ret == nil then
                error("Unsupported joint type "..joint.kind.name)
            end
            return ret
        end
        local metaargs = txtConfig.meta.func_roys_model.args
        local function value_access_code(expression)
            return property_value_accessor(expression,
                metaargs.inertia_constants, metaargs.inertia_parameters,
                metaargs.geometry_constants, metaargs.geometry_parameters)
        end
        local function expand_xtree_data(joint)
            local items = {}
            for i,ct in genutils.poly_ipairs(joint_transform_map[joint].primitives) do
                local f   = joint_transform_map.toSpatialV2Function(ct)
                local arg = value_access_code(ct.amount)
                if joint_transform_map.isRotation(ct) then
                    items[i] = f..'('..arg..')'
                else
                    local vec3 = {"0.0","0.0","0.0"}
                    vec3[ ct.axis.value + 1 ] = arg
                    items[i] = f..'(['..table.concat(vec3, ',')..'])'
                end
            end
            if #items == 0 then
                return "eye(6,6)"
            end
            return table.concat(items, " * ")
        end
        env.jointType = jointType
        env.thisfunc  = txtConfig.meta.func_roys_model
        env.xtree     = expand_xtree_data
        env.valueof   = value_access_code
        local ok,text = genutils.tpl_eval(templates.roys_model, env)
        return ok,text
    end


    local function generator_init_function()
        local ok,text = genutils.tpl_eval(templates.init_function, env)
        return ok,text
    end


    local function generator_test_script()
        env.tests = {
            id = "test_id",
            jsim = "test_jsim",
            fd = "test_fd",
        }
        if robot.isFloatingBase then
            for k,v in pairs(env.tests) do
                env.tests[k] = v .. "_fb"
            end
        end
        local ok,text = genutils.tpl_eval(templates.test_script, env)
        return ok,text
    end

    return {
        inertia_constants  = generator_inertia_constants,
        geometry_parameters= generator_geometry_parameters,
        inertia_parameters = generator_inertia_parameters,
        inertia_properties = generator_inertia_properties,
        transforms_container = generator_transforms_container,
        inverse_dynamics   = generator_inverse_dynamics,
        jsim               = generator_jsim,
        forward_dynamics   = generator_forward_dynamics,
        roys_model         = generator_roys_model,
        init_function      = generator_init_function,
        test_script        = generator_test_script,
    }
end

return {
    getGenerators = getAllGenerators
}
