local function scalar_tpl_utils(scalar_t, class, do_templates)
    if do_templates == nil then
        do_templates = true
    end
    if do_templates then
        return {
            heading  = 'template<typename ' .. scalar_t .. '>',
            scalar_t = scalar_t,
            class= {
                name = class,
                in_qualifier = class .. '<'..scalar_t..'>'
            },
            suffix = '<'..scalar_t..'>'
        }
    else
        return {
            heading  = '',
            scalar_t = scalar_t,
            class= {
                name = class,
                in_qualifier = class
            },
            suffix = ''
        }
    end
end


local function commons(robot, transforms, configurator)
    local tfMeta = configurator.transformsContainerMeta
    return {
        jointIdentifier = configurator.txtCfg.ids.joint,
        linkIdentifier  = configurator.txtCfg.ids.link,
        spatialVectorIndex = configurator.data.iitrbd.spatialVectorIndex,
        iitrbd = configurator.data.iitrbd,
        scalarTpl = function(class)
            return scalar_tpl_utils(configurator.txtCfg.types.scalar, class, configurator.templateAll())
        end,

        constantValueAccess = nil, -- will be set by the main Generator

        transformsContainerMeta = tfMeta,

        link_CT_parent = function(link, container)
            local ctMeta = transforms.link_CT_parent__byLink[link]
            return container .. '.' .. tfMeta.members.transform(ctMeta)
        end,

        --TODO do no hardcode here the strings and the policy on how to
        -- construct the member function; should get them from config
        parent_XF_link = function(link, container)
            -- note here I am asking for the opposite polarity, link-to-parent
            local ctMeta = transforms.link_CT_parent__byLink[link]
            return container .. '.' .. tfMeta.members.transform(ctMeta) ..
                '.' .. ctMeta.ct.rightFrame.name .. '_XF_' .. ctMeta.ct.leftFrame.name .. '()'
        end,
        link_XM_parent = function(link, container)
            local ctMeta = transforms.link_CT_parent__byLink[link]
            return container .. '.' .. tfMeta.members.transform(ctMeta) ..
                '.' .. ctMeta.ct.leftFrame.name .. '_XM_' .. ctMeta.ct.rightFrame.name .. '()'
        end
    }
end

cppcommon = commons -- deprecate

rcg__cppcommon = commons

return commons
