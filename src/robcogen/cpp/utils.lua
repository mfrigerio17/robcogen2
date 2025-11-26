local i_iterator_decorator = RCG.utils.templates.i_iterator_decorator


local function ns_utils(ns_names)
    local iter_factory = function() return ipairs(ns_names) end
    local nsopen  = function(name) return 'namespace ' .. name .. ' {' end
    local nsclose = function(name) return '}' end
    return {
        open  = i_iterator_decorator(iter_factory, nsopen),
        close = i_iterator_decorator(iter_factory, nsclose),
        qualifier = table.concat(ns_names, '::')
    }
end


cpputils = {
  ns_utils = ns_utils
}

rcg__cpputils = cpputils
