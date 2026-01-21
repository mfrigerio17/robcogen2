-- Python dictionaries after python 3.7 preserve insertion order.
-- Thus, regular python iteration over dictionaries would be sorted.

-- A key-value Lua iterator over the given python dictionary, using the given
-- python iterator over the keys of the dictionary
local function items_iterator( dict, keys_iterator )
    local iterator, invariant, ctrl = keys_iterator()
    local myiterator = function()
        local key = iterator(invariant, ctrl)
        ctrl = key
        if key ~= nil then
            return key, dict[key]
        end
    end
    return myiterator, invariant, ctrl
end

-- The python dictionary of links for the given robot model.
-- Does not include the robot base unless asked in the option (see code).
local function getLinksDict(robot, option)
    local opt   = option or ""
    local links = robot.linksNoBase
    if (opt=="include_base") or (opt=="include_base_if_floating" and robot.isFloatingBase )then
        links = robot.tree.links
    end
    return links
end


--- An iterator over the links of the robot except the base, following the
-- numbering in the robot model.
-- The base is included if the given options is "include_base" or
-- "include_base_if_floating"
--
local function sorted_links(robot, option)

    local links = getLinksDict(robot, option)
    return items_iterator(links, function() return python.iter(links) end )
end

--- An iterator over the links of the robot except the base, in the reversed
-- order as the one specified by the numbering in the robot model.
-- That means, for example, that the leafs of the kinematic tree will be listed
-- first.
-- See `sorted_links` for the optional argument.
--
local function sorted_links_reversed(robot, option)
    local links = getLinksDict(robot, option)
    return items_iterator(links,
        function() return python.iter(
            python.as_attrgetter(links).__reversed__()) end )
end


local function sorted_joints(robot)
    return items_iterator(robot.tree.joints,
                      function() return python.iter(robot.tree.joints) end )
end


local function _python_iterator_factory(reversed)
    local pyiter
    if reversed then
        pyiter = function(items) return python.iter(python.as_attrgetter(items).__reversed__()) end
    else
        pyiter = function(items) return python.iter(items) end
    end
    -- Do not use the 'if' to decide the argument of python.iter, and define
    -- 'pyiter' only once, even though it is tempting. We need it like this
    -- because 'reversed' is itself a kind of iterator, thus we must ensure it
    -- is called everytime the factory that we are returning is called.
    -- We cannot call it once and use the value for the closure.

    return pyiter
end

--- Create an iterator factory for the links of the given robot.
-- The iterator returns <name,link> pairs, ordered according to the robot model.
-- Pass a true flag to this method to have iteration in the opposite order.
-- The returned iterator factory also takes an optional flag, which controls
-- whether the robot base is included in the iteration (defaults to NO).
--
-- Strictly speaking, this function is a factory of factories: it returns a
-- function like 'pairs' (which is an iterator factory), which needs to be
-- _called_ when used with the generic for. But each returned factory is itself
-- a closure on the robot links and the "reversed" option, so that they do not
-- need to be passed when doing the iteration.
--
local function get_sorted_links_iter_factory(robot, reversed)
    local pyiter = _python_iterator_factory(reversed)
    return function(opt_include_base)
        local links = robot.linksNoBase
        if opt_include_base then links = robot.tree.links end
        return items_iterator(links, function() return pyiter(links) end )
    end
end

--- Create an iterator factory for the joints of the given robot.
-- The iterator returns <name,joint> pairs, ordered according to the robot model.
-- Pass a true flag to this method to have iteration in the opposite order.
local function get_sorted_joints_iter_factory(robot, reversed)
    local pyiter = _python_iterator_factory(reversed)
    return function()
        return items_iterator(robot.tree.joints,
                               function() return pyiter(robot.tree.joints) end )
    end
end

return {
    sorted_links = sorted_links,
    sorted_links_reversed = sorted_links_reversed,
    sorted_joints = sorted_joints,
    get_sorted_links_iter_factory = get_sorted_links_iter_factory,
    get_sorted_joints_iter_factory = get_sorted_joints_iter_factory,
}
