
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
    -- Python dictionaries after python 3.7 preserve insertion order.
    -- Thus, regular python iteration would be sorted
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

return {
    sorted_links = sorted_links,
    sorted_links_reversed = sorted_links_reversed,
    sorted_joints = sorted_joints,
}
