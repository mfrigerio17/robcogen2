local tplengine = require('template-text')

local function tpleval(text, env, opts, included)
    local options = opts or {}
    options.xtendStyle = true
    local ok, ret = tplengine.tload(text, options, env, included)
    if not ok then
        -- TODO use some kind of logging?
        error("Error while loading a text template: " .. table.concat(ret, "\n"), 2)
    end
    ok, ret = ret.evaluate(options)
    if not ok then
        error("Error while evaluating a text template: " .. table.concat(ret, "\n"), 2)
    end
    return ok, ret
end


local function i_iterator_decorator(iter_factory, single_item_decorator)
  return function()
    local iter, inv, ctrl = iter_factory()
    return function()
      local i, item = iter(inv, ctrl)
      ctrl = i
      if item ~= nil then
        return i, single_item_decorator( item )
      end
    end, inv, ctrl
  end
end


local function comma_separated_list(it_factory)
  local args = {}
  for k, value in it_factory() do
    table.insert(args, value)
  end
  return table.concat(args, ', ')
end

--- Iterator decorator that adds the given separator between items
--
local function i_iterator_with_separator(iter_factory, separator)

    -- Need to "look ahead" to the next item to check if it is the last, because
    -- the separator must be appended up to the second-last element, but not to
    -- the very last

    local iter, inv, ctrl = iter_factory()
    local i, item = iter(inv, ctrl)
    local myiter = function()
        if item ~= nil then
            ctrl = i
            local reti, ret = i, item   -- default return value, save current ones
            i, item = iter(inv, ctrl)   -- peek the next i,item pair
            if item then       -- the current one is not the last
                return reti, ret, separator
            end
            return reti, ret,  ""
        end
    end
    return myiter, inv, ctrl
end

-- TODO this should be called and use -i-pairs, not pairs
-- because we assume/need ordered sequences

local function poly_pairs(d)
    if type(d) == "table" then
        return pairs(d)
    elseif type(d) == "userdata" then -- we assume it is a python dictionary
        -- python dictionaries after python 3.7 preserve insertion order
        local iterator, invariant, ctrl = python.iter( d )
        local myiterator = function()
            local item = iterator(invariant, ctrl)
            ctrl = item
            if item ~= nil then
                return item, d[item]
            end
        end
        return myiterator, invariant, ctrl
    else
        error("Unsupported object type for 'poly_pairs'", 2)
    end
end


local function poly_ipairs(d)
    if type(d) == "table" then
        return ipairs(d)
    elseif type(d) == "userdata" then -- we assume it is a python dictionary
        -- python dictionaries after python 3.7 preserve insertion order
        local iterator, invariant, ctrl = python.iter( d )
        local i = 0
        local myiterator = function()
            local item = iterator(invariant, ctrl)
            ctrl = item
            if item ~= nil then
                i = i + 1
                return i, item
            end
        end
        return myiterator, invariant, 0
    else
        error("Unsupported object type for 'poly_ipairs'", 2)
    end
end

return {
    tpl_eval = tpleval,
    comma_separated_list = comma_separated_list,
    i_iterator_decorator = i_iterator_decorator,
    i_iterator_with_separator = i_iterator_with_separator,
    poly_pairs = poly_pairs,
    poly_ipairs = poly_ipairs,
}
