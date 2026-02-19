local function commons(robot, transforms, configurator)

    local function spatialVectorIndex(joint)
        local lookup = { prismatic=6, revolute=3 }
        if lookup[joint.kind.name] == nil then
            error("Unknown spatial vector index for joint "..joint.name,2)
        end
        return lookup[joint.kind.name]
    end

    local function jointStateVectorIndex(joint)
        return robot.tree.jointNum(joint)
    end

    local function symbolicVariableToJoint(variable)
        return robot.kinematics.symVarToJoint[variable]
    end

    local function linkArrayIndex(link)
    -- Matlab arrays indexing is 1-based.
    -- We assume any array to have data only for the links that move (i.e. not the fixed base).
    -- The numeric ID of the robot base is always 0 (fixed or floating)
        if robot.isFloatingBase then
            return robot.tree.linkNum(link)+1
        else
            return robot.tree.linkNum(link)
        end
    end

    local function link_X_parent__matrixMetadata(link, matrixType)
        local ctMetadata = transforms.link_X_parent__tfMetadata(link)
        if ctMetadata then
            return (transforms.allMatricesMetadata(matrixType))[ctMetadata.name]
        end
    end


    return {
        linear_Z_coordinate = 6,
        spatialVectorIndex = spatialVectorIndex,
        jointStateVectorIndex = jointStateVectorIndex,
        symbolicVariableToJoint = symbolicVariableToJoint,
        linkArrayIndex = linkArrayIndex,
        link_X_parent__matrixMetadata = link_X_parent__matrixMetadata,
    }
end


return commons
