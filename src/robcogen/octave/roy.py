'''
Some extra facilities to help with the generation of the robot model in Roy
Featherstone's spatialV2 format
'''

from kgprim.ct.models import TransformPolarity
import kgprim.ct.frommotions as ct_from_motion

from kgprim.motions import MotionStep

def toSpatialV2Function(primitiveCoordinateTransform):
    if primitiveCoordinateTransform.kind == MotionStep.Kind.Rotation :
        axis = primitiveCoordinateTransform.axis.name.lower()
        return "rot"+axis
    else:
        return "xlt"


def buildXTreeData(robotModel):
    kin = robotModel.kinematics
    xtree = {}
    for joint in robotModel.tree.joints.values() :
        pose = kin.getJointWrtPredecessorPose(joint)
        ct = ct_from_motion.toCoordinateTransform(
            pose,
            polarity = TransformPolarity.movedFrameOnTheLeft,
            primitives_polarity = TransformPolarity.movedFrameOnTheLeft)
        xtree[joint] = ct
    xtree['toSpatialV2Function'] = toSpatialV2Function
    xtree['isRotation'] = lambda ct : (ct.kind == MotionStep.Kind.Rotation)
    return xtree
