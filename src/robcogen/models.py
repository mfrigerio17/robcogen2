from enum import Enum, auto
import itertools
import numbers
from dataclasses import dataclass

import robmodel.connectivity
import robmodel.treeutils
import robmodel.jposes

import kgprim.core
import kgprim.values as expr
import kgprim.motions as motions
import kgprim.ct as ct
from kgprim.ct.models import CoordinateTransformPlaceholder
import kgprim.ct.repr.mxrepr as mxrepr
from kgprim.ct.repr.mxrepr import MatrixRepresentation
from kgprim.ct.repr.mxrepr import MatrixReprMetadata

from robcogen import logger


class TransformsModelWrapper:
    def __init__(self, robot, uderDesiredTransforms):
        posesSpecsModel = motions.PosesSpec(name=robot.name, poses=robot.kinematics.allPosesSpecs())

        # Default desired transforms. In the form `child_CT_parent`
        # This means that in the generated C++ code using the iit-rbd backend, the
        # transform view 'A_X_B' is 'child_CT_parent', the 'B_X_A' is viceversa
        tree   = robot.tree
        frames = robot.frames
        desTransforms = []
        links = []
        for l in tree.links.values() :
            if l != tree.base :
                leftFrame = frames.linkFrames[ l ]
                rightFrame= frames.linkFrames[ robot.treeutils.parent(l) ]
                cttoken = CoordinateTransformPlaceholder(leftFrame=leftFrame, rightFrame=rightFrame)
                desTransforms.append( cttoken )
                links.append( l )

        ctModel = ct.frommotions.motionsToCoordinateTransforms(posesSpecsModel, desTransforms, robot.name+'-geometry')
        ctModelMeta = ct.metadata.TransformsModelMetadata(ctModel)

        allMxMeta = {}
        for mxType in [MatrixRepresentation.homogeneous,
                        MatrixRepresentation.spatial_motion, MatrixRepresentation.spatial_force ]:
            byName = {}
            for ctMeta in ctModelMeta.transformsMetadata :
                MX     = mxrepr.symbolic[mxType](ctMeta.ct)
                mxMeta = MatrixReprMetadata(ctMeta, MX, mxType)
                byName[ctMeta.name] = mxMeta

            allMxMeta[mxType] = byName;

        self.mxMetadataByType = allMxMeta

        link_CT_parent__byLink = {}
        for i in range(0, len(links) ) :
            link_CT_parent__byLink[ links[i] ] = ctModelMeta.transformsMetadata[i]

        self.robot = robot
        self.ctModelMeta = ctModelMeta
        self.link_CT_parent__byLink = link_CT_parent__byLink

    @property
    def modelMetadata(self):
        return self.ctModelMeta

    def allMatricesMetadata(self, matrixType=MatrixRepresentation.homogeneous):
        if matrixType not in self.mxMetadataByType:
            logger.error("No metadata for matrix type '{}'".format(matrixType))
            return []
        return self.mxMetadataByType[matrixType]

    def link_X_parent__tfMetadata(self, link):
        return self.link_CT_parent__byLink.get(link)



class IPField(Enum):
    '''
    Enumeration of all the scalar values of a rigid body inertia
    '''
    mass = 1
    comx = 2
    comy = 4
    comz = 8
    ixx  = 16
    iyy  = 32
    izz  = 64
    ixy  = 128
    ixz  = 256
    iyz  = 512

@dataclass(eq=True, frozen=True)
class InertiaPropertyBacktrackingData:
    link    : kgprim.core.RigidBody
    ipfield : IPField

@dataclass(eq=True, frozen=True)
class KinematicPropertyBacktrackingData:
    pose  : kgprim.core.Pose
    mstep : kgprim.motions.MotionStep


class ExpressionsOfAQuantity:
    '''
    The sequence of unique `kgprim.values.Expression`s for a given quantity
    appearing in the robot model.
    A "quantity" is either a `kgprim.values.Parameter` or `kgprim.values.Constant`
    which appears in the definition of an attribute of the robot model.
    '''

    def __init__(self, quantity):
        '''
        Parameters:

        - `quantity`: either a `kgprim.values.Parameter` or `kgprim.values.Constant` which
           is part of the definition a property of the robot model
        '''
        self.quantity_ = quantity
        self.expressions_ = dict() # preserves insertion order

    def addExpression(self, expr, backtrackingData):
        '''
        Add a `kgprim.values.Expression` to this list.
        The argument of the expression must be the same quantity stored in this
        instance.
        '''
        if expr.arg != self.quantity:
            logger.error("Argument mismatch: got an expression of '%s', expecting '%s'", expr.arg.name, self.quantity.name)
        else:
            seq = self.expressions_.setdefault(expr, [])
            seq.append(backtrackingData)

    @property
    def quantity(self):
        return self.quantity_

    @property
    def expressions(self):
        return self.expressions_.keys()

    def backtrackingData(self, expression):
        return self.expressions_.get(expression, [])

    def isIdentity(self, expression):
        '''
        Returns true if the given expression is simply the identity, i.e., it is
        just a wrapper of a symbol.
        '''
        # see the API of kgprim.values.Expression
        return expression.arg.symbol == expression.expr


class RobotModel:
    '''
    The robot model data structure used by `robcogen.*` classes and functions.

    This class composes together the different aspects of a robot model as
    represented by the classes of the `robmodel` package.

    It also adds some metadata and functions which are required by the robcogen
    code generators.
    '''

    def __init__(self, geometryModel, inertia, floatingBase=False):
        self.fb = floatingBase
        self.frames    = geometryModel.framesModel
        self.tree      = geometryModel.connectivityModel
        self.treeutils = robmodel.treeutils.TreeUtils(self.tree)
        self.kinematics= RobotKinematics(geometryModel)
        self.inertia   = RobotInertia(self, inertia)
        #TODO save the link-frame inertia properties

        movingLinks = self.tree.links # the default, by-name dictionary of links
        if not floatingBase:
            movingLinks = self.tree.links.copy() # shallow copy
            movingLinks.pop(self.tree.base.name)
        self.movLinks = movingLinks

        noBase = self.tree.links.copy() # shallow copy
        noBase.pop(self.tree.base.name)
        self.linksNoBase = noBase

        self.hasPrismaticJoint = any([j.kind==robmodel.connectivity.JointKind.prismatic
                                            for j in self.tree.joints.values()])
        self.hasRevoluteJoint  = any([j.kind==robmodel.connectivity.JointKind.revolute
                                            for j in self.tree.joints.values()])

    @property
    def name(self):
        return self.tree.name

    @property
    def base(self):
        return self.tree.base

    @property
    def isFloatingBase(self):
        return self.fb

    @property
    def movingLinks(self):
        return self.movLinks

    @property
    def hasParametricGeometry(self):
        return len(self.kinematics.parameters) > 0

    @property
    def DOFs(self):
        return self.tree.nJ + 6*self.isFloatingBase
        # NOTE: this relies on the assumption that any joint is 1 DOF...

    @property
    def properDOFs(self):
        return self.tree.nJ
        # NOTE: this relies on the assumption that any joint is 1 DOF...

    def allConstantsIter(self):
        return itertools.chain(
                self.inertia.constants,
                self.kinematics.constants.keys() )




ipgetter = {
    IPField.mass : (lambda bodyInertia : bodyInertia.mass),
    IPField.comx : (lambda bodyInertia : bodyInertia.com.x),
    IPField.comy : (lambda bodyInertia : bodyInertia.com.y),
    IPField.comz : (lambda bodyInertia : bodyInertia.com.z),
    IPField.ixx  : (lambda bodyInertia : bodyInertia.moments.ixx),
    IPField.iyy  : (lambda bodyInertia : bodyInertia.moments.iyy),
    IPField.izz  : (lambda bodyInertia : bodyInertia.moments.izz),
    IPField.ixy  : (lambda bodyInertia : bodyInertia.moments.ixy),
    IPField.ixz  : (lambda bodyInertia : bodyInertia.moments.ixz),
    IPField.iyz  : (lambda bodyInertia : bodyInertia.moments.iyz)
}

class RobotInertia:
    '''
    A wrapper of the container `robmodel.inertia.RobotLinksInertia`, with
    additional metadata required by RobCoGen internals.
    '''

    def __init__(self, robot, inertia):
        self.parameters_ = {}
        self.constants_  = {}
        self.pflags = {}
        self.cflags = {}
        # since python3.7 dictionaries preserve insertion order
        self.inputModel = inertia

        self._isParametric = False
        for link in robot.tree.links.values():
            self.cflags[link] = set()
            self.pflags[link] = RobotInertia.ParametricFlags()
            self._registerProperties(link, inertia.byLink(link))

    @property
    def isParametric(self):
        return self._isParametric

    def isParameter(self, candidate):
        return candidate in self.parameters_

    def isConstant(self, candidate):
        return candidate in self.constants

    def _registerProperties(self, link, ip):
        if ip is None:
            return
        for __, field in IPField.__members__.items():
            prop = ipgetter[field](ip)

            if isinstance(prop, expr.Expression) :
                backtrack = InertiaPropertyBacktrackingData(link, field)
                quantity = prop.arg

                if isinstance(quantity, expr.Parameter) :
                    exprs = self.parameters_.setdefault(quantity, ExpressionsOfAQuantity(quantity))
                    exprs.addExpression(prop, backtrack)
                    self.pflags[link].add( field )
                    self._isParametric = True

                elif isinstance(quantity, expr.Constant) :
                    exprs = self.constants_.setdefault(quantity, ExpressionsOfAQuantity(quantity))
                    exprs.addExpression(prop, backtrack)
                    self.cflags[link].add( field )
            else:
                assert( isinstance(prop, numbers.Number) )

    @property
    def constants(self):
        return self.constants_.keys()

    def expressionsOfConstant(self, constant):
        return self.constants_[constant]

    @property
    def parameters(self):
        return self.parameters_.keys()

    @property
    def parametric_flags(self):
        return self.pflags

    @property
    def actual_data(self):
        return self.inputModel

    class ParametricFlags:
        com = IPField.comx.value + IPField.comy.value + IPField.comz.value
        im  = IPField.ixx.value + IPField.iyy.value + IPField.izz.value +\
              IPField.ixy.value + IPField.ixz.value + IPField.iyz.value

        def __init__(self):
            self.flags = 0

        def add(self, field):
            self.flags = self.flags + field.value

        def allParametric(self):
            return self.parametricMass() and self.parametricCoM() and self.parametricTensor()

        def parametricMass(self):
            return bool(self.flags & IPField.mass.value)

        def parametricCoM(self):
            return bool(self.flags & RobotInertia.ParametricFlags.com)

        def parametricTensor(self):
            return bool(self.flags & RobotInertia.ParametricFlags.im)


from kgprim.motions import MotionStep

class RobotKinematics:
    def __init__(self, in_geometry):
        jointPoses = robmodel.jposes.JointPoses(
            in_geometry.connectivityModel,
            in_geometry.framesModel,
            in_geometry.jointAxes)

        self.poseSpecByPose = {**in_geometry.byPose, **jointPoses.poseSpecByPose}
        self.jointPosesByJoint = jointPoses.poseSpecByJoint
        self.geomPosesByJoint  = in_geometry.byJoint
        self.jointToSymVar = jointPoses.jointToSymVar
        self.symVarToJoint = jointPoses.symVarToJoint
        self.baseFrame = in_geometry.framesModel.linkFrames[ in_geometry.connectivityModel.base ]

        self.constants   = {}
        self.parameters_ = {}
        self.registerArguments()

    @property
    def parameters(self):
        return self.parameters_.keys()

    def isParameter(self, candidate):
        return candidate in self.parameters

    def isConstant(self, candidate):
        return candidate in self.constants

    def allPosesSpecs(self):
        return self.poseSpecByPose.values()

    def getPoseSpec(self, pose):
        return self.poseSpecByPose[pose]

    def getJointWrtPredecessorPose(self, joint):
        return self.geomPosesByJoint[joint]

    def getSuccessorWrtJointPose(self, joint):
        return self.jointPosesByJoint[joint]

    def registerArguments(self):
        for poseSpec in self.poseSpecByPose.values() :
            motionPath = poseSpec.motion
            for motionSequence in motionPath.sequences :
                for motionStep in motionSequence.steps :
                    amount = motionStep.amount
                    if isinstance(amount, expr.Expression) :
                        backtrack = KinematicPropertyBacktrackingData(poseSpec.pose, motionStep)
                        quantity = amount.arg

                        if isinstance(quantity, expr.Parameter) :
                            if quantity not in self.parameters_ :
                                self.parameters_[quantity] = ExpressionsOfAQuantity(quantity)
                            self.parameters_[quantity].addExpression( amount, backtrack )

                        elif isinstance(quantity, expr.Constant) :
                            if quantity not in self.constants :
                                self.constants[quantity] = ExpressionsOfAQuantity(quantity)
                            self.constants[quantity].addExpression( amount, backtrack )


