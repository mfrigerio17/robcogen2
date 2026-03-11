#include "inertia_properties.h"


hyq::rcg2::InertiaProperties::InertiaProperties()
{
    com_trunk = Vector3(ModelConstants::trunk_comx, ModelConstants::trunk_comy, 0.0);
    tensor_trunk.fill(
        ModelConstants::trunk_mass,
        com_trunk,
        iit::rbd::Utils::buildInertiaTensor<Scalar>(ModelConstants::trunk_ixx,ModelConstants::trunk_iyy,ModelConstants::trunk_izz,ModelConstants::trunk_ixy,ModelConstants::trunk_ixz,ModelConstants::trunk_iyz)
    );
    com_LF_hipassembly = Vector3(ModelConstants::LF_hipassembly_comx, 0.0, ModelConstants::LF_hipassembly_comz);
    tensor_LF_hipassembly.fill(
        ModelConstants::LF_hipassembly_mass,
        com_LF_hipassembly,
        iit::rbd::Utils::buildInertiaTensor<Scalar>(ModelConstants::LF_hipassembly_ixx,ModelConstants::LF_hipassembly_iyy,ModelConstants::LF_hipassembly_izz,ModelConstants::LF_hipassembly_ixy,ModelConstants::LF_hipassembly_ixz,ModelConstants::LF_hipassembly_iyz)
    );
    com_LF_upperleg = Vector3(ModelConstants::LF_upperleg_comx, ModelConstants::LF_upperleg_comy, 0.0);
    tensor_LF_upperleg.fill(
        ModelConstants::LF_upperleg_mass,
        com_LF_upperleg,
        iit::rbd::Utils::buildInertiaTensor<Scalar>(ModelConstants::LF_upperleg_ixx,ModelConstants::LF_upperleg_iyy,ModelConstants::LF_upperleg_izz,ModelConstants::LF_upperleg_ixy,ModelConstants::LF_upperleg_ixz,ModelConstants::LF_upperleg_iyz)
    );
    com_LF_lowerleg = Vector3(ModelConstants::LF_lowerleg_comx, ModelConstants::LF_lowerleg_comy, ModelConstants::LF_lowerleg_comz);
    tensor_LF_lowerleg.fill(
        ModelConstants::LF_lowerleg_mass,
        com_LF_lowerleg,
        iit::rbd::Utils::buildInertiaTensor<Scalar>(ModelConstants::LF_lowerleg_ixx,ModelConstants::LF_lowerleg_iyy,ModelConstants::LF_lowerleg_izz,0.0,0.0,0.0)
    );
    com_RF_hipassembly = Vector3(ModelConstants::RF_hipassembly_comx, 0.0, ModelConstants::RF_hipassembly_comz);
    tensor_RF_hipassembly.fill(
        ModelConstants::RF_hipassembly_mass,
        com_RF_hipassembly,
        iit::rbd::Utils::buildInertiaTensor<Scalar>(ModelConstants::RF_hipassembly_ixx,ModelConstants::RF_hipassembly_iyy,ModelConstants::RF_hipassembly_izz,ModelConstants::RF_hipassembly_ixy,ModelConstants::RF_hipassembly_ixz,ModelConstants::RF_hipassembly_iyz)
    );
    com_RF_upperleg = Vector3(ModelConstants::RF_upperleg_comx, ModelConstants::RF_upperleg_comy, 0.0);
    tensor_RF_upperleg.fill(
        ModelConstants::RF_upperleg_mass,
        com_RF_upperleg,
        iit::rbd::Utils::buildInertiaTensor<Scalar>(ModelConstants::RF_upperleg_ixx,ModelConstants::RF_upperleg_iyy,ModelConstants::RF_upperleg_izz,ModelConstants::RF_upperleg_ixy,ModelConstants::RF_upperleg_ixz,ModelConstants::RF_upperleg_iyz)
    );
    com_RF_lowerleg = Vector3(ModelConstants::RF_lowerleg_comx, ModelConstants::RF_lowerleg_comy, ModelConstants::RF_lowerleg_comz);
    tensor_RF_lowerleg.fill(
        ModelConstants::RF_lowerleg_mass,
        com_RF_lowerleg,
        iit::rbd::Utils::buildInertiaTensor<Scalar>(ModelConstants::RF_lowerleg_ixx,ModelConstants::RF_lowerleg_iyy,ModelConstants::RF_lowerleg_izz,0.0,0.0,0.0)
    );
    com_LH_hipassembly = Vector3(ModelConstants::LH_hipassembly_comx, 0.0, ModelConstants::LH_hipassembly_comz);
    tensor_LH_hipassembly.fill(
        ModelConstants::LH_hipassembly_mass,
        com_LH_hipassembly,
        iit::rbd::Utils::buildInertiaTensor<Scalar>(ModelConstants::LH_hipassembly_ixx,ModelConstants::LH_hipassembly_iyy,ModelConstants::LH_hipassembly_izz,ModelConstants::LH_hipassembly_ixy,ModelConstants::LH_hipassembly_ixz,ModelConstants::LH_hipassembly_iyz)
    );
    com_LH_upperleg = Vector3(ModelConstants::LH_upperleg_comx, ModelConstants::LH_upperleg_comy, 0.0);
    tensor_LH_upperleg.fill(
        ModelConstants::LH_upperleg_mass,
        com_LH_upperleg,
        iit::rbd::Utils::buildInertiaTensor<Scalar>(ModelConstants::LH_upperleg_ixx,ModelConstants::LH_upperleg_iyy,ModelConstants::LH_upperleg_izz,ModelConstants::LH_upperleg_ixy,ModelConstants::LH_upperleg_ixz,ModelConstants::LH_upperleg_iyz)
    );
    com_LH_lowerleg = Vector3(ModelConstants::LH_lowerleg_comx, ModelConstants::LH_lowerleg_comy, ModelConstants::LH_lowerleg_comz);
    tensor_LH_lowerleg.fill(
        ModelConstants::LH_lowerleg_mass,
        com_LH_lowerleg,
        iit::rbd::Utils::buildInertiaTensor<Scalar>(ModelConstants::LH_lowerleg_ixx,ModelConstants::LH_lowerleg_iyy,ModelConstants::LH_lowerleg_izz,0.0,0.0,0.0)
    );
    com_RH_hipassembly = Vector3(ModelConstants::RH_hipassembly_comx, 0.0, ModelConstants::RH_hipassembly_comz);
    tensor_RH_hipassembly.fill(
        ModelConstants::RH_hipassembly_mass,
        com_RH_hipassembly,
        iit::rbd::Utils::buildInertiaTensor<Scalar>(ModelConstants::RH_hipassembly_ixx,ModelConstants::RH_hipassembly_iyy,ModelConstants::RH_hipassembly_izz,ModelConstants::RH_hipassembly_ixy,ModelConstants::RH_hipassembly_ixz,ModelConstants::RH_hipassembly_iyz)
    );
    com_RH_upperleg = Vector3(ModelConstants::RH_upperleg_comx, ModelConstants::RH_upperleg_comy, 0.0);
    tensor_RH_upperleg.fill(
        ModelConstants::RH_upperleg_mass,
        com_RH_upperleg,
        iit::rbd::Utils::buildInertiaTensor<Scalar>(ModelConstants::RH_upperleg_ixx,ModelConstants::RH_upperleg_iyy,ModelConstants::RH_upperleg_izz,ModelConstants::RH_upperleg_ixy,ModelConstants::RH_upperleg_ixz,ModelConstants::RH_upperleg_iyz)
    );
    com_RH_lowerleg = Vector3(ModelConstants::RH_lowerleg_comx, ModelConstants::RH_lowerleg_comy, ModelConstants::RH_lowerleg_comz);
    tensor_RH_lowerleg.fill(
        ModelConstants::RH_lowerleg_mass,
        com_RH_lowerleg,
        iit::rbd::Utils::buildInertiaTensor<Scalar>(ModelConstants::RH_lowerleg_ixx,ModelConstants::RH_lowerleg_iyy,ModelConstants::RH_lowerleg_izz,0.0,0.0,0.0)
    );
}


void hyq::rcg2::InertiaProperties::updateParameters(const RuntimeInertiaParams& fresh)
{
    parameters = fresh;  // trivial bit-copy is fine
}

