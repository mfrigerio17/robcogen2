#include "inertia_properties.h"


fancy::rcg2::InertiaProperties::InertiaProperties()
{
    com_link1 = Vector3(ModelConstants::link1_comx, 0.0, 0.0);
    tensor_link1.fill(
        parameters.m1,
        com_link1,
        iit::rbd::Utils::buildInertiaTensor<Scalar>(ModelConstants::link1_ixx,ModelConstants::link1_iyy,ModelConstants::link1_izz,0.0,0.0,0.0)
    );
    com_link2 = Vector3(0.0, 0.0, ModelConstants::link2_comz);
    tensor_link2.fill(
        ModelConstants::link2_mass,
        com_link2,
        iit::rbd::Utils::buildInertiaTensor<Scalar>(ModelConstants::link2_ixx,ModelConstants::link2_iyy,ModelConstants::link2_izz,0.0,0.0,0.0)
    );
    com_link3 = Vector3(ModelConstants::link3_comx, 0.0, 0.0);
    tensor_link3.fill(
        ModelConstants::link3_mass,
        com_link3,
        iit::rbd::Utils::buildInertiaTensor<Scalar>(ModelConstants::link3_ixx,ModelConstants::link3_iyy,ModelConstants::link3_izz,0.0,0.0,0.0)
    );
    com_link4 = Vector3(0.0, 0.0, ModelConstants::link4_comz);
    tensor_link4.fill(
        ModelConstants::link4_mass,
        com_link4,
        iit::rbd::Utils::buildInertiaTensor<Scalar>(ModelConstants::link4_ixx,ModelConstants::link4_iyy,ModelConstants::link4_izz,0.0,0.0,0.0)
    );
    com_link5 = Vector3(ModelConstants::link5_comx, 0.0, 0.0);
    tensor_link5.fill(
        ModelConstants::link5_mass,
        com_link5,
        iit::rbd::Utils::buildInertiaTensor<Scalar>(ModelConstants::link5_ixx,ModelConstants::link5_iyy,ModelConstants::link5_izz,0.0,0.0,0.0)
    );
}


void fancy::rcg2::InertiaProperties::updateParameters(const RuntimeInertiaParams& fresh)
{
    parameters = fresh;  // trivial bit-copy is fine
    tensor_link1.changeMass(parameters.m1);
}

