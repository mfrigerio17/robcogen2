#ifndef RCG2_HYQ_INERTIA_PROPERTIES_H
#define RCG2_HYQ_INERTIA_PROPERTIES_H

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"
#include "constants.h"

namespace hyq {
namespace rcg2 {


/**
 * A container for the runtime parameters of the inertia of the robot.
 *
 * Inertia parameters are non-constant inertia-properties, symbolically
 * defined in the robot model.
 * As the value of the parameters must be resolved at runtime, we refer
 * to them as "runtime parameters", "runtime dynamics parameters",
 * "runtime inertia parameters", etc.
 *
 * Unfortunately, the literature commonly refers to the inertia-properties
 * as "inertia parameters". Do not confuse them. In RobCoGen, the parameters
 * are the non-constant values of the properties.
 */
struct RuntimeInertiaParams
{
    RuntimeInertiaParams() {
        defaults();
    }
    void defaults() {
    }
};


/**
 * A container for the inertial properties of the links of the robot
 */

class InertiaProperties
{
public:
    InertiaProperties();
    ~InertiaProperties() {};

    const InertiaMatrix& getTensor_trunk() const {
        return tensor_trunk;
    }
    const InertiaMatrix& getTensor_LF_hipassembly() const {
        return tensor_LF_hipassembly;
    }
    const InertiaMatrix& getTensor_LF_upperleg() const {
        return tensor_LF_upperleg;
    }
    const InertiaMatrix& getTensor_LF_lowerleg() const {
        return tensor_LF_lowerleg;
    }
    const InertiaMatrix& getTensor_RF_hipassembly() const {
        return tensor_RF_hipassembly;
    }
    const InertiaMatrix& getTensor_RF_upperleg() const {
        return tensor_RF_upperleg;
    }
    const InertiaMatrix& getTensor_RF_lowerleg() const {
        return tensor_RF_lowerleg;
    }
    const InertiaMatrix& getTensor_LH_hipassembly() const {
        return tensor_LH_hipassembly;
    }
    const InertiaMatrix& getTensor_LH_upperleg() const {
        return tensor_LH_upperleg;
    }
    const InertiaMatrix& getTensor_LH_lowerleg() const {
        return tensor_LH_lowerleg;
    }
    const InertiaMatrix& getTensor_RH_hipassembly() const {
        return tensor_RH_hipassembly;
    }
    const InertiaMatrix& getTensor_RH_upperleg() const {
        return tensor_RH_upperleg;
    }
    const InertiaMatrix& getTensor_RH_lowerleg() const {
        return tensor_RH_lowerleg;
    }
    Scalar getMass_trunk() const {
        return tensor_trunk.getMass();
    }
    Scalar getMass_LF_hipassembly() const {
        return tensor_LF_hipassembly.getMass();
    }
    Scalar getMass_LF_upperleg() const {
        return tensor_LF_upperleg.getMass();
    }
    Scalar getMass_LF_lowerleg() const {
        return tensor_LF_lowerleg.getMass();
    }
    Scalar getMass_RF_hipassembly() const {
        return tensor_RF_hipassembly.getMass();
    }
    Scalar getMass_RF_upperleg() const {
        return tensor_RF_upperleg.getMass();
    }
    Scalar getMass_RF_lowerleg() const {
        return tensor_RF_lowerleg.getMass();
    }
    Scalar getMass_LH_hipassembly() const {
        return tensor_LH_hipassembly.getMass();
    }
    Scalar getMass_LH_upperleg() const {
        return tensor_LH_upperleg.getMass();
    }
    Scalar getMass_LH_lowerleg() const {
        return tensor_LH_lowerleg.getMass();
    }
    Scalar getMass_RH_hipassembly() const {
        return tensor_RH_hipassembly.getMass();
    }
    Scalar getMass_RH_upperleg() const {
        return tensor_RH_upperleg.getMass();
    }
    Scalar getMass_RH_lowerleg() const {
        return tensor_RH_lowerleg.getMass();
    }
    const Vector3& getCOM_trunk() const {
        return com_trunk;
    }
    const Vector3& getCOM_LF_hipassembly() const {
        return com_LF_hipassembly;
    }
    const Vector3& getCOM_LF_upperleg() const {
        return com_LF_upperleg;
    }
    const Vector3& getCOM_LF_lowerleg() const {
        return com_LF_lowerleg;
    }
    const Vector3& getCOM_RF_hipassembly() const {
        return com_RF_hipassembly;
    }
    const Vector3& getCOM_RF_upperleg() const {
        return com_RF_upperleg;
    }
    const Vector3& getCOM_RF_lowerleg() const {
        return com_RF_lowerleg;
    }
    const Vector3& getCOM_LH_hipassembly() const {
        return com_LH_hipassembly;
    }
    const Vector3& getCOM_LH_upperleg() const {
        return com_LH_upperleg;
    }
    const Vector3& getCOM_LH_lowerleg() const {
        return com_LH_lowerleg;
    }
    const Vector3& getCOM_RH_hipassembly() const {
        return com_RH_hipassembly;
    }
    const Vector3& getCOM_RH_upperleg() const {
        return com_RH_upperleg;
    }
    const Vector3& getCOM_RH_lowerleg() const {
        return com_RH_lowerleg;
    }
    Scalar getTotalMass() const {
        return
            getMass_trunk()+
            getMass_LF_hipassembly()+
            getMass_LF_upperleg()+
            getMass_LF_lowerleg()+
            getMass_RF_hipassembly()+
            getMass_RF_upperleg()+
            getMass_RF_lowerleg()+
            getMass_LH_hipassembly()+
            getMass_LH_upperleg()+
            getMass_LH_lowerleg()+
            getMass_RH_hipassembly()+
            getMass_RH_upperleg()+
            getMass_RH_lowerleg()
        ;
    }


    /*!
     * Set new inertia parameters.
     * A change in the parameters triggers the update of the inertia
     * properties modeled by this instance.
     */
    void updateParameters(const RuntimeInertiaParams&);

private:
    RuntimeInertiaParams parameters;

    InertiaMatrix tensor_trunk;
    InertiaMatrix tensor_LF_hipassembly;
    InertiaMatrix tensor_LF_upperleg;
    InertiaMatrix tensor_LF_lowerleg;
    InertiaMatrix tensor_RF_hipassembly;
    InertiaMatrix tensor_RF_upperleg;
    InertiaMatrix tensor_RF_lowerleg;
    InertiaMatrix tensor_LH_hipassembly;
    InertiaMatrix tensor_LH_upperleg;
    InertiaMatrix tensor_LH_lowerleg;
    InertiaMatrix tensor_RH_hipassembly;
    InertiaMatrix tensor_RH_upperleg;
    InertiaMatrix tensor_RH_lowerleg;
    Vector3 com_trunk;
    Vector3 com_LF_hipassembly;
    Vector3 com_LF_upperleg;
    Vector3 com_LF_lowerleg;
    Vector3 com_RF_hipassembly;
    Vector3 com_RF_upperleg;
    Vector3 com_RF_lowerleg;
    Vector3 com_LH_hipassembly;
    Vector3 com_LH_upperleg;
    Vector3 com_LH_lowerleg;
    Vector3 com_RH_hipassembly;
    Vector3 com_RH_upperleg;
    Vector3 com_RH_lowerleg;
};

}
}


#endif
