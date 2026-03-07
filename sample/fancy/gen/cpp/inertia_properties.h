#ifndef RCG2_FANCY_INERTIA_PROPERTIES_H
#define RCG2_FANCY_INERTIA_PROPERTIES_H

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"
#include "constants.h"

namespace fancy {
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
    Scalar m1;
    RuntimeInertiaParams() {
        defaults();
    }
    void defaults() {
        m1 = 1;
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

    const InertiaMatrix& getTensor_link1() const {
        return tensor_link1;
    }
    const InertiaMatrix& getTensor_link2() const {
        return tensor_link2;
    }
    const InertiaMatrix& getTensor_link3() const {
        return tensor_link3;
    }
    const InertiaMatrix& getTensor_link4() const {
        return tensor_link4;
    }
    const InertiaMatrix& getTensor_link5() const {
        return tensor_link5;
    }
    Scalar getMass_link1() const {
        return tensor_link1.getMass();
    }
    Scalar getMass_link2() const {
        return tensor_link2.getMass();
    }
    Scalar getMass_link3() const {
        return tensor_link3.getMass();
    }
    Scalar getMass_link4() const {
        return tensor_link4.getMass();
    }
    Scalar getMass_link5() const {
        return tensor_link5.getMass();
    }
    const Vector3& getCOM_link1() const {
        return com_link1;
    }
    const Vector3& getCOM_link2() const {
        return com_link2;
    }
    const Vector3& getCOM_link3() const {
        return com_link3;
    }
    const Vector3& getCOM_link4() const {
        return com_link4;
    }
    const Vector3& getCOM_link5() const {
        return com_link5;
    }
    Scalar getTotalMass() const {
        return
            getMass_link1()+
            getMass_link2()+
            getMass_link3()+
            getMass_link4()+
            getMass_link5()
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

    InertiaMatrix tensor_link1;
    InertiaMatrix tensor_link2;
    InertiaMatrix tensor_link3;
    InertiaMatrix tensor_link4;
    InertiaMatrix tensor_link5;
    Vector3 com_link1;
    Vector3 com_link2;
    Vector3 com_link3;
    Vector3 com_link4;
    Vector3 com_link5;
};

}
}


#endif
