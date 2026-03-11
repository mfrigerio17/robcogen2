#ifndef HYQ_GEOMETRY_TRANSFORMS_GEN_H
#define HYQ_GEOMETRY_TRANSFORMS_GEN_H

#include "rbd_types.h"
#include "declarations.h"
#include "constants.h"
#include <iit/rbd/compact_transform.h>
#include <iit/rbd/scalar_traits.h>

namespace hyq {
namespace rcg2 {



using scalar_t = typename ScalarTraits::Scalar;



/**
 * Derived constant expressions
 */
struct Constants
{
};


struct ModelParameters
{
};


struct Parameters
{

    Parameters() {}
    explicit Parameters(const ModelParameters& params) {

    }

    Parameters& operator=(const ModelParameters& params) {

        return *this;
    }
};



using state_t = JointState;

template<typename ACTUAL>
struct Transform : public iit::rbd::TransformBase<scalar_t>,
    public iit::rbd::StateDependentBase<state_t, ACTUAL>
{
    using Base = iit::rbd::TransformBase<scalar_t>;
    Transform() : Base(0) {} // calls explicit constructor setting data to 0
};

using A_XM_B = iit::rbd::A_XM_B< scalar_t >;
using B_XM_A = iit::rbd::B_XM_A< scalar_t >;
using A_XF_B = iit::rbd::A_XF_B< scalar_t >;
using B_XF_A = iit::rbd::B_XF_A< scalar_t >;
using A_XH_B = iit::rbd::A_XH_B< scalar_t >;
using B_XH_A = iit::rbd::B_XH_A< scalar_t >;


struct LF_hipassembly_X_trunk : public Transform<LF_hipassembly_X_trunk>
{
    LF_hipassembly_X_trunk();
    const LF_hipassembly_X_trunk& update(const state_t&);

    A_XM_B LF_hipassembly_XM_trunk() const { return this->template as<A_XM_B>(); }
    B_XM_A trunk_XM_LF_hipassembly() const { return this->template as<B_XM_A>(); }
    A_XF_B LF_hipassembly_XF_trunk() const { return this->template as<A_XF_B>(); }
    B_XF_A trunk_XF_LF_hipassembly() const { return this->template as<B_XF_A>(); }
    A_XH_B LF_hipassembly_XH_trunk() const { return this->template as<A_XH_B>(); }
    B_XH_A trunk_XH_LF_hipassembly() const { return this->template as<B_XH_A>(); }

};


struct LF_upperleg_X_LF_hipassembly : public Transform<LF_upperleg_X_LF_hipassembly>
{
    LF_upperleg_X_LF_hipassembly();
    const LF_upperleg_X_LF_hipassembly& update(const state_t&);

    A_XM_B LF_upperleg_XM_LF_hipassembly() const { return this->template as<A_XM_B>(); }
    B_XM_A LF_hipassembly_XM_LF_upperleg() const { return this->template as<B_XM_A>(); }
    A_XF_B LF_upperleg_XF_LF_hipassembly() const { return this->template as<A_XF_B>(); }
    B_XF_A LF_hipassembly_XF_LF_upperleg() const { return this->template as<B_XF_A>(); }
    A_XH_B LF_upperleg_XH_LF_hipassembly() const { return this->template as<A_XH_B>(); }
    B_XH_A LF_hipassembly_XH_LF_upperleg() const { return this->template as<B_XH_A>(); }

};


struct LF_lowerleg_X_LF_upperleg : public Transform<LF_lowerleg_X_LF_upperleg>
{
    LF_lowerleg_X_LF_upperleg();
    const LF_lowerleg_X_LF_upperleg& update(const state_t&);

    A_XM_B LF_lowerleg_XM_LF_upperleg() const { return this->template as<A_XM_B>(); }
    B_XM_A LF_upperleg_XM_LF_lowerleg() const { return this->template as<B_XM_A>(); }
    A_XF_B LF_lowerleg_XF_LF_upperleg() const { return this->template as<A_XF_B>(); }
    B_XF_A LF_upperleg_XF_LF_lowerleg() const { return this->template as<B_XF_A>(); }
    A_XH_B LF_lowerleg_XH_LF_upperleg() const { return this->template as<A_XH_B>(); }
    B_XH_A LF_upperleg_XH_LF_lowerleg() const { return this->template as<B_XH_A>(); }

};


struct RF_hipassembly_X_trunk : public Transform<RF_hipassembly_X_trunk>
{
    RF_hipassembly_X_trunk();
    const RF_hipassembly_X_trunk& update(const state_t&);

    A_XM_B RF_hipassembly_XM_trunk() const { return this->template as<A_XM_B>(); }
    B_XM_A trunk_XM_RF_hipassembly() const { return this->template as<B_XM_A>(); }
    A_XF_B RF_hipassembly_XF_trunk() const { return this->template as<A_XF_B>(); }
    B_XF_A trunk_XF_RF_hipassembly() const { return this->template as<B_XF_A>(); }
    A_XH_B RF_hipassembly_XH_trunk() const { return this->template as<A_XH_B>(); }
    B_XH_A trunk_XH_RF_hipassembly() const { return this->template as<B_XH_A>(); }

};


struct RF_upperleg_X_RF_hipassembly : public Transform<RF_upperleg_X_RF_hipassembly>
{
    RF_upperleg_X_RF_hipassembly();
    const RF_upperleg_X_RF_hipassembly& update(const state_t&);

    A_XM_B RF_upperleg_XM_RF_hipassembly() const { return this->template as<A_XM_B>(); }
    B_XM_A RF_hipassembly_XM_RF_upperleg() const { return this->template as<B_XM_A>(); }
    A_XF_B RF_upperleg_XF_RF_hipassembly() const { return this->template as<A_XF_B>(); }
    B_XF_A RF_hipassembly_XF_RF_upperleg() const { return this->template as<B_XF_A>(); }
    A_XH_B RF_upperleg_XH_RF_hipassembly() const { return this->template as<A_XH_B>(); }
    B_XH_A RF_hipassembly_XH_RF_upperleg() const { return this->template as<B_XH_A>(); }

};


struct RF_lowerleg_X_RF_upperleg : public Transform<RF_lowerleg_X_RF_upperleg>
{
    RF_lowerleg_X_RF_upperleg();
    const RF_lowerleg_X_RF_upperleg& update(const state_t&);

    A_XM_B RF_lowerleg_XM_RF_upperleg() const { return this->template as<A_XM_B>(); }
    B_XM_A RF_upperleg_XM_RF_lowerleg() const { return this->template as<B_XM_A>(); }
    A_XF_B RF_lowerleg_XF_RF_upperleg() const { return this->template as<A_XF_B>(); }
    B_XF_A RF_upperleg_XF_RF_lowerleg() const { return this->template as<B_XF_A>(); }
    A_XH_B RF_lowerleg_XH_RF_upperleg() const { return this->template as<A_XH_B>(); }
    B_XH_A RF_upperleg_XH_RF_lowerleg() const { return this->template as<B_XH_A>(); }

};


struct LH_hipassembly_X_trunk : public Transform<LH_hipassembly_X_trunk>
{
    LH_hipassembly_X_trunk();
    const LH_hipassembly_X_trunk& update(const state_t&);

    A_XM_B LH_hipassembly_XM_trunk() const { return this->template as<A_XM_B>(); }
    B_XM_A trunk_XM_LH_hipassembly() const { return this->template as<B_XM_A>(); }
    A_XF_B LH_hipassembly_XF_trunk() const { return this->template as<A_XF_B>(); }
    B_XF_A trunk_XF_LH_hipassembly() const { return this->template as<B_XF_A>(); }
    A_XH_B LH_hipassembly_XH_trunk() const { return this->template as<A_XH_B>(); }
    B_XH_A trunk_XH_LH_hipassembly() const { return this->template as<B_XH_A>(); }

};


struct LH_upperleg_X_LH_hipassembly : public Transform<LH_upperleg_X_LH_hipassembly>
{
    LH_upperleg_X_LH_hipassembly();
    const LH_upperleg_X_LH_hipassembly& update(const state_t&);

    A_XM_B LH_upperleg_XM_LH_hipassembly() const { return this->template as<A_XM_B>(); }
    B_XM_A LH_hipassembly_XM_LH_upperleg() const { return this->template as<B_XM_A>(); }
    A_XF_B LH_upperleg_XF_LH_hipassembly() const { return this->template as<A_XF_B>(); }
    B_XF_A LH_hipassembly_XF_LH_upperleg() const { return this->template as<B_XF_A>(); }
    A_XH_B LH_upperleg_XH_LH_hipassembly() const { return this->template as<A_XH_B>(); }
    B_XH_A LH_hipassembly_XH_LH_upperleg() const { return this->template as<B_XH_A>(); }

};


struct LH_lowerleg_X_LH_upperleg : public Transform<LH_lowerleg_X_LH_upperleg>
{
    LH_lowerleg_X_LH_upperleg();
    const LH_lowerleg_X_LH_upperleg& update(const state_t&);

    A_XM_B LH_lowerleg_XM_LH_upperleg() const { return this->template as<A_XM_B>(); }
    B_XM_A LH_upperleg_XM_LH_lowerleg() const { return this->template as<B_XM_A>(); }
    A_XF_B LH_lowerleg_XF_LH_upperleg() const { return this->template as<A_XF_B>(); }
    B_XF_A LH_upperleg_XF_LH_lowerleg() const { return this->template as<B_XF_A>(); }
    A_XH_B LH_lowerleg_XH_LH_upperleg() const { return this->template as<A_XH_B>(); }
    B_XH_A LH_upperleg_XH_LH_lowerleg() const { return this->template as<B_XH_A>(); }

};


struct RH_hipassembly_X_trunk : public Transform<RH_hipassembly_X_trunk>
{
    RH_hipassembly_X_trunk();
    const RH_hipassembly_X_trunk& update(const state_t&);

    A_XM_B RH_hipassembly_XM_trunk() const { return this->template as<A_XM_B>(); }
    B_XM_A trunk_XM_RH_hipassembly() const { return this->template as<B_XM_A>(); }
    A_XF_B RH_hipassembly_XF_trunk() const { return this->template as<A_XF_B>(); }
    B_XF_A trunk_XF_RH_hipassembly() const { return this->template as<B_XF_A>(); }
    A_XH_B RH_hipassembly_XH_trunk() const { return this->template as<A_XH_B>(); }
    B_XH_A trunk_XH_RH_hipassembly() const { return this->template as<B_XH_A>(); }

};


struct RH_upperleg_X_RH_hipassembly : public Transform<RH_upperleg_X_RH_hipassembly>
{
    RH_upperleg_X_RH_hipassembly();
    const RH_upperleg_X_RH_hipassembly& update(const state_t&);

    A_XM_B RH_upperleg_XM_RH_hipassembly() const { return this->template as<A_XM_B>(); }
    B_XM_A RH_hipassembly_XM_RH_upperleg() const { return this->template as<B_XM_A>(); }
    A_XF_B RH_upperleg_XF_RH_hipassembly() const { return this->template as<A_XF_B>(); }
    B_XF_A RH_hipassembly_XF_RH_upperleg() const { return this->template as<B_XF_A>(); }
    A_XH_B RH_upperleg_XH_RH_hipassembly() const { return this->template as<A_XH_B>(); }
    B_XH_A RH_hipassembly_XH_RH_upperleg() const { return this->template as<B_XH_A>(); }

};


struct RH_lowerleg_X_RH_upperleg : public Transform<RH_lowerleg_X_RH_upperleg>
{
    RH_lowerleg_X_RH_upperleg();
    const RH_lowerleg_X_RH_upperleg& update(const state_t&);

    A_XM_B RH_lowerleg_XM_RH_upperleg() const { return this->template as<A_XM_B>(); }
    B_XM_A RH_upperleg_XM_RH_lowerleg() const { return this->template as<B_XM_A>(); }
    A_XF_B RH_lowerleg_XF_RH_upperleg() const { return this->template as<A_XF_B>(); }
    B_XF_A RH_upperleg_XF_RH_lowerleg() const { return this->template as<B_XF_A>(); }
    A_XH_B RH_lowerleg_XH_RH_upperleg() const { return this->template as<A_XH_B>(); }
    B_XH_A RH_upperleg_XH_RH_lowerleg() const { return this->template as<B_XH_A>(); }

};



struct Transforms
{
    Transforms();

    void update(const state_t&);

    LF_hipassembly_X_trunk m_LF_hipassembly_X_trunk;
    LF_upperleg_X_LF_hipassembly m_LF_upperleg_X_LF_hipassembly;
    LF_lowerleg_X_LF_upperleg m_LF_lowerleg_X_LF_upperleg;
    RF_hipassembly_X_trunk m_RF_hipassembly_X_trunk;
    RF_upperleg_X_RF_hipassembly m_RF_upperleg_X_RF_hipassembly;
    RF_lowerleg_X_RF_upperleg m_RF_lowerleg_X_RF_upperleg;
    LH_hipassembly_X_trunk m_LH_hipassembly_X_trunk;
    LH_upperleg_X_LH_hipassembly m_LH_upperleg_X_LH_hipassembly;
    LH_lowerleg_X_LH_upperleg m_LH_lowerleg_X_LH_upperleg;
    RH_hipassembly_X_trunk m_RH_hipassembly_X_trunk;
    RH_upperleg_X_RH_hipassembly m_RH_upperleg_X_RH_hipassembly;
    RH_lowerleg_X_RH_upperleg m_RH_lowerleg_X_RH_upperleg;

};



}
}


#endif
