#ifndef FANCY_GEOMETRY_TRANSFORMS_GEN_H
#define FANCY_GEOMETRY_TRANSFORMS_GEN_H

#include "rbd_types.h"
#include "declarations.h"
#include "constants.h"
#include <iit/rbd/compact_transform.h>
#include <iit/rbd/scalar_traits.h>

namespace fancy {
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
    scalar_t ll{1.0};
};


struct Parameters
{
    scalar_t ll;

    Parameters() {}
    explicit Parameters(const ModelParameters& params) {
        ll = params.ll;

    }

    Parameters& operator=(const ModelParameters& params) {
        ll = params.ll;

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


struct link1_X_base0 : public Transform<link1_X_base0>
{
    link1_X_base0();
    const link1_X_base0& update(const state_t&);

    A_XM_B link1_XM_base0() const { return this->template as<A_XM_B>(); }
    B_XM_A base0_XM_link1() const { return this->template as<B_XM_A>(); }
    A_XF_B link1_XF_base0() const { return this->template as<A_XF_B>(); }
    B_XF_A base0_XF_link1() const { return this->template as<B_XF_A>(); }
    A_XH_B link1_XH_base0() const { return this->template as<A_XH_B>(); }
    B_XH_A base0_XH_link1() const { return this->template as<B_XH_A>(); }

};


struct link2_X_link1 : public Transform<link2_X_link1>
{
    link2_X_link1(const Parameters& params);
    const link2_X_link1& update(const state_t&);

    A_XM_B link2_XM_link1() const { return this->template as<A_XM_B>(); }
    B_XM_A link1_XM_link2() const { return this->template as<B_XM_A>(); }
    A_XF_B link2_XF_link1() const { return this->template as<A_XF_B>(); }
    B_XF_A link1_XF_link2() const { return this->template as<B_XF_A>(); }
    A_XH_B link2_XH_link1() const { return this->template as<A_XH_B>(); }
    B_XH_A link1_XH_link2() const { return this->template as<B_XH_A>(); }

protected:
    const Parameters& parameters;
};


struct link3_X_link2 : public Transform<link3_X_link2>
{
    link3_X_link2();
    const link3_X_link2& update(const state_t&);

    A_XM_B link3_XM_link2() const { return this->template as<A_XM_B>(); }
    B_XM_A link2_XM_link3() const { return this->template as<B_XM_A>(); }
    A_XF_B link3_XF_link2() const { return this->template as<A_XF_B>(); }
    B_XF_A link2_XF_link3() const { return this->template as<B_XF_A>(); }
    A_XH_B link3_XH_link2() const { return this->template as<A_XH_B>(); }
    B_XH_A link2_XH_link3() const { return this->template as<B_XH_A>(); }

};


struct link4_X_link3 : public Transform<link4_X_link3>
{
    link4_X_link3();
    const link4_X_link3& update(const state_t&);

    A_XM_B link4_XM_link3() const { return this->template as<A_XM_B>(); }
    B_XM_A link3_XM_link4() const { return this->template as<B_XM_A>(); }
    A_XF_B link4_XF_link3() const { return this->template as<A_XF_B>(); }
    B_XF_A link3_XF_link4() const { return this->template as<B_XF_A>(); }
    A_XH_B link4_XH_link3() const { return this->template as<A_XH_B>(); }
    B_XH_A link3_XH_link4() const { return this->template as<B_XH_A>(); }

};


struct link5_X_link4 : public Transform<link5_X_link4>
{
    link5_X_link4();
    const link5_X_link4& update(const state_t&);

    A_XM_B link5_XM_link4() const { return this->template as<A_XM_B>(); }
    B_XM_A link4_XM_link5() const { return this->template as<B_XM_A>(); }
    A_XF_B link5_XF_link4() const { return this->template as<A_XF_B>(); }
    B_XF_A link4_XF_link5() const { return this->template as<B_XF_A>(); }
    A_XH_B link5_XH_link4() const { return this->template as<A_XH_B>(); }
    B_XH_A link4_XH_link5() const { return this->template as<B_XH_A>(); }

};



struct Transforms
{
    Transforms();
    Transforms(const ModelParameters& initial);
    void updateParams(const ModelParameters& mp) {
        parameters = mp;
    }

    void update(const state_t&);

    link1_X_base0 m_link1_X_base0;
    link2_X_link1 m_link2_X_link1;
    link3_X_link2 m_link3_X_link2;
    link4_X_link3 m_link4_X_link3;
    link5_X_link4 m_link5_X_link4;

protected:
    Parameters parameters;
};



}
}


#endif
