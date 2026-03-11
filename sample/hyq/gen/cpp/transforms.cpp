#include <transforms.h>

using hyq::rcg2::scalar_t;




hyq::rcg2::LF_hipassembly_X_trunk::LF_hipassembly_X_trunk()
{
    this->ct.a_R_b(0,0) = 0;
    this->ct.a_R_b(1,0) = 0;
    this->ct.a_R_b(2,0) = -1;
    this->ct.a_R_b(2,1) = 0;
    this->ct.a_R_b(2,2) = 0;
    this->ct.r_ab_a(2) = ModelConstants::LF_HAA_tx;
    
    
    
    
}


const typename hyq::rcg2::LF_hipassembly_X_trunk& hyq::rcg2::LF_hipassembly_X_trunk::update(const state_t& q)
{
    scalar_t s__q0 = ScalarTraits::sin( q(0) );
    scalar_t c__q0 = ScalarTraits::cos( q(0) );

    this->ct.a_R_b(0,1) = -s__q0;
    this->ct.a_R_b(0,2) = -c__q0;
    this->ct.r_ab_a(0) = ModelConstants::LF_HAA_ty*s__q0;
    this->ct.a_R_b(1,1) = -c__q0;
    this->ct.a_R_b(1,2) = s__q0;
    this->ct.r_ab_a(1) = ModelConstants::LF_HAA_ty*c__q0;
    return *this;
}



hyq::rcg2::LF_upperleg_X_LF_hipassembly::LF_upperleg_X_LF_hipassembly()
{
    this->ct.a_R_b(0,1) = 0;
    this->ct.a_R_b(1,1) = 0;
    this->ct.a_R_b(2,0) = 0;
    this->ct.a_R_b(2,1) = -1;
    this->ct.a_R_b(2,2) = 0;
    this->ct.r_ab_a(2) = 0;
    
    
    
    
}


const typename hyq::rcg2::LF_upperleg_X_LF_hipassembly& hyq::rcg2::LF_upperleg_X_LF_hipassembly::update(const state_t& q)
{
    scalar_t s__q1 = ScalarTraits::sin( q(1) );
    scalar_t c__q1 = ScalarTraits::cos( q(1) );

    this->ct.a_R_b(0,0) = c__q1;
    this->ct.a_R_b(0,2) = s__q1;
    this->ct.r_ab_a(0) = -ModelConstants::LF_HFE_tx*c__q1;
    this->ct.a_R_b(1,0) = -s__q1;
    this->ct.a_R_b(1,2) = c__q1;
    this->ct.r_ab_a(1) = ModelConstants::LF_HFE_tx*s__q1;
    return *this;
}



hyq::rcg2::LF_lowerleg_X_LF_upperleg::LF_lowerleg_X_LF_upperleg()
{
    this->ct.a_R_b(0,2) = 0;
    this->ct.a_R_b(1,2) = 0;
    this->ct.a_R_b(2,0) = 0;
    this->ct.a_R_b(2,1) = 0;
    this->ct.a_R_b(2,2) = 1;
    this->ct.r_ab_a(2) = 0;
    
    
    
    
}


const typename hyq::rcg2::LF_lowerleg_X_LF_upperleg& hyq::rcg2::LF_lowerleg_X_LF_upperleg::update(const state_t& q)
{
    scalar_t s__q2 = ScalarTraits::sin( q(2) );
    scalar_t c__q2 = ScalarTraits::cos( q(2) );

    this->ct.a_R_b(0,0) = c__q2;
    this->ct.a_R_b(0,1) = s__q2;
    this->ct.r_ab_a(0) = -ModelConstants::LF_KFE_tx*c__q2;
    this->ct.a_R_b(1,0) = -s__q2;
    this->ct.a_R_b(1,1) = c__q2;
    this->ct.r_ab_a(1) = ModelConstants::LF_KFE_tx*s__q2;
    return *this;
}



hyq::rcg2::RF_hipassembly_X_trunk::RF_hipassembly_X_trunk()
{
    this->ct.a_R_b(0,0) = 0;
    this->ct.a_R_b(1,0) = 0;
    this->ct.a_R_b(2,0) = 1;
    this->ct.a_R_b(2,1) = 0;
    this->ct.a_R_b(2,2) = 0;
    this->ct.r_ab_a(2) = -ModelConstants::RF_HAA_tx;
    
    
    
    
}


const typename hyq::rcg2::RF_hipassembly_X_trunk& hyq::rcg2::RF_hipassembly_X_trunk::update(const state_t& q)
{
    scalar_t s__q3 = ScalarTraits::sin( q(3) );
    scalar_t c__q3 = ScalarTraits::cos( q(3) );

    this->ct.a_R_b(0,1) = s__q3;
    this->ct.a_R_b(0,2) = -c__q3;
    this->ct.r_ab_a(0) = -ModelConstants::RF_HAA_ty*s__q3;
    this->ct.a_R_b(1,1) = c__q3;
    this->ct.a_R_b(1,2) = s__q3;
    this->ct.r_ab_a(1) = -ModelConstants::RF_HAA_ty*c__q3;
    return *this;
}



hyq::rcg2::RF_upperleg_X_RF_hipassembly::RF_upperleg_X_RF_hipassembly()
{
    this->ct.a_R_b(0,1) = 0;
    this->ct.a_R_b(1,1) = 0;
    this->ct.a_R_b(2,0) = 0;
    this->ct.a_R_b(2,1) = 1;
    this->ct.a_R_b(2,2) = 0;
    this->ct.r_ab_a(2) = 0;
    
    
    
    
}


const typename hyq::rcg2::RF_upperleg_X_RF_hipassembly& hyq::rcg2::RF_upperleg_X_RF_hipassembly::update(const state_t& q)
{
    scalar_t s__q4 = ScalarTraits::sin( q(4) );
    scalar_t c__q4 = ScalarTraits::cos( q(4) );

    this->ct.a_R_b(0,0) = c__q4;
    this->ct.a_R_b(0,2) = -s__q4;
    this->ct.r_ab_a(0) = -ModelConstants::RF_HFE_tx*c__q4;
    this->ct.a_R_b(1,0) = -s__q4;
    this->ct.a_R_b(1,2) = -c__q4;
    this->ct.r_ab_a(1) = ModelConstants::RF_HFE_tx*s__q4;
    return *this;
}



hyq::rcg2::RF_lowerleg_X_RF_upperleg::RF_lowerleg_X_RF_upperleg()
{
    this->ct.a_R_b(0,2) = 0;
    this->ct.a_R_b(1,2) = 0;
    this->ct.a_R_b(2,0) = 0;
    this->ct.a_R_b(2,1) = 0;
    this->ct.a_R_b(2,2) = 1;
    this->ct.r_ab_a(2) = 0;
    
    
    
    
}


const typename hyq::rcg2::RF_lowerleg_X_RF_upperleg& hyq::rcg2::RF_lowerleg_X_RF_upperleg::update(const state_t& q)
{
    scalar_t s__q5 = ScalarTraits::sin( q(5) );
    scalar_t c__q5 = ScalarTraits::cos( q(5) );

    this->ct.a_R_b(0,0) = c__q5;
    this->ct.a_R_b(0,1) = s__q5;
    this->ct.r_ab_a(0) = -ModelConstants::RF_KFE_tx*c__q5;
    this->ct.a_R_b(1,0) = -s__q5;
    this->ct.a_R_b(1,1) = c__q5;
    this->ct.r_ab_a(1) = ModelConstants::RF_KFE_tx*s__q5;
    return *this;
}



hyq::rcg2::LH_hipassembly_X_trunk::LH_hipassembly_X_trunk()
{
    this->ct.a_R_b(0,0) = 0;
    this->ct.a_R_b(1,0) = 0;
    this->ct.a_R_b(2,0) = -1;
    this->ct.a_R_b(2,1) = 0;
    this->ct.a_R_b(2,2) = 0;
    this->ct.r_ab_a(2) = ModelConstants::LH_HAA_tx;
    
    
    
    
}


const typename hyq::rcg2::LH_hipassembly_X_trunk& hyq::rcg2::LH_hipassembly_X_trunk::update(const state_t& q)
{
    scalar_t s__q6 = ScalarTraits::sin( q(6) );
    scalar_t c__q6 = ScalarTraits::cos( q(6) );

    this->ct.a_R_b(0,1) = -s__q6;
    this->ct.a_R_b(0,2) = -c__q6;
    this->ct.r_ab_a(0) = ModelConstants::LH_HAA_ty*s__q6;
    this->ct.a_R_b(1,1) = -c__q6;
    this->ct.a_R_b(1,2) = s__q6;
    this->ct.r_ab_a(1) = ModelConstants::LH_HAA_ty*c__q6;
    return *this;
}



hyq::rcg2::LH_upperleg_X_LH_hipassembly::LH_upperleg_X_LH_hipassembly()
{
    this->ct.a_R_b(0,1) = 0;
    this->ct.a_R_b(1,1) = 0;
    this->ct.a_R_b(2,0) = 0;
    this->ct.a_R_b(2,1) = -1;
    this->ct.a_R_b(2,2) = 0;
    this->ct.r_ab_a(2) = 0;
    
    
    
    
}


const typename hyq::rcg2::LH_upperleg_X_LH_hipassembly& hyq::rcg2::LH_upperleg_X_LH_hipassembly::update(const state_t& q)
{
    scalar_t s__q7 = ScalarTraits::sin( q(7) );
    scalar_t c__q7 = ScalarTraits::cos( q(7) );

    this->ct.a_R_b(0,0) = c__q7;
    this->ct.a_R_b(0,2) = s__q7;
    this->ct.r_ab_a(0) = -ModelConstants::LH_HFE_tx*c__q7;
    this->ct.a_R_b(1,0) = -s__q7;
    this->ct.a_R_b(1,2) = c__q7;
    this->ct.r_ab_a(1) = ModelConstants::LH_HFE_tx*s__q7;
    return *this;
}



hyq::rcg2::LH_lowerleg_X_LH_upperleg::LH_lowerleg_X_LH_upperleg()
{
    this->ct.a_R_b(0,2) = 0;
    this->ct.a_R_b(1,2) = 0;
    this->ct.a_R_b(2,0) = 0;
    this->ct.a_R_b(2,1) = 0;
    this->ct.a_R_b(2,2) = 1;
    this->ct.r_ab_a(2) = 0;
    
    
    
    
}


const typename hyq::rcg2::LH_lowerleg_X_LH_upperleg& hyq::rcg2::LH_lowerleg_X_LH_upperleg::update(const state_t& q)
{
    scalar_t s__q8 = ScalarTraits::sin( q(8) );
    scalar_t c__q8 = ScalarTraits::cos( q(8) );

    this->ct.a_R_b(0,0) = c__q8;
    this->ct.a_R_b(0,1) = s__q8;
    this->ct.r_ab_a(0) = -ModelConstants::LH_KFE_tx*c__q8;
    this->ct.a_R_b(1,0) = -s__q8;
    this->ct.a_R_b(1,1) = c__q8;
    this->ct.r_ab_a(1) = ModelConstants::LH_KFE_tx*s__q8;
    return *this;
}



hyq::rcg2::RH_hipassembly_X_trunk::RH_hipassembly_X_trunk()
{
    this->ct.a_R_b(0,0) = 0;
    this->ct.a_R_b(1,0) = 0;
    this->ct.a_R_b(2,0) = 1;
    this->ct.a_R_b(2,1) = 0;
    this->ct.a_R_b(2,2) = 0;
    this->ct.r_ab_a(2) = -ModelConstants::RH_HAA_tx;
    
    
    
    
}


const typename hyq::rcg2::RH_hipassembly_X_trunk& hyq::rcg2::RH_hipassembly_X_trunk::update(const state_t& q)
{
    scalar_t s__q9 = ScalarTraits::sin( q(9) );
    scalar_t c__q9 = ScalarTraits::cos( q(9) );

    this->ct.a_R_b(0,1) = s__q9;
    this->ct.a_R_b(0,2) = -c__q9;
    this->ct.r_ab_a(0) = -ModelConstants::RH_HAA_ty*s__q9;
    this->ct.a_R_b(1,1) = c__q9;
    this->ct.a_R_b(1,2) = s__q9;
    this->ct.r_ab_a(1) = -ModelConstants::RH_HAA_ty*c__q9;
    return *this;
}



hyq::rcg2::RH_upperleg_X_RH_hipassembly::RH_upperleg_X_RH_hipassembly()
{
    this->ct.a_R_b(0,1) = 0;
    this->ct.a_R_b(1,1) = 0;
    this->ct.a_R_b(2,0) = 0;
    this->ct.a_R_b(2,1) = 1;
    this->ct.a_R_b(2,2) = 0;
    this->ct.r_ab_a(2) = 0;
    
    
    
    
}


const typename hyq::rcg2::RH_upperleg_X_RH_hipassembly& hyq::rcg2::RH_upperleg_X_RH_hipassembly::update(const state_t& q)
{
    scalar_t s__q10 = ScalarTraits::sin( q(10) );
    scalar_t c__q10 = ScalarTraits::cos( q(10) );

    this->ct.a_R_b(0,0) = c__q10;
    this->ct.a_R_b(0,2) = -s__q10;
    this->ct.r_ab_a(0) = -ModelConstants::RH_HFE_tx*c__q10;
    this->ct.a_R_b(1,0) = -s__q10;
    this->ct.a_R_b(1,2) = -c__q10;
    this->ct.r_ab_a(1) = ModelConstants::RH_HFE_tx*s__q10;
    return *this;
}



hyq::rcg2::RH_lowerleg_X_RH_upperleg::RH_lowerleg_X_RH_upperleg()
{
    this->ct.a_R_b(0,2) = 0;
    this->ct.a_R_b(1,2) = 0;
    this->ct.a_R_b(2,0) = 0;
    this->ct.a_R_b(2,1) = 0;
    this->ct.a_R_b(2,2) = 1;
    this->ct.r_ab_a(2) = 0;
    
    
    
    
}


const typename hyq::rcg2::RH_lowerleg_X_RH_upperleg& hyq::rcg2::RH_lowerleg_X_RH_upperleg::update(const state_t& q)
{
    scalar_t s__q11 = ScalarTraits::sin( q(11) );
    scalar_t c__q11 = ScalarTraits::cos( q(11) );

    this->ct.a_R_b(0,0) = c__q11;
    this->ct.a_R_b(0,1) = s__q11;
    this->ct.r_ab_a(0) = -ModelConstants::RH_KFE_tx*c__q11;
    this->ct.a_R_b(1,0) = -s__q11;
    this->ct.a_R_b(1,1) = c__q11;
    this->ct.r_ab_a(1) = ModelConstants::RH_KFE_tx*s__q11;
    return *this;
}





hyq::rcg2::Transforms::Transforms() :
m_LF_hipassembly_X_trunk(),m_LF_upperleg_X_LF_hipassembly(),m_LF_lowerleg_X_LF_upperleg(),m_RF_hipassembly_X_trunk(),m_RF_upperleg_X_RF_hipassembly(),m_RF_lowerleg_X_RF_upperleg(),m_LH_hipassembly_X_trunk(),m_LH_upperleg_X_LH_hipassembly(),m_LH_lowerleg_X_LH_upperleg(),m_RH_hipassembly_X_trunk(),m_RH_upperleg_X_RH_hipassembly(),m_RH_lowerleg_X_RH_upperleg()
{}



void hyq::rcg2::Transforms::update(const state_t& q)
{
    m_LF_hipassembly_X_trunk.update(q);
    m_LF_upperleg_X_LF_hipassembly.update(q);
    m_LF_lowerleg_X_LF_upperleg.update(q);
    m_RF_hipassembly_X_trunk.update(q);
    m_RF_upperleg_X_RF_hipassembly.update(q);
    m_RF_lowerleg_X_RF_upperleg.update(q);
    m_LH_hipassembly_X_trunk.update(q);
    m_LH_upperleg_X_LH_hipassembly.update(q);
    m_LH_lowerleg_X_LH_upperleg.update(q);
    m_RH_hipassembly_X_trunk.update(q);
    m_RH_upperleg_X_RH_hipassembly.update(q);
    m_RH_lowerleg_X_RH_upperleg.update(q);
}



