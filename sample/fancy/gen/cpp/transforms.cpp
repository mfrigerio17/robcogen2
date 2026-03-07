#include <transforms.h>

using fancy::rcg2::scalar_t;




fancy::rcg2::link1_X_base0::link1_X_base0()
{
    this->ct.a_R_b(0,2) = 0;
    this->ct.r_ab_a(0) = 0;
    this->ct.a_R_b(1,2) = 0;
    this->ct.r_ab_a(1) = 0;
    this->ct.a_R_b(2,0) = 0;
    this->ct.a_R_b(2,1) = 0;
    this->ct.a_R_b(2,2) = 1;
    this->ct.r_ab_a(2) = 0;
    
    
    
    
}


const typename fancy::rcg2::link1_X_base0& fancy::rcg2::link1_X_base0::update(const state_t& q)
{
    scalar_t s__q0 = ScalarTraits::sin( q(0) );
    scalar_t c__q0 = ScalarTraits::cos( q(0) );

    this->ct.a_R_b(0,0) = c__q0;
    this->ct.a_R_b(0,1) = s__q0;
    this->ct.a_R_b(1,0) = -s__q0;
    this->ct.a_R_b(1,1) = c__q0;
    return *this;
}



fancy::rcg2::link2_X_link1::link2_X_link1(const Parameters& params)
    : parameters(params)
{
    this->ct.a_R_b(0,0) = 1;
    this->ct.a_R_b(0,1) = 0;
    this->ct.a_R_b(0,2) = 0;
    this->ct.a_R_b(1,0) = 0;
    this->ct.a_R_b(1,1) = 0;
    this->ct.a_R_b(1,2) = -1;
    this->ct.r_ab_a(1) = 0;
    this->ct.a_R_b(2,0) = 0;
    this->ct.a_R_b(2,1) = 1;
    this->ct.a_R_b(2,2) = 0;
    
    
    
    
}


const typename fancy::rcg2::link2_X_link1& fancy::rcg2::link2_X_link1::update(const state_t& q)
{

    this->ct.r_ab_a(0) = -parameters.ll;
    this->ct.r_ab_a(2) = -q(1);
    return *this;
}



fancy::rcg2::link3_X_link2::link3_X_link2()
{
    this->ct.a_R_b(0,2) = 0;
    this->ct.r_ab_a(0) = 0;
    this->ct.a_R_b(1,2) = 0;
    this->ct.r_ab_a(1) = 0;
    this->ct.a_R_b(2,0) = 0;
    this->ct.a_R_b(2,1) = 0;
    this->ct.a_R_b(2,2) = 1;
    this->ct.r_ab_a(2) = -ModelConstants::jC_tz;
    
    
    
    
}


const typename fancy::rcg2::link3_X_link2& fancy::rcg2::link3_X_link2::update(const state_t& q)
{
    scalar_t s__q2 = ScalarTraits::sin( q(2) );
    scalar_t c__q2 = ScalarTraits::cos( q(2) );

    this->ct.a_R_b(0,0) = c__q2;
    this->ct.a_R_b(0,1) = s__q2;
    this->ct.a_R_b(1,0) = -s__q2;
    this->ct.a_R_b(1,1) = c__q2;
    return *this;
}



fancy::rcg2::link4_X_link3::link4_X_link3()
{
    this->ct.a_R_b(0,0) = 1;
    this->ct.a_R_b(0,1) = 0;
    this->ct.a_R_b(0,2) = 0;
    this->ct.r_ab_a(0) = -ModelConstants::jD_tx;
    this->ct.a_R_b(1,0) = 0;
    this->ct.a_R_b(1,1) = 0;
    this->ct.a_R_b(1,2) = 1;
    this->ct.r_ab_a(1) = 0;
    this->ct.a_R_b(2,0) = 0;
    this->ct.a_R_b(2,1) = -1;
    this->ct.a_R_b(2,2) = 0;
    
    
    
    
}


const typename fancy::rcg2::link4_X_link3& fancy::rcg2::link4_X_link3::update(const state_t& q)
{

    this->ct.r_ab_a(2) = -q(3);
    return *this;
}



fancy::rcg2::link5_X_link4::link5_X_link4()
{
    this->ct.a_R_b(0,2) = 0;
    this->ct.r_ab_a(0) = 0;
    this->ct.a_R_b(1,2) = 0;
    this->ct.r_ab_a(1) = 0;
    this->ct.a_R_b(2,0) = 0;
    this->ct.a_R_b(2,1) = 0;
    this->ct.a_R_b(2,2) = 1;
    this->ct.r_ab_a(2) = -ModelConstants::jE_tz;
    
    
    
    
}


const typename fancy::rcg2::link5_X_link4& fancy::rcg2::link5_X_link4::update(const state_t& q)
{
    scalar_t s__q4 = ScalarTraits::sin( q(4) );
    scalar_t c__q4 = ScalarTraits::cos( q(4) );

    this->ct.a_R_b(0,0) = c__q4;
    this->ct.a_R_b(0,1) = s__q4;
    this->ct.a_R_b(1,0) = -s__q4;
    this->ct.a_R_b(1,1) = c__q4;
    return *this;
}





fancy::rcg2::Transforms::Transforms(const ModelParameters& initial) :
m_link1_X_base0(),m_link2_X_link1(parameters),m_link3_X_link2(),m_link4_X_link3(),m_link5_X_link4(), parameters(initial)
{}


fancy::rcg2::Transforms::Transforms() :
m_link1_X_base0(),m_link2_X_link1(parameters),m_link3_X_link2(),m_link4_X_link3(),m_link5_X_link4(), parameters(/*do we want the default instance of ModelParameters ?*/)
{}



void fancy::rcg2::Transforms::update(const state_t& q)
{
    m_link1_X_base0.update(q);
    m_link2_X_link1.update(q);
    m_link3_X_link2.update(q);
    m_link4_X_link3.update(q);
    m_link5_X_link4.update(q);
}



