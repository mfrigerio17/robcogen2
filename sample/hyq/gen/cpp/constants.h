#ifndef RCG2_HYQ_CONSTANTS_H
#define RCG2_HYQ_CONSTANTS_H

#include "rbd_types.h"

namespace hyq {
namespace rcg2 {

struct ModelConstants {
    static constexpr Scalar trunk_mass{55.4};
    static constexpr Scalar trunk_comx{-0.024};
    static constexpr Scalar trunk_comy{0.0023};
    static constexpr Scalar trunk_ixx{1.688238};
    static constexpr Scalar trunk_iyy{8.665299};
    static constexpr Scalar trunk_izz{9.24356};
    static constexpr Scalar trunk_ixy{-0.028789};
    static constexpr Scalar trunk_ixz{0.277694};
    static constexpr Scalar trunk_iyz{0.00382};
    static constexpr Scalar LF_hipassembly_mass{2.93};
    static constexpr Scalar LF_hipassembly_comx{0.04263};
    static constexpr Scalar LF_hipassembly_comz{0.16931};
    static constexpr Scalar LF_hipassembly_ixx{0.134705};
    static constexpr Scalar LF_hipassembly_iyy{0.144171};
    static constexpr Scalar LF_hipassembly_izz{0.011033};
    static constexpr Scalar LF_hipassembly_ixy{3.6e-05};
    static constexpr Scalar LF_hipassembly_ixz{0.022734};
    static constexpr Scalar LF_hipassembly_iyz{5.1e-05};
    static constexpr Scalar LF_upperleg_mass{2.638};
    static constexpr Scalar LF_upperleg_comx{0.15074};
    static constexpr Scalar LF_upperleg_comy{-0.02625};
    static constexpr Scalar LF_upperleg_ixx{0.005495};
    static constexpr Scalar LF_upperleg_iyy{0.087136};
    static constexpr Scalar LF_upperleg_izz{0.089871};
    static constexpr Scalar LF_upperleg_ixy{-0.007418};
    static constexpr Scalar LF_upperleg_ixz{-0.000102};
    static constexpr Scalar LF_upperleg_iyz{-2.1e-05};
    static constexpr Scalar LF_lowerleg_mass{0.881};
    static constexpr Scalar LF_lowerleg_comx{0.1254};
    static constexpr Scalar LF_lowerleg_comy{0.0005};
    static constexpr Scalar LF_lowerleg_comz{-0.0001};
    static constexpr Scalar LF_lowerleg_ixx{0.000468};
    static constexpr Scalar LF_lowerleg_iyy{0.026409};
    static constexpr Scalar LF_lowerleg_izz{0.026181};
    static constexpr Scalar RF_hipassembly_mass{2.93};
    static constexpr Scalar RF_hipassembly_comx{0.04263};
    static constexpr Scalar RF_hipassembly_comz{-0.16931};
    static constexpr Scalar RF_hipassembly_ixx{0.134705};
    static constexpr Scalar RF_hipassembly_iyy{0.144171};
    static constexpr Scalar RF_hipassembly_izz{0.011033};
    static constexpr Scalar RF_hipassembly_ixy{-3.6e-05};
    static constexpr Scalar RF_hipassembly_ixz{-0.022734};
    static constexpr Scalar RF_hipassembly_iyz{5.1e-05};
    static constexpr Scalar RF_upperleg_mass{2.638};
    static constexpr Scalar RF_upperleg_comx{0.15074};
    static constexpr Scalar RF_upperleg_comy{-0.02625};
    static constexpr Scalar RF_upperleg_ixx{0.005495};
    static constexpr Scalar RF_upperleg_iyy{0.087136};
    static constexpr Scalar RF_upperleg_izz{0.089871};
    static constexpr Scalar RF_upperleg_ixy{-0.007418};
    static constexpr Scalar RF_upperleg_ixz{-0.000102};
    static constexpr Scalar RF_upperleg_iyz{-2.1e-05};
    static constexpr Scalar RF_lowerleg_mass{0.881};
    static constexpr Scalar RF_lowerleg_comx{0.1254};
    static constexpr Scalar RF_lowerleg_comy{0.0005};
    static constexpr Scalar RF_lowerleg_comz{-0.0001};
    static constexpr Scalar RF_lowerleg_ixx{0.000468};
    static constexpr Scalar RF_lowerleg_iyy{0.026409};
    static constexpr Scalar RF_lowerleg_izz{0.026181};
    static constexpr Scalar LH_hipassembly_mass{2.93};
    static constexpr Scalar LH_hipassembly_comx{0.04263};
    static constexpr Scalar LH_hipassembly_comz{-0.16931};
    static constexpr Scalar LH_hipassembly_ixx{0.134705};
    static constexpr Scalar LH_hipassembly_iyy{0.144171};
    static constexpr Scalar LH_hipassembly_izz{0.011033};
    static constexpr Scalar LH_hipassembly_ixy{-3.6e-05};
    static constexpr Scalar LH_hipassembly_ixz{-0.022734};
    static constexpr Scalar LH_hipassembly_iyz{5.1e-05};
    static constexpr Scalar LH_upperleg_mass{2.638};
    static constexpr Scalar LH_upperleg_comx{0.15074};
    static constexpr Scalar LH_upperleg_comy{0.02625};
    static constexpr Scalar LH_upperleg_ixx{0.005495};
    static constexpr Scalar LH_upperleg_iyy{0.087136};
    static constexpr Scalar LH_upperleg_izz{0.089871};
    static constexpr Scalar LH_upperleg_ixy{0.007418};
    static constexpr Scalar LH_upperleg_ixz{0.000102};
    static constexpr Scalar LH_upperleg_iyz{-2.1e-05};
    static constexpr Scalar LH_lowerleg_mass{0.881};
    static constexpr Scalar LH_lowerleg_comx{0.1254};
    static constexpr Scalar LH_lowerleg_comy{-0.0005};
    static constexpr Scalar LH_lowerleg_comz{0.0001};
    static constexpr Scalar LH_lowerleg_ixx{0.000468};
    static constexpr Scalar LH_lowerleg_iyy{0.026409};
    static constexpr Scalar LH_lowerleg_izz{0.026181};
    static constexpr Scalar RH_hipassembly_mass{2.93};
    static constexpr Scalar RH_hipassembly_comx{0.04263};
    static constexpr Scalar RH_hipassembly_comz{0.16931};
    static constexpr Scalar RH_hipassembly_ixx{0.134705};
    static constexpr Scalar RH_hipassembly_iyy{0.144171};
    static constexpr Scalar RH_hipassembly_izz{0.011033};
    static constexpr Scalar RH_hipassembly_ixy{3.6e-05};
    static constexpr Scalar RH_hipassembly_ixz{0.022734};
    static constexpr Scalar RH_hipassembly_iyz{5.1e-05};
    static constexpr Scalar RH_upperleg_mass{2.638};
    static constexpr Scalar RH_upperleg_comx{0.15074};
    static constexpr Scalar RH_upperleg_comy{0.02625};
    static constexpr Scalar RH_upperleg_ixx{0.005495};
    static constexpr Scalar RH_upperleg_iyy{0.087136};
    static constexpr Scalar RH_upperleg_izz{0.089871};
    static constexpr Scalar RH_upperleg_ixy{0.007418};
    static constexpr Scalar RH_upperleg_ixz{0.000102};
    static constexpr Scalar RH_upperleg_iyz{-2.1e-05};
    static constexpr Scalar RH_lowerleg_mass{0.881};
    static constexpr Scalar RH_lowerleg_comx{0.1254};
    static constexpr Scalar RH_lowerleg_comy{-0.0005};
    static constexpr Scalar RH_lowerleg_comz{0.0001};
    static constexpr Scalar RH_lowerleg_ixx{0.000468};
    static constexpr Scalar RH_lowerleg_iyy{0.026409};
    static constexpr Scalar RH_lowerleg_izz{0.026181};
    static constexpr Scalar LF_HAA_tx{0.3735};
    static constexpr Scalar LF_HAA_ty{0.207};
    static constexpr Scalar LF_HFE_tx{0.08};
    static constexpr Scalar LF_KFE_tx{0.35};
    static constexpr Scalar RF_HAA_tx{0.3735};
    static constexpr Scalar RF_HAA_ty{-0.207};
    static constexpr Scalar RF_HFE_tx{0.08};
    static constexpr Scalar RF_KFE_tx{0.35};
    static constexpr Scalar LH_HAA_tx{-0.3735};
    static constexpr Scalar LH_HAA_ty{0.207};
    static constexpr Scalar LH_HFE_tx{0.08};
    static constexpr Scalar LH_KFE_tx{0.35};
    static constexpr Scalar RH_HAA_tx{-0.3735};
    static constexpr Scalar RH_HAA_ty{-0.207};
    static constexpr Scalar RH_HFE_tx{0.08};
    static constexpr Scalar RH_KFE_tx{0.35};
    static constexpr Scalar LF_foot_tx{0.33};
    static constexpr Scalar RF_foot_tx{0.33};
    static constexpr Scalar LH_foot_tx{0.33};
    static constexpr Scalar RH_foot_tx{0.33};
};

}
}


#endif
