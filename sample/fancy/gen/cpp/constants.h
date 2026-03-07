#ifndef RCG2_FANCY_CONSTANTS_H
#define RCG2_FANCY_CONSTANTS_H

#include "rbd_types.h"

namespace fancy {
namespace rcg2 {

struct ModelConstants {
    static constexpr Scalar link1_comx{0.5};
    static constexpr Scalar link1_ixx{0.0025};
    static constexpr Scalar link1_iyy{0.33458};
    static constexpr Scalar link1_izz{0.33458};
    static constexpr Scalar link2_mass{1.0};
    static constexpr Scalar link2_comz{0.5};
    static constexpr Scalar link2_ixx{0.33458};
    static constexpr Scalar link2_iyy{0.33458};
    static constexpr Scalar link2_izz{0.0025};
    static constexpr Scalar link3_mass{1.0};
    static constexpr Scalar link3_comx{0.5};
    static constexpr Scalar link3_ixx{0.0025};
    static constexpr Scalar link3_iyy{0.33458};
    static constexpr Scalar link3_izz{0.33458};
    static constexpr Scalar link4_mass{1.0};
    static constexpr Scalar link4_comz{0.5};
    static constexpr Scalar link4_ixx{0.33458};
    static constexpr Scalar link4_iyy{0.33458};
    static constexpr Scalar link4_izz{0.0025};
    static constexpr Scalar link5_mass{1.0};
    static constexpr Scalar link5_comx{0.5};
    static constexpr Scalar link5_ixx{0.0025};
    static constexpr Scalar link5_iyy{0.33458};
    static constexpr Scalar link5_izz{0.33458};
    static constexpr Scalar jC_tz{1.0};
    static constexpr Scalar jD_tx{1.0};
    static constexpr Scalar jE_tz{1.0};
    static constexpr Scalar ee_tx{1.0};
};

}
}


#endif
