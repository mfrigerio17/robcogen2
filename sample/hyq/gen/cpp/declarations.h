#ifndef RCG2_HYQ_DECLARATIONS_H
#define RCG2_HYQ_DECLARATIONS_H

#include <iit/rbd/data_map.h>
#include "rbd_types.h"

namespace hyq {
namespace rcg2 {

constexpr int JointSpaceDimension{12};
constexpr int jointsCount{12};
/** The total number of rigid bodies that can move */
constexpr int linksCount{13};

typedef Matrix<12, 1> Column12d;
typedef Column12d JointState;

enum JointIDs {
    LF_HAA, LF_HFE, LF_KFE, RF_HAA, RF_HFE, RF_KFE, LH_HAA, LH_HFE, LH_KFE, RH_HAA, RH_HFE, RH_KFE
};

enum LinkIDs {
    trunk, LF_hipassembly, LF_upperleg, LF_lowerleg, RF_hipassembly, RF_upperleg, RF_lowerleg, LH_hipassembly, LH_upperleg, LH_lowerleg, RH_hipassembly, RH_upperleg, RH_lowerleg
};

static constexpr const JointIDs orderedJointIDs[jointsCount] = {
    LF_HAA, LF_HFE, LF_KFE, RF_HAA, RF_HFE, RF_KFE, LH_HAA, LH_HFE, LH_KFE, RH_HAA, RH_HFE, RH_KFE
};

static constexpr const LinkIDs orderedLinkIDs[linksCount] = {
    trunk, LF_hipassembly, LF_upperleg, LF_lowerleg, RF_hipassembly, RF_upperleg, RF_lowerleg, LH_hipassembly, LH_upperleg, LH_lowerleg, RH_hipassembly, RH_upperleg, RH_lowerleg
};

template<typename T>
using LinkDataMap = iit::rbd::DataMap<T, linksCount, LinkIDs>;

template<typename T>
using JointDataMap = iit::rbd::DataMap<T, jointsCount, JointIDs>;

}
}
#endif
