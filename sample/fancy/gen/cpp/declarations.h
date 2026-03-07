#ifndef RCG2_FANCY_DECLARATIONS_H
#define RCG2_FANCY_DECLARATIONS_H

#include <iit/rbd/data_map.h>
#include "rbd_types.h"

namespace fancy {
namespace rcg2 {

constexpr int JointSpaceDimension{5};
constexpr int jointsCount{5};
/** The total number of rigid bodies that can move */
constexpr int linksCount{5};

typedef Matrix<5, 1> Column5d;
typedef Column5d JointState;

enum JointIDs {
    jA, jB, jC, jD, jE
};

enum LinkIDs {
    link1, link2, link3, link4, link5
};

static constexpr const JointIDs orderedJointIDs[jointsCount] = {
    jA, jB, jC, jD, jE
};

static constexpr const LinkIDs orderedLinkIDs[linksCount] = {
    link1, link2, link3, link4, link5
};

template<typename T>
using LinkDataMap = iit::rbd::DataMap<T, linksCount, LinkIDs>;

template<typename T>
using JointDataMap = iit::rbd::DataMap<T, jointsCount, JointIDs>;

}
}
#endif
