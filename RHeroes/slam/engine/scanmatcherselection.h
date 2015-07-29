/*
 * scanmatcherselection.h
 *
 *  Created on: 27/dic/2012
 *      Author: Mladen Mazuran
 */

#ifndef SCANMATCHERSELECTION_H_
#define SCANMATCHERSELECTION_H_

namespace SLAM {
namespace Engine {

enum ScanMatcherSelection {
    LiGriffithsICLSelection,
    ClassicICLSelection,
    FilteredICLSelection,
    RANSACMatcherSelection
};

} /* namespace Engine */
} /* namespace SLAM */

#endif /* SCANMATCHERSELECTION_H_ */
