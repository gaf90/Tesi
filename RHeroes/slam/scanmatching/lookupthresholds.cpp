/*
 * lookupthresholds.cpp
 *
 *  Created on: 27/dic/2012
 *      Author: Mladen Mazuran
 */

#include "associationelseberg.h"
#include "associationamigoni.h"
#include "associationligriffiths.h"
#include "associationprobabilistic.h"
#include "associationposecentric.h"
#include "shared/config.h"

namespace SLAM {
namespace ScanMatching {

const double &AssociationElseberg::lookupThreshold              =
        Config::SLAM::lookupThresholdElseberg;
const double &AssociationElseberg::broadLookupThreshold         =
        Config::SLAM::broadLookupThresholdElseberg;

const double &AssociationAmigoni::lookupThreshold               =
        Config::SLAM::lookupThresholdAmigoni;
const double &AssociationAmigoni::broadLookupThreshold          =
        Config::SLAM::broadLookupThresholdAmigoni;

const double &AssociationLiGriffiths::lookupThreshold           =
        Config::SLAM::lookupThresholdLiGriffiths;
const double &AssociationLiGriffiths::broadLookupThreshold      =
        Config::SLAM::broadLookupThresholdLiGriffiths;

const double &AssociationProbabilistic::lookupThreshold         =
        Config::SLAM::lookupThresholdProbabilistic;
const double &AssociationProbabilistic::broadLookupThreshold    =
        Config::SLAM::broadLookupThresholdProbabilistic;

const double &AssociationPoseCentric::lookupThreshold         =
        Config::SLAM::lookupThresholdPoseCentric;
const double &AssociationPoseCentric::broadLookupThreshold    =
        Config::SLAM::broadLookupThresholdPoseCentric;

} /* namespace ScanMatching */
} /* namespace SLAM */
