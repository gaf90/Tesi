/*
 * frontier.cpp
 *
 *  Created on: 24/mar/2012
 *      Author: Mladen Mazuran
 */

#include "frontier.h"

namespace SLAM {
namespace Geometry {

LoggerStream &operator<<(LoggerStream &stream, const Frontier *front)
{
    return stream << *front;
}

} /* namespace Geometry */
} /* namespace SLAM */
