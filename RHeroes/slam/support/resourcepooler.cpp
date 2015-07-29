/*
 * resourcepooler.cpp
 *
 *  Created on: 23/apr/2012
 *      Author: Mladen Mazuran
 */

#include "resourcepooler.h"
#include "shared/logger.h"

namespace SLAM {
namespace Support {

ResourcePooler::ResourcePooler()
{
}

ResourcePooler::~ResourcePooler()
{
	if(pointers.size() > 0) {
		ldbg << "Memory leak in ResourcePooler, can't deallocate void * pointer casts" << endl;
	}
}

} /* namespace Support */
} /* namespace SLAM */
