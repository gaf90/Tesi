/*
 * resourcepooler.h
 *
 *  Created on: 23/apr/2012
 *      Author: Mladen Mazuran
 */

#ifndef RESOURCEPOOLER_H_
#define RESOURCEPOOLER_H_

#include <QHash>

namespace SLAM {
namespace Support {

class ResourcePooler {
public:
	ResourcePooler();
	virtual ~ResourcePooler();

	template <typename T> T   *acquire(T *resource);
	template <typename T> void release(T *resource);
	template <typename T> int  count  (T *resource);

private:
	typedef QHash<void *, int> PointerHash;
	PointerHash pointers;
};

template <typename T>
inline T *ResourcePooler::acquire(T *resource)
{
	void *pointer = static_cast<void *>(resource);

	PointerHash::iterator i = pointers.find(pointer);

	if(i != pointers.end() && i.key() == pointer) {
		i.value()++;
	} else {
		pointers.insert(pointer, 1);
	}

	return resource;
}

template <typename T>
inline void ResourcePooler::release(T *resource)
{
	void *pointer = static_cast<void *>(resource);

	PointerHash::iterator i = pointers.find(pointer);

	if(i != pointers.end() && i.key() == pointer) {
		int &n = i.value();
		n--;
		if(n == 0) {
			pointers.remove(pointer);
			delete resource;
		}
	}
}

template <typename T>
inline int ResourcePooler::count(T *resource)
{
	void *pointer = static_cast<void *>(resource);

	PointerHash::iterator i = pointers.find(pointer);

	if(i != pointers.end() && i.key() == pointer) {
		return i.value();
	} else {
		return 0;
	}
}

} /* namespace Support */
} /* namespace SLAM */

#endif /* RESOURCEPOOLER_H_ */
