/*
 * fforeach.h
 *
 *  Created on: 05/gen/2013
 *      Author: Mladen Mazuran
 */

#ifndef FFOREACH_H_
#define FFOREACH_H_

#include <qglobal.h>

/*
    Faster foreach version with no copying. Doesn't work with temporaries, use standard foreach in
    such cases
*/
#if defined(Q_CC_GNU) && !defined(Q_CC_INTEL) && !defined(Q_CC_RVCT)

namespace __internal {

template <typename T>
struct FastForeachContainer {
    FastForeachContainer(T &t) : b(1), i(t.begin()), e(t.end()) {}
    int b; typename T::iterator i, e;
};

template <typename T>
struct FastForeachContainer<const T> {
    FastForeachContainer(const T &t) : b(1), i(t.begin()), e(t.end()) {}
    int b; typename T::const_iterator i, e;
};

} /* namespace __internal */

#   define fforeach(decl, var) \
    for(::__internal::FastForeachContainer<__typeof__(var)> __f_container(var); \
        __f_container.b && __f_container.i != __f_container.e; \
        ++__f_container.i, __f_container.b--) \
        for(decl = *__f_container.i;; __extension__({ __f_container.b++; break; }))
#else
#   define fforeach Q_FOREACH
#endif

#endif /* FFOREACH_H_ */
