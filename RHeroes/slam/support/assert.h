/*
 * assert.h
 *
 *  Created on: 05/gen/2013
 *      Author: Mladen Mazuran
 */

#ifndef ASSERT_H_
#define ASSERT_H_

namespace SLAM {

#define static_assert(x)    CompileTimeAssertion<(x)>::valid()
#define delayed_false(Cls)  DelayedFalseEvaluation<Cls>::delayer

template <bool>
struct CompileTimeAssertion
{
};

template <>
struct CompileTimeAssertion<true>
{
    static inline void valid() {}
};

template <typename T>
struct DelayedFalseEvaluation
{
    static const bool delayer = false;
};

} /* namespace SLAM */

#endif /* ASSERT_H_ */
