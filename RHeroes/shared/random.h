/*
 * random.h
 *
 *  Created on: 22/apr/2012
 *      Author: Mladen Mazuran
 */

#ifndef RANDOM_H_
#define RANDOM_H_

/* RandomLib generates unused parameter warnings which I don't like */
#if defined(__GNUC__) || defined(__clang__) || defined(__llvm__)
#	pragma GCC diagnostic ignored "-Wunused-parameter"
#	include <RandomLib/Random.hpp>
#	include <RandomLib/NormalDistribution.hpp>
#	pragma GCC diagnostic warning "-Wunused-parameter"
#else
#	include <RandomLib/Random.hpp>
#	include <RandomLib/NormalDistribution.hpp>
#endif

#include "stepwisecdf.h"
#include <Eigen/Core>
#include <Eigen/Cholesky>

namespace Shared {

using namespace Eigen;

class Random {
public:
	static int integer();
	static int integer(int a, int b);

	static double uniform();
	static double uniform(double a, double b);

	template <typename T>
    static T sample(const StepwiseCDF<T> &cdf);

	static double normal(double mu = 0, double sigma = 1);

	static VectorXd multiNormal(int rows);

	template <typename Derived>
    static VectorXd multiNormal(
            const MatrixBase<Derived> &mu);

	template <typename Derived1, typename Derived2>
	static VectorXd multiNormal(
	        const MatrixBase<Derived1> &mu,
	        const MatrixBase<Derived2> &sigma);

	template <typename Derived1, typename Derived2>
    static VectorXd multiNormalRoot(
            const MatrixBase<Derived1> &mu,
            const MatrixBase<Derived2> &sigmaRoot);

	template <typename Derived1, typename Derived2>
    static VectorXd multiNormalRootUncorrelated(
            const MatrixBase<Derived1> &mu,
            const MatrixBase<Derived2> &sigmaRoot);



private:
	static struct RandomState {
		RandomLib::Random r;
		RandomLib::NormalDistribution<double> n;

		RandomState() {
			r.Reseed();
		}
	} state;
};

inline int Random::integer() {
	return state.r.Integer();
}

inline int Random::integer(int a, int b) {
	return state.r.IntegerC(a, b);
}

inline double Random::uniform() {
	return state.r.Real();
}

inline double Random::uniform(double a, double b) {
	return (b - a) * state.r.Real() + a;
}

template <typename T>
inline T Random::sample(const StepwiseCDF<T> &cdf) {
    return cdf.inverseLookup(uniform());
}

inline double Random::normal(double mu, double sigma) {
	return state.n(state.r, mu, sigma);
}

inline VectorXd Random::multiNormal(int rows) {
    VectorXd ret(rows);
    for(int i = 0; i < rows; i++) {
        ret[i] = state.n(state.r);
    }
    return ret;
}

template <typename Derived>
inline VectorXd Random::multiNormal(const MatrixBase<Derived> &mu) {
    return mu + multiNormal(mu.rows());
}

template <typename Derived1, typename Derived2>
inline VectorXd Random::multiNormal(
        const MatrixBase<Derived1> &mu,
        const MatrixBase<Derived2> &sigma) {
    if(sigma.cols() == 1) {
        VectorXd ret(mu.rows());
        for(int i = 0; i < mu.rows(); i++) {
            ret[i] = mu[i] + std::sqrt(sigma(i,0)) * state.n(state.r);
        }
        return ret;
    /*} else if(sigma.isDiagonal()) {
        VectorXd ret(mu.rows());
        for(int i = 0; i < mu.rows(); i++) {
            ret[i] = mu[i] + std::sqrt(sigma(i,i)) * state.n(state.r);
        }
        return ret;*/
    } else {
        return mu + sigma.llt().matrixL() * multiNormal(mu.rows());
    }
}

template <typename Derived1, typename Derived2>
inline VectorXd Random::multiNormalRoot(
        const MatrixBase<Derived1> &mu,
        const MatrixBase<Derived2> &sigmaRoot) {
    return mu + sigmaRoot * multiNormal(mu.rows());
}

template <typename Derived1, typename Derived2>
inline VectorXd Random::multiNormalRootUncorrelated(
        const MatrixBase<Derived1> &mu,
        const MatrixBase<Derived2> &sigmaRoot) {
    VectorXd ret(mu.rows());
    for(int i = 0; i < mu.rows(); i++) {
        ret[i] = mu[i] + sigmaRoot[i] * state.n(state.r);
    }
    return ret;
}

} /* namespace Shared */

#endif /* RANDOM_H_ */
