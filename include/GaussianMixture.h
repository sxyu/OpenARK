#pragma once
#include "Version.h"
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
namespace ark{
    /** Gaussian Mixture Model */
    class GaussianMixture { 
    public:
        /** load Gaussian Mixture parameters from 'path' */
        void load(const std::string & path);

        /** get number of Gaussian mixture components */
        inline int numComponents() const { return nComps; };

        template<class T>
        /** Compute PDF at 'input' */
        T pdf(const Eigen::Matrix<T, Eigen::Dynamic, 1> & x) const {
            T prob = 0.0;
            typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> mat_t;
            for (int i = 0; i < nComps; ++i) {
                Eigen::TriangularView<Eigen::MatrixXd, Eigen::Lower> L(cov_cho[i]);
                auto residual = (L.transpose() * (x - mean.row(i).transpose()));
                prob += consts[i] * ceres::exp(-0.5 * residual.squaredNorm());
            }
            return prob;
        }

        template<class T>
        /** Compute Ceres residual vector (squaredNorm of output vector is equal to min_i -log(c_i pdf_i(x))) */
        Eigen::Matrix<T, Eigen::Dynamic, 1> residual(const Eigen::Matrix<T, Eigen::Dynamic, 1> & x) {
            T bestProb = T(0);
            typedef Eigen::Matrix<T, Eigen::Dynamic, 1> vec_t;
            typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> mat_t;
            vec_t ans;
            for (int i = 0; i < nComps; ++i) {
                Eigen::TriangularView<Eigen::MatrixXd, Eigen::Lower> L(cov_cho[i]);
                vec_t residual(nDims + 1);
                residual[nDims] = T(0);
                residual.head(nDims) = L.transpose() * (x - mean.row(i).transpose()) * sqrt(0.5);
                T p = residual.squaredNorm() - T(consts_log[i]);
                if (p < bestProb || !i) {
                    bestProb = p;
                    residual[nDims] = T(sqrt(-consts_log[i]));
                    ans = residual;
                }
            }
            return ans;
        }
    private:
        int nComps, nDims;
        Eigen::VectorXd weight;
        Eigen::MatrixXd mean;
        std::vector<Eigen::MatrixXd> cov;

        // leading constants
        Eigen::VectorXd consts, consts_log;
        // cholesky decomposition of inverse: cov^-1 = cov_cho * cov_cho^T
        std::vector<Eigen::MatrixXd> cov_cho;
    };
}
