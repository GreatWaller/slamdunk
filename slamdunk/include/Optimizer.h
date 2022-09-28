#pragma once

#include "Common.h"
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_impl.h>


namespace slamdunk {
	class VertexPose : public g2o::BaseVertex<6, SE3> {
		//EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		virtual void setToOriginImpl() override {
			_estimate = SE3();
		}

		virtual void oplusImpl(const double* update)override {
			Vec6 vec;
			vec << update[0], update[1], update[2], update[3], update[4],
				update[5];
			_estimate = SE3::exp(vec) * _estimate;
		}

		virtual bool read(std::istream& in) override {
			return true;
		}

		virtual bool write(std::ostream& out) const override {
			return true;
		}

	};

	/// <summary>
	/// 估计位姿的一元边
	/// </summary>
	class EdgeProjectionPoseOnly : public g2o::BaseUnaryEdge<2, Vec2, VertexPose> {
	private:
		Vec3 mPosition;
		Mat33 mK;

	public:
		EdgeProjectionPoseOnly(const Vec3& pos, const Mat33& K)
			: mPosition(pos), mK(K)
		{

		}
		/// <summary>
		/// error
		/// </summary>
		void computeError() override {
			const VertexPose* vertex = static_cast<VertexPose*>(_vertices[0]);
			SE3 T = vertex->estimate();
			Vec3 posPixel = mK * (T * mPosition);

			posPixel /= posPixel[2];
			_error = _measurement - posPixel.head<2>();
		}

		/// <summary>
		/// jacobian
		/// </summary>
		void linearizeOplus() override {
			const VertexPose* vertex = static_cast<VertexPose*>(_vertices[0]);
			SE3 T = vertex->estimate();
			Vec3 posCamera = T * mPosition;
			double fx = mK(0, 0);
			double fy = mK(1, 1);
			double X = posCamera[0];
			double Y = posCamera[1];
			double Z = posCamera[2];
			double Zinv = 1.0 / (Z + 1e-18);
			double Zinv2 = Zinv * Zinv;

			_jacobianOplusXi << -fx * Zinv, 0, fx* X* Zinv2, fx* X* Y* Zinv2,
				-fx - fx * X * X * Zinv2, fx* Y* Zinv, 0, -fy * Zinv,
				fy* Y* Zinv2, fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2,
				-fy * X * Zinv;
		}

		bool read(std::istream& in) override { return true; }
		bool write(std::ostream& out) const override { return true; }

	};

}