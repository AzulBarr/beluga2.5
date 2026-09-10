#ifndef BELUGASLAM_CORE_POSE_GRAPH_COST_HPP
#define BELUGASLAM_CORE_POSE_GRAPH_COST_HPP
#include <ceres/ceres.h>
#include "pose_graph_residual.hpp"

class AnalyticPoseGraphCost final : public ceres::SizedCostFunction<3,3,3> {
public:
    AnalyticPoseGraphCost(double dx, double dy, double angle, double wt=1, double wr=1,
                          double ox=0, double oy=0, double oa=0)
        : residual_(dx,dy,angle,wt,wr,ox,oy,oa) {}
    bool Evaluate(double const* const* p, double* r, double** j) const override {
        return residual_.evaluate(p[0],p[1],r,j ? j[0] : nullptr,j ? j[1] : nullptr);
    }
private:
    belugaslam::PoseGraphResidual residual_;
};

struct PoseGraphEdgeError {
    PoseGraphEdgeError(double dx, double dy, double dtheta, double weight_translation = 1.0, double weight_rotation = 1.0,
                       double offset_x = 0.0, double offset_y = 0.0, double offset_angle = 0.0)
        : dx_(dx), dy_(dy), dtheta_(dtheta), 
          weight_translation_(weight_translation), weight_rotation_(weight_rotation),
          offset_x_(offset_x), offset_y_(offset_y), offset_angle_(offset_angle) {}

    template <typename T>
    bool operator()(const T* const pose_i, const T* const pose_j, T* residuals) const {
        // A live submap can be a fixed offset from a shared rigid group variable.
        T xi = pose_i[0] + ceres::cos(pose_i[2]) * T(offset_x_) - ceres::sin(pose_i[2]) * T(offset_y_);
        T yi = pose_i[1] + ceres::sin(pose_i[2]) * T(offset_x_) + ceres::cos(pose_i[2]) * T(offset_y_);
        T theta_i = pose_i[2] + T(offset_angle_);

        T xj = pose_j[0];
        T yj = pose_j[1];
        T theta_j = pose_j[2];

        T cos_theta_i = ceres::cos(theta_i);
        T sin_theta_i = ceres::sin(theta_i);

        // Relative position of j in i's frame
        T dx_ij = xj - xi;
        T dy_ij = yj - yi;

        T local_x = cos_theta_i * dx_ij + sin_theta_i * dy_ij;
        T local_y = -sin_theta_i * dx_ij + cos_theta_i * dy_ij;

        // Residuals scaled by weights
        residuals[0] = (local_x - T(dx_)) * T(weight_translation_);
        residuals[1] = (local_y - T(dy_)) * T(weight_translation_);
        
        T diff_theta = (theta_j - theta_i) - T(dtheta_);
        residuals[2] = ceres::atan2(ceres::sin(diff_theta), ceres::cos(diff_theta)) * T(weight_rotation_);

        return true;
    }

    static ceres::CostFunction* Create(double dx, double dy, double dtheta, double weight_translation = 1.0, double weight_rotation = 1.0,
                                      double offset_x = 0.0, double offset_y = 0.0, double offset_angle = 0.0,
                                      bool use_analytic = true) {
        if (use_analytic) return new AnalyticPoseGraphCost(dx, dy, dtheta, weight_translation, weight_rotation, offset_x, offset_y, offset_angle);
        return new ceres::AutoDiffCostFunction<PoseGraphEdgeError, 3, 3, 3>(
            new PoseGraphEdgeError(dx, dy, dtheta, weight_translation, weight_rotation, offset_x, offset_y, offset_angle));
    }

    double dx_, dy_, dtheta_;
    double weight_translation_, weight_rotation_;
    double offset_x_, offset_y_, offset_angle_;
};

#endif
