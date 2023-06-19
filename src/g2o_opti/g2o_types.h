#pragma once
#include "common/basetype.h"
#include "common/nav_state.h"
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/robust_kernel.h>
#include "imu_preinteg/imu_preintegration.h"

/**
 * 旋转在前的SO3+t类型pose，6自由度，存储时伪装为g2o::VertexSE3，供g2o_viewer查看
 */
class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d> 
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexPose() {}
    bool read(std::istream& is) override {
        double data[7];
        for (int i = 0; i < 7; i++) {
            is >> data[i];
        }
        setEstimate(Sophus::SE3d(Quaterniond(data[6], data[3], data[4], data[5]), Vector3d(data[0], data[1], data[2])));
    }
    bool write(std::ostream& os) const override {
        os << "VERTEX_SE3:QUAT ";
        os << id() << " ";
        Quaterniond q = _estimate.unit_quaternion();
        os << _estimate.translation().transpose() << " ";
        os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << std::endl;
        return true;
    }
    virtual void setToOriginImpl() {}

    virtual void oplusImpl(const double* update_) {
        _estimate.so3() = _estimate.so3() * Sophus::SO3d::exp(Eigen::Map<const Vector3d>(&update_[0]));  // 旋转部分
        _estimate.translation() += Eigen::Map<const Vector3d>(&update_[3]);                     // 平移部分
        updateCache();
    }
};

/**
 * 速度顶点，单纯的Vec3d
 */
class VertexVelocity : public g2o::BaseVertex<3, Vector3d> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexVelocity() {}

    virtual bool read(std::istream& is) { return false; }
    virtual bool write(std::ostream& os) const { return false; }

    virtual void setToOriginImpl() { _estimate.setZero(); }

    virtual void oplusImpl(const double* update_) { _estimate += Eigen::Map<const Vector3d>(update_); }
};

/**
 * 陀螺零偏顶点，亦为Vec3d，从速度顶点继承
 */
class VertexGyroBias : public VertexVelocity {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexGyroBias() {}
};

/**
 * 加计零偏顶点，Vec3d，亦从速度顶点继承
 */
class VertexAccBias : public VertexVelocity {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexAccBias() {}
};

/**
 * 陀螺随机游走
 */
class EdgeGyroRW : public g2o::BaseBinaryEdge<3, Vector3d, VertexGyroBias, VertexGyroBias> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeGyroRW() {}
    virtual bool read(std::istream& is) { return false; }
    virtual bool write(std::ostream& os) const { return false; }

    void computeError() {
        const auto* VG1 = dynamic_cast<const VertexGyroBias*>(_vertices[0]);
        const auto* VG2 = dynamic_cast<const VertexGyroBias*>(_vertices[1]);
        _error = VG2->estimate() - VG1->estimate();
    }

    virtual void linearizeOplus() {
        _jacobianOplusXi = -Matrix3d::Identity();
        _jacobianOplusXj.setIdentity();
    }

    Eigen::Matrix<double, 6, 6> GetHessian() {
        linearizeOplus();
        Eigen::Matrix<double, 3, 6> J;
        J.block<3, 3>(0, 0) = _jacobianOplusXi;
        J.block<3, 3>(0, 3) = _jacobianOplusXj;
        return J.transpose() * information() * J;
    }
};

/**
 * 加计随机游走
 */
class EdgeAccRW : public g2o::BaseBinaryEdge<3, Vec3d, VertexAccBias, VertexAccBias> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeAccRW() {}

    virtual bool read(std::istream& is) { return false; }
    virtual bool write(std::ostream& os) const { return false; }

    void computeError() {
        const auto* VA1 = dynamic_cast<const VertexAccBias*>(_vertices[0]);
        const auto* VA2 = dynamic_cast<const VertexAccBias*>(_vertices[1]);
        _error = VA2->estimate() - VA1->estimate();
    }

    virtual void linearizeOplus() {
        _jacobianOplusXi = -Matrix3d::Identity();
        _jacobianOplusXj.setIdentity();
    }

    Eigen::Matrix<double, 6, 6> GetHessian() {
        linearizeOplus();
        Eigen::Matrix<double, 3, 6> J;
        J.block<3, 3>(0, 0) = _jacobianOplusXi;
        J.block<3, 3>(0, 3) = _jacobianOplusXj;
        return J.transpose() * information() * J;
    }
};

/**
 * 对上一帧IMU pvq bias的先验
 * info 由外部指定，通过时间窗口边缘化给出
 *
 * 顶点顺序：pose, v, bg, ba
 * 残差顺序：R, p, v, bg, ba，15维
 */
class EdgePriorPoseNavState : public g2o::BaseMultiEdge<15, Matrix<double, 15, 15>> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgePriorPoseNavState(const NavStated& state, const Matrix<double, 15, 15>& info);

    virtual bool read(std::istream& is) { return false; }
    virtual bool write(std::ostream& os) const { return false; }

    void computeError();
    virtual void linearizeOplus();

    Eigen::Matrix<double, 15, 15> GetHessian() {
        linearizeOplus();
        Eigen::Matrix<double, 15, 15> J;
        J.block<15, 6>(0, 0) = _jacobianOplus[0];
        J.block<15, 3>(0, 6) = _jacobianOplus[1];
        J.block<15, 3>(0, 9) = _jacobianOplus[2];
        J.block<15, 3>(0, 12) = _jacobianOplus[3];
        return J.transpose() * information() * J;
    }

    NavStated state_;
};

/**
 * 6 自由度的GNSS
 * 误差的角度在前，平移在后
 */
class EdgeGNSS : public g2o::BaseUnaryEdge<6, Sophus::SE3d, VertexPose> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeGNSS() = default;
    EdgeGNSS(VertexPose* v, const Sophus::SE3d& obs) {
        setVertex(0, v);
        setMeasurement(obs);
    }

    void computeError() override {
        VertexPose* v = (VertexPose*)_vertices[0];
        _error.head<3>() = (_measurement.so3().inverse() * v->estimate().so3()).log();
        _error.tail<3>() = v->estimate().translation() - _measurement.translation();
    };

    void linearizeOplus() override {
        VertexPose* v = (VertexPose*)_vertices[0];
        // jacobian 6x6
        _jacobianOplusXi.setZero();
        _jacobianOplusXi.block<3, 3>(0, 0) = (_measurement.so3().inverse() * v->estimate().so3()).jr_inv();  // dR/dR
        _jacobianOplusXi.block<3, 3>(3, 3) = Matrix3d::Identity();                                              // dp/dp
    }

    Matrix<double, 6, 6> GetHessian() {
        linearizeOplus();
        return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
    }

    virtual bool read(std::istream& in) { return true; }
    virtual bool write(std::ostream& out) const { return true; }

   private:
};

/**
 * 只有平移的GNSS
 * 此时需要提供RTK外参 TBG，才能正确施加约束
 */
class EdgeGNSSTransOnly : public g2o::BaseUnaryEdge<3, Vector3d, VertexPose> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     * 指定位姿顶点、RTK观测 t_WG、外参TGB
     * @param v
     * @param obs
     */
    EdgeGNSSTransOnly(VertexPose* v, const Vector3d& obs, const Sophus::SE3d& TBG) : TBG_(TBG) {
        setVertex(0, v);
        setMeasurement(obs);
    }

    void computeError() override {
        VertexPose* v = (VertexPose*)_vertices[0];
        _error = (v->estimate() * TBG_).translation() - _measurement;
    };

    // void linearizeOplus() override {
    //     VertexPose* v = (VertexPose*)_vertices[0];
    //     // jacobian 6x6
    //     _jacobianOplusXi.setZero();
    //     _jacobianOplusXi.block<3, 3>(0, 0) = (_measurement.so3().inverse() * v->estimate().so3()).jr_inv();  // dR/dR
    //     _jacobianOplusXi.block<3, 3>(3, 3) = Mat3d::Identity();                                              // dp/dp
    // }

    virtual bool read(std::istream& in) { return true; }
    virtual bool write(std::ostream& out) const { return true; }

   private:
    Sophus::SE3d TBG_;
};

/**
 * 6 自由度相对运动
 * 误差的平移在前，角度在后
 * 观测：T12
 */
class EdgeRelativeMotion : public g2o::BaseBinaryEdge<6, Sophus::SE3d, VertexPose, VertexPose> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeRelativeMotion() = default;
    EdgeRelativeMotion(VertexPose* v1, VertexPose* v2, const Sophus::SE3d& obs) {
        setVertex(0, v1);
        setVertex(1, v2);
        setMeasurement(obs);
    }

    void computeError() override {
        VertexPose* v1 = (VertexPose*)_vertices[0];
        VertexPose* v2 = (VertexPose*)_vertices[1];
        Sophus::SE3d T12 = v1->estimate().inverse() * v2->estimate();
        _error = (_measurement.inverse() * v1->estimate().inverse() * v2->estimate()).log();
    };

    virtual bool read(std::istream& is) override {
        double data[7];
        for (int i = 0; i < 7; i++) {
            is >> data[i];
        }
        Quaterniond q(data[6], data[3], data[4], data[5]);
        q.normalize();
        setMeasurement(Sophus::SE3d(q, Vector3d(data[0], data[1], data[2])));
        for (int i = 0; i < information().rows() && is.good(); i++) {
            for (int j = i; j < information().cols() && is.good(); j++) {
                is >> information()(i, j);
                if (i != j) information()(j, i) = information()(i, j);
            }
        }
        return true;
    }

    virtual bool write(std::ostream& os) const override {
        os << "EDGE_SE3:QUAT ";
        auto* v1 = static_cast<VertexPose*>(_vertices[0]);
        auto* v2 = static_cast<VertexPose*>(_vertices[1]);
        os << v1->id() << " " << v2->id() << " ";
        Sophus::SE3d m = _measurement;
        Eigen::Quaterniond q = m.unit_quaternion();
        os << m.translation().transpose() << " ";
        os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << " ";

        // information matrix
        for (int i = 0; i < information().rows(); i++) {
            for (int j = i; j < information().cols(); j++) {
                os << information()(i, j) << " ";
            }
        }
        os << std::endl;
        return true;
    }

   private:
};

/**
 * 3维 轮速计观测边
 * 轮速观测世界速度在自车坐标系下矢量, 3维情况下假设自车不会有y和z方向速度
 */
class EdgeEncoder3D : public g2o::BaseUnaryEdge<3, Vector3d, VertexVelocity> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeEncoder3D() = default;

    /**
     * 构造函数需要知道世界系下速度
     * @param v0
     * @param speed
     */
    EdgeEncoder3D(VertexVelocity* v0, const Vector3d& speed) {
        setVertex(0, v0);
        setMeasurement(speed);
    }

    void computeError() override {
        VertexVelocity* v0 = (VertexVelocity*)_vertices[0];
        _error = v0->estimate() - _measurement;
    }

    void linearizeOplus() override { _jacobianOplusXi.setIdentity(); }

    virtual bool read(std::istream& in) { return true; }
    virtual bool write(std::ostream& out) const { return true; }
};

/**
 * NDT误差模型
 * 残差是 Rp+t-mu，info为NDT内部估计的info
 */
class EdgeNDT : public g2o::BaseUnaryEdge<3, Vector3d, VertexPose> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeNDT() = default;

    /// 需要查询NDT内部的体素，这里用一个函数式给设置过去
    using QueryVoxelFunc = std::function<bool(const Vector3d& query_pt, Vector3d& mu, Matrix3d& info)>;

    EdgeNDT(VertexPose* v0, const Vector3d& pt, QueryVoxelFunc func) {
        setVertex(0, v0);
        pt_ = pt;
        query_ = func;

        Vector3d q = v0->estimate().so3() * pt_ + v0->estimate().translation();
        if (query_(q, mu_, info_)) {
            setInformation(info_);
            valid_ = true;
        } else {
            valid_ = false;
        }
    }

    bool IsValid() const { return valid_; }

    Matrix<double, 6, 6> GetHessian() {
        linearizeOplus();
        return _jacobianOplusXi.transpose() * info_ * _jacobianOplusXi;
    }

    /// 残差计算
    void computeError() override {
        VertexPose* v0 = (VertexPose*)_vertices[0];
        Vector3d q = v0->estimate().so3() * pt_ + v0->estimate().translation();

        if (query_(q, mu_, info_)) {
            _error = q - mu_;
            setInformation(info_);
            valid_ = true;
        } else {
            valid_ = false;
            _error.setZero();
            setLevel(1);
        }
    }

    /// 线性化
    void linearizeOplus() override {
        if (valid_) {
            VertexPose* v0 = (VertexPose*)_vertices[0];
            Sophus::SO3d R = v0->estimate().so3();

            _jacobianOplusXi.setZero();
            _jacobianOplusXi.block<3, 3>(0, 0) = -R.matrix() * Sophus::SO3d::hat(pt_);  // 对R
            _jacobianOplusXi.block<3, 3>(0, 3) = Matrix3d::Identity();            // 对p
        } else {
            _jacobianOplusXi.setZero();
        }
    }

    virtual bool read(std::istream& in) { return true; }
    virtual bool write(std::ostream& out) const { return true; }

   private:
    QueryVoxelFunc query_;
    Vector3d pt_ = Vector3d::Zero();
    Vector3d mu_ = Vector3d::Zero();
    Matrix3d info_ = Matrix3d::Identity();
    bool valid_ = false;
};

/// 与预积分相关的vertex, edge
/**
 * 预积分边
 * 连接6个顶点：上一帧的pose, v, bg, ba，下一帧的pose, v
 * 观测量为9维，即预积分残差, 顺序：R, v, p
 * information从预积分类中获取，构造函数中计算
 */
class EdgeInertial : public g2o::BaseMultiEdge<9, Matrix<double, 9, 1>> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * 构造函数中需要指定预积分类对象
     * @param preinteg  预积分对象指针
     * @param gravity   重力矢量
     * @param weight    权重
     */
    EdgeInertial(std::shared_ptr<IMUPreintegration> preinteg, const Vector3d& gravity, double weight = 1.0);

    bool read(std::istream& is) override { return false; }
    bool write(std::ostream& os) const override { return false; }

    void computeError() override;
    void linearizeOplus() override;

    Eigen::Matrix<double, 24, 24> GetHessian() {
        linearizeOplus();
        Eigen::Matrix<double, 9, 24> J;
        J.block<9, 6>(0, 0) = _jacobianOplus[0];
        J.block<9, 3>(0, 6) = _jacobianOplus[1];
        J.block<9, 3>(0, 9) = _jacobianOplus[2];
        J.block<9, 3>(0, 12) = _jacobianOplus[3];
        J.block<9, 6>(0, 15) = _jacobianOplus[4];
        J.block<9, 3>(0, 21) = _jacobianOplus[5];
        return J.transpose() * information() * J;
    }

   private:
    const double dt_;
    std::shared_ptr<IMUPreintegration> preint_ = nullptr;
    Vector3d grav_;
};