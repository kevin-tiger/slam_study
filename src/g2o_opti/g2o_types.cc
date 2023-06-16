#include "g2o_opti/g2o_types.h"

EdgePriorPoseNavState::EdgePriorPoseNavState(const NavStated& state, const Matrix<double, 15, 15>& info) {
    resize(4);
    state_ = state;
    setInformation(info);
}

void EdgePriorPoseNavState::computeError() {
    auto* vp = dynamic_cast<const VertexPose*>(_vertices[0]);
    auto* vv = dynamic_cast<const VertexVelocity*>(_vertices[1]);
    auto* vg = dynamic_cast<const VertexGyroBias*>(_vertices[2]);
    auto* va = dynamic_cast<const VertexAccBias*>(_vertices[3]);

    const Vector3d er = Sophus::SO3d(state_.R_.matrix().transpose() * vp->estimate().so3().matrix()).log();
    const Vector3d ep = vp->estimate().translation() - state_.p_;
    const Vector3d ev = vv->estimate() - state_.v_;
    const Vector3d ebg = vg->estimate() - state_.bg_;
    const Vector3d eba = va->estimate() - state_.ba_;

    _error << er, ep, ev, ebg, eba;
}

void EdgePriorPoseNavState::linearizeOplus() {
    const auto* vp = dynamic_cast<const VertexPose*>(_vertices[0]);
    const Vector3d er = Sophus::SO3d(state_.R_.matrix().transpose() * vp->estimate().so3().matrix()).log();

    /// 注意有3个index, 顶点的，自己误差的，顶点内部变量的
    _jacobianOplus[0].setZero();
    _jacobianOplus[0].block<3, 3>(0, 0) = Sophus::SO3d::jr_inv(er);    // dr/dr
    _jacobianOplus[0].block<3, 3>(3, 3) = Matrix3d::Identity();  // dp/dp
    _jacobianOplus[1].setZero();
    _jacobianOplus[1].block<3, 3>(6, 0) = Matrix3d::Identity();  // dv/dv
    _jacobianOplus[2].setZero();
    _jacobianOplus[2].block<3, 3>(9, 0) = Matrix3d::Identity();  // dbg/dbg
    _jacobianOplus[3].setZero();
    _jacobianOplus[3].block<3, 3>(12, 0) = Matrix3d::Identity();  // dba/dba
}