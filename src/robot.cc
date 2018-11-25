/**
 * @file robot.cc
 * @author SAITO Naoya (saito3110.naoya@gmail.com)
 * @brief Extend Choreonoid Body Control
 * @version 0.1
 * @date 2018-11-23
 *
 * @copyright Copyright (c) 2018
 *
 */

#include "../include/robot.h"

#include <iostream>

#include <cnoid/BodyLoader>
#include <cnoid/EigenUtil>
#include <cnoid/Jacobian>

namespace con {

/**
 * @brief load model file to cnoid::Body
 *
 * @param model_file_path: It must correspond to cnoid
 * @return true: successed load model
 * @return false: failed load model
 */
bool Robot::loadModel(const std::string &model_file_path) {
  cnoid::BodyLoader bl;
  if (!bl.load(this, model_file_path)) {
    std::cerr << "[Robot] Failed model load" << std::endl;
    return false;
  }
  return true;
}

/**
 * @brief need to run once this method before run "calcIKforWalking"
 *
 * @param right_foot_base_link_name
 * @param right_foot_sole_link_name
 * @param left_foot_base_link_name
 * @param left_foot_sole_link_name
 */
void Robot::setFootLinks(const std::string &right_foot_base_link_name,
                         const std::string &right_foot_sole_link_name,
                         const std::string &left_foot_base_link_name,
                         const std::string &left_foot_sole_link_name) {
  foot_base_[right] = this->link(right_foot_base_link_name);
  foot_sole_[right] = this->link(right_foot_sole_link_name);
  foot_base_[left] = this->link(left_foot_base_link_name);
  foot_sole_[left] = this->link(left_foot_sole_link_name);
}

/**
 * @brief calc inverse kinematics for walking motion
 *
 * @param sup_leg: support foot. con::right or con::left
 * @param ref_com_pos: reference position of center of mass
 * @param ref_waist_rot: reference rotation of waist
 * @param ref_feet_pos: reference position of feet[con::right, con::left]
 * @param ref_feet_rot: reference rotation of feet[con::right, con::left]
 * @return true: successed inverse kinematics
 * @return false: failed inverse kinematics
 */
bool Robot::calcIKforWalking(const rl &sup_leg, const Vector3 &ref_com_pos,
                             const Matrix3 &ref_waist_rot,
                             const Vector3 ref_feet_pos[2],
                             const Matrix3 ref_feet_rot[2]) {
  if (sup_leg == both or sup_leg == null) {
    return false;
  }
  const rl sw_leg = reverse_foot(sup_leg);

  // set variables
  constexpr int MAX_IK_ITERATION = 50;
  constexpr double LAMBDA = 0.9;
  cnoid::JointPathPtr legs =
      getCustomJointPath(this, foot_sole_[sup_leg], foot_sole_[sw_leg]);
  cnoid::JointPathPtr sup_leg_path =
      getCustomJointPath(this, foot_sole_[sup_leg], this->rootLink());

  const int n = legs->numJoints();
  std::vector<double> qorg(n);
  for (int i = 0; i < n; ++i) {
    qorg[i] = legs->joint(i)->q();
  }

  // set val conf
  dmatrix J_com(3, numJoints());
  dmatrix J_sup(6, sup_leg_path->numJoints());
  dmatrix J_leg(6, n);
  dmatrix J(12, n);
  dmatrix JJ;
  Eigen::ColPivHouseholderQR<dmatrix> QR;
  double damping_constant_sqr = 1e-12;

  dvector v(12);
  dvector dq(n);

  double max_IK_error_sqr = 1.0e-16;
  double errsqr = max_IK_error_sqr * 100.0;
  bool converged = false;

  for (int k = 0; k < MAX_IK_ITERATION; k++) {
    // calc velocity
    Vector3 cur_com(calcCenterOfMass());
    Vector3 dp_com(ref_com_pos - cur_com);
    Vector3 omega_waist(rootLink()->R() *
                        cnoid::omegaFromRot(Matrix3(
                            rootLink()->R().transpose() * ref_waist_rot)));
    Vector3 dp_swg(ref_feet_pos[sw_leg] - legs->endLink()->p());
    Vector3 omega_swg(
        legs->endLink()->R() *
        cnoid::omegaFromRot(
            Matrix3(legs->endLink()->R().transpose() * ref_feet_rot[sw_leg])));

    v.head<3>() = dp_com;
    v.segment<3>(3) = omega_waist;
    v.segment<3>(6) = dp_swg;
    v.segment<3>(9) = omega_swg;

    errsqr = v.squaredNorm();
    if (errsqr < max_IK_error_sqr) {
      converged = true;
      break;
    } else if (MAX_IK_ITERATION == 1) {
      converged = true;
    }

    ///// calc jacobians
    cnoid::Link *base = legs->baseLink();
    calcCMJacobian(this, base, J_com);  // calc com jacobian
    sup_leg_path->calcJacobian(J_sup);  // calc sup jacobian
    legs->calcJacobian(J_leg);          // calc legs jacobian
    J = dmatrix::Zero(12, n);           // calc all Jacobian
    // set com jacobian
    for (int i = 0; i < n; i++) {
      int id = legs->joint(i)->jointId();
      for (int j = 0; j < 3; j++) {
        J(j, i) = J_com(j, id);
      }
    }
    // set sup jacobian
    for (int i = 0, m = sup_leg_path->numJoints(); i < m; i++) {
      for (int j = 3; j < 6; j++) {
        J(j, i) = J_sup(j, i);
      }
    }
    // set legs jacobian
    for (int i = 0; i < n; i++) {
      for (int j = 0; j < 6; j++) {
        J(j + 6, i) = J_leg(j, i);
      }
    }

    // solve quation
    JJ = J * J.transpose() +
         damping_constant_sqr * dmatrix::Identity(J.rows(), J.rows());
    dq = J.transpose() * QR.compute(JJ).solve(v);
    for (int i = 0; i < n; i++) {
      legs->joint(i)->q() += LAMBDA * dq(i);
    }
    legs->calcForwardKinematics();
    calcForwardKinematics();
  }

  if (!converged) {
    for (int i = 0; i < n; ++i) {
      legs->joint(i)->q() = qorg[i];
    }
    legs->calcForwardKinematics();
    calcForwardKinematics();
  }

  return converged;
}

/**
 * @brief update body model based waist pose
 *
 * @param waist_pos
 * @param waist_rot
 */
void Robot::update(const Vector3 &waist_pos, const Matrix3 &waist_rot) {
  rootLink()->p() = waist_pos;
  rootLink()->R() = waist_rot;
  calcForwardKinematics();
}

/**
 * @brief update body model based support foot pose
 *
 * @param sup_leg: con::right or con::left
 * @param sup_sole_pos
 * @param sup_sole_rot
 */
void Robot::update(const rl &sup_leg, const Vector3 &sup_sole_pos,
                   const Matrix3 &sup_sole_rot) {
  if (sup_leg == both or sup_leg == null) {
    std::cerr << "[Robot] Robot Can't update.\n"
              << "You have to specify right or left to sup_leg.\n"
              << "specified sup_leg: " << sup_leg << std::endl;
    return;
  }

  foot_sole_[sup_leg]->p() = sup_sole_pos;
  foot_sole_[sup_leg]->R() = sup_sole_rot;

  cnoid::JointPathPtr legs = getCustomJointPath(
      this, foot_sole_[sup_leg], foot_sole_[reverse_foot(sup_leg)]);
  legs->calcForwardKinematics();
  calcForwardKinematics();
}

/**
 * @brief update body model by joint angles
 *
 * @param angles: All joint angles. angles[i] and joint id need to be same.
 */
void Robot::update(const std::vector<double> angles) {
  if (static_cast<int>(angles.size()) != this->numJoints()) {
    std::cerr << "[Robot] angles size and robot dof are not same.\n"
              << "angles.size: " << angles.size()
              << "robot dof: " << this->numJoints() << std::endl;
  }
  for (size_t i = 0, n = angles.size(); i < n; ++i) {
    this->joint(i)->q() = angles[i];
  }
  calcForwardKinematics();
}

/**
 * @brief calc joint angle along trajectory
 *
 * @param base_link_name
 * @param tip_link_name
 * @param tip_pos_trajetory
 * @param tip_rot_trajetory
 * @param calced_angles: joint angles[trajectory.size][number of joints]
 * @return true: successed
 * @return false: failed
 */
bool Robot::calcTrajetory(const std::string &base_link_name,
                          const std::string &tip_link_name,
                          const std::vector<Vector3> &tip_pos_trajetory,
                          const std::vector<Matrix3> &tip_rot_trajetory,
                          std::vector<std::vector<double>> *calced_angles) {
  // get specified joint path
  cnoid::JointPathPtr joint_path = getCustomJointPath(
      this, this->link(base_link_name), this->link(tip_link_name));
  if (joint_path->empty()) {
    std::cerr << "[Robot] Failed get joint path" << std::endl;
    return false;
  }

  // error handling of trajectory size
  if (tip_pos_trajetory.size() != tip_rot_trajetory.size()) {
    std::cerr
        << "[Robot] not match reference position length and rotation length¥n"
        << "tip_pos_trajectory length: " << tip_pos_trajetory.size() << "¥n"
        << "tip_rot_trajectory length: " << tip_rot_trajetory.size()
        << std::endl;
    return false;
  }

  // calc joint angles along trajectory
  const int n = joint_path->numJoints();
  for (size_t i = 0, m = tip_pos_trajetory.size(); i < m; ++i) {
    if (!joint_path->calcInverseKinematics(tip_pos_trajetory[i],
                                           tip_rot_trajetory[i])) {
      std::cerr << "[Robot] Failed inverse kinematics¥n"
                << "roop: " << i << "¥n"
                << "tip_pos: ¥n" << tip_pos_trajetory[i] << "¥n"
                << "tip_rot: ¥n" << tip_rot_trajetory[i] << std::endl;
      return false;
    }
    std::vector<double> angles(n);
    for (int j = 0; j < n; ++i) {
      angles[j] = joint_path->joint(j)->q();
    }
    calced_angles->push_back(angles);
  }

  return true;
}

}  // namespace con
