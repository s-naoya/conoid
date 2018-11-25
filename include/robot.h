/**
 * @file robot.h
 * @author SAITO Naoya (saito3110.naoya@gmail.com)
 * @brief Extend Choreonoid Body Control
 * @version 0.1
 * @date 2018-11-23
 *
 * @copyright Copyright (c) 2018
 *
 */
#ifndef CONOID_ROBOT_H_
#define CONOID_ROBOT_H_

#include <string>

#include <cnoid/Body>

#include "utility.h"

namespace con {

class Robot : public cnoid::Body {
 public:
  Robot() {}

  // setting
  bool loadModel(const std::string &model_file_path);
  void setFootLinks(const std::string &r_foot_base_link_name,
                    const std::string &r_foot_sole_link_name,
                    const std::string &l_foot_base_link_name,
                    const std::string &l_foot_sole_link_name);

  // kinematics
  bool calcIKforWalking(const rl &sup_leg, const Vector3 &ref_com_pos,
                        const Matrix3 &ref_waist_rot,
                        const Vector3 ref_feet_pos[2],
                        const Matrix3 ref_feet_rot[2]);
  void update(const Vector3 &waist_pos, const Matrix3 &waist_rot);
  void update(const rl &sup_foot, const Vector3 &sup_sole_pos,
              const Matrix3 &sup_sole_rot);

  bool calcTrajetory(const std::string &base_link_name,
                     const std::string &tip_link_name,
                     const std::vector<Vector3> &tip_pos_trajetory,
                     const std::vector<Matrix3> &tip_rot_trajetory,
                     std::vector<std::vector<double>> *calced_angles);

 private:
  cnoid::Link *foot_base_[2], *foot_sole_[2];
};

using RobotPtr = cnoid::ref_ptr<Robot>;

}  // namespace con

#endif  // CONOID_ROBOT_H
