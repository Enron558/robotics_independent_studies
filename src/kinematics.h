#pragma once

#include <BasicLinearAlgebra.h>
#include "utils.h"

struct KinematicsConfig
{
  // float l1 = 8.0; // length of thigh to foot
  // float l2 = 11.0; // length of hip to thigh
  // float hip = 3.5; // length between hip and starting of thigh
  float l[3];
};

BLA::Matrix<3,3> rotation_matrix(const float theta, const char axis)
{
  if (axis == ('xz' || 'zx')){
    BLA::Matrix<3,3> rotation_matrix = ((cos(theta), 0, -sin(theta)),
                                      (0, 1, 0),
                                      (sin(theta), 0, cos(theta)));
    return rotation_matrix;
  }
  else if (axis == ('zy' || 'yz')){
    BLA::Matrix<3,3> rotation_matrix = ((1, 0, 0),
                                      (0, cos(theta), sin(theta)),
                                      (0, -sin(theta), cos(theta)));
    return rotation_matrix;
  }
  else if (axis == ('xy' || 'yx')){
    BLA::Matrix<3,3> rotation_matrix = ((cos(theta), sin(theta), 0),
                                      (-sin(theta), cos(theta), 0),
                                      (0, 0, 1));
    return rotation_matrix;
  }
  else{
    return 0;
  }
} 

BLA::Matrix<3,3> translation_matrix(const float value, const char axis)
{
  if (axis == 'x'){
    BLA::Matrix<3> T = (value,
                        0,
                        0);
  }
  else if (axis == 'y'){
    BLA::Matrix<3> T = (0,
                        value,
                        0);
  }
  else if (axis == 'z'){
    BLA::Matrix<3> T = (0,
                        0,
                        value);
  }
} 

// TODO: Step 12. Implement forward kinematics
BLA::Matrix<3> forward_kinematics(const BLA::Matrix<3> &joint_angles, const KinematicsConfig &config)
{
  /* Computes forward kinematics for the 3DOF robot arm/leg.
  Returns the cartesian coordinates of the end-effector corresponding to the given joint angles and leg configuration. 
  */

  /* Suggested Implementation
      Refer to Slide 38 in the FK Lecture Slides
      Parameters: Joint angles for each motor, config for the joint offsets
      Create helper functions to perform a rotation and translation together
        Parameters: theta, x, y, z
        Return: 4x4 Matrix for the corresponding translation (homogeneous coordinates)
      Call each transformation helper function together in this FK function, returning the cartesian coordinates in x, y, z
      Return: 3x1 Vector (BLA::Matrix<3,1>) for the x, y, z cartesian coordinates
  */ 
  
  return BLA::Matrix<3>(0, 0, 0);
}

BLA::Matrix<3> inverse_kinematics(const BLA::Matrix<3> &target_location, const KinematicsConfig &config)
{
  return BLA::Matrix<3>(0, 0, 0);
}

enum class BodySide
{
  kLeft,
  kRight,
  kUnspecified
};

BLA::Matrix<3> correct_for_actuator_direction(const BLA::Matrix<3> &joint_vector, const BodySide &side)
{
  return {joint_vector(0), joint_vector(1), joint_vector(2)};
}
