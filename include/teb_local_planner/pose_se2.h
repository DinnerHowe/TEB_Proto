#pragma once

#include <g2o/stuff/misc.h>

#include <Eigen/Core>
#include <teb_local_planner/misc.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>

namespace teb_local_planner {
class PoseSE2 {
public:
  PoseSE2() {setZero();}

  PoseSE2(const Eigen::Ref<const Eigen::Vector2d>& position, double theta) {
      position_ = position;
      theta_ = theta;
  }

  PoseSE2(double x, double y, double theta) {
      position_.coeffRef(0) = x;
      position_.coeffRef(1) = y;
      theta_ = theta;
  }

  PoseSE2(const geometry_msgs::Pose& pose) {
      position_.coeffRef(0) = pose.position.x;
      position_.coeffRef(1) = pose.position.y;
      theta_ = tf::getYaw( pose.orientation );
  }

  PoseSE2(const tf::Pose& pose) {
      position_.coeffRef(0) = pose.getOrigin().getX();
      position_.coeffRef(1) = pose.getOrigin().getY();
      theta_ = tf::getYaw( pose.getRotation() );
  }

  PoseSE2(const PoseSE2& pose) {
      position_ = pose.position_;
      theta_ = pose.theta_;
  }

  ~PoseSE2() {}

  Eigen::Vector2d& position() {return position_;}

  const Eigen::Vector2d& position() const {return position_;}

  double& x() {return position_.coeffRef(0);}

  const double& x() const {return position_.coeffRef(0);}

  double& y() {return position_.coeffRef(1);}

  const double& y() const {return position_.coeffRef(1);}

  double& theta() {return theta_;}

  const double& theta() const {return theta_;}

  void setZero() {
    position_.setZero();
    theta_ = 0;
  }

  Eigen::Vector2d orientationUnitVec() const {return Eigen::Vector2d(std::cos(theta_), std::sin(theta_));}

  void scale(double factor) {
    position_ *= factor;
    theta_ = g2o::normalize_theta( theta_*factor );
  }

  void plus(const double* pose_as_array) {
    position_.coeffRef(0) += pose_as_array[0];
    position_.coeffRef(1) += pose_as_array[1];
    theta_ = g2o::normalize_theta( theta_ + pose_as_array[2] );
  }

  void averageInPlace(const PoseSE2& pose1, const PoseSE2& pose2) {
    position_ = (pose1.position_ + pose2.position_)/2;
    theta_ = g2o::average_angle(pose1.theta_, pose2.theta_);
  }

  static PoseSE2 average(const PoseSE2& pose1, const PoseSE2& pose2) {
    return PoseSE2((pose1.position_ + pose2.position_)/2 , g2o::average_angle(pose1.theta_, pose2.theta_) );
  }

  void rotateGlobal(double angle, bool adjust_theta=true) {
    double new_x = std::cos(angle) * position_.x() - std::sin(angle) * position_.y();
    double new_y = std::sin(angle) * position_.x() + std::cos(angle) * position_.y();
    position_.x() = new_x;
    position_.y() = new_y;
    if (adjust_theta) {
      theta_ = g2o::normalize_theta(theta_+angle);
    }
  }

  void toPoseMsg(geometry_msgs::Pose& pose) const {
    pose.position.x = position_.x();
    pose.position.y = position_.y();
    pose.position.z = 0;
    pose.orientation = tf::createQuaternionMsgFromYaw(theta_);
  }

  PoseSE2& operator=( const PoseSE2& rhs ) {
    if (&rhs != this) {
      position_ = rhs.position_;
      theta_ = rhs.theta_;
    }
    return *this;
  }

  PoseSE2& operator+=(const PoseSE2& rhs) {
    position_ += rhs.position_;
    theta_ = g2o::normalize_theta(theta_ + rhs.theta_);
    return *this;
  }

  friend PoseSE2 operator+(PoseSE2 lhs, const PoseSE2& rhs) {
    return lhs += rhs;
  }

  PoseSE2& operator-=(const PoseSE2& rhs) {
    position_ -= rhs.position_;
    theta_ = g2o::normalize_theta(theta_ - rhs.theta_);
    return *this;
  }

  friend PoseSE2 operator-(PoseSE2 lhs, const PoseSE2& rhs) {
    return lhs -= rhs;
  }

  friend PoseSE2 operator*(PoseSE2 pose, double scalar) {
    pose.position_ *= scalar;
    pose.theta_ *= scalar;
    return pose;
  }
 
  friend PoseSE2 operator*(double scalar, PoseSE2 pose) {
    pose.position_ *= scalar;
    pose.theta_ *= scalar;
    return pose;
  }

	friend std::ostream& operator<< (std::ostream& stream, const PoseSE2& pose) {
		std::cout << "x: " << pose.position_[0]
              << " y: " << pose.position_[1]
              << " theta: " << pose.theta_;
    return stream;
	}

private:
  Eigen::Vector2d position_;
  double theta_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace teb_local_planner

