
#include <teb_local_planner/recovery_behaviors.h>
#include <ros/ros.h>
#include <limits>
#include <functional>
#include <numeric>
#include <g2o/stuff/misc.h>

namespace teb_local_planner
{

// ============== FailureDetector Implementation ===================

void FailureDetector::update(const geometry_msgs::Twist& twist, double v_max, double v_backwards_max, double omega_max, double v_eps, double omega_eps)
{
    if (buffer_.capacity() == 0)
        return;
    
    VelMeasurement measurement;
    measurement.v = twist.linear.x; // just consider linear velocity in x-direction in the robot frame for now
    measurement.omega = twist.angular.z;
    
    if (measurement.v > 0 && v_max>0)
        measurement.v /= v_max;
    else if (measurement.v < 0 && v_backwards_max > 0)
        measurement.v /= v_backwards_max;
    
    if (omega_max > 0)
        measurement.omega /= omega_max;
    
    buffer_.push_back(measurement);
    
    // immediately compute new state
    detect(v_eps, omega_eps);
}

void FailureDetector::clear()
{
    buffer_.clear();
    oscillating_ = false;
}

bool FailureDetector::isOscillating() const
{
    return oscillating_;
}

bool FailureDetector::detect(double v_eps, double omega_eps)
{
    oscillating_ = false;
    
    if (buffer_.size() < buffer_.capacity()/2) // we start detecting only as soon as we have the buffer filled at least half
        return false;

    double n = (double)buffer_.size();
            
    // compute mean for v and omega
    double v_mean=0;
    double omega_mean=0;
    int omega_zero_crossings = 0;
    for (int i=0; i < n; ++i)
    {
        v_mean += buffer_[i].v;
        omega_mean += buffer_[i].omega;
        if ( i>0 && g2o::sign(buffer_[i].omega) != g2o::sign(buffer_[i-1].omega) )
            ++omega_zero_crossings;
    }
    v_mean /= n;
    omega_mean /= n;

    if (std::abs(v_mean) < v_eps && std::abs(omega_mean) < omega_eps && omega_zero_crossings>1 ) 
    {
        oscillating_ = true;
    }
//     ROS_INFO_STREAM("v: " << std::abs(v_mean) << ", omega: " << std::abs(omega_mean) << ", zero crossings: " << omega_zero_crossings);
    return oscillating_;
}
    
    

} // namespace teb_local_planner
