#include <deque>
#include <memory>

#include "transform/rigid_transform.h"
#include "transform/transform.h"
#include <Eigen/Core>

namespace Time_Align{

/**
 * store INSvelocity data from INS
 */
struct InsVelocityData
{
    double timestamp;
    Eigen::Vector3d linear_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
//    double acc_x;///<right
//    double acc_y;///<forward
//    double acc_z;///<up
    InsVelocityData(double time, Eigen::Vector3d linear_vel, Eigen::Vector3d angular_vel)
    {
        timestamp = time;
        linear_velocity = linear_vel;
        angular_velocity = angular_vel;
    }
};

class PoseExtrapolator {
public:
    /*
     * init start time.
     */
    PoseExtrapolator(double time) : oldest_time_(time){}
    ~PoseExtrapolator() = default;

    /**
     * store insvelocity data to deque
     * @param insvelocityData
     */
    void AddInsVelocityData(const InsVelocityData& insvelocityData);

    /**
     * get pose in current time to last time.
     * @param last_time
     * @param curren_time
     * @return
     */
    ::ivcommon::transform::Rigid3d ExtrapolatePoseAckermann(double last_time, double curren_time);

    /**
     * only restore insvelocity data from oldest time in case the deque is too large.
     * @param time
     */
    void reset_oldest_time(double time)
    {
        oldest_time_ = time;
        while (!insvelocity_data_.empty() && insvelocity_data_.front().timestamp < oldest_time_)
            insvelocity_data_.pop_front();
    }

private:
    std::deque<InsVelocityData> insvelocity_data_;
    double oldest_time_;

};


}
