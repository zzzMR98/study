#include "pose_extrapolator.h"

namespace Time_Align{

void PoseExtrapolator::AddInsVelocityData(const InsVelocityData& insvelocityData)
{
    insvelocity_data_.push_back(insvelocityData);
}

ivcommon::transform::Rigid3d PoseExtrapolator::ExtrapolatePoseAckermann(double last_time, double curren_time)
{
    Eigen::Vector3d translation_vehicle = Eigen::Vector3d::Zero();
    Eigen::Quaterniond rotation_vehicle = Eigen::Quaterniond::Identity();

    if(!insvelocity_data_.empty()) {
        int insvelocitynum = 0;
        Eigen::Vector3d insvelocity_sum = Eigen::Vector3d::Zero();
        Eigen::Vector3d insangularvelocity_sum = Eigen::Vector3d::Zero();
        for (const auto &insvelocity_data : insvelocity_data_) {
            if (insvelocity_data.timestamp >= last_time && insvelocity_data.timestamp <= curren_time) {
                insvelocity_sum += insvelocity_data.linear_velocity;
                insangularvelocity_sum += insvelocity_data.angular_velocity;
                insvelocitynum++;
            } else if (insvelocity_data.timestamp > curren_time)
                break;
        }

        if(insvelocitynum>0)
        {
            double time_diff = curren_time - last_time;
            translation_vehicle = insvelocity_sum / insvelocitynum*time_diff;
            rotation_vehicle = ::ivcommon::transform::AngleAxisVectorToRotationQuaternion<double>(insangularvelocity_sum * time_diff/ insvelocitynum);
        }

    }
    return std::move(ivcommon::transform::Rigid3d(translation_vehicle, rotation_vehicle));
}



}