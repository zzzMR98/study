//
// Created by zzh on 19-3-15.
//
#include <ros/ros.h>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include <transform/transform.h>
#include <ceres/ceres.h>
#include <glog/logging.h>

class RotationCostFunctor {
public:
    explicit RotationCostFunctor(const std::pair<::ivcommon::transform::Rigid3d, ivcommon::transform::Rigid3d>& LI_relativepose, bool degradation_move = false)
    {
        LI_relativepose_ = LI_relativepose;
        degradation_move_ = degradation_move;
    }
    ~RotationCostFunctor() = default;

    template <typename T>
    bool operator()(const T* pose_L2I , T* residual) const {
        Eigen::Map<Eigen::Matrix<T, 3, 1>> mutable_residual(residual);
        const Eigen::Quaternion<T> temp_pose_L2I(pose_L2I[3], pose_L2I[0], pose_L2I[1], pose_L2I[2]);

        if(degradation_move_)
        {
            Eigen::Matrix<T, 3, 1> relative_trans_L = LI_relativepose_.first.translation().cast<T>();
            Eigen::Matrix<T, 3, 1> relative_trans_I = LI_relativepose_.second.translation().cast<T>();
            mutable_residual = temp_pose_L2I * relative_trans_L - relative_trans_I;
        } else
        {
            const Eigen::Quaternion<T> temp_relativepose_L = LI_relativepose_.first.rotation().cast<T>();
            const Eigen::Quaternion<T> temp_relativepose_I = LI_relativepose_.second.rotation().cast<T>();
            mutable_residual = (temp_relativepose_I * temp_pose_L2I * ( temp_pose_L2I * temp_relativepose_L).inverse()).vec();
        }


        return true;
    }
private:
    std::pair<::ivcommon::transform::Rigid3d, ivcommon::transform::Rigid3d> LI_relativepose_;
    bool degradation_move_;

};

class TranslationCostFunctor {
public:
    explicit TranslationCostFunctor(const std::pair<::ivcommon::transform::Rigid3d, ivcommon::transform::Rigid3d>& LI_relativepose, const Eigen::Quaterniond& quat_L2I, bool degradation_move = false)
    {
        LI_relativepose_ = LI_relativepose;
        quat_L2I_ = quat_L2I;
        degradation_move_ = degradation_move;
    }

    template <typename T>
    bool operator()(const T* trans_L2I, T* residual) const
    {
        Eigen::Map<Eigen::Matrix<T, 3, 1>> mutable_residual(residual);
        const Eigen::Quaternion<T> temp_relativepose_I = LI_relativepose_.second.rotation().cast<T>();//Rm
        const Eigen::Matrix<T, 3, 1> temp_relativetrans_I = LI_relativepose_.second.translation().cast<T>();//tm
        const Eigen::Matrix<T, 3, 1> temp_relativetrans_L = LI_relativepose_.first.translation().cast<T>();//tn

        if(degradation_move_)
        {
            const Eigen::Matrix<T, 3, 1> trans_unique(trans_L2I[0], trans_L2I[1], (T)0);
            mutable_residual = (temp_relativepose_I.toRotationMatrix() - Eigen::Matrix<T, 3, 3>::Identity()) * (quat_L2I_.cast<T>() * trans_unique)
                               - quat_L2I_.cast<T>() * temp_relativetrans_L + temp_relativetrans_I;
        } else
        {
            const Eigen::Matrix<T, 3, 1> temp_trans_L2I(trans_L2I[0], trans_L2I[1], trans_L2I[2]);
            mutable_residual = temp_relativepose_I * temp_trans_L2I + temp_relativetrans_I - quat_L2I_.cast<T>() * temp_relativetrans_L - temp_trans_L2I;
        }

        return true;
    }

private:
    std::pair<::ivcommon::transform::Rigid3d, ivcommon::transform::Rigid3d> LI_relativepose_;
    Eigen::Quaterniond quat_L2I_;
    bool degradation_move_;
};

class Calib_Node
{
public:
    Calib_Node()
    {
        std::string tempstr, tempdir;
        ros::param::get("~load_dir", tempdir);
        std::stringstream sstr(tempdir);
        while(sstr >> tempstr)
            load_dirs_.push_back(tempstr);

        ros::param::get("~save_dir", save_dir_);
        quat_L2I_ = Eigen::Quaterniond::Identity();
        trans_L2I_ = Eigen::Vector3d::Identity();
        degradation_move_ = false;
        ros::param::get("~pos_x", trans_L2I_[0]);
        ros::param::get("~pos_y", trans_L2I_[1]);
        ros::param::get("~pos_z", trans_L2I_[2]);
        ros::param::get("~degradation_move", degradation_move_);
        s_degration_ = trans_L2I_[2];


        loaddata();
    }
    ~Calib_Node() = default;

    void loaddata();
    void filterdata();
    void hand_eye_calib();
    void save();

private:
    std::vector<std::pair<::ivcommon::transform::Rigid3d, ivcommon::transform::Rigid3d>> LI_data_;
    std::vector<std::pair<::ivcommon::transform::Rigid3d, ivcommon::transform::Rigid3d>> LI_relativepose_;
    std::vector<std::pair<::ivcommon::transform::Rigid3d, ivcommon::transform::Rigid3d>> LI_relativepose__without_rotation_;
    Eigen::Quaterniond quat_L2I_;
    Eigen::Vector3d trans_L2I_;
    double s_degration_;
    std::string save_dir_;
    std::vector<std::string> load_dirs_;
    bool degradation_move_;

};

/**
 * data arrangement: timestamp slam_position slam_pose ins_position ins_pose slam_time - ins_time
 */
void Calib_Node::loaddata()
{
    for(const auto& source : load_dirs_ )
    {
        std::string filename = source + "/trajectory.txt";
        std::ifstream file(filename.c_str());
        Eigen::Vector3d temp_transL, temp_eulerL, temp_transI, temp_eulerI; //yaw pitch roll in degree
        double starttime = 0, timediff = 0;
        while(file >> starttime)
        {
            file >> temp_transL(0) >> temp_transL(1) >> temp_transL(2)
                 >> temp_eulerL(0) >> temp_eulerL(1) >> temp_eulerL(2)
                 >> temp_transI(0) >> temp_transI(1) >> temp_transI(2)
                 >> temp_eulerI(0) >> temp_eulerI(1) >> temp_eulerI(2)
                 >> timediff;

            LI_data_.emplace_back(std::make_pair(ivcommon::transform::Rigid3d (temp_transL, ivcommon::transform::RollPitchYaw(temp_eulerL(2)*M_PI/180, temp_eulerL(1)*M_PI/180, temp_eulerL(0)*M_PI/180)),
                                                 ivcommon::transform::Rigid3d (temp_transI, ivcommon::transform::RollPitchYaw(temp_eulerI(2)*M_PI/180, temp_eulerI(1)*M_PI/180, temp_eulerI(0)*M_PI/180))));
        }
        file.close();
    }
    LOG(INFO) << "load data done, " << LI_data_.size() << " pairs in total ";
}

void Calib_Node::filterdata()
{
    auto last_LIdata = LI_data_[0];
    for(const auto& LIdata : LI_data_ )
    {
        auto temp_pose_I = LIdata.second.inverse() * last_LIdata.second;
        auto temp_pose_L = LIdata.first.inverse() * last_LIdata.first;
        double rot_diff_INS = fabs(last_LIdata.second.rotation().angularDistance(LIdata.second.rotation()));
        if(degradation_move_)
        {
            if(rot_diff_INS < 1*M_PI/180 && temp_pose_I.translation().norm() > 2)// no rotation, only translation
            {
                LI_relativepose__without_rotation_.emplace_back(std::make_pair(temp_pose_L, temp_pose_I));
                last_LIdata = LIdata;
            }
            else if (rot_diff_INS > 8*M_PI/180 && temp_pose_I.translation().norm() > 1)// rotation and translation
            {
                LI_relativepose_.emplace_back(std::make_pair(temp_pose_L, temp_pose_I));
                last_LIdata = LIdata;
            }
            else if(rot_diff_INS > 30*M_PI/180)// only rotation
                last_LIdata = LIdata;
        } else
        {
            if( temp_pose_I.translation().norm() > 1 && rot_diff_INS > 5*M_PI/180 )//rotation and translation
            {
                LI_relativepose_.emplace_back(std::make_pair(temp_pose_L, temp_pose_I));
                last_LIdata = LIdata;
                LOG(INFO) << "translation : " << temp_pose_I.translation().norm() << " , and rotation : " << rot_diff_INS*180/M_PI;
            }
            else if (temp_pose_I.translation().norm() > 6 || rot_diff_INS > 30*M_PI/180)// only rotation or only translation
            {
                last_LIdata = LIdata;
            }
        }

    }

    if(degradation_move_)
        LOG(INFO) << "filter data done, " << LI_relativepose__without_rotation_.size() <<" pairs without rotation are selected. ";

    LOG(INFO) << "filter data done, " << LI_relativepose_.size() <<" pairs are selected. ";
}

void Calib_Node::hand_eye_calib() {
    //first optimize pose
    ceres::Problem problem_rot;
    ceres::Solver::Options options;
    options.use_nonmonotonic_steps = false;
    options.max_num_iterations = 50;
    options.linear_solver_type = ceres::DENSE_QR;
    options.num_threads = 1;
    ceres::LocalParameterization *local_parameterization_current = new ceres::EigenQuaternionParameterization();
    problem_rot.AddParameterBlock(quat_L2I_.coeffs().data(), 4, local_parameterization_current);


    for (const auto &relativepose : degradation_move_ ? LI_relativepose__without_rotation_ : LI_relativepose_)
    {
        problem_rot.AddResidualBlock(
                new ceres::AutoDiffCostFunction<RotationCostFunctor, 3, 4>(
                        new RotationCostFunctor(relativepose, degradation_move_))
                , nullptr, quat_L2I_.coeffs().data());
    }

    ceres::Solver::Summary summary_rot;
    ceres::Solve(options, &problem_rot,&summary_rot);

    ceres::Problem problem_trans;

    for(const auto &relativepose : LI_relativepose_)
    {
        problem_trans.AddResidualBlock(
                new ceres::AutoDiffCostFunction<TranslationCostFunctor, 3 ,3>(
                        new TranslationCostFunctor(relativepose, quat_L2I_, degradation_move_))
                , nullptr, trans_L2I_.data());
    }

    ceres::Solver::Summary summary_trans;
    ceres::Solve(options, &problem_trans,&summary_trans);
    if(degradation_move_)
    {
        trans_L2I_[3] = s_degration_;
        trans_L2I_ = quat_L2I_ * trans_L2I_;
    }

}

void Calib_Node::save() {
    std::string filename = save_dir_ + "/calib_param.txt";
    std::ofstream file(filename);

    auto pose_calibparam_d = ivcommon::transform::toRollPitchYaw(quat_L2I_)*180*M_1_PI;
    file<< "this file is auto-generated by INS_LIDAR program\n"
        << "<param name=\"pos_x\" value=\"" << trans_L2I_[0] << "\"/>" << std::endl
        << "<param name=\"pos_y\" value=\"" << trans_L2I_[1] << "\"/>" << std::endl
        << "<param name=\"pos_z\" value=\"" << trans_L2I_[2] << "\"/>" << std::endl
        << "<param name=\"roll\" value=\"" << pose_calibparam_d[0] << "\"/>" << std::endl
        << "<param name=\"pitch\" value=\"" << pose_calibparam_d[1] << "\"/>" << std::endl
        << "<param name=\"yaw\" value=\"" << pose_calibparam_d[2] << "\"/>" << std::endl;
    file.close();

    filename = save_dir_ + "/calib_data.txt";
    file.open(filename);
    Eigen::Vector3d INS_trans_calibrated;
    ivcommon::transform::Rigid3d pose_L2I(trans_L2I_, quat_L2I_);
    for(const auto& LIdata : LI_data_)
    {
        INS_trans_calibrated = (LIdata.second * pose_L2I).translation();
        file << std::fixed << std::setprecision(3)
             << LIdata.first.translation()[0] << " " << LIdata.first.translation()[1] << " " << LIdata.first.translation()[2] << " "
             << LIdata.second.translation()[0] << " " << LIdata.second.translation()[1] << " " << LIdata.second.translation()[2] << " "
             << INS_trans_calibrated[0] << " " << INS_trans_calibrated[1] << " " << INS_trans_calibrated[2] << std::endl;
    }

    file.close();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "INS_LIDAR_calibration");
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true; //设置输出到屏幕的日志显示相应颜色
    google::InstallFailureSignalHandler();

    Calib_Node node;
    node.filterdata();
    node.hand_eye_calib();
    node.save();

    return 0;
}

