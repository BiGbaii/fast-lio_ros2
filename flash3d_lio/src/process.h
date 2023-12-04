#include <rclcpp/rclcpp.hpp>
#include <livox_interfaces/msg/custom_msg.hpp>
#include "pcl_conversions/pcl_conversions.h"
#include <sensor_msgs/msg/imu.hpp>
#include "common_lib.h"
#include "use-ikfom.hpp"

#include <vector>
/*lidar preprocess 
1.livox_ros_driver::CustomMsg to PointCloudXYZI
2.down_sampleing
*/
class Lidar_preprocess
{
public:
    Lidar_preprocess():blind(3),point_filter_num(3),N_SCANS(6){};
    ~Lidar_preprocess()=default;

    void process(const livox_interfaces::msg::CustomMsg::UniquePtr &msg, PointCloudXYZI::Ptr &pcl_out);

    double blind;
    int point_filter_num, N_SCANS;
    PointCloudXYZI pl_surf,pl_tmp;
    //PointCloudXYZI pl_buff[6];//max 线束 此处仅针对类似livox的固态相机,需要根据其线束设置

private:
    void avia_handler(const livox_interfaces::msg::CustomMsg::UniquePtr &msg);

};

/*imu preprocess
1.forward
2.backward
*/
#define MinInitIMUNum (10)

class Imu_process
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW //eigen内存对齐

    Imu_process():b_first_frame(true),imu_need_init(true),init_iter_num(1)
    {
        Q=process_noise_cov();
        cov_acc       = V3D(0.1, 0.1, 0.1);
        cov_gyr       = V3D(0.1, 0.1, 0.1);
        cov_bias_gyr  = V3D(0.0001, 0.0001, 0.0001);
        cov_bias_acc  = V3D(0.0001, 0.0001, 0.0001);
        mean_acc      = V3D(0, 0, -1.0);
        mean_gyr      = V3D(0, 0, 0);
        lidar2imu_T=V3D(0.04165, 0.02326, -0.0284);//ONLY FOR AVIA
        lidar2imu_R=Eye3d;
        last_imu.reset(new sensor_msgs::msg::Imu());
    };
    ~Imu_process()=default;

    void process(const MeasureGroup &meas,esekfom::esekf<state_ikfom,12,input_ikfom> &kf_state,PointCloudXYZI::Ptr cur_pcl_un_);
    //void set_cov_acc(const V3D &a);
    //void set_cov_gyr(const V3D &g);
    //void set_extrinsic(const V3D &tran,const M3D &rot);
    //void set_cov_bias_acc(const V3D &ba);
    //void set_cov_bias_gyr(const V3D &bg);
    void set_param(const V3D&a,const V3D&g,const V3D &tran,const M3D &rot,const V3D &ba,const V3D &bg);

    V3D cov_acc;
    V3D cov_gyr;
    V3D cov_acc_scale;
    V3D cov_gyr_scale;
    V3D cov_bias_acc;
    V3D cov_bias_gyr;
    V3D lidar2imu_T;
    M3D lidar2imu_R;
    Eigen::Matrix<double, 12, 12> Q;
    
private:
    void Imu_init(const MeasureGroup &meas,esekfom::esekf<state_ikfom,12,input_ikfom> &kf_state,int &N);
    void UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_out);
    void Reset();
     
    void UndistortPcl2(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_out);

    bool b_first_frame;
    bool imu_need_init;
    int  init_iter_num;

    V3D mean_acc;
    V3D mean_gyr;
    V3D acc_s_last;
    V3D angvel_last;
    sensor_msgs::msg::Imu::ConstPtr last_imu;//上一次组合 最后一帧imu 保存起来
    vector<Pose6D> IMUpose;


    double last_lidar_end_time_;

};