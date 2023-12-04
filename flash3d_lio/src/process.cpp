#include "process.h"

void Lidar_preprocess::process(const livox_interfaces::msg::CustomMsg::UniquePtr &msg,PointCloudXYZI::Ptr &pcl_out)
{
    avia_handler(msg);
    *pcl_out=pl_surf;
}

void Lidar_preprocess::avia_handler(const livox_interfaces::msg::CustomMsg::UniquePtr &msg)
{
    pl_surf.clear();
    pl_tmp.clear();

    int pl_size= msg->point_num;

    pl_surf.reserve(pl_size);
    pl_tmp.resize(pl_size);

    int valid_num=0;

    for(unsigned i=1;i<pl_size;i++)
    {
        if( msg->points[i].line<N_SCANS && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))//一次回波或者2次回波的点
        {
            valid_num++;
            if(valid_num%point_filter_num==0)
            {
                pl_tmp[i].x=msg->points[i].x;
                pl_tmp[i].y=msg->points[i].y;
                pl_tmp[i].z=msg->points[i].z;
                pl_tmp[i].intensity=msg->points[i].reflectivity;
                pl_tmp[i].curvature=msg->points[i].offset_time/(float)1000000;  //unit：ms
            
                if(((abs(pl_tmp[i].x - pl_tmp[i-1].x) > 1e-7) 
                    || (abs(pl_tmp[i].y - pl_tmp[i-1].y) > 1e-7)
                    || (abs(pl_tmp[i].z - pl_tmp[i-1].z) > 1e-7))
                    && (pl_tmp[i].x * pl_tmp[i].x + pl_tmp[i].y * pl_tmp[i].y + pl_tmp[i].z * pl_tmp[i].z > (blind * blind)))
                    {
                        pl_surf.push_back(pl_tmp[i]);//只取有效间隔的点 且在blid之外的点  全部作为平面点
                    }

            }
        }
    }
}


void Imu_process::process(const MeasureGroup&meas,esekfom::esekf<state_ikfom,12,input_ikfom>&kf_state,PointCloudXYZI::Ptr cur_pcl_un_)
{

    if(meas.imu.empty()) return;
    if(imu_need_init)
    {
        Imu_init(meas,kf_state,init_iter_num);

        last_imu = meas.imu.back();

        state_ikfom imu_state = kf_state.get_x();
        if(init_iter_num>MinInitIMUNum)
        {
            imu_need_init=false;

            printf("IMU Initial Done!\n");
        }
        return;
    }

    UndistortPcl(meas,kf_state,*cur_pcl_un_);
}



void Imu_process::Reset() 
{
  // ROS_WARN("Reset ImuProcess");
  mean_acc      = V3D(0, 0, -1.0);
  mean_gyr      = V3D(0, 0, 0);
  angvel_last       = Zero3d;
  imu_need_init    = true;
  init_iter_num     = 1;
  IMUpose.clear();//imu位姿清空
  last_imu.reset(new sensor_msgs::msg::Imu());
}

void Imu_process::Imu_init(const MeasureGroup&meas,esekfom::esekf<state_ikfom,12,input_ikfom>&kf_state,int &N)
{
    /** 1. initializing the gravity, gyro bias, acc and gyro covariance
     ** 2. normalize the acceleration measurenments to unit gravity **/
    V3D cur_acc,cur_gyr;

    if(b_first_frame)
    {
        Reset();
        N=1;
        b_first_frame = false;
        const auto &imu_acc = meas.imu.front()->linear_acceleration;//从测量中获取imu初始时的加速度
        const auto &gyr_acc = meas.imu.front()->angular_velocity;////从测量中获取imu初始时的角速度
        mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
        mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
    }
    for(const auto &imu:meas.imu)
    {
        const auto &imu_acc = imu->linear_acceleration;
        const auto &gyr_acc = imu->angular_velocity;
        cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
        cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

         //第N次的平均值迭代 acc用于初始化重力 gyr用于初始化零偏bg  ba和重力耦合难以初始化
        mean_acc      += (cur_acc - mean_acc) / N;
        mean_gyr      += (cur_gyr - mean_gyr) / N;

        N++;
    }

    state_ikfom init_state = kf_state.get_x();//状态参数X_
    init_state.grav = S2(- mean_acc / mean_acc.norm() * G_m_s2);//平均测量的单位方向向量 * 重力加速度预设值
    init_state.bg  = mean_gyr;
    init_state.offset_T_L_I = lidar2imu_T;//外参
    init_state.offset_R_L_I = lidar2imu_R;
    kf_state.change_x(init_state);//初始化状态传入

    esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();//对应的协方差矩阵P_  23*23 这里重力用的2维
    init_P.setIdentity();//先置为单位阵
    init_P(6,6) = init_P(7,7) = init_P(8,8) = 0.00001;//外参R协方差置为0.00001
    init_P(9,9) = init_P(10,10) = init_P(11,11) = 0.00001;//外参T协方差置为0.00001
    init_P(15,15) = init_P(16,16) = init_P(17,17) = 0.0001;//bg
    init_P(18,18) = init_P(19,19) = init_P(20,20) = 0.001;//ba
    init_P(21,21) = init_P(22,22) = 0.00001; //重力
    kf_state.change_P(init_P);//初始化协方差传入P_
    last_imu = meas.imu.back();//将imu最后一帧保存起来

}


void Imu_process::UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_out)
{
    /*** add the imu of the last frame-tail to the of current frame-head ***/
    auto tmp_imu=meas.imu;
    tmp_imu.push_front(last_imu);
    const auto& imu_beg_time=get_time_sec(tmp_imu.front()->header.stamp);
    const auto& imu_end_time=get_time_sec(tmp_imu.back()->header.stamp);
    const auto& pcl_beg_time=meas.lidar_beg_time;
    const auto& pcl_end_time=meas.lidar_end_time;

    //sort 
    //printf("the points number of one meas,%d",meas.lidar->size());
    pcl_out=*(meas.lidar);
    sort(pcl_out.points.begin(),pcl_out.points.end(),[](const PointType& a,const PointType b){
        return a.curvature<b.curvature;
    });

    /*** Initialize IMU pose ***/
    state_ikfom imu_state = kf_state.get_x();//上一次的后验估计作为此次的初始状态
    IMUpose.clear();
    IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));//依次为时间，acc，gyr，v,p,R

    //前向传播
    V3D avrg_acc,avrg_gyr;
    double dt;
    input_ikfom in;//输入 w a
    for(auto it_imu=tmp_imu.begin();it_imu<(tmp_imu.end()-1);it_imu++)
    {
        auto&& head=*(it_imu);
        auto&& tail=*(it_imu+1);

        avrg_acc << 0.5*(head->linear_acceleration.x+tail->linear_acceleration.x),
                    0.5*(head->linear_acceleration.y+tail->linear_acceleration.y),
                    0.5*(head->linear_acceleration.z+tail->linear_acceleration.z);
        avrg_gyr << 0.5*(head->angular_velocity.x+tail->angular_velocity.x),
                    0.5*(head->angular_velocity.y+tail->angular_velocity.y),
                    0.5*(head->angular_velocity.z+tail->angular_velocity.z);

        avrg_acc = avrg_acc * G_m_s2 / mean_acc.norm(); // - state_inout.ba; 
        in.acc=avrg_acc;
        in.gyro=avrg_gyr;

        if(get_time_sec(head->header.stamp) < last_lidar_end_time_)
        {
             dt = get_time_sec(tail->header.stamp)- last_lidar_end_time_;
            // dt = tail->header.stamp.toSec() - pcl_beg_time;
        }else{
            dt=get_time_sec(tail->header.stamp)- get_time_sec(head->header.stamp);
        }
        Q.block<3,3>(0,0).diagonal()=cov_gyr;
        Q.block<3,3>(3,3).diagonal()=cov_acc;
        Q.block<3,3>(6,6).diagonal()=cov_bias_gyr;
        Q.block<3,3>(9,9).diagonal()=cov_bias_acc;

        kf_state.predict(dt,Q,in);//预测
        
        /* save the poses at each IMU measurements */
        imu_state=kf_state.get_x();
        angvel_last=avrg_gyr-imu_state.bg;//世界坐标系下的角速度 w
        acc_s_last  = imu_state.rot * (avrg_acc - imu_state.ba); //state ikofm的 rot就是我们通常的body->world
        for(int i=0; i<3; i++)
        {
            acc_s_last[i] += imu_state.grav[i];//世界坐标系下的状态量加速度 a  可见书P52公式
        }
        double &&offs_t =get_time_sec(tail->header.stamp) - pcl_beg_time;
        IMUpose.push_back(set_pose6d(offs_t, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));//依次为时间，acc，gyr，v,p,R

    }

    /*** calculated the pos and attitude prediction at the frame-end ***/ //帧末尾位置和姿态的预测
    double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
    dt = note * (pcl_end_time - imu_end_time);             //最后一帧的imu可能早于点云 也可能晚于点云   （应该是早于点云 在整合的时候保证了）
    kf_state.predict(dt, Q, in);
  
    imu_state = kf_state.get_x();//预测后的状态 
    last_imu = meas.imu.back();//保存最后一帧imu
    last_lidar_end_time_ = pcl_end_time;//更新这一次结束的时间戳

    //后向传播
    /*** undistort each lidar point (backward propagation) ***/  //把所有点都变换到扫描结束时刻的坐标系下
    if(pcl_out.points.begin()==pcl_out.points.end()) return;
    auto it_pcl=pcl_out.points.end()-1;
    //TODO
    M3D R_imu;
    V3D vel_imu,pos_imu,acc_imu,gyr_imu;

    for(auto it_kp=IMUpose.end()-1;it_kp != IMUpose.begin();it_kp--)
    {
        auto head=it_kp-1;
        auto tail=it_kp;
        R_imu<<MAT_FROM_ARRAY(head->rot);
        vel_imu<<VEC_FROM_ARRAY(head->vel);
        pos_imu<<VEC_FROM_ARRAY(head->pos);
        acc_imu<<VEC_FROM_ARRAY(tail->acc);
        gyr_imu<<VEC_FROM_ARRAY(tail->gyr);
        for(;it_pcl->curvature/double(1000)>head->offset_time;it_pcl--)
        {
            dt=it_pcl->curvature/double(1000)-head->offset_time;
            /*
                Pc=Tcb*Pb
                Pc={Tgc}-1*Tgb*Pb
                {Tgc}-1*Tgb=『{Rc}-1*Rb  {Rc}-1*(tb-tc)
                                  0                 1 』
            */
            M3D R_i(R_imu*Exp(gyr_imu,dt));
            V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
            V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);//(tb-tc)
            //去了畸变 原理即是把前一帧 到 后一帧 中间所有点的坐标都归化到最后一帧上 请见上方公式 注意 重新回到了lidar坐标系下
            V3D P_compensate = imu_state.offset_R_L_I.conjugate() * (imu_state.rot.conjugate() * (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) - imu_state.offset_T_L_I);// not accurate!
            
            // save Undistorted points and their rotation
            it_pcl->x = P_compensate(0);
            it_pcl->y = P_compensate(1);
            it_pcl->z = P_compensate(2);

            if (it_pcl == pcl_out.points.begin()) break;
        }
    }

}


void Imu_process::set_param(const V3D&a,const V3D&g,const V3D &tran,const M3D &rot,const V3D &ba,const V3D &bg)
{
    cov_acc_scale=a;
    cov_gyr_scale=g;
    lidar2imu_T=tran;
    lidar2imu_R=rot;
    cov_bias_acc=ba;
    cov_bias_gyr=bg;
}