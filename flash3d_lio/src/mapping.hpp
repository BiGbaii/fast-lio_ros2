#ifndef USE_MAPPING_H
#define USE_MAPPING_H

#include<mutex>
#include<condition_variable>
#include<omp.h>
#include<memory>

#include<rclcpp/rclcpp.hpp>
#include"common_lib.h"
#include"use-ikfom.hpp"
#include"ikd-Tree/ikd_Tree.h"
#include<livox_interfaces/msg/custom_msg.hpp>
#include<nav_msgs/msg/path.hpp>
#include<nav_msgs/msg/odometry.hpp>
#include<geometry_msgs/msg/quaternion.hpp>
#include<geometry_msgs/msg/transform_stamped.hpp>
#include<tf2_ros/transform_broadcaster.h>
#include<pcl/filters/voxel_grid.h>

#include "process.h"

//多线程
mutex mtx_buffer;
condition_variable sig_buffer;

//string root_dir = ROOT_DIR;

//ikd-tree
KD_TREE<PointType> ikdtree;// ikd-tree
/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;//状态 噪声维度 输入
state_ikfom state_point;
vect3 pos_lid;//world系下的lidar坐标

shared_ptr<Imu_process> p_imu2(new Imu_process());//imu前后传播指针
shared_ptr<Lidar_preprocess> lidar_pre2(new Lidar_preprocess());//lidar前后传播指针

PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());//去畸变后特征点
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());//去畸变后降采样 lidar系

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;


void point_body2world(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

BoxPointType localmap_point;//ikd-tree中局部地图包围盒的角点
bool Localmap_Initialized = false;//局部地图是否初始化
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
vector<BoxPointType> cub_needrm; //ikd-tree中，地图需要移除的包围盒序列
int kdtree_delete_counter = 0;
double cube_len = 0;

void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;

    V3D Pos_lid=pos_lid;
    if (!Localmap_Initialized){ //初始化局部地图包围盒角点，以为w系下lidar位置为中心,得到长宽高1000*1000*1000的局部地图
        for (int i = 0; i < 3; i++){
            localmap_point.vertex_min[i] = Pos_lid(i) - cube_len / 2.0;
            localmap_point.vertex_max[i] = Pos_lid(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }

    bool need_move=false;
    double dist_to_map_edge[2][3];
    for(int i=0;i<3;i++)
    {
        dist_to_map_edge[0][i]=fabs(pos_lid(i)-localmap_point.vertex_min[i]);
        dist_to_map_edge[1][i]=fabs(localmap_point.vertex_max[i]-pos_lid(i));
        if(dist_to_map_edge[0][i]<= MOV_THRESHOLD*DET_RANGE || dist_to_map_edge[1][i] <= MOV_THRESHOLD*DET_RANGE) need_move=true;
    }

    if(need_move)
    {
        BoxPointType new_localmap_point,tmp_localmap_point;
        new_localmap_point=localmap_point;
        double mov_dist=DET_RANGE * (MOV_THRESHOLD -1);
        for(int i=0;i<3;i++)
        {
            tmp_localmap_point=localmap_point;
            if(dist_to_map_edge[0][i]<= MOV_THRESHOLD*DET_RANGE)
            {
                new_localmap_point.vertex_max[i] -= mov_dist;
                new_localmap_point.vertex_min[i] -= mov_dist;
                tmp_localmap_point.vertex_min[i] =localmap_point.vertex_max[i]-mov_dist ;
                cub_needrm.push_back(tmp_localmap_point);
            }else if(dist_to_map_edge[0][i]<= MOV_THRESHOLD*DET_RANGE)
            {
                new_localmap_point.vertex_max[i] += mov_dist;
                new_localmap_point.vertex_min[i] += mov_dist;
                tmp_localmap_point.vertex_min[i] =localmap_point.vertex_min[i]+mov_dist ;
                cub_needrm.push_back(tmp_localmap_point);
            }
        }

        PointVector points_history;
        ikdtree.acquire_removed_points(points_history);//返回被剔除的点  对应points_cache_collect()

        if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    }

}

//传感器信息获取 专用于livox
deque<double>                     time_buffer;//lidar 时间戳缓存队列
deque<PointCloudXYZI::Ptr>        lidar_buffer;//特征提取或间隔采样后的lidar
deque<sensor_msgs::msg::Imu::ConstPtr> imu_buffer;//imu 缓存队列


double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;//时间同步位  表imu和lidar是否已经时间对齐
bool   time_sync_en = false;
// 订阅器sub_pcl的回调函数：接收Livox激光雷达的点云数据
void livox_pcl_cbk(const livox_interfaces::msg::CustomMsg::UniquePtr msg) 
{
    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    double cur_time=get_time_sec(msg->header.stamp);

    if (cur_time < last_timestamp_lidar)//如果当前扫描的激光雷达数据的时间戳比上一次扫描的激光雷达数据的时间戳早，需要将激光雷达数据缓存队列清空
    {
        printf("lidar loop back, clear buffer!\n");
        lidar_buffer.clear();
    }
    last_timestamp_lidar = get_time_sec(msg->header.stamp);
    
    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    lidar_pre2->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);
    
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

// 订阅器sub_imu的回调函数：接收IMU数据，将IMU数据保存到IMU数据缓存队列中
double time_diff_lidar_to_imu = 0.0;
void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in) 
{
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));

    msg->header.stamp = get_ros_time(get_time_sec(msg_in->header.stamp) - time_diff_lidar_to_imu);//外部检校
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = \
        get_ros_time(timediff_lidar_wrt_imu + get_time_sec(msg_in->header.stamp));//偏移检校
    }

    double timestamp = get_time_sec(msg->header.stamp);

    mtx_buffer.lock();

    // 如果当前IMU的时间戳小于上一个时刻IMU的时间戳，则IMU数据有误，将IMU数据缓存队列清空
    if (timestamp < last_timestamp_imu)
    {
        printf("imu loop back, clear buffer!");
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

//将imu与对应的lidar点云放在一起
bool lidar_pushed=false;
double lidar_end_time=0,lidar_mean_scantime = 0;
int scan_num=0;
bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        if (meas.lidar->points.size() <= 1) // time too little   //
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            printf("Too few input point cloud!\n");
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num ++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);//一次lidar scan结束的时间
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = get_time_sec(imu_buffer.front()->header.stamp);
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))//当每一个imu的数据的时间总是早于最后一个点云的时间  （最后一个晚于）
    {
        imu_time = get_time_sec(imu_buffer.front()->header.stamp);
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());//记录此imu
        imu_buffer.pop_front();//pop出 轮到下一个imu
    }

    lidar_buffer.pop_front();//pop出一轮lidar数据
    time_buffer.pop_front();//pop 这一轮lidar数据第一个点的时间记录
    lidar_pushed = false;//重置lidar 到下一轮
    return true;
}

//输出参数
//nav_msgs::msg::Path path; 位姿 有里程计了 默认不发布 
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::Quaternion geoQuat;

//发布里程计
void set_posestamp(geometry_msgs::msg::PoseWithCovariance & out)// 设置输出的t,q，在publish_odometry，publish_path调用
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
    
}
void publish_odometry(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped ,\
                       const std::unique_ptr<tf2_ros::TransformBroadcaster> &tf_br)
{
    odomAftMapped.header.frame_id="camera_init";
    odomAftMapped.child_frame_id="body";
    odomAftMapped.header.stamp=get_ros_time(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    pubOdomAftMapped->publish(odomAftMapped);

    auto P =kf.get_P();
    for(int i= 0;i<6;++i)
    {
        int k=i<3 ? i+3 : i-3;
        odomAftMapped.pose.covariance[i*6+0] =P(k,3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);

    }

    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id="camera_init";
    tf.child_frame_id="body";
    tf.transform.translation.x=odomAftMapped.pose.pose.position.x;
    tf.transform.translation.y=odomAftMapped.pose.pose.position.y;
    tf.transform.translation.z=odomAftMapped.pose.pose.position.z;
    tf.transform.rotation.w=odomAftMapped.pose.pose.orientation.w;
    tf.transform.rotation.x=odomAftMapped.pose.pose.orientation.x;
    tf.transform.rotation.y=odomAftMapped.pose.pose.orientation.y;
    tf.transform.rotation.z=odomAftMapped.pose.pose.orientation.z;

    tf_br->sendTransform(tf);

}


PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());

//发布点云
bool pcd_save_en=false;int pcd_save_interval=-1,pcd_index=0;
bool scan_pub_en=true;
bool dense_pub_en=true;
void publish_save_frame_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull )
{
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort:feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size,1));

        for(int i=0;i<size;i++)
        {
            point_body2world(&laserCloudFullRes->points[i],&laserCloudWorld->points[i]);
        }

        *pcl_wait_save += *laserCloudWorld;

        //publish
        sensor_msgs::msg::PointCloud2 laserCloudMsg;
        pcl::toROSMsg(*laserCloudWorld,laserCloudMsg);
        laserCloudMsg.header.stamp=get_ros_time(lidar_end_time);
        laserCloudMsg.header.frame_id="camera_init";
        pubLaserCloudFull->publish(laserCloudMsg);

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    
}


//观测模型
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());//去畸变后降采样 world系
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));//特征点在地图中对应点的法向量
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));//有效特征点点
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));//对应点法向量
vector<PointVector>  Nearest_Points; //每个点的最邻近序类
float res_last[100000] = {0.0};//残差 点到面距离
bool   point_selected_feat[100000] = {0};//是否为平面特征点
bool   extrinsic_est_en = false;
int    effect_feat_num = 0,feats_down_size = 0;
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
double res_mean_last = 0.05, total_residual = 0.0;//残差平均值 残存总和
void h_share_model(state_ikfom &s,esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    laserCloudOri->clear(); 

    //是否多线程 下面这个for循环
    #ifdef MP_EN  
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif

     /*
        * 有效点的判断
        * 1. 将当前点变换到世界系下
        * 2. 在局部地图上使用kd_tree 临近搜索NUM_MATCH_POINTS个点
        * 3. 判断这些点是否构成平面
        * 4. 判断点离这个平面够不够近(达到阈值)
        * 5. 满足上述条件，设置为有效点。
    */
    for(int i=0;i<feats_down_size;i++)
    {
        PointType &point_body  = feats_down_body->points[i]; 
        PointType &point_world = feats_down_world->points[i]; 

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
        auto &near_point=Nearest_Points[i];


        ikdtree.Nearest_Search(point_world,NUM_MATCH_POINTS,near_point,pointSearchSqDis);
         // 是否搜索到足够的点以及最远的点到当前点的距离足够小(太远，就不认为这俩在一个平面)
        if (near_point.size()<NUM_MATCH_POINTS||pointSearchSqDis[NUM_MATCH_POINTS-1]>5) continue;

        VF(4) pabcd;//平面法向量
        if(esti_plane(pabcd,near_point,0.1f))//判断是否构成平面
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);///计算点到平面的距离
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());//计算点和平面的夹角，夹角越小S越大
            if (s > 0.9)//大于阈值，则认为该点是有效点
            {
                point_selected_feat[i] = true;//置为有效点
                normvec->points[i].x = pabcd(0);//存储其法向量 用于计算H
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                //res_last[i] = abs(pd2);//残差
            }
        }
    }

    effect_feat_num=0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_feat[i])
        {
            laserCloudOri->points[effect_feat_num] = feats_down_body->points[i];//有效点存入
            corr_normvect->points[effect_feat_num] = normvec->points[i];//有效法向量存入
            effect_feat_num ++;
        }
    }

    if (effect_feat_num < 1)
    {
        ekfom_data.valid = false;
        printf("No Effective Points! \n");
        return;
    }

    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effect_feat_num, 12); 
    ekfom_data.h.resize(effect_feat_num);

    for (int i = 0; i < effect_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; //论文中是 R P ```` 
                                                                                                                            //在ikfom库中实现为 P R R(外参) T(外参)````
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }

}

//地图动态更新
bool flg_EKF_inited=false;
double filter_size_map_min = 0, filter_size_surf_min = 0;
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;

    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);

    for(int i=0;i <feats_down_body->size();++i)
    {
        point_body2world(&(feats_down_body->points[i]),&(feats_down_world->points[i]));

        /* decide if need add to map */
        /*  1.新点落在那个体素盒子内
            2.最邻近的点如果在这个体素盒子外，直接插入
            3.最邻近在体素盒子内，判断最邻近和新点谁离体素中心最近
        */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            PointType downsample_result, mid_point; 
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min || fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min || fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false);   

}

bool flg_first_scan = true, flg_exit = false;
double first_lidar_time = 0.0;
int    NUM_MAX_ITERATIONS = 0;
const double  INIT_TIME = 0.1;
const double  LASER_POINT_COV = 0.001;
string lid_topic,imu_topic;

double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
vector<double>       extrinsic_T(3, 0.0);//lidar-imu 外参T
vector<double>       extrinsic_R(9, 0.0);//lidar-imu 外参R
V3D Lidar_T_wrt_IMU(Zero3d);//lidar->imu    T
M3D Lidar_R_wrt_IMU(Eye3d);//lidar->imu     R


class LioNode :public rclcpp::Node
{
public:
    LioNode(std::string name):Node(name)
    {
        RCLCPP_INFO(this->get_logger(),"INIT LioNode!");
        this->declare_parameter<bool>("publish.scan_publish_en",true);
        this->declare_parameter<bool>("publish.dense_publish_en",true);
        this->declare_parameter<int>("max_iteration",3);
        this->declare_parameter<string>("common.lid_topic","/livox/lidar");
        this->declare_parameter<string>("common.imu_topic","/livox/imu");
        this->declare_parameter<bool>("common.time_sync_en",false);
        this->declare_parameter<double>("common.time_offset_lidar_to_imu",0.0);
        this->declare_parameter<double>("filter_size_surf",0.3);
        this->declare_parameter<double>("filter_size_map",0.3);
        this->declare_parameter<double>("cube_side_length",1000);
        this->declare_parameter<float>("mapping.det_range",300.f);

        this->declare_parameter<double>("mapping.gyr_cov",0.1);
        this->declare_parameter<double>("mapping.acc_cov",0.1);
        this->declare_parameter<double>("mapping.b_gyr_cov",0.0001);
        this->declare_parameter<double>("mapping.b_acc_cov",0.0001);

        this->declare_parameter<double>("preprocess.blind",2);
        this->declare_parameter<int>("point_filter_num",3);
        this->declare_parameter<bool>("mapping.extrinsic_est_en",false);
        this->declare_parameter<bool>("pcd_save.pcd_save_en",false);
        this->declare_parameter<int>("pcd_save.interval",-1);
        this->declare_parameter<vector<double>>("mapping.extrinsic_T",vector<double>());
        this->declare_parameter<vector<double>>("mapping.extrinsic_R",vector<double>());

        this->get_parameter_or<bool>("publish.scan_publish_en",scan_pub_en,true);
        this->get_parameter_or<bool>("publish.dense_publish_en",dense_pub_en,true);
        this->get_parameter_or<int>("max_iteration",NUM_MAX_ITERATIONS,3);
        this->get_parameter_or<string>("common.lid_topic",lid_topic,"/livox/lidar");
        this->get_parameter_or<string>("common.imu_topic", imu_topic,"/livox/imu");
        this->get_parameter_or<bool>("common.time_sync_en", time_sync_en, false);
        this->get_parameter_or<double>("common.time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
        this->get_parameter_or<double>("filter_size_surf",filter_size_surf_min,0.3);
        this->get_parameter_or<double>("filter_size_map",filter_size_map_min,0.3);
        this->get_parameter_or<double>("cube_side_length",cube_len,1000);
        this->get_parameter_or<float>("mapping.det_range",DET_RANGE,300.f);
        this->get_parameter_or<double>("mapping.gyr_cov",gyr_cov,0.1);
        this->get_parameter_or<double>("mapping.acc_cov",acc_cov,0.1);
        this->get_parameter_or<double>("mapping.b_gyr_cov",b_gyr_cov,0.0001);
        this->get_parameter_or<double>("mapping.b_acc_cov",b_acc_cov,0.0001);
        this->get_parameter_or<double>("preprocess.blind", lidar_pre2->blind, 2);
        this->get_parameter_or<int>("point_filter_num", lidar_pre2->point_filter_num, 3);
        this->get_parameter_or<bool>("mapping.extrinsic_est_en", extrinsic_est_en, false);
        this->get_parameter_or<bool>("pcd_save.pcd_save_en", pcd_save_en, true);
        this->get_parameter_or<int>("pcd_save.interval", pcd_save_interval, -1);
        this->get_parameter<vector<double>>("mapping.extrinsic_T",extrinsic_T);
        this->get_parameter<vector<double>>("mapping.extrinsic_R",extrinsic_R);

        RCLCPP_INFO(this->get_logger(), "Node init 0.");
        //Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinsic_T);//lidar->imu
        //Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinsic_R);
        RCLCPP_INFO(this->get_logger(), "Node init 2.");
        p_imu2->set_param(V3D(acc_cov, acc_cov, acc_cov),V3D(gyr_cov, gyr_cov, gyr_cov),Lidar_T_wrt_IMU,Lidar_R_wrt_IMU, \
                            V3D(b_acc_cov, b_acc_cov, b_acc_cov),V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
        RCLCPP_INFO(this->get_logger(), "Node init finished1.");
        memset(res_last, -1000.0f, sizeof(res_last));//残差 点到平面距离
        
        RCLCPP_INFO(this->get_logger(), "Node init 3.");
        downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);// VoxelGrid滤波器参数，即进行滤波时的创建的体素边长为filter_size_surf_min

        double epsi[23] = {0.001};
        fill(epsi, epsi+23, 0.001);
        kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

        sub_lidar = this->create_subscription<livox_interfaces::msg::CustomMsg>(lid_topic,20,&livox_pcl_cbk);
        sub_imu   = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic,20,&imu_cbk);
        pub_lidar_with_save = this->create_publisher<sensor_msgs::msg::PointCloud2>("/flash3d_lio_cloud",20);
        pub_odometry = this->create_publisher<nav_msgs::msg::Odometry>("/flash3d_lio_odometry",20);
        tf_br_=std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(10.0));
        timer= rclcpp::create_timer(this, this->get_clock(), period_ms, std::bind(&LioNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Node init finished.");

    }

private:
    rclcpp::Subscription<livox_interfaces::msg::CustomMsg>::SharedPtr sub_lidar;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar_with_save;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_br_;

    rclcpp::TimerBase::SharedPtr timer;

    void timer_callback()
    {
        if(sync_packages(Measures)) // 将激光雷达点云数据和IMU数据从缓存队列中取出，进行时间对齐，并保存到Measures中
        {
            RCLCPP_INFO(this->get_logger(),"points Number of Measures:%ld",Measures.lidar->size());
            RCLCPP_INFO(this->get_logger(),"IMU Number of Measures:%ld",Measures.imu.size());

            if (flg_first_scan)//是第一轮数据吗
            {
                first_lidar_time = Measures.lidar_beg_time;
                //P_imu2->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                return;
            }


            p_imu2->process(Measures, kf, feats_undistort);//imu的前向传播和后向传播
            

            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;//lidar在世界坐标系下位置

            if (feats_undistort->empty() || (feats_undistort == NULL))//如果点云数据为空，则代表了激光雷达没有完成去畸变，此时还不能初始化成功
            {
                RCLCPP_INFO(this->get_logger(),"No point, skip this scan!\n");
                return;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \  
                            false : true;//一次的组合数据世界大于第一次组合数据的时间
            

             /* 动态调整地图大小*/
            lasermap_fov_segment();

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);//降采样后的点云数据
            feats_down_size = feats_down_body->points.size();
            
            /*** initialize the map kdtree ***/
            if(ikdtree.Root_Node == nullptr)
            {
                if(feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);//kdtree的降采样分辨率 应该把他设置的与点的降采样分辨率一致
                    feats_down_world->resize(feats_down_size);
                    for(int i = 0; i < feats_down_size; i++)
                    {
                        point_body2world(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    ikdtree.Build(feats_down_world->points);
                }
                return;
            }
            int featsFromMapNum = ikdtree.validnum();//有效节点
            int kdtree_size_st = ikdtree.size();//全部节点


            if (feats_down_size < 5)
            {
                RCLCPP_INFO(this->get_logger(),"No point, skip this scan!\n");
                return;
            }

            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);

            
            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);//这一更新kf
            state_point = kf.get_x();
            std::cout<<state_point.rot<<" "<<state_point.pos.transpose()<<'\n';
            V3D euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            /*** add the feature points to map kdtree ***/
            map_incremental();
            /******* Publish odometry *******/
            publish_odometry(pub_odometry,tf_br_);

             /******* Publish points *******/
            if (scan_pub_en || pcd_save_en)      publish_save_frame_world(pub_lidar_with_save);
        }

    }
};

#endif






