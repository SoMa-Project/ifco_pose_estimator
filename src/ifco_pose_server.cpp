// Copyright 2019 Ocado Innovation Limited

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <vector>
#include <cmath>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include "ifco_pose_estimator/ifco_pose.h"

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class IfcoPoseServer
{
    private:
        
        ros::NodeHandle nh_;
        ros::Publisher pcl_pub_;
        ros::ServiceClient planning_scene_diff_client_;

        PointCloud filtered_msg_, model_ifco_cloud_;

        bool use_initial_pose_estimate_;
        uint8_t A_high_, A_low_;
        unsigned long icp_iterations_;
        double icp_fitness_, ifco_width_, ifco_length_, ifco_height_;
        std::string pcl_topic_;
        Eigen::Matrix4f cam_ifco_tf_;
        Eigen::Affine3d initial_pose_estimate_;

        pcl::PointXYZ create_ifco_model_point(int i_x, int i_y, int i_z, int divisor);
        void create_ifco_model();
        void filter_pcl(const sensor_msgs::PointCloud2 scene_pcl);
        void performICP();
        Eigen::Matrix4f getGuessFromPCL();
        void add_ifco_to_planning_scene(Eigen::Affine3d base_in_cam);
        void addBox(std::string id, std::string frame_id, geometry_msgs::Pose box_pose, double dim_x, double dim_y, double dim_z);
        void add_ifco_side(std::string id, std::string frame_id, Eigen::Affine3d base_in_cam, double pos_x, double pos_y, double pos_z, double dim_x, double dim_y, double dim_z);
        void acquire_ifco_params();
        
    public:
        IfcoPoseServer(ros::NodeHandle& n, unsigned long A_high, unsigned long A_low, unsigned long icp_iterations, std::string pcl_topic, bool use_initial_pose_estimate, Eigen::Affine3d initial_pose_estimate);  
        bool compute_ifco_pose(ifco_pose_estimator::ifco_pose::Request  &req, ifco_pose_estimator::ifco_pose::Response &res);
        
};

IfcoPoseServer::IfcoPoseServer(ros::NodeHandle& n, unsigned long A_high, unsigned long A_low, unsigned long icp_iterations, std::string pcl_topic, bool use_initial_pose_estimate, Eigen::Affine3d initial_pose_estimate):nh_(n), icp_iterations_(icp_iterations), pcl_topic_(pcl_topic), use_initial_pose_estimate_(use_initial_pose_estimate), initial_pose_estimate_(initial_pose_estimate)
{
    if(A_high > 255 || A_low > 255){
        throw std::runtime_error("A_low and A_high must be between 0 and 255");
    }
    if(A_low >= A_high){
        throw std::runtime_error("A_low must be lower than A_high");
    }
    A_high_ = static_cast<uint8_t>(A_high);
    A_low_ = static_cast<uint8_t>(A_low); 
    pcl_pub_ = nh_.advertise<PointCloud> ("model_pcl", 1, true);
}

void IfcoPoseServer::filter_pcl(const sensor_msgs::PointCloud2 scene_pcl_msg){
    PointCloudRGB scene_pcl;
    pcl::fromROSMsg(scene_pcl_msg, scene_pcl);

    if (!scene_pcl.isOrganized()) throw std::runtime_error("This service only works for organized pointclouds."); 
    
    cv::Mat scene_img(scene_pcl.height, scene_pcl.width, CV_8UC3);

    for (int h=0; h<scene_img.rows; h++) {
        for (int w=0; w<scene_img.cols; w++) {
            pcl::PointXYZRGB point = scene_pcl.at(w, h);

            scene_img.at<cv::Vec3b>(h,w)[0] = point.b;
            scene_img.at<cv::Vec3b>(h,w)[1] = point.g;
            scene_img.at<cv::Vec3b>(h,w)[2] = point.r;
        }
    }

    cv::Mat LAB_img(scene_pcl.height, scene_pcl.width, CV_8UC3);
    cv::Mat channelA(scene_pcl.height, scene_pcl.width, CV_8UC1);

    cv::cvtColor(scene_img, LAB_img, cv::COLOR_BGR2Lab);
    
    int from_to[] = { 1,0 };
    cv::mixChannels(&LAB_img, 1, &channelA, 1, from_to, 1);

    cv::inRange(channelA, A_low_, A_high_, channelA);
    
    filtered_msg_.header = scene_pcl.header;
    filtered_msg_.height = 1;
    filtered_msg_.points.clear();

    int k = 0;
    for(int i=0; i<channelA.rows; i++){
        for(int j=0; j<channelA.cols; j++){
            if(channelA.at<uint8_t>(i,j) > 0  && !(std::isnan(scene_pcl[k].x) || std::isnan(scene_pcl[k].y) || std::isnan(scene_pcl[k].z))){
                pcl::PointXYZ point(scene_pcl[k].x, scene_pcl[k].y, scene_pcl[k].z);
                filtered_msg_.points.push_back (point);
            }
            k++;
        }
    }
    filtered_msg_.width = (filtered_msg_.points).size();

    boost::filesystem::create_directory("debug_folder");
    cv::imwrite("debug_folder/ifco_debug.png", channelA);
}

pcl::PointXYZ IfcoPoseServer::create_ifco_model_point(int i_x, int i_y, int i_z, int divisor) {
    double x = static_cast<double>(i_x)/divisor;
    double y = static_cast<double>(i_y)/divisor;
    double z = static_cast<double>(i_z)/divisor;
    return pcl::PointXYZ(ifco_length_*(x-0.5), ifco_width_*(y-0.5), ifco_height_*z);
};

void IfcoPoseServer::create_ifco_model()
{
    model_ifco_cloud_.height = 1;
    model_ifco_cloud_.is_dense = true;
    model_ifco_cloud_.points.clear();
    int model_resolution_divisor = 20;

    // create the long sides of the ifco
    for (int i_x = 0; i_x <= model_resolution_divisor; i_x ++){
        for(int i_z = -model_resolution_divisor; i_z <= 0; i_z ++){
            model_ifco_cloud_.points.push_back(create_ifco_model_point(i_x, 0, i_z, model_resolution_divisor));
            model_ifco_cloud_.points.push_back(create_ifco_model_point(i_x, model_resolution_divisor, i_z, model_resolution_divisor));
        }
    }
    
    // create the short sides of the ifco
    for(int i_y = 1; i_y < model_resolution_divisor; i_y ++){
        for(int i_z = -model_resolution_divisor; i_z <= 0; i_z ++){
            model_ifco_cloud_.points.push_back(create_ifco_model_point(0, i_y, i_z, model_resolution_divisor));
            model_ifco_cloud_.points.push_back(create_ifco_model_point(model_resolution_divisor, i_y, i_z, model_resolution_divisor));
        }
    }
    
    // create the ifco surface
    for (int i_x = 1; i_x < model_resolution_divisor; i_x ++){
        for(int i_y = 1; i_y < model_resolution_divisor; i_y ++){
            model_ifco_cloud_.points.push_back(create_ifco_model_point(i_x, i_y, 0, model_resolution_divisor));
        }   
    }
    model_ifco_cloud_.width = (model_ifco_cloud_.points).size();
    ROS_INFO("The ifco model was created");
}

Eigen::Matrix4f IfcoPoseServer::getGuessFromPCL()
{
    Eigen::Affine3f transform_guess = Eigen::Affine3f::Identity();
    double x = 0, y = 0, z = 0;
    for(int i=0; i < filtered_msg_.width; i++){
        x += filtered_msg_.points[i].x/filtered_msg_.width;
        y += filtered_msg_.points[i].y/filtered_msg_.width;
        z += filtered_msg_.points[i].z/filtered_msg_.width;
    }
    transform_guess.translation() << x, y, z;

    // -45 degrees around x axis (given the initial IFCO model orientation, the IFCO model will have to be eventually rotated by 0 - 90 degrees)
    transform_guess.rotate (Eigen::AngleAxisf (-M_PI/4, Eigen::Vector3f::UnitX()));
    
    return transform_guess.matrix();
}

void IfcoPoseServer::performICP() 
{    
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(model_ifco_cloud_.makeShared());
    icp.setInputTarget(filtered_msg_.makeShared());
    icp.setMaximumIterations (icp_iterations_);
                
    PointCloud aligned_sim_ifco_cloud;
    icp.align(aligned_sim_ifco_cloud);
                
    icp_fitness_ = icp.getFitnessScore();
    ROS_INFO("ICP score: %.10f", icp_fitness_);

    // Obtain the transformation that aligned cloud_source to cloud_source_registered
    Eigen::Matrix4f transformation = icp.getFinalTransformation();
    pcl::transformPointCloud (model_ifco_cloud_, model_ifco_cloud_, transformation);
    cam_ifco_tf_ = transformation * cam_ifco_tf_;

    model_ifco_cloud_.header = filtered_msg_.header;
}

void IfcoPoseServer::addBox(std::string id, std::string frame_id, geometry_msgs::Pose box_pose, double dim_x, double dim_y, double dim_z) 
{
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;

    /* The id of the object is used to identify it. */
    collision_object.id = id;

    /* Define a box to add to the world. */
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = dim_x;
    primitive.dimensions[1] = dim_y;
    primitive.dimensions[2] = dim_z;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    
    ROS_INFO("Adding %s into the world.", id.c_str());

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(collision_object);
    planning_scene.is_diff = true;

    moveit_msgs::ApplyPlanningScene srv;
    srv.request.scene = planning_scene;
    planning_scene_diff_client_.call(srv);
}

void IfcoPoseServer::add_ifco_side(std::string id, std::string frame_id, Eigen::Affine3d base_in_cam, double pos_x, double pos_y, double pos_z, double dim_x, double dim_y, double dim_z)
{
    geometry_msgs::Pose box_pose;
    Eigen::Affine3d side_in_cam;
    Eigen::Vector4d side_in_base, side_in_cam_pos;
    side_in_cam.linear() = base_in_cam.linear();
    side_in_base << pos_x, pos_y, pos_z, 1;
    side_in_cam_pos = base_in_cam.matrix() * side_in_base;
    side_in_cam.translation() = side_in_cam_pos.head(3);
    tf::poseEigenToMsg(side_in_cam, box_pose);
    addBox(id, frame_id, box_pose, dim_x, dim_y, dim_z);
}

void IfcoPoseServer::add_ifco_to_planning_scene(Eigen::Affine3d base_in_cam)
{
    planning_scene_diff_client_ = nh_.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
    planning_scene_diff_client_.waitForExistence();

    add_ifco_side("ifco_base", "camera", base_in_cam, 0, 0, 0, ifco_length_, ifco_width_, 0.02);
    
    add_ifco_side("wall1", "camera", base_in_cam, ifco_length_/2, 0, -ifco_height_/2, 0.02, ifco_width_, ifco_height_);

    add_ifco_side("wall2", "camera", base_in_cam, 0, ifco_width_/2, -ifco_height_/2, ifco_length_, 0.02, ifco_height_);

    add_ifco_side("wall3", "camera", base_in_cam, -ifco_length_/2, 0, -ifco_height_/2, 0.02, ifco_width_, ifco_height_);

    add_ifco_side("wall4", "camera", base_in_cam, 0, -ifco_width_/2, -ifco_height_/2, ifco_length_, 0.02, ifco_height_);
}

void IfcoPoseServer::acquire_ifco_params()
{
    while (!nh_.getParam("/ifco/length", ifco_length_) || !nh_.getParam("/ifco/width", ifco_width_) || !nh_.getParam("/ifco/height", ifco_height_))
    {
        ROS_INFO("IFCO specification params have not been published yet");
        ROS_INFO("Publish /ifco/length, /ifco/width and /ifco/height");
    }
}

bool IfcoPoseServer::compute_ifco_pose(ifco_pose_estimator::ifco_pose::Request &req, ifco_pose_estimator::ifco_pose::Response &res)
{

    acquire_ifco_params();

    create_ifco_model();

    boost::shared_ptr<sensor_msgs::PointCloud2 const> shared_scene_pcl_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(pcl_topic_, nh_, ros::Duration(10.));

    if (shared_scene_pcl_msg == NULL) throw std::runtime_error("Unable to acquire scene pointcloud msg");

    filter_pcl(*shared_scene_pcl_msg);

    // Get an initial guess of where lies the centre of the ifco
    if(use_initial_pose_estimate_){
        cam_ifco_tf_ = initial_pose_estimate_.matrix().cast<float>();
    }
    else{
        cam_ifco_tf_ = getGuessFromPCL();
    }
    

    pcl::transformPointCloud (model_ifco_cloud_, model_ifco_cloud_, cam_ifco_tf_);

    performICP();

    pcl_pub_.publish(model_ifco_cloud_);
    
    res.fitness = icp_fitness_;

    Eigen::Affine3f cam_ifco_affine;
    cam_ifco_affine.matrix() = cam_ifco_tf_;

    tf::poseEigenToMsg(cam_ifco_affine.cast<double>(), res.pose);

    if(req.publish_ifco){
        add_ifco_to_planning_scene(cam_ifco_affine.cast<double>());
    }
    
    return true;
}

// C++98 alternative to std::stoul (c++11 could not be used due to PCL issues)
// Taken from here: https://stackoverflow.com/questions/35749900/c98-alternative-to-stdstoul
// Modified to throw an out_of_range exception for negative numbers
unsigned long custom_stoul (std::string const& str, size_t *idx = 0, int base = 10) {
    if (str[0] == '-'){
        throw std::out_of_range(str);
    }
    char *endp;
    unsigned long value = strtoul(str.c_str(), &endp, base);
    if (endp == str.c_str()) {
        throw std::invalid_argument(str);
    }
    if (value == ULONG_MAX && errno == ERANGE) {
        throw std::out_of_range(str);
    }
    if (idx) {
        *idx = endp - str.c_str();
    }
    return value;
}

// C++98 alternative to std::stod (c++11 could not be used due to PCL issues)
double custom_stod (std::string const& str, size_t *idx = 0) {
    char *endp;
    double value = strtod(str.c_str(), &endp);
    if (endp == str.c_str()) {
        throw std::invalid_argument(str);
    }
    if (value == HUGE_VAL && errno == ERANGE) {
        throw std::out_of_range(str);
    }
    if (idx) {
        *idx = endp - str.c_str();
    }
    return value;
}

Eigen::Affine3d construct_pose_from_string_vector(std::vector<std::string> pose_strs){
    if(pose_strs.size() != 7){
        throw std::runtime_error("The initial_pose_estimate must be in the px py pz qx qy qz qw format");
    }
    std::vector<double> pose_doubles;
    for (std::vector<std::string>::iterator it = pose_strs.begin() ; it != pose_strs.end(); ++it){
        pose_doubles.push_back(custom_stod(*it));
    }
    Eigen::Affine3d initial_pose_estimate;
    initial_pose_estimate.translation() << pose_doubles[0], pose_doubles[1], pose_doubles[2];

    double DENORMALIZED_QUATERNION_THRESHOLD = 0.001;
    if(fabs(pose_doubles[3]*pose_doubles[3] + pose_doubles[4]*pose_doubles[4] + pose_doubles[5]*pose_doubles[5] + pose_doubles[6]*pose_doubles[6] - 1.0) > DENORMALIZED_QUATERNION_THRESHOLD){
        throw std::runtime_error("The quaternion of the initial_pose_estimate is not normalized");
    }
    Eigen::Matrix3d initial_pose_estimate_rot;
    initial_pose_estimate_rot = Eigen::Quaterniond(pose_doubles[6], pose_doubles[3], pose_doubles[4], pose_doubles[5]);
    initial_pose_estimate.linear() = initial_pose_estimate_rot;
    return initial_pose_estimate;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ifco_pose_server");
    ros::NodeHandle n;

    if(argc < 6){
        std::cout << "Usage 1: rosrun ifco_pose_estimator ifco_pose_server A_high A_low icp_iterations pcl_topic use_initial_pose_estimate initial_pose_estimate (optional)" << std::endl;
        std::cout << "Usage 2: roslaunch ifco_pose_estimator launch_ifco_pose_server.launch" << std::endl;
        return 0;
    }
    try{
        const unsigned long A_high = custom_stoul(argv[1]);
        const unsigned long A_low = custom_stoul(argv[2]);
        const unsigned long icp_iterations = custom_stoul(argv[3]);
        const std::string pcl_topic = argv[4];
        const std::string use_initial_pose_estimate_st = argv[5];
        if(use_initial_pose_estimate_st.length() != 1 || use_initial_pose_estimate_st[0] < '0' || use_initial_pose_estimate_st[0] > '1') throw std::runtime_error("The use_initial_pose_estimate parameter must be either 0 or 1");
        const bool use_initial_pose_estimate = ( use_initial_pose_estimate_st[0] == '1' );
        Eigen::Affine3d initial_pose_estimate = Eigen::Affine3d::Identity();
        if(use_initial_pose_estimate){
            if(argc != 7) throw std::runtime_error("You want to use a custom initial pose estimate for your IFCO, but no such estimate is provided. Or maybe you passed more than 6 arguments!");
            std::vector<std::string> pose_strs;
            boost::split(pose_strs, argv[6], boost::is_any_of("\t "));
            initial_pose_estimate = construct_pose_from_string_vector(pose_strs);
        }
        IfcoPoseServer ifco_pose_server(n, A_high, A_low, icp_iterations, pcl_topic, use_initial_pose_estimate, initial_pose_estimate);
        ros::ServiceServer service = n.advertiseService("ifco_pose", &IfcoPoseServer::compute_ifco_pose, &ifco_pose_server);
        ROS_INFO("Ready to compute the ifco pose.");
        ros::spin();
        return 0;
    } catch (const std::invalid_argument& e) {
        std::cout << "Argument with value " << e.what() << " is invalid" << std::endl;
        return 0;
    } catch (const std::out_of_range& e) {
        std::cout << "Argument with value " << e.what() << " is out of range" << std::endl;
        return 0;
    } catch (const std::runtime_error& e){
        std::cout << e.what() << std::endl;
    }
    
}
