#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "libpixyusb2.h"
#include <thread>
#include <mutex>
#include <pixy_camera/ObjectInfo.h>
#include <pixy_camera/ObjectInfoArrayStamped.h>
#include <algorithm>
#include <stdexcept>

//TODO: fix the yaml file get param, the namespace stuff doesnt work as well
template <std::size_t N>
boost::array<double, N> toBoostArray(const std::vector<double>& vec) {
    if (vec.size() != N) {
        throw std::runtime_error("Size mismatch in vector-to-boost::array conversion");
    }
    boost::array<double, N> arr;
    std::copy(vec.begin(), vec.end(), arr.begin());
    return arr;
}

class PixyCamera{
    public:
        PixyCamera(ros::NodeHandle& nh, bool debug, std::string mode, const std::string& yaml_file);
        void run();

    private:
        ros::NodeHandle& nh_;
        
        ros::Publisher image_pub_;
        ros::Publisher cameraInfo_pub_;
        ros::Publisher block_pub_;
        ros::Publisher bearing_pub_;

        Pixy2 pixy_;
        std::mutex pixy_mutex_;

        bool debug_;
        std::string mode_;

        ros::Rate blockRate_;
        ros::Rate imageRate_;
        ros::Rate cameraInfoRate_;

        struct CameraParameters{
            std::vector<double> D;
            std::vector<double> K;
            std::vector<double> R;
            std::vector<double> P;

            CameraParameters()  // Constructor to initialize the parameters
            : K{213.161906, 0.000000, 154.841998,
                0.000000, 213.521038, 114.983980,
                0.000000, 0.000000, 1.000000},
              R{1, 0, 0,
                0, 1, 0,
                0, 0, 1},
              P{170.476436, 0.000000, 154.428043, 0.000000,
                0.000000, 193.474659, 117.728365, 0.000000,
                0.000000, 0.000000, 1.000000, 0.000000},
              D{-0.352043, 0.137879, 0.001953, 0.000628, 0.000000} {}

        };

        CameraParameters camera_params_;

        void lineDetection();
        void blockDetection();
        void videoStreaming();
        void cameraInfo();
};

PixyCamera::PixyCamera(ros::NodeHandle& nh, bool debug, std::string mode, const std::string& yaml_file): nh_(nh), debug_(debug), mode_(mode), blockRate_(60), imageRate_(1), cameraInfoRate_(1) {
    if (!yaml_file.empty()) {
        std::string namespace_ = nh.getNamespace();
        size_t pos = namespace_.find('/', 1);
        namespace_ = namespace_.substr(0, pos);
        ROS_INFO("Loading camera parameters from: %s", yaml_file.c_str());
        nh_.setParam("camera/filename", yaml_file); // Store file name in parameter server
        nh_.getParam(namespace_ + "/distortion_coefficients/data", camera_params_.D);
        nh_.getParam(namespace_ + "/camera_matrix/data", camera_params_.K);
        nh_.getParam(namespace_ + "/rectification_matrix/data", camera_params_.R);
        nh_.getParam(namespace_ + "/projection_matrix/data", camera_params_.P);
    } else {
        ROS_WARN("No YAML file specified. Using default camera parameters.");
    }

    image_pub_ = nh_.advertise<sensor_msgs::Image>("pixy_image", 10);
    cameraInfo_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("camera_info", 10);
    block_pub_ = nh_.advertise<pixy_camera::ObjectInfoArrayStamped>("objectsInfo", 10);

    if (pixy_.init() < 0) {
        ROS_ERROR("Failed to initialize Pixy2");
        throw std::runtime_error("Pixy2 initialization failed");
    }

    if (mode_ == "color"){
        pixy_.changeProg("color_connected_components");
    }
    if (mode_ == "barcode"){
        pixy_.changeProg("line");
    }
}

void PixyCamera::lineDetection(){
    // features: LINE_BARCODE
    pixy_camera::ObjectInfoArrayStamped ObjArraymsg;
    while (ros::ok()){
        {
        std::lock_guard<std::mutex> lock(pixy_mutex_);
        pixy_.line.getAllFeatures(0x04);
        }
        if (pixy_.line.numBarcodes){
            for (int i = 0; i < pixy_.line.numBarcodes; ++i){
                if (debug_){
                    printf ("  Barcode %d: ", i + 1);
                    pixy_.line.barcodes[i].print();
                }
                ObjArraymsg.header.stamp = ros::Time::now();
                ObjArraymsg.header.frame_id = "camera";

                pixy_camera::ObjectInfo objInfoMsg;
                objInfoMsg.signature = pixy_.line.barcodes[i].m_code;
                objInfoMsg.center_x = pixy_.line.barcodes[i].m_x;
                objInfoMsg.center_y = pixy_.line.barcodes[i].m_y;
                ObjArraymsg.objects.push_back(objInfoMsg);
            }
        } else{
            if (debug_){
                ROS_INFO("No blocks detected");
            }
        }
        block_pub_.publish(ObjArraymsg);
        ObjArraymsg.objects.clear();
        blockRate_.sleep();
    }
}

void PixyCamera::blockDetection(){
    pixy_camera::ObjectInfoArrayStamped ObjArraymsg;
    
    while (ros::ok()){
        {
            std::lock_guard<std::mutex> lock(pixy_mutex_);
            pixy_.ccc.getBlocks();
        }

        if (pixy_.ccc.numBlocks){
            if(debug_){
                ROS_INFO("Detected %d blocks", pixy_.ccc.numBlocks);
            }
            for (int i = 0; i < pixy_.ccc.numBlocks; i++) {
                if(debug_){
                    ROS_INFO("Block %d: Signature=%d, X=%d, Y=%d, Width=%d, Height=%d", 
                            i, 
                            pixy_.ccc.blocks[i].m_signature, 
                            pixy_.ccc.blocks[i].m_x, 
                            pixy_.ccc.blocks[i].m_y, 
                            pixy_.ccc.blocks[i].m_width, 
                            pixy_.ccc.blocks[i].m_height);
                }
                ObjArraymsg.header.stamp = ros::Time::now();
                ObjArraymsg.header.frame_id = "camera";

                pixy_camera::ObjectInfo objInfoMsg;
                objInfoMsg.signature = pixy_.ccc.blocks[i].m_signature;
                objInfoMsg.center_x = pixy_.ccc.blocks[i].m_x;
                objInfoMsg.center_y = pixy_.ccc.blocks[i].m_y;
                objInfoMsg.width = pixy_.ccc.blocks[i].m_width;
                objInfoMsg.height = pixy_.ccc.blocks[i].m_height;
                objInfoMsg.angle = pixy_.ccc.blocks[i].m_angle;
                objInfoMsg.index = pixy_.ccc.blocks[i].m_index;
                objInfoMsg.age = pixy_.ccc.blocks[i].m_age;

                ObjArraymsg.objects.push_back(objInfoMsg);
            }
        } else{
            if (debug_){
                ROS_INFO("No blocks detected");
            }
        }
        block_pub_.publish(ObjArraymsg);
        ObjArraymsg.objects.clear();
        blockRate_.sleep();
    }
}

void PixyCamera::videoStreaming(){
    uint8_t *bayerFrame;

    while (ros::ok()){
        {
            std::lock_guard<std::mutex> lock(pixy_mutex_);

            pixy_.m_link.stop(); // Stop Pixy processing to fetch raw frames

            if (pixy_.m_link.getRawFrame(&bayerFrame) < 0){
                ROS_WARN("Failed to get frame from Pixy2");
                pixy_.m_link.resume();
                continue;
            }
            pixy_.m_link.resume(); // Resume Pixy processing
        }

        // Convert Bayer to RGB using OpenCV
        cv::Mat bayer_image(PIXY2_RAW_FRAME_HEIGHT, PIXY2_RAW_FRAME_WIDTH, CV_8UC1, (void *)bayerFrame);
        cv::Mat rgb_image;
        cv::cvtColor(bayer_image, rgb_image, cv::COLOR_BayerBG2RGB);

        // Publish as a ROS Image message
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb_image).toImageMsg();
        image_pub_.publish(msg);

        imageRate_.sleep();
    }
}

void PixyCamera::cameraInfo(){
    pixy_.getResolution();
    if (debug_){
        ROS_INFO("Resolution, width = %d, height = %d", pixy_.frameWidth, pixy_.frameHeight);
    }

    while (ros::ok()){
        sensor_msgs::CameraInfo msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "camera";
        msg.height = pixy_.frameHeight;
        msg.width = pixy_.frameWidth;
        msg.distortion_model = "plumb_bob";
        msg.D = camera_params_.D;
        msg.K = toBoostArray<9>(camera_params_.K);
        msg.R = toBoostArray<9>(camera_params_.R);
        msg.P = toBoostArray<12>(camera_params_.P);
        // // msg.binning_x = 
        // // msg.binning_y = 
        // // msg.roi = 
        cameraInfo_pub_.publish(msg);
        cameraInfoRate_.sleep();
    }
}

void PixyCamera::run() {
    std::thread block_thread;
    if (mode_ == "color"){
        block_thread = std::thread(&PixyCamera::blockDetection, this);
    }
    std::thread lineDetection_thread;
    if (mode_ == "barcode"){
        lineDetection_thread = std::thread(&PixyCamera::lineDetection, this);
    }
    std::thread cameraInfo_thread(&PixyCamera::cameraInfo, this);
    std::thread video_thread;
    if (debug_){
        video_thread = std::thread(&PixyCamera::videoStreaming, this);
    }

    if (mode_ == "color"){
        block_thread.join();
    }
    if (mode_ == "barcode"){
        lineDetection_thread.join();
    }
    cameraInfo_thread.join();
    if (debug_){
        video_thread.join();
    }
}


int main(int argc, char **argv){
    ros::init(argc, argv, "pixy_camera");
    ros::NodeHandle nh("~");

    bool debug = false;
    nh.getParam("debug", debug);
    
    std::string mode;
    if (!nh.getParam("mode", mode)) {
        mode = "color"; 
    }

    std::string yaml_file;
    if (!nh.getParam("camera_yaml", yaml_file)) {
        ROS_ERROR("Failed to get parameter 'camera_yaml'");
        return -1;
    }

    try {
        PixyCamera PixyCamera(nh, debug, mode, yaml_file);
        PixyCamera.run();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
        return -1;
    }

    if(debug){
        ROS_INFO("Debug mode is ON");
    }
    ROS_INFO("Start Detection");

    return 0;
}

// rosparam load ~/ITRL/kth_thesis/src/pixy_camera/parameter/pixy2camera.yaml
// rosrun pixy_camera pixy_node ~/ITRL/kth_thesis/src/pixy_camera/parameter/pixy2camera.yaml
