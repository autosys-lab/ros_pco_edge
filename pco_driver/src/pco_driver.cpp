//
// Created by george on 10/6/21.
//

#include <pco_driver/pco_driver.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pco_camera_driver");

PCODriver::PCODriver(const rclcpp::NodeOptions &options) : Node("pco_camera_driver", options){
    // Declare any parameters needed
    this->declare_parameter<std::string>("frame_id", "pco_camera");
    this->declare_parameter<std::string>("camera_calibration_file", "file://config/camera.yaml");
    this->declare_parameter<int>("desired_framerate", 10);
    this->declare_parameter<int>("camera_id", 0);

    // Initialise ROS objects
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    camera_info_pub_ = image_transport::create_camera_publisher(this, "image", custom_qos_profile);

    camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this);

    camera_info_manager_->loadCameraInfo(this->get_parameter("camera_calibration_file").as_string());

}

bool PCODriver::initialiseCamera() {
    int board = this->get_parameter("camera_id").as_int();
    RCLCPP_INFO_STREAM(LOGGER, "Starting up camera on board " << board);
    pco_camera_ = std::make_shared<CPco_com_usb>();
    pco_error_ = pco_camera_->Open_Cam((WORD)board);
    if(pco_error_!=PCO_NOERROR)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Failed to open camera with ERROR: \n" << pco_error_ << "\n\nExiting\n");
        return false;
    }

    RCLCPP_INFO_STREAM(LOGGER, "Starting up grabber ");
    pco_grabber = std::make_shared<CPco_grab_usb>((CPco_com_usb*)pco_camera_.get());
    pco_error_ = pco_grabber->Open_Grabber(board);
    if(pco_error_!=PCO_NOERROR)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Failed to open grabber with ERROR: \n" << pco_error_ << "\n\nExiting\n");
        return false;
    }
    return false;
}

bool PCODriver::startCamera() {
    return false;
}

void PCODriver::imageCallback() {

}

PCODriver::~PCODriver() {
    pco_grabber->Close_Grabber();
    pco_camera_->Close_Cam();

}
