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

    if(!initialiseCamera()){
        RCLCPP_ERROR_STREAM(LOGGER, "Failed to initialise camera");
        return;
    }
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
    pco_grabber_ = std::make_shared<CPco_grab_usb>((CPco_com_usb*)pco_camera_.get());
    pco_error_ = pco_grabber_->Open_Grabber(board);
    if(pco_error_!=PCO_NOERROR)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Failed to open grabber with ERROR: \n" << pco_error_ << "\n\nExiting\n");
        return false;
    }

    pco_error_ = pco_grabber_->Set_Grabber_Timeout(10000);
    if(pco_error_!=PCO_NOERROR)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Failed to set grabber timeout with ERROR: \n" << pco_error_ << "\n\nExiting\n");
        return false;
    }

    pco_error_ = pco_camera_->PCO_SetCameraToCurrentTime();
    if(pco_error_!=PCO_NOERROR)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Failed to set camera to current time with ERROR: \n" << pco_error_ << "\n\nExiting\n");
        return false;
    }

    pco_error_ = pco_camera_->PCO_ResetSettingsToDefault();
    if(pco_error_!=PCO_NOERROR)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Failed to reset the settings to default ERROR: \n" << pco_error_ << "\n\nExiting\n");
        return false;
    }

    pco_error_ = pco_camera_->PCO_SetTimestampMode(2);
    if(pco_error_!=PCO_NOERROR)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Failed to set timestamp mode of the camera with ERROR: \n" << pco_error_ << "\n\nExiting\n");
        return false;
    }

    pco_error_ = pco_camera_->PCO_SetTimebase(1, 1);
    if(pco_error_!=PCO_NOERROR)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Failed to set timebase of the camera with ERROR: \n" << pco_error_ << "\n\nExiting\n");
        return false;
    }

    pco_error_ = pco_camera_->PCO_SetDelayExposure(0, 5000);
    if(pco_error_!=PCO_NOERROR)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Failed to set the delay and exposure time of the camera with ERROR: \n" << pco_error_ << "\n\nExiting\n");
        return false;
    }

    RCLCPP_INFO_STREAM(LOGGER, "Determining image configuration");
    pco_error_ = pco_grabber_->Get_actual_size(&image_width_, &image_height_, &bit_per_pixel_);
    if(pco_error_!=PCO_NOERROR)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Failed to determine image size with ERROR: \n" << pco_error_ << "\n\nExiting\n");
        return false;
    }

    WORD bit_alignment;
    pco_error_ = pco_camera_->PCO_GetBitAlignment(&bit_alignment); // 0 is MSB (big endian), 1 is LSB (little endian)

    RCLCPP_INFO_STREAM(LOGGER, "Image width: " << image_width_ << "\nImage height: " << image_height_
                        << "Bits per pixel: " << bit_per_pixel_);

    // Setup the buffer for the images
    pco_buffer_.assign(image_height_ * image_width_, 0);
    image_msg_->data.assign(image_height_ * image_width_, 0);
    image_msg_->width = image_width_;
    image_msg_->height = image_height_;
    image_msg_->step = (bit_per_pixel_ / 8) * image_width_;   //row length in bytes
    image_msg_->is_bigendian = bit_alignment == 0;
    image_msg_->encoding = sensor_msgs::image_encodings::MONO16;
    image_msg_->header.frame_id = this->get_parameter("frame_id").as_string();

    RCLCPP_INFO_STREAM(LOGGER, "Preparing camera to record");
    pco_error_ = pco_camera_->PCO_ArmCamera();
    if(pco_error_!=PCO_NOERROR)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Failed to arm the camera ERROR: \n" << pco_error_ << "\n\nExiting\n");
        return false;
    }
    pco_error_ = pco_grabber_->PostArm();
    if(pco_error_!=PCO_NOERROR)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Failed in the post arming setup of the grabber ERROR: \n" << pco_error_ << "\n\nExiting\n");
        return false;
    }

    pco_error_ = pco_camera_->PCO_SetRecordingState(1);
    if(pco_error_!=PCO_NOERROR)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Failed to set the camera to record ERROR: \n" << pco_error_ << "\n\nExiting\n");
        return false;
    }

    auto duration = std::chrono::milliseconds(int(1.0/ this->get_parameter("desired_framerate").as_double() * 1000));
    timer_ = this->create_wall_timer(duration, std::bind(&PCODriver::imageCallback, this));
    RCLCPP_INFO_STREAM(LOGGER, "Camera is now recording");

    return true;
}

void PCODriver::imageCallback() {
    // Get latest image from camera/grabber
    pco_error_ = pco_grabber_->Acquire_Image(pco_buffer_.data());
    //convert to ros image
    std::copy(pco_buffer_.begin(), pco_buffer_.end(), image_msg_->data.begin());
    //publish with the camera info
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg_(
            new sensor_msgs::msg::CameraInfo(camera_info_manager_->getCameraInfo()));

    rclcpp::Time timestamp = this->get_clock()->now();

    image_msg_->header.stamp = timestamp;
    image_msg_->header.frame_id = this->get_parameter("frame_id").as_string();

    camera_info_msg_->header.stamp = timestamp;
    camera_info_msg_->header.frame_id = this->get_parameter("frame_id").as_string();

    camera_info_pub_.publish(image_msg_, camera_info_msg_);

}

PCODriver::~PCODriver() {
    pco_grabber_->Close_Grabber();
    pco_camera_->Close_Cam();

}


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(PCODriver)