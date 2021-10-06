//
// Created by george on 10/6/21.
//

#ifndef PCO_DRIVER_PCO_DRIVER_H
#define PCO_DRIVER_PCO_DRIVER_H

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

#include <opencv4/opencv2/imgproc.hpp>

// PCO camera
#include <pco_common/pco_classes/Cpco_com.h>
#include <pco_driver/pco_classes/Cpco_grab_usb.h>
#include <pco_common/pco_include/file12.h>

class PCODriver : public rclcpp::Node{
public:
    explicit PCODriver(const rclcpp::NodeOptions &options);

    ~PCODriver();
    bool initialiseCamera();

    bool startCamera();

private:
    rclcpp::TimerBase::SharedPtr timer_;

    std::string frame_id_;
    uint image_height_;
    uint image_width_;
    double fps_;
    int camera_id;

    std::chrono::steady_clock::time_point last_frame_;

    std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
    image_transport::CameraPublisher camera_info_pub_;

    std::shared_ptr<sensor_msgs::msg::Image> image_msg_;

    void imageCallback();

    // PCO camera components
    std::shared_ptr<CPco_com> pco_camera_;
    std::shared_ptr<CPco_grab_usb> pco_grabber;
    CPco_Log* mylog=NULL;
    DWORD pco_error_;

    std::vector<WORD> pco_buffer_;

};


#endif //PCO_DRIVER_PCO_DRIVER_H
