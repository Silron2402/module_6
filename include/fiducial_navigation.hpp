#ifndef arucoVision_HPP
#define arucoVision_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// Объявим класс для обработки инфоррмации системы технического зрения и навигации по реперным маркерам
class ArucoVision
{
    // Приватные члены класса
private:
    ros::NodeHandle &n_;
    std::string arucoName_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_; // подписка на изображение с камеры;
    ros::Subscriber cameraSub_;  // подписка на изображение с камеры;

    void rosNodeInit();

    // ros::Subscriber cameraSub_;  // подписка на изображение с камеры;

public:
    ArucoVision(ros::NodeHandle &n_, const std::string &arucoName = "mavros");
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
};

#endif