#include "fiducial_navigation.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <ros/ros.h>
#include <chrono>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
/*
ArucoVision::ArucoVision(ros::NodeHandle &n)
    : n_(n)
{
    rosNodeInit();
};*/
ArucoVision::ArucoVision(ros::NodeHandle &n, const std::string &arucoName) : n_(n), arucoName_(arucoName), it_(n)
{
    rosNodeInit();
}

void ArucoVision::rosNodeInit()
	{
		// Подписываемся на состояние и положение БЛА
		//stateSub_ = n_.subscribe<mavros_msgs::State>("/mavros/state", 10, &UavController::uavStateCallback, this);
		
		// Инициализация подписки на топик камеры ЛА
        //image_sub_ = n_.subscribe<image_transport::ImageTransport>("iris/camera1/image_raw", 1, &ArucoVision::imageCallback, this);
		cameraSub_ = n_.subscribe<sensor_msgs::Image>("/iris/camera1/image_raw", 1, &ArucoVision::imageCallback, this);
		
		// Инициализация подписки на топик реального положения ЛА
		//localPositionSub_ = n_.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, &UavController::realPositionCallback, this);
		
		// Инициализируем publisher для целевого состояния ЛА
		//setPointPub_ = n_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
	}

//Обработка изображения
    void ArucoVision::imageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        try
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            // Обработка изображения
            cv::imshow("Image Window", cv_ptr->image);
            cv::waitKey(3);
            // Автоматическая конвертация в OpenCV Mat
            cv::Mat cv_image = cv_bridge::toCvShare(msg, "bgr8")->image;

            // Дальнейшая обработка изображения
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("Ошибка конвертации изображения: %s", e.what());
        }
    }