#ifndef camReader_HPP
#define camReader_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <vector>

// Объявим класс для чтения информации о камере коптера
class CamReader
{
    // Приватные члены класса
private:
    ros::NodeHandle &n_;
    ros::Subscriber cameraInfo_;  // подписка на данные о камере
    std::string camName_;
    cv::Mat cameraMatrix; //  Матрица камеры
    std::vector<double> distCoeffs;   //  Коэффициенты искажений

    void rosNodeInit();

public:
    CamReader(ros::NodeHandle &n_, const std::string &camName = "mavros");
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
    cv::Mat get_cameraMatrix(); //Вывод матрицы камеры
    std::vector<double> get_distCoeff(); //Получение данных о коэффициенте дисторсии
};

#endif