#ifndef imageGetter_HPP
#define imageGetter_HPP

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

// Объявим класс для чтения информации о камере коптера
class ImageGetter
{
    // Приватные члены класса
private:
    ros::NodeHandle &n_;
    std::string camName_;
    ros::Subscriber cameraSub_;  // подписка на изображение с камеры;
    cv::Mat cv_image;  //изображение в формате OpenCV
    
    void rosNodeInit();

public:
    ImageGetter(ros::NodeHandle &n_, const std::string &camName = "mavros");
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    

    cv::Mat get_cameraMatrix(); //Вывод матрицы камеры
    std::vector<double> get_distCoeff(); //Получение данных о коэффициенте дисторсии
};



#endif