#include "image_getter.hpp"
#include <iostream>
#include <vector>
#include <sensor_msgs/Image.h>

// Конструктор класса
ImageGetter::ImageGetter(ros::NodeHandle &n, const std::string &uavName)
    : n_(n), camName_(uavName)
{
    rosNodeInit();
}

// метод для обработки задаваемого положения БЛА
void ImageGetter::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        // Обработка изображения
        cv::imshow("Image Window", cv_ptr->image);
        cv::waitKey(3);
        // преобразование изображения из формата ROS в формат OpenCV.
        cv_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Ошибка конвертации изображения: %s", e.what());
    }
}