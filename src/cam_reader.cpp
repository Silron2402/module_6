#include "cam_reader.hpp"
#include <iostream>
#include <vector>
#include <sensor_msgs/CameraInfo.h>

//Конструктор класса
CamReader::CamReader(ros::NodeHandle &n, const std::string &uavName)
    : n_(n), camName_(uavName)
{
    rosNodeInit();
}

void CamReader::rosNodeInit()
{
    // Инициализация подписки на топик камеры ЛА
    cameraInfo_ = n_.subscribe<sensor_msgs::CameraInfo>("/iris/camera1/camera_info", 1, &CamReader::cameraInfoCallback, this);
}

//Обработчик полученной информации
void CamReader::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
    // Сохранение матрицы калибровки
    cameraMatrix = cv::Mat(3, 3, CV_64F);
    cameraMatrix.at<double>(0, 0) = msg->K[0];
    cameraMatrix.at<double>(0, 1) = msg->K[1];
    cameraMatrix.at<double>(0, 2) = msg->K[2];

    cameraMatrix.at<double>(1, 0) = msg->K[3];
    cameraMatrix.at<double>(1, 1) = msg->K[4];
    cameraMatrix.at<double>(1, 2) = msg->K[5];

    cameraMatrix.at<double>(2, 0) = msg->K[6];
    cameraMatrix.at<double>(2, 1) = msg->K[7];
    cameraMatrix.at<double>(2, 2) = msg->K[8];

    // Получение коэффициентов дисторсии
    distCoeffs = msg->D;
}

// вывод матрицы калибровки 
cv::Mat CamReader::get_cameraMatrix()
{
    //std::cout << cameraMatrix <<std::endl;
    return cameraMatrix;
}

// вывод коэффициентов дисторсии 
std::vector<double> CamReader::get_distCoeff()
{
    return distCoeffs;
}