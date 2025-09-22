#ifndef arucoMatrix_HPP
#define arucoMatrix_HPP

#include <vector>
#include <iostream>
#include <map>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

class MarkerPosition
{
private:
    // Приватные члены класса
    ros::NodeHandle &n_;
    std::string uavName_;
    cv::Mat cv_image;  //изображение в формате OpenCV
    cv::Point2f point_0; //Начальные координаты системы в системе изображения (левый нижний угол)
    //Координаты левого нижнего маркера с системе координат изображения (пиксели)
    double x0_;
    double y0_;
    double marker_length = 0.3;

    // Получаем координаты точки
    double x; // Координата X
    double y; // Координата Y
    double z; // Координата Z
    int id;   // ID маркера

public:
    // Конструктор
    MarkerPosition(ros::NodeHandle &n, const std::string &uavName = "mavros"); // конструктор класса
    //Получение изображений
    std::map<int, std::vector<cv::Point3f>> getMarkerMatrix(); // используется словарь, в котором ключ - ID маркера, а значение - Четырехмерная матрица
    void printMarkerPositions(const std::vector<std::vector<MarkerPosition>> &matrix);
    //std::vector<std::vector<MarkerPosition>> createMarkerMatrix();
};


#endif