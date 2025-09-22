#include "aruco_matrix.hpp"

#include <vector>
#include <iostream>
#include <map>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

// Конструктор класса
MarkerPosition::MarkerPosition(ros::NodeHandle &n, const std::string &uavName)
   : n_(n), uavName_(uavName)
{
    //	rosNodeInit();
}

std::map<int, std::vector<cv::Point3f>> MarkerPosition::getMarkerMatrix()
{
    // Создаем словарь маркеров
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
    // Векторы для хранения результатов
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    try
    {

        cv_image = cv::imread("/home/andrew/catkin_ws/src/module_6/src/board.jpg");
        if (cv_image.empty())
        {
            std::cerr << "Ошибка: не удалось загрузить изображение!" << std::endl;
            // Дополнительные проверки:
            std::cout << "Текущий рабочий каталог: " << getcwd(NULL, 0) << std::endl;
        }
        else
        {
            std::cout << "Изображение успешно загружено!" << std::endl;
        }

        // Обнаруживаем маркеры
        cv::aruco::detectMarkers(cv_image, dictionary, corners, ids);
        // Выводим результаты
        for (size_t i = 0; i < ids.size(); ++i)
        {
            //std::cout << "Найден маркер ID: " << ids[i] << std::endl;
            // std::cout << "Координаты углов: " << corners[i] << std::endl;
        }
        // Поместим начало мировой СК в угол 0 маркера 99
        // Получим координаты вектора смещения МСК и начала координат доски
        point_0 = corners[99].at(0);
        // Определим значения смещений координат в пикселях
        x0_ = point_0.y;
        y0_ = point_0.x;
        // Расчитаем длину маркера в пикселях
        // Ввиду того, что координаты по оси y отсчитываются вниз,
        // совместим ось x рисунка с осью y МСК поэтому
        int delta_x_ = corners[99].at(2).y - x0_; // размер по оси y МСК
        int delta_y_ = corners[99].at(2).x - y0_; // размер по оси Х МСК
        //std::cout << delta_x_ << std::endl;
        //std::cout << delta_y_ << std::endl;
        /*
                //Расчитаем расстояние между маркерами в пикселях
                int delta_ly_ = corners[98].at(0).x - corners[99].at(1).x;
                int delta_lx_ = corners[89].at(0).y - corners[99].at(3).y;
                std::cout << delta_lx_ << std::endl;
                std::cout << delta_ly_ << std::endl;

                cv::circle(cv_image, corners[98].at(0), 15, cv::Scalar(0, 0, 255), -1);
                cv::circle(cv_image, corners[98].at(1), 15, cv::Scalar(0, 0, 255), -1);
                cv::circle(cv_image, corners[98].at(2), 15, cv::Scalar(0, 0, 255), -1);
                cv::circle(cv_image, corners[98].at(3), 15, cv::Scalar(0, 0, 255), -1);

                cv::circle(cv_image, corners[99].at(0), 15, cv::Scalar(255, 0, 0), -1);
                cv::circle(cv_image, corners[99].at(1), 15, cv::Scalar(255, 0, 0), -1);
                cv::circle(cv_image, corners[99].at(2), 15, cv::Scalar(255, 0, 0), -1);
                cv::circle(cv_image, corners[99].at(3), 15, cv::Scalar(255, 0, 0), -1);

                std::cout <<  corners[97].at(0)   << std::endl;
                std::cout <<  corners[97].at(1)   << std::endl;
                std::cout <<  corners[97].at(2)   << std::endl;
                std::cout <<  corners[97].at(3)   << std::endl;

                std::cout <<  corners[98].at(0)   << std::endl;
                std::cout <<  corners[98].at(1)   << std::endl;
                std::cout <<  corners[98].at(2)   << std::endl;
                std::cout <<  corners[98].at(3)   << std::endl;

                std::cout <<  corners[99].at(0)   << std::endl;
                std::cout <<  corners[99].at(1)   << std::endl;
                std::cout <<  corners[99].at(2)   << std::endl;
                std::cout <<  corners[99].at(3)   << std::endl;*/

        // Создаем словарь для 100 маркеров
        std::map<int, std::vector<cv::Point3f>> markers;
        for (size_t k = 0; k < corners.size(); k++)
        {
            markers[ids.at(k)] = std::vector<cv::Point3f>(4); // вектор, заполненный нулями
            //std::cout << k << std::endl;

            // Заполняем матрицу известными координатами
            // Примерные значения (нужно замерить реальные координаты)
            for (size_t i = 0; i < 4; i++)
            {
                markers[ids.at(k)].at(i).x = (corners[k].at(i).y - x0_) * marker_length / delta_x_;
                markers[ids.at(k)].at(i).y = (corners[k].at(i).x - y0_) * marker_length / delta_y_;
                // markers[0].at(i).z = 0.0; не используется, т.к. доска расположена на плоскости
            }
           // std::cout << markers[ids.at(k)] << std::endl;
            //std::cout << ids.at(k) << std::endl;
        }
        return markers;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Ошибка открытия изображения: %s", e.what());
        return std::map<int, std::vector<cv::Point3f>>();
    }
}
