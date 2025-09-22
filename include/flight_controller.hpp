#ifndef uavController_HPP
#define uavController_HPP

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "cam_reader.hpp"
#include "aruco_matrix.hpp"

// Обьявим класс для контроллера БЛА
// этот класс включает методы для управления аппаратом
//

namespace uav_controller
{
	class UavController
	{
		public:
		UavController(ros::NodeHandle &n, const std::string &uavName="mavros"); //конструктор класса
		

		/**
			* @brief Метод переводит аппарат в состояние arm/disarm
			* Состояние arm - аппарат готов к движению при получении комманд
			* управления начинает движение
			*
			* Состояние disarm - аппарат не готов к движению при поступлении
			* комманд управления не начинает движение
			*
			* @param cmd - смена состояния
			* True - перевод аппарата в состояние arm
			* False - перевод аппарата в состояние disarm
			*/

		//метод для арминга дрона. На вход принимает команду True/False
		void arm(bool cmd);
		/**
		   * @brief метод производит рассчет желаемых управляющих воздействий
		   *  и пересылает сообщение типа mavros_msgs::PositionTarget
		   * в топик <имя_ла>/setpoint_raw/local (как правило mavros/setpoint_raw/local)
		   */
		//Метод расчета и отправки положения точки 
		void calculateAndSendSetpoint();
		//Метод расчета и отправки положения точки при \взлет
		void takeoffSendSetpoint();
        bool do_takeoff(double target_altitude);

		void markers_w();
		
		void run();

		private:
        //Приватные члены класса
		CamReader camReader;  // Объявляем член класса
		MarkerPosition marker; // класс для получения маркеров
		ros::NodeHandle &n_;
		std::string uavName_;
		mavros_msgs::PositionTarget setPoint_;        // объект сообщения для задающего воздействия
		mavros_msgs::State currentState_;             // объект сообщения о состоянии аппарата
		geometry_msgs::PoseStamped currentPoseLocal_; // объект сообщения о положении и ориентации
		geometry_msgs::PoseStamped desPose_;          // объект сообщения о требуемом положении БЛА
		ros::Subscriber localPositionSub_;
		ros::Subscriber desPoseSub_;
		ros::Subscriber stateSub_;
		ros::Publisher setPointPub_;		
		ros::ServiceClient setModeClient_;
		mavros_msgs::SetMode setModeName_;
        ros::Subscriber cameraSub_;  // подписка на изображение с камеры;
		ros::Subscriber cameraInfo_;  // подписка на данные о камере
        //sensor_msgs::CameraInfo cam_Info;

		cv::Mat cv_image;  //изображение в формате OpenCV
        std::vector<int> ids;
		std::vector<std::vector<cv::Point2f>> corners;
		cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
		cv::Mat cameraMatrix; //  Матрица камеры
        std::vector<double> distCoeffs;   //  Коэффициенты искажений
		std::map<int, std::vector<cv::Point3f>> Markers_dict; //словарь с маркерами и их координатами в мировой СК
		

		double target_alt;  //Целевая высота взлета дрона

		void rosNodeInit();
		void setPointTypeInit();
		void uavStateCallback(const mavros_msgs::State::ConstPtr &msg);
		void imageCallback(const sensor_msgs::ImageConstPtr& msg);
		void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
		void realPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &currentPoseLocal);
		void offboard_enable(bool enable);
		bool change_mode(const std::string &mode);
		

		
		

	};
};// namespace uav_controller

#endif