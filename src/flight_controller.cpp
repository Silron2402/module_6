#include "flight_controller.hpp"
#include "fiducial_navigation.hpp"
#include <mavros_msgs/CommandBool.h>
#include <iostream>
#include <chrono>
#include "transform.hpp"
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.h>
#include "fiducial_navigation.hpp"
#include "cam_reader.hpp"
#include "image_getter.hpp"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TwistStamped.h>

namespace uav_controller
{

	UavController::UavController(ros::NodeHandle &n, const std::string &uavName)
		: n_(n), uavName_(uavName), camReader(n), marker(n)
	{
		rosNodeInit();
	}

	// Метод, выполняющий арминг аппарата
	void UavController::arm(bool cmd)
	{
		if (!currentState_.armed)
		{
			mavros_msgs::CommandBool arm_cmd; // переменная mavros отвечающая за команду, принимающая значение истина/ложь
			arm_cmd.request.value = cmd;	  // передача значения команды в пространство сообщений  MavRos
											  // определим сервис, с нодой n_ ?отвечающий за арминг дрона
			ros::ServiceClient arming_client = n_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
			arming_client.call(arm_cmd); // непосредстванно арминг путем вызова сервиса arming_client
			// проверим результат работы команды
			if (arm_cmd.response.success) // отклик на вызов сервиса True (арминг получился)
			{
				std::cout << "Successfull Arming!" << std::endl; // сообщение в консоль об успехе
			}
			else
			{
				std::cerr << "Fail Arming!" << std::endl; // вывод сообщения об ошибке
			}
		}
	}

	void UavController::rosNodeInit()
	{
		// Подписываемся на состояние и положение БЛА
		stateSub_ = n_.subscribe<mavros_msgs::State>("/mavros/state", 10, &UavController::uavStateCallback, this);

		// Инициализация подписки на топик odometry
		//odometry_ = n_.subscribe<nav_msgs::Odometry>("/mavros/global_position/local", 1, &UavController::odometryCallback, this);
		odometrySub_ = n_.subscribe<nav_msgs::Odometry>("/mavros/global_position/local", 1, &UavController::odometryCallback, this);

		// Инициализация подписки на топик камеры ЛА
		cameraSub_ = n_.subscribe<sensor_msgs::Image>("/iris/camera1/image_raw", 1, &UavController::imageCallback, this);

		// Инициализация подписки на топик реального положения ЛА
		localPositionSub_ = n_.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, &UavController::realPositionCallback, this);

		// Инициализируем publisher для целевого состояния ЛА
		setPointPub_ = n_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);

		// Создаём publisher для публикации положения по данным камеры
        setPosePub_ = n_.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);

		// Создаём publisher для публикации скорости
        setVelPub_ = n_.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);

	}

	void UavController::uavStateCallback(const mavros_msgs::State::ConstPtr &msg)
	{
		currentState_ = *msg;
	}

	// метод для обработки задаваемого положения БЛА
	void UavController::imageCallback(const sensor_msgs::ImageConstPtr &msg)
	{
		try
		{
			cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			// Обработка изображения
			// cv::imshow("Image Window", cv_ptr->image);
			// cv::waitKey(3);
			// преобразование изображения из формата ROS в формат OpenCV.
			cv_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
		}
		catch (cv_bridge::Exception &e)
		{
			ROS_ERROR("Ошибка конвертации изображения: %s", e.what());
		}
	}

    void UavController::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
		odometry_.x = msg->pose.pose.position.x;
        odometry_.y = msg->pose.pose.position.y;
        odometry_.z = msg->pose.pose.position.z;
		orientation_.w = msg ->pose.pose.orientation.w;
		orientation_.x = msg ->pose.pose.orientation.x;
		orientation_.y = msg ->pose.pose.orientation.y;
		orientation_.z = msg ->pose.pose.orientation.z;
    }

    // метод для обработки реального положения БЛА
	void UavController::realPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &currentPoseLocal)
	{
		currentPoseLocal_ = *currentPoseLocal;
	}

	// метод для установки ограничений значений в пределах заданного диапазона.
	double clip(double value, double min_val, double max_val)
	{
		if (value < min_val)
		{
			return min_val;
		}
		else if (value > max_val)
		{
			return max_val;
		}
		else
		{
			return value;
		}
	}

	// взлетный режим
	bool UavController::do_takeoff(double target_altitude)
	{
		if (!currentState_.armed)
		{
			arm(true);
		}

		// double ascent_speed = 1.0;                       // Установим скорость взлета 1 м\с
		// double timeout = 1 / ascent_speed + 10.0; // установим таймаут исходя из скорости взлета с небольшим запасом
		auto start_time = ros::Time::now();
		bool altitude_reached = false;
		ros::Rate rate(10);
		bool result = false;
		target_alt = target_altitude;
		// ArucoVision vision(n_);
		while (ros::ok() && !altitude_reached)
		{
			// run();
			change_mode("OFFBOARD");
			if (currentState_.mode == "OFFBOARD" && currentState_.armed)
			{
				ROS_INFO("Ready for OFFBOARD control!");
			}
			else
			{
				ROS_ERROR("Failed to enter OFFBOARD mode");
			}
			ros::spinOnce();
			markers_w();
			//takeoffSendSetpoint();

			// vision.imageCallback
			//  Проверим достижение заданной высоты с точностью в 10 см
			if (std::abs(currentPoseLocal_.pose.position.z - target_altitude) < 0.1)
			{
				altitude_reached = true;
				std::cout << "takeoff altitude" << currentPoseLocal_.pose.position.z << std::endl;
				ROS_INFO("Target altitude reached.");
				result = true;
			}

			/*
						// Проверяем не вышел ли взлет за таймаут
						if ((ros::Time::now() - start_time).toSec() > timeout)
						{
							ROS_WARN("Timeout reached without achieving target altitude.");
							return false;
							break;
						}*/
			rate.sleep();
		}
		return result;
	}

	// Метод выполняет расчет желаемой линейной скорости БЛА и скорости угла рыскания
	void UavController::calculateAndSendSetpoint()
	{
		double des_x, des_y, des_z;

		// Получение целевых координат БЛА
		des_x = 0;			// desPose_.pose.position.x;
		des_y = 0;			// desPose_.pose.position.y;
		des_z = target_alt; //.pose.position.z;

		// Вывод результатов
		std::cout << "Target: [x = " << des_x << " y = " << des_y << " z = " << des_z << " ]" << std::endl;

		// Получение текущих координат БЛА
		double x = currentPoseLocal_.pose.position.x;
		double y = currentPoseLocal_.pose.position.y;
		double z = currentPoseLocal_.pose.position.z;
		// Вывод результатов
		std::cout << "Position: [x = " << x << " y = " << y << " z = " << z << " ]" << std::endl;

		// Получение данных от текущей ориентации БЛА
		double w = currentPoseLocal_.pose.orientation.w;
		double x1 = currentPoseLocal_.pose.orientation.x;
		double y1 = currentPoseLocal_.pose.orientation.y;
		double z1 = currentPoseLocal_.pose.orientation.z;

		// Получение текущих углов ориентации БЛА
		EulerAngles ddd;
		ddd = quaternionToEuler(w, x1, y1, z1);
		double pitch = ddd.pitch;
		double roll = ddd.roll;
		double yaw = ddd.yaw;
		// Вывод результатов
		std::cout << "Orientation: [roll = " << roll << " pitch = " << pitch << " yaw = " << yaw << " ]" << std::endl;

		double err_x, err_y, err_z, target_yaw, err_yaw, V_x, V_y, V_z, yaw_rate;

		// Расчет ошибки по положению
		err_x = des_x - x;
		err_y = des_y - y;
		err_z = des_z - z;

		// Расчет целевого значения угла рыскания
		target_yaw = std::atan2(err_y, err_x);
		// Вывод результатов
		std::cout << "Target yaw = " << target_yaw << std::endl;

		// Расчет ошибки по углу рыскания
		err_yaw = target_yaw - yaw;

		// Расчет целевой скорости в глобальной системе координат
		V_x = 0.75 * err_x;		   // 0.75 - коэффициент П-регулятора
		V_y = 0.75 * err_y;		   // 0.75 - коэффициент П-регулятора
		V_z = 0.22 * err_z;		   // 0.22 - коэффициент П-регулятора
		yaw_rate = 0.95 * err_yaw; // 0.95 - коэффициент П-регулятора

		// выведем результат в формате [-pi, pi]
		yaw_rate = std::atan2(std::sin(yaw_rate), std::cos(yaw_rate));
		// Вывод результатов
		std::cout << "yaw_rate = " << yaw_rate << std::endl;

		// Установим лимиты целевых скоростей
		double V_x1 = clip(V_x, -2, 2);
		double V_y1 = clip(V_y, -2, 2);
		double V_z1 = clip(V_z, -1, 1);

		setPointTypeInit();
		setPoint_.velocity.x = V_x1;
		setPoint_.velocity.y = V_y1;
		setPoint_.velocity.z = V_z1;
		setPoint_.yaw_rate = yaw_rate;

		// отправка
		setPointPub_.publish(setPoint_);
	}

	// Метод выполняет расчет желаемой линейной скорости БЛА и скорости угла рыскания
	void UavController::takeoffSendSetpoint()
	{
		double des_x, des_y, des_z;

		// Получение целевых координат БЛА
		des_x = 0;
		des_y = 0;
		des_z = target_alt;

		// Вывод результатов
		std::cout << "Target: [x = " << des_x << " y = " << des_y << " z = " << des_z << " ]" << std::endl;
        
		// Получение текущих координат БЛА
        std::cout << "Одометрия:" << std::endl;
        std::cout << "X_od: " << odometry_.x << std::endl;
        std::cout << "Y_od: " << odometry_.y << std::endl;
        std::cout << "Z_od: " << odometry_.z << std::endl;

        double x = odometry_.x;
        double y = odometry_.y;
        double z = odometry_.z;
		
		//double x = currentPoseLocal_.pose.position.x;
		//double y = currentPoseLocal_.pose.position.y;
		//double z = currentPoseLocal_.pose.position.z;
		// Вывод результатов
		std::cout << "Position: [x = " << x << " y = " << y << " z = " << z << " ]" << std::endl;

		// Получение данных от текущей ориентации БЛА
		double w = orientation_.w;
		double x1 = orientation_.x;
		double y1 = orientation_.y;
		double z1 = orientation_.z;
		/*
		double w = currentPoseLocal_.pose.orientation.w;
		double x1 = currentPoseLocal_.pose.orientation.x;
		double y1 = currentPoseLocal_.pose.orientation.y;
		double z1 = currentPoseLocal_.pose.orientation.z;*/

		// Получение текущих углов ориентации БЛА
		EulerAngles ddd;
		ddd = quaternionToEuler(w, x1, y1, z1);
		double pitch = ddd.pitch;
		double roll = ddd.roll;
		double yaw = ddd.yaw;
		// Вывод результатов
		std::cout << "Orientation: [roll = " << roll << " pitch = " << pitch << " yaw = " << yaw << " ]" << std::endl;

		double err_x, err_y, err_z, target_yaw, err_yaw, V_x, V_y, V_z, yaw_rate;

		// Расчет ошибки по положению
		err_x = des_x - x;
		err_y = des_y - y;
		err_z = des_z - z;

		// Расчет целевого значения угла рыскания
		target_yaw = std::atan2(err_y, err_x);
		// Вывод результатов
		std::cout << "Target yaw = " << target_yaw << std::endl;

		// Расчет ошибки по углу рыскания
		err_yaw = target_yaw - yaw;

		// Расчет целевой скорости в глобальной системе координат
		V_x = 0.75 * err_x;		  // 0.75 - коэффициент П-регулятора
		V_y = 0.75 * err_y;		  // 0.75 - коэффициент П-регулятора
		V_z = 0.87 * err_z;		  // 0.22 - коэффициент П-регулятора
		yaw_rate = 0.5 * err_yaw; // 0.95 - коэффициент П-регулятора

		// выведем результат в формате [-pi, pi]
		yaw_rate = std::atan2(std::sin(yaw_rate), std::cos(yaw_rate));

		// Установим лимиты целевых скоростей
		double V_x1 = clip(V_x, -2, 2);
		double V_y1 = clip(V_y, -2, 2);
		double V_z1 = clip(V_z, -1, 1);

		setPointTypeInit();
		setPoint_.velocity.x = V_x1;
		setPoint_.velocity.y = V_y1;
		setPoint_.velocity.z = V_z1;
		setPoint_.yaw_rate = yaw_rate;

		// Вывод результатов
		std::cout << "yaw_rate = " << yaw_rate << std::endl;
		std::cout << "V_x = " << V_x1 << std::endl;
		std::cout << "V_y = " << V_y1 << std::endl;
		std::cout << "V_z = " << V_z1 << std::endl;
		// std::cout << "fhfh " << setPoint_ << std::endl;
       
		
		// Заполняем header
        vel_msg.header.stamp = ros::Time::now();
		vel_msg.header.frame_id = "base_link";  // Система координат дрона
        //vel_msg.header.frame_id = "takeoff";  // или "vision", "camera"

        // Заполняем позицию (пример: x=1.0, y=2.0, z=1.5)
        vel_msg.twist.linear.x = V_x1;
        vel_msg.twist.linear.y = V_y1;
        vel_msg.twist.linear.z = V_z1;

        // Заполняем ориентацию (кватернион)
        vel_msg.twist.angular.x = 0;
		vel_msg.twist.angular.y = 0;
		vel_msg.twist.angular.z = yaw_rate;
   
        // Публикуем сообщение
        setVelPub_.publish(vel_msg);

        //ROS_INFO("Published pose: x=%.2f, y=%.2f, z=%.2f",
        //         pose_msg.pose.position.x,
        //         pose_msg.pose.position.y,
        //         pose_msg.pose.position.z);

		// отправка
        //setPointPub_.publish(setPoint_);
	}

	void UavController::setPointTypeInit()
	{
		// задаем тип используемого нами сообщения для желаемых параметров управления аппаратом
		// приведенная ниже конфигурация соответствует управлению линейной скоростью ЛА
		// и угловой скоростью аппарата в канале рыскания(yaw)
		uint16_t setpointTypeMask = mavros_msgs::PositionTarget::IGNORE_PX + mavros_msgs::PositionTarget::IGNORE_PY + mavros_msgs::PositionTarget::IGNORE_PZ + mavros_msgs::PositionTarget::IGNORE_AFX + mavros_msgs::PositionTarget::IGNORE_AFY + mavros_msgs::PositionTarget::IGNORE_AFZ + mavros_msgs::PositionTarget::IGNORE_YAW;
		// при помощи конфигурации вышеприведенным образом переменной setpointTypeMask
		// можно настроить управление аппаратом посредством передачи(положения аппарата и углового положения в канале рыскания)

		// Конфигурация системы координат в соответствии с которой задаются параметры управления ЛА
		// при setpointCoordinateFrame = 1 управление происходит в неподвижной СК (локальная неподвижная СК инициализируется при работе навигационной системы)
		// при использовании ГНСС или optical flow является стартовой, при использовании других НС начало координат соответствует таковому у выбранной
		//  навигационной системы(например оси выходят из центра реперного маркера).
		// setpointCoordinateFrame = 8 соответствует управлению аппаратом в связных нормальных осях (подвижная СК центр которой находится в центре масс ЛА)
		// в действительности без какой либо настройки, совпадает с системой координат инерциальной навигационной системы.
		uint16_t setpointCoordinateFrame = 1;

		// Присваиваем наши параметры задающего воздействия полям класса нашего сообщения
		setPoint_.type_mask = setpointTypeMask;
		setPoint_.coordinate_frame = setpointCoordinateFrame;
	}

	// Смена режима полета
	bool UavController::change_mode(const std::string &mode)
	{
		mavros_msgs::SetMode sm;
		sm.request.custom_mode = mode;

		ros::ServiceClient client = n_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
		return client.call(sm) && sm.response.mode_sent;

		if (client.call(sm))
		{
			if (sm.response.mode_sent)
			{
				ROS_INFO("Mode changed to %s", mode.c_str());
				return true;
			}
			else
			{
				ROS_ERROR("Failed to change mode to %s", mode.c_str());
				return false;
			}
		}
		else
		{
			ROS_ERROR("Service call failed for mode change");
			return false;
		}
	}

	// обработка маркеров
	void UavController::markers_w()
	{
		// Получим данные из словаря
		Markers_dict = marker.getMarkerMatrix();
		
		//  Обнуляем векторы перед новым поиском
		ids.clear();
		corners.clear();

		//Получим параметры камеры;
		cv::Mat cameraMatrix = camReader.get_cameraMatrix();
	
		// Получение параметров камеры
		distCoeffs = camReader.get_distCoeff();
		if (!cameraMatrix.empty())
		{
			ROS_INFO("Camera matrix received:");
			std::cout << cameraMatrix << std::endl;
			std::cout << distCoeffs.at(0) << std::endl;
			std::cout << distCoeffs.at(1) << std::endl;
		}

		if (!cameraMatrix.empty() && !distCoeffs.empty())
		{
			try
			{
				// Создаем вектор с координатами точек в МСК
				std::vector<cv::Point3f> objectPoints;

				std::vector<cv::Point2f> imagePoints;
				// cv::namedWindow("Image Window", cv::WINDOW_NORMAL);

				// Параметры для вывода текста
				int fontFace = cv::FONT_HERSHEY_SIMPLEX;
				double fontScale = 0.4;
				cv::Scalar color = cv::Scalar(0, 0, 255);
				int thickness = 1;

				// Обнаружение маркеров
				cv::aruco::detectMarkers(cv_image, dictionary, corners, ids);

				// Обработка найденных маркеров
				if (!ids.empty())
				{
					ROS_INFO("Found %d markers", ids.size());

					//создаем структуры rvec tvec
					cv::Mat rvec, tvec;

                    // очистка векторов
					objectPoints.clear(); 
					imagePoints.clear();

					for (size_t i = 0; i < ids.size(); ++i)
					{
						// Обработка изображения
						std::vector<cv::Point3f> result = Markers_dict[ids.at(i)];
						objectPoints.push_back(result.at(0));
						objectPoints.push_back(result.at(1));
						objectPoints.push_back(result.at(2));
						objectPoints.push_back(result.at(3));

						// Форматируем координаты в строку
						std::stringstream ss;
                        std::stringstream ss2;

						ss << "X: " << result.at(0).x << ", Y: " << result.at(0).y;
						std::string text = ss.str();
						ss2 << "X: " << result.at(2).x << ", Y: " << result.at(2).y;
						std::string text2 = ss2.str();
						
						cv::circle(cv_image, corners[i].at(0), 5, cv::Scalar(0, 0, 255), 2);
						cv::circle(cv_image, corners[i].at(2), 5, cv::Scalar(0, 255, 0), 2);
						imagePoints.push_back(corners[i].at(0));
						imagePoints.push_back(corners[i].at(1));
						imagePoints.push_back(corners[i].at(2));
						imagePoints.push_back(corners[i].at(3));
						
						cv::Point textOrg(corners[i].at(0).x + 10, corners[i].at(0).y + 10);
						cv::Point textOrg2(corners[i].at(2).x + 10, corners[i].at(2).y + 10);
						cv::putText(cv_image, text, textOrg, fontFace, fontScale, color, thickness, cv::LINE_AA);
						cv::putText(cv_image, text2, textOrg2, fontFace, fontScale, color, thickness, cv::LINE_AA);
					}

					bool success = cv::solvePnP(
						objectPoints,
						imagePoints,
						cameraMatrix,
						distCoeffs,
						rvec,
						tvec,
						false,
						cv::SOLVEPNP_ITERATIVE);

					if (success)
					{
						std::cout << "Вектор поворота: " << rvec << std::endl;
						std::cout << "Вектор переноса: " << tvec << std::endl;
						std::cout << "\nКоординаты дрона (tvec):" << std::endl;
						std::cout << "X: " << tvec.at<double>(0, 0) << std::endl;
						std::cout << "Y: " << tvec.at<double>(1, 0) << std::endl;
						std::cout << "Z: " << tvec.at<double>(2, 0) << std::endl;
						cv::Mat rotationMatrix;
                        cv::Rodrigues(rvec, rotationMatrix);
						std::cout << rvec << std::endl;
						// Транспонируем матрицу вращения
                        cv::Mat R_t = rotationMatrix.t();
                        // Вычисляем позицию камеры
                        cv::Mat camera_position = -R_t * tvec;

						std::cout << "Позиция камеры в мировых координатах:" << std::endl;
                        std::cout << "X_MSK: " << camera_position.at<double>(0, 0) << std::endl;
                        std::cout << "Y_MSK: " << camera_position.at<double>(1, 0) << std::endl;
                        std::cout << "Z_MSK: " << camera_position.at<double>(2, 0) << std::endl;

						std::cout << "Одометрия:" << std::endl;
                        std::cout << "X_od: " << odometry_.x << std::endl;
                        std::cout << "Y_od: " << odometry_.y << std::endl;
                        std::cout << "Z_od: " << odometry_.z << std::endl;

						std::cout << "Расхождение:" << std::endl;
                        std::cout << "Delta_X: " << abs(odometry_.x - camera_position.at<double>(0, 0)) << std::endl;
						std::cout << "Delta_Y: " << abs(odometry_.y - camera_position.at<double>(1, 0)) << std::endl;
						std::cout << "Delta_Z: " << abs(odometry_.z - camera_position.at<double>(2, 0)) << std::endl;
                        
						// Вывод результатов
						// Заполняем header
                        pose_msg.header.stamp = ros::Time::now();
		                pose_msg.header.frame_id = "vision";  // Система координат дрона
    
                        // Заполняем позицию (пример: x=1.0, y=2.0, z=1.5)
                        pose_msg.pose.position.x = camera_position.at<double>(0, 0);
						pose_msg.pose.position.y = camera_position.at<double>(1, 0);
						pose_msg.pose.position.z = camera_position.at<double>(2, 0);
       
                        // Заполняем ориентацию (кватернион)
                        pose_msg.pose.orientation.w = 0;
		                pose_msg.pose.orientation.x = 0;
                 		pose_msg.pose.orientation.y = 0;
						pose_msg.pose.orientation.z = 0;

                        // Публикуем сообщение
                        setPosePub_.publish(pose_msg);



						//cv::circle(cv_image, {tvec.at<double>(0, 0), tvec.at<double>(1, 0)}, 5, cv::Scalar(0, 255, 0), 2);
						//cv::Point textOrg(tvec.at<double>(0, 0) + 10 , tvec.at<double>(1, 0) + 10);
						// Форматируем координаты в строку
						//std::stringstream ss;
						//ss << "X: " << tvec.at<double>(0, 0) << ", Y: " << tvec.at<double>(1, 0);
						//std::string text = ss.str();
						//cv::putText(cv_image, text, textOrg, fontFace, fontScale, color, thickness, cv::LINE_AA);

					}
					else
					{
						std::cerr << "Ошибка при решении PnP" << std::endl;
					}

					if (cv_image.empty())
					{
						std::cout << "empty image!!!" << std::endl;
					}
					else
					{
						std::cout << "image is ok" << std::endl;
					}

					// Отображаем изображение
					cv::imshow("Image Window", cv_image);

					// Добавляем задержку для корректного отображения
					cv::waitKey(3); // Увеличиваем время ожидания*/

				}
				else
				{
					std::cout << "i can't see!!!!" << std::endl;
				}
			}
			catch (cv_bridge::Exception &e)
			{
				ROS_ERROR("Ошибка конвертации: %s", e.what());
			}
		}
	}

	// Активация автоматического режима полета
	void UavController::offboard_enable(bool enable)
	{
		if (enable)
		{
			// offboard_timer_.stop();
			// return;

			// Установка начального положения.
			// set_position(0, 0, 0, 0);
			// Запускаем таймер отправки целевого положения
			// offboard_timer_.start();
			// Немного ждем пока сообщения начнут приходить на автопилот
			// ros::Rate rate(1.0); set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
			// rate.sleep();
			// Переключение режима на OFFBOARD

			// Переключение режима на OFFBOARD
			change_mode("OFFBOARD");
		}
	}

	void UavController::run()
	{
		// Ожидаем подключения к автопилоту
		while (ros::ok() && !currentState_.connected)
		{
			offboard_enable(true);
			change_mode("OFFBOARD");
			ros::spinOnce();
			ros::Rate rate(10);
			rate.sleep();
		}

		// Запускаем режим offboard
		
		// do_takeoff(2.0); // Взлетаем на 2 метра....

		// Методы для следования в точку и  посадки можно сделать по аналогии.
	}

} // namespace uav_controller