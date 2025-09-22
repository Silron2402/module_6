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
		
		// Инициализация подписки на топик желаемого положения ЛА
		//cameraInfo_ = n_.subscribe<sensor_msgs::CameraInfo>("/iris/camera1/camera_info", 1, &UavController::cameraInfoCallback, this);

        // Инициализация подписки на топик камеры ЛА
		cameraSub_ = n_.subscribe<sensor_msgs::Image>("/iris/camera1/image_raw", 1, &UavController::imageCallback, this);
		
		// Инициализация подписки на топик реального положения ЛА
		localPositionSub_ = n_.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, &UavController::realPositionCallback, this);
		
		// Инициализируем publisher для целевого состояния ЛА
		setPointPub_ = n_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
	}

    void UavController::uavStateCallback(const mavros_msgs::State::ConstPtr &msg)
    {
		currentState_ = *msg;
	}

	//метод для обработки задаваемого положения БЛА
	void UavController::imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
	    try
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            // Обработка изображения
            //cv::imshow("Image Window", cv_ptr->image);
            //cv::waitKey(3);
			// преобразование изображения из формата ROS в формат OpenCV.
			cv_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("Ошибка конвертации изображения: %s", e.what());
        }
	}

	//метод для обработки реального положения БЛА
	void UavController::realPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &currentPoseLocal)
	{
        currentPoseLocal_ = *currentPoseLocal;
	}

	/*void UavController::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
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


	}*/

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

	//взлетный режим
    bool UavController::do_takeoff(double target_altitude)
    {
        if (!currentState_.armed)
		{
            arm(true);
		}

        //double ascent_speed = 1.0;                       // Установим скорость взлета 1 м\с
        //double timeout = 1 / ascent_speed + 10.0; // установим таймаут исходя из скорости взлета с небольшим запасом
        auto start_time = ros::Time::now();
        bool altitude_reached = false;
        ros::Rate rate(30);
		bool result = false;
		target_alt = target_altitude;
		//ArucoVision vision(n_);
        while (ros::ok() && !altitude_reached)
        {
			rate.sleep();
			ros::spinOnce();
			run();
			takeoffSendSetpoint();
			
			//vision.imageCallback
			// Проверим достижение заданной высоты с точностью в 10 см
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
        }
		return result;
    }

	// Метод выполняет расчет желаемой линейной скорости БЛА и скорости угла рыскания
	void UavController::calculateAndSendSetpoint()
	{
		double des_x, des_y, des_z;

		// Получение целевых координат БЛА
		des_x = 0;//desPose_.pose.position.x;
		des_y = 0;//desPose_.pose.position.y;
		des_z = target_alt;//.pose.position.z;

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
		//std::cout << "fhfh " << setPoint_ << std::endl;
		

		// отправка
		setPointPub_.publish(setPoint_);
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
    }

	// обраьотка маркеров
	void UavController::markers_w()
	{
        //Получим данные из словаря
		Markers_dict = marker.getMarkerMatrix();
        //Проверим работоспособность 
		//std::cout << Markers_dict[1] << std::endl;
		// Обнуляем векторы перед новым поиском
		ids.clear();
		corners.clear();
		//CamReader cam(n_);
	
		cv::Mat cameraMatrix = camReader.get_cameraMatrix();
		/*
		if (cameraMatrix.empty())
		{
			std::cout << "fhgdhgdhgdgdghgdh" << std::endl;
		}
		else{
		    std::cout << "working" << std::endl;
      	}*/
	    //Получение параметров камеры
		distCoeffs = camReader.get_distCoeff();
		if (!cameraMatrix.empty())
        {
            ROS_INFO("Camera matrix received:");
            std::cout << cameraMatrix << std::endl;
            std::cout << distCoeffs.at(0) << std::endl;
            std::cout << distCoeffs.at(1) << std::endl;
            //ROS_INFO("%s", matrix.dump().c_str());
        }

		if (!cameraMatrix.empty() && !distCoeffs.empty())
		{
			try
			{
				// Создаем окно для отображения
				//cv::namedWindow("Image Window", cv::WINDOW_NORMAL);

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
					
			//		int marker_id = ids[0];
					cv::Mat rvec, tvec;

					for (size_t i = 0; i < ids.size(); ++i)
					{
						// Обработка изображения

						std::vector<cv::Point3f> result = Markers_dict[ids.at(i)];
						// Форматируем координаты в строку
                        std::stringstream ss;
                        ss << "X: " << result.at(0).x << ", Y: " << result.at(0).y;
                        std::string text = ss.str();
						cv::circle(cv_image,corners[i].at(0), 5, cv::Scalar(0, 0, 255), 2);
						cv::Point textOrg(corners[i].at(0).x + 10, corners[i].at(0).y + 10);
						cv::putText(cv_image, text, textOrg, fontFace, fontScale, color, thickness, cv::LINE_AA);

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

			//	cv::aruco::estimatePoseSingleMarkers(corners, 0.3, cameraMatrix, distCoeffs, rvec, tvec);

				//	double roll = rvec.at<double>(0);
				//	double pitch = rvec.at<double>(1);
			//		double yaw = rvec.at<double>(2);

				//	std::cout << roll << std::endl;

					// Преобразование rvec в матрицу вращения
					// cv::Mat rotation_matrix;

					// cv::Rodrigues(rvec, rotation_matrix);

					// cam_Info.
					//  Здесь можно добавить обработку координат маркеров

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
            //offboard_timer_.stop();
            //return;
        
        // Установка начального положения.
        //set_position(0, 0, 0, 0);
        // Запускаем таймер отправки целевого положения
        //offboard_timer_.start();
        // Немного ждем пока сообщения начнут приходить на автопилот
        //ros::Rate rate(1.0); set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        //rate.sleep();
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
            ros::spinOnce();
			ros::Rate rate(10);
            rate.sleep();
        }

        // Запускаем режим offboard
        offboard_enable(true);
        //do_takeoff(2.0); // Взлетаем на 2 метра....

        // Методы для следования в точку и  посадки можно сделать по аналогии.
    }

} // namespace uav_controller