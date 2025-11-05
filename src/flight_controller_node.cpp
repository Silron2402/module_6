#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "flight_controller.hpp"
#include <mavros_msgs/State.h>
#include "cam_reader.hpp"
#include <vector>
#include "aruco_matrix.hpp"


// Объявим точку входа в исполняемом файле
int main(int argc, char **argv)
{
    // Инициализация ноды и регистрация ее в master node
    // с именем uav_controller_node
    ros::init(argc, argv, "uav_controller_node");
    
    // cоздаем экземпляр класса ноды
    ros::NodeHandle n;

    geometry_msgs::PoseStamped desPose;
    ros::Rate rate(30);

    // создаем экземпляр класса системы управления БЛА
    uav_controller::UavController controller(n);
    
    //Выполним арминг дрона  
    controller.arm(true);
      

    // Немного ждем пока сообщения начнут приходить на автопилот
    //ros::Rate rate(1.0);
    rate.sleep();
    
    //Зададим высоту взлета дрона
    double target_altitude = 2;

   //Выполним взлет дрона
    controller.do_takeoff(target_altitude);
   
    while (ros::ok())
    {
        // Делаем шаг системы
        // После вызова данной комманды если в очереди были сообщения
        // произойдет чтение сообщение из очереди и вызов callback
        ros::spinOnce();
        controller.run();

        // Получим текущие координаты дрона
        //controller.calculateAndSendSetpoint(); //.pose.position.x;
        controller.takeoffSendSetpoint();
        controller.markers_w();
        // controller.arm(true);
        //  Ожидание для поддержания частоты работы системы
        //  ожидание проходит с учетом времени между текущей и предыдущей итерациями
        //  что позволяет более точно регулировать частоту работы системы
        rate.sleep();


    }    
       
}