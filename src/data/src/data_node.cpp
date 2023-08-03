#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <my_msgs/data_driver_info.h>
#include <my_msgs/data_driver_control.h>
#include <my_msgs/data_driver_odometr.h>
#include <my_msgs/Command.h>
#include <my_msgs/data_iot_info.h>
#include <my_msgs/data_iot_distance.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <wiringPiI2C.h>

#include <softPwm.h>

nav_msgs::Odometry odom;
my_msgs::Command msg_head_receive; // Полученное сообщение из топика

my_msgs::data_iot_info msg_iot_info_send;         // Сообщение которое публикуем в Топик
my_msgs::data_iot_distance msg_iot_distance_send; // Сообщение которое публикуем в Топик

my_msgs::data_driver_info msg_driver_info_send;       // Сообщение которое публикуем в Топик
my_msgs::data_driver_control msg_driver_control_send; // Сообщение которое публикуем в Топик
my_msgs::data_driver_odometr msg_driver_odometr_send; // Сообщение которое публикуем в Топик

#include "MyKalman.h"
#include "config.h"
#include "data2driver.h"
#include "data2iot.h"
#include "code.h"
//#include "MyClass.h"
// ssh key 22

int main(int argc, char **argv)
{

    ROS_INFO("%s -------------------------------------------------", NN);
    ROS_WARN("%s START Data Module HighLevel  Raspberry Pi 4B !888! ", NN);
    ROS_ERROR("%s -------------------------------------------------", NN);

    // MyClass myClass; // Объявляем свою локальную перемнную класса и дальше работаем внутри этого класса

    ros::init(argc, argv, "data_node");
    ros::NodeHandle nh;
    ros::Publisher str_pub_data_driver_info = nh.advertise<my_msgs::data_driver_info>("data_driver_info", 16);          //Это мы публикуем структуру
    ros::Publisher str_pub_data_driver_control = nh.advertise<my_msgs::data_driver_control>("data_driver_control", 16); //Это мы публикуем структуру
    ros::Publisher str_pub_data_driver_odometr = nh.advertise<my_msgs::data_driver_odometr>("data_driver_odometr", 16); //Это мы публикуем структуру
    ros::Publisher str_pub_data_iot_info = nh.advertise<my_msgs::data_iot_info>("data_iot_info", 16);                         //Это мы публикуем структуру
    ros::Publisher str_pub_data_iot_distance = nh.advertise<my_msgs::data_iot_distance>("data_iot_distance", 16);                  //Это мы публикуем структуру

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    ros::Subscriber command_sub = nh.subscribe("command_topic", 16, message_callback_Command); // Это мы подписываемся на то что публигует Главная нода для Body
    tf::TransformBroadcaster odom_broadcaster;

    ros::Time current_time; // Время ROS

    ros::Rate r(RATE);

    wiringPiSetup();                    // Инициализация библиотеки
    set_PIN_Led();                      // Устанавливаем и обьявляем пины. для вывода анализатора светодиодов и всего прочего
    init_SPI(SPI_CHANNAL_0, SPI_SPEED); // Инициализация нужного канала SPI
    init_SPI(SPI_CHANNAL_1, SPI_SPEED); // Инициализация нужного канала SPI

    // Setup I2C communication
    // int fd = wiringPiI2CSetup(DEVICE_ID_STM);
    // if (fd == -1)
    // {
    //     ROS_WARN("%s Failed to init I2C communication = %i.", NN, fd);
    //     return -1;
    // }
    // ROS_INFO("%s I2C communication successfully setup = %i.", NN, fd);

    // laser.setParametr1 (1, 0.1);
    // laser.setParametr2(5, 0.1); // Установка параетров фильтрации в фидьтре Каламан. Можно подбирать как получится
    // // uzi.setParametr1 (0.5, 0.1);
    // uzi.setParametr2(5, 0.1);

    // SoftPwmCreate(PIN_PWM, 96, 100);
    // softPwmStop (PIN_PWM);

    while (ros::ok())
    {
        Led_Blink(PIN_LED_BLUE, 1000); // Мигание светодиодом, что цикл работает
        ros::spinOnce();               // Обновление в данных в ядре ROS, по этой команде происходит вызов функции обратного вызова

        //----------------------------------------------------------------------------------------------------------
        // wiringPiI2CWriteReg8(fd, REG_POWER_CTL, 0b00001000);
        // wiringPiI2CWrite(fd, 0b00001111);

        //----------------------------------------------------------------------------------------------------------
        Collect_Data2Iot(); //Собираем рабочие данные в структуру для передачи считывая из топиков
        // printData_To_Control();                                            // Выводим на печать то что отправляем в Control
        rez_data = sendData2Iot(SPI_CHANNAL_0, Iot2Data, Data2Iot); //
        data_Iot_all++;
        // printDataFrom_Control();
        if (rez_data) // Если пришли хорошие данные то обрабатываем их и публикуем данные в ROS
        {
            digitalWrite(PIN_LED_GREEN, 0); // Гасим светодиод пришли хорошие данные
            // ROS_INFO("Data ok! ");
            dataProcessing_Control();                  // Обрабатываем данные
            str_pub_data_iot_info.publish(msg_iot_info_send); //Публикация полученных данных
            str_pub_data_iot_distance.publish(msg_iot_distance_send); //Публикация полученных данных
        }
        if (!rez_data) // Если пришли плохие данные то выводим ошибку
        {
            digitalWrite(PIN_LED_GREEN, 1); // Включаем светодиод пришли плохие данныеc

            ROS_WARN("%s Flag_bedData chek_sum BED Iot", NN);
        }

        ROS_INFO("%s 0 channal data_Iot_all =    %i, data_Iot_bed    = %i", NN, data_Iot_all, data_Iot_bed);
        //----------------------------------------------------------------------------------------------------------

        Collect_Data2Driver(); //Собираем рабочие данные в структуру для передачи считывая данные из топика ноды Head
        //printData_To_Body();
        rez_data = sendData2Driver(SPI_CHANNAL_1, Driver2Data, Data2Driver); ////  Отправляем данные на нижний уровень
        data_driver_all++;
        // printDataFrom_Body();

        if (rez_data) // Если пришли хорошие данные то обрабатываем их и публикуем данные в ROS
        {
            digitalWrite(PIN_LED_GREEN, 0); // Гасим светодиод пришли хорошие данные
            // ROS_INFO("Data ok! ");
            dataProcessing_Body(); // Обрабатываем данные
                                   // setOdomToTf(nh, odom_broadcaster, current_time); // Функция которая одометрию пищет куда нужно, передаем этой функции все переменные котороые создали в гласной функции main
            // transOdom();

            str_pub_data_driver_info.publish(msg_driver_info_send); //Публикация полученных данных
            str_pub_data_driver_control.publish(msg_driver_control_send); //Публикация полученных данных
            str_pub_data_driver_odometr.publish(msg_driver_odometr_send); //Публикация полученных данных
        }

        if (!rez_data) // Если пришли плохие данные то выводим ошибку
        {
            data_driver_bed++;
            digitalWrite(PIN_LED_GREEN, 1); // Включаем светодиод пришли плохие данные
            ROS_WARN("%s Flag_bedData chek_sum BED Driver", NN);
        }
        ROS_INFO("%s 1 channal data_Driver_all = %i, data_Driver_bed = %i", NN, data_driver_all, data_driver_bed);

        ros::spinOnce(); // Обновление в данных в ядре ROS
        r.sleep();       // Интеллектуальная задержка на указанную частоту
    }
    // softPwmStop (PIN_PWM);
    digitalWrite(PIN_LED_GREEN, 0); // Выключаем светодиоды при выходе
    digitalWrite(PIN_LED_BLUE, 0);
    return 0;
}
