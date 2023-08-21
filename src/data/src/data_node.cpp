#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>

#include <data/Struct_Car.h>
#include <data/Struct_Control.h>
#include <data/Struct_Encoder.h>
#include <data/Struct_IMU.h>
#include <data/Struct_Led.h>
#include <data/Struct_Odom.h>
#include <data/Struct_Sensor.h>
#include <data/Struct_Servo.h>
#include <data/Struct_ServoOut.h>

#include <data/Struct_Data2Driver.h>
#include <data/Struct_Driver2Data.h>

#include <data/data_iot_info.h>
#include <data/data_iot_distance.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>

nav_msgs::Odometry odom;

data::data_iot_info msg_iot_info_send;         // Сообщение которое публикуем в Топик
data::data_iot_distance msg_iot_distance_send; // Сообщение которое публикуем в Топик

data::Struct_Driver2Data msg_Driver2Data; // Сообщение которое публикуем в Топик
data::Struct_Data2Driver msg_Head2Data; // Полученное сообщение из топика Head в Data

#include "data_code/MyKalman.h"
#include "data_code/config.h"
#include "data_code/data2driver.h"
#include "data_code/data2iot.h"
#include "data_code/code.h"
//#include "data_code/MyClass.h"
// ssh key 22

int main(int argc, char **argv)
{

    ROS_INFO("%s -------------------------------------------------------", NN);
    ROS_WARN("%s START Data Module HighLevel  Raspberry Pi 4B ver. 1.00 ", NN);
    ROS_ERROR("%s -------------------------------------------------------", NN);

    // MyClass myClass; // Объявляем свою локальную перемнную класса и дальше работаем внутри этого класса

    ros::init(argc, argv, "data_node");
    ros::NodeHandle nh;
    
    ros::Subscriber subscriber_Head2Data = nh.subscribe("Head2Data", 16, callback_Head2Data); // Это мы подписываемся на то что публигует Head для Data

    ros::Publisher publish_Driver2Data = nh.advertise<data::Struct_Driver2Data>("Driver2Data", 16); //Это мы публикуем структуру которую получили с драйвера

    ros::Publisher str_pub_data_iot_info = nh.advertise<data::data_iot_info>("data_iot_info", 16);                         //Это мы публикуем структуру
    ros::Publisher str_pub_data_iot_distance = nh.advertise<data::data_iot_distance>("data_iot_distance", 16);                  //Это мы публикуем структуру

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    
    tf::TransformBroadcaster odom_broadcaster;

    ros::Time current_time; // Время ROS

    ros::Rate r(RATE);

    wiringPiSetup();                    // Инициализация библиотеки
    set_PIN_Led();                      // Устанавливаем и обьявляем пины. для вывода анализатора светодиодов и всего прочего
    init_SPI(SPI_CHANNAL_0, SPI_SPEED); // Инициализация нужного канала SPI
    init_SPI(SPI_CHANNAL_1, SPI_SPEED); // Инициализация нужного канала SPI

    while (ros::ok())
    {
        Led_Blink(PIN_LED_BLUE, 1000); // Мигание светодиодом, что цикл работает
        ros::spinOnce();               // Обновление в данных в ядре ROS, по этой команде происходит вызов функции обратного вызова

        // //----------------------------------------------------------------------------------------------------------
        // Collect_Data2Iot(); //Собираем рабочие данные в структуру для передачи считывая из топиков
        // // printData_To_Control();                                            // Выводим на печать то что отправляем в Control
        // rez_data = sendData2Iot(SPI_CHANNAL_0, Iot2Data, Data2Iot); //
        // data_Iot_all++;
        // // printDataFrom_Control();
        // if (rez_data) // Если пришли хорошие данные то обрабатываем их и публикуем данные в ROS
        // {
        //     digitalWrite(PIN_LED_GREEN, 0); // Гасим светодиод пришли хорошие данные
        //     // ROS_INFO("Data ok! ");
        //     dataProcessing_Control();                  // Обрабатываем данные
        //     str_pub_data_iot_info.publish(msg_iot_info_send); //Публикация полученных данных
        //     str_pub_data_iot_distance.publish(msg_iot_distance_send); //Публикация полученных данных
        // }
        // if (!rez_data) // Если пришли плохие данные то выводим ошибку
        // {
        //     digitalWrite(PIN_LED_GREEN, 1); // Включаем светодиод пришли плохие данные

        //     ROS_WARN("%s Flag_bedData chek_sum BED Iot", NN);
        // }

        // ROS_INFO("%s 0 channal data_Iot_all =    %i, data_Iot_bed    = %i", NN, data_Iot_all, data_Iot_bed);
        // //----------------------------------------------------------------------------------------------------------

        collect_Data2Driver(); //Собираем рабочие данные в структуру для передачи считывая данные из топика ноды Head
        //printData_To_Body();
        rez_data = sendData2Driver(SPI_CHANNAL_1, Driver2Data, Data2Driver); ////  Отправляем данные на нижний уровень
        data_driver_all++;
        printData_From_Driver();

        if (rez_data) // Если пришли хорошие данные то обрабатываем их и публикуем данные в ROS
        {
            //digitalWrite(PIN_LED_GREEN, 0); // Гасим светодиод пришли хорошие данные
            // ROS_INFO("Data ok! ");
            processing_Driver2Data(); // данные от драйвера записываем их в структуру для публикации в топике

            // setOdomToTf(nh, odom_broadcaster, current_time); // Функция которая одометрию пищет куда нужно, передаем этой функции все переменные котороые создали в гласной функции main
            // transOdom();

            publish_Driver2Data.publish(msg_Driver2Data); //Публикация полученных данных
        }

        if (!rez_data) // Если пришли плохие данные то выводим ошибку
        {
            data_driver_bed++;
            //digitalWrite(PIN_LED_GREEN, 1); // Включаем светодиод пришли плохие данные
            ROS_WARN("%s Flag_bedData chek_sum BED Driver", NN);
        }
        ROS_INFO("%s 1 channal data_Driver_all = %i, data_Driver_bed = %i", NN, data_driver_all, data_driver_bed);

        ros::spinOnce(); // Обновление в данных в ядре ROS
        r.sleep();       // Интеллектуальная задержка на указанную частоту
    }
    // softPwmStop (PIN_PWM);
    //digitalWrite(PIN_LED_GREEN, 0); // Выключаем светодиоды при выходе
    digitalWrite(PIN_LED_BLUE, 0);
    return 0;
}
