#include </opt/ros/melodic/include/ros/ros.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>

// Непоянтно азчем это
//#include <data/data_iot_info.h>
//#include <data/data_iot_distance.h>

#include <data/Struct_Control.h>
#include <data/Struct_Car.h>
#include <data/Struct_Encoder.h>
#include <data/Struct_IMU.h>
#include <data/Struct_Led.h>
#include <data/Struct_Odom.h>
#include <data/Struct_Sensor.h>
#include <data/Struct_Servo.h>
#include <data/Struct_ServoOut.h>

#include <data/Struct_Data2Driver.h>
#include <data/Struct_Driver2Data.h>

#include <data/Struct_ModulMotor.h>
#include <data/Struct_ModulLidar.h>
#include <data/Struct_ModulMicric.h>
#include <data/Struct_Info_SPI.h>

#include <data/topicAngle.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>

nav_msgs::Odometry odom;

data::Struct_Info_SPI msg_spi; // Это структуры которые мы заполняем и потом публикуем

data::Struct_ModulMotor msg_modul_motor;   // Это структуры сообщений которые мы заполняем и потом публикуем
data::Struct_ModulLidar msg_modul_lidar;   // Это структуры которые мы заполняем и потом публикуем
data::Struct_ModulMicric msg_modul_micric; // Это структуры которые мы заполняем и потом публикуем

data::Struct_Driver2Data msg_Driver2Data; // Это структуры которые мы заполняем и потом публикуем
data::Struct_Data2Driver msg_Head2Data;   // Полученное сообщение из топика Head в Data
data::topicAngle msg_topicAngle;   // Полученное сообщение из топика 

#include "data_code/config.h"
#include "data_code/data2driver.h"
#include "data_code/data2modul.h"
#include "data_code/code.h"
//  ssh key 22

int main(int argc, char **argv)
{

    ROS_INFO("%s -------------------------------------------------------", NN);
    ROS_WARN("%s START Data Module HighLevel  Raspberry Pi 4B ver. 4.00 ", NN);
    ROS_ERROR("%s -------------------------------------------------------", NN);

    // MyClass myClass; // Объявляем свою локальную перемнную класса и дальше работаем внутри этого класса

    ros::init(argc, argv, "data_node");
    ros::NodeHandle nh;
    ros::Time current_time; // Время ROS
    ros::Rate r(RATE);

    ros::Subscriber subscriber_Head2Data = nh.subscribe("Head2Data", 16, callback_Head2Data);       // Это мы подписываемся на то что публигует Head для Data
    ros::Subscriber subscriber_Angle = nh.subscribe("angle", 16, callback_Angle);       // Это мы подписываемся на то что публигует Head для Data

    ros::Publisher publish_Driver2Data = nh.advertise<data::Struct_Driver2Data>("Driver2Data", 16); // Это мы публикуем структуру которую получили с драйвера

    ros::Publisher publish_ModulMotor = nh.advertise<data::Struct_ModulMotor>("modulMotor", 16);    // Это мы создаем публикатор и определяем название топика в рос
    ros::Publisher publish_ModulLidar = nh.advertise<data::Struct_ModulLidar>("modulLidar", 16);    // Это мы создаем публикатор и определяем название топика в рос
    ros::Publisher publish_ModulMicric = nh.advertise<data::Struct_ModulMicric>("modulMicric", 16); // Это мы создаем публикатор и определяем название топика в рос
    ros::Publisher publish_Spi = nh.advertise<data::Struct_Info_SPI>("infoSpi", 16);                // Это мы создаем публикатор и определяем название топика в рос

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    wiringPiSetup();                    // Инициализация библиотеки
    set_PIN_Led();                      // Устанавливаем и обьявляем пины. для вывода анализатора светодиодов и всего прочего
    init_SPI(SPI_CHANNAL_0, SPI_SPEED); // Инициализация нужного канала SPI
    init_SPI(SPI_CHANNAL_1, SPI_SPEED); // Инициализация нужного канала SPI
    int aaa[10];

    while (ros::ok())
    {
        Led_Blink(PIN_LED_BLUE, 500); // Мигание светодиодом, что цикл работает
        ros::spinOnce();              // Обновление в данных в ядре ROS, по этой команде происходит вызов функции обратного вызова

        // //----------------------------------------------------------------------------------------------------------
        Collect_Data2Modul(); // Собираем рабочие данные в структуру для передачи считывая из топиков
        // // printData_To_Control();                                            // Выводим на печать то что отправляем в Control
        rez_data = sendData2Modul(SPI_CHANNAL_0, Modul2Data, Data2Modul); //
        data_modul_all++;
        // // printDataFrom_Control();
        if (rez_data) // Если пришли хорошие данные то обрабатываем их и публикуем данные в ROS
        {
            //  ROS_INFO("Data ok! ");
            dataProcessing_Modul(); // Обрабатываем данные

            publish_ModulMotor.publish(msg_modul_motor);   // Публикация полученных данных
            publish_ModulLidar.publish(msg_modul_lidar);   // Публикация полученных данных
            publish_ModulMicric.publish(msg_modul_micric); // Публикация полученных данных
        }
        if (!rez_data) // Если пришли плохие данные то выводим ошибку
        {
            ROS_WARN("%s Flag_bedData chek_sum BED Modul", NN);
        }

        // ROS_INFO("%s channal 0, all = %i, bed = %i", NN, data_modul_all, data_modul_bed);
        //  //----------------------------------------------------------------------------------------------------------
        /*
                collect_Data2Driver(); // Собираем рабочие данные в структуру для передачи считывая данные из топика ноды Head
                // printData2Driver();
                rez_data = sendData2Driver(SPI_CHANNAL_1, Driver2Data, Data2Driver); ////  Отправляем данные на нижний уровень
                data_driver_all++;
                // printData_From_Driver();

                if (rez_data) // Если пришли хорошие данные то обрабатываем их и публикуем данные в ROS
                {
                    // digitalWrite(PIN_LED_GREEN, 0); // Гасим светодиод пришли хорошие данные
                    //  ROS_INFO("Data ok! ");
                    processing_Driver2Data(); //  данные от драйвера записываем их в структуру для публикации в топике

                    // setOdomToTf(nh, odom_broadcaster, current_time); // Функция которая одометрию пищет куда нужно, передаем этой функции все переменные котороые создали в гласной функции main
                    // transOdom();

                    publish_Driver2Data.publish(msg_Driver2Data); // Публикация полученных данных
                }

                if (!rez_data) // Если пришли плохие данные то выводим ошибку
                {
                    data_driver_bed++;
                    // digitalWrite(PIN_LED_GREEN, 1); // Включаем светодиод пришли плохие данные
                    ROS_WARN("%s Flag_bedData chek_sum BED Driver", NN);
                }
                // ROS_INFO("%s 1 channal data_Driver_all = %i, data_Driver_bed = %i", NN, data_driver_all, data_driver_bed);
        */
        processingSPI();              // Сбор данных обмена по SPI
        publish_Spi.publish(msg_spi); // Публикация собранных данных по обмену по шине SPI
        ros::spinOnce();              // Обновление в данных в ядре ROS
        r.sleep();                    // Интеллектуальная задержка на указанную частоту
    }
    // softPwmStop (PIN_PWM);
    // digitalWrite(PIN_LED_GREEN, 0); // Выключаем светодиоды при выходе
    // digitalWrite(PIN_LED_BLUE, 0);
    return 0;
}
