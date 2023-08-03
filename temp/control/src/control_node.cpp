#include <ros/ros.h>
#include <my_msgs/Control.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>


#include "config.h"
#include "code.h"

int main(int argc, char **argv)
{
    printf(" -------------------------------------------------\n");
    printf(" START Control Module HighLevel ROS Raspberry Pi 4B !!! \n");
    printf(" -------------------------------------------------\n");

    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;
    ros::Publisher str_pub = nh.advertise<my_msgs::Control>("control_topic", 16); //Это мы публикуем структуру
    // ros::Subscriber HL_Body_sub = nh.subscribe("body_topic",16, message_callback);  // Это мы подписываемся на то что публигует Главная нода для Body
    ros::Rate r(RATE); // Частота в Герцах - задержка

    wiringPiSetup(); // Инициализация библиотеки
    set_PIN_Led();   // Устанавливаем и обьявляем пины. для вывода анализатора светодиодов и всего прочего
    init_SPI(SPI_CHANNAL,SPI_SPEED);      // Инициализация нужного канала SPI


    while (ros::ok())
    {
        Led_Blink(PIN_LED_GREEN, 1000); // Мигание светодиодом, что цикл работает

        Collect_DataHL_Control(); //Собираем рабочие данные в структуру для передачи считывая из топиков

        // DataHL_Body.command = 444666;
        // DataHL_Body.napravlenie = 16;
        // DataHL_Body.radius = 0.6666;
        // DataHL_Body.speed = 9993.66;
        // DataHL_Body.time = 999966;
        DataHL_Control.id += 1;
        DataHL_Control.gaz1_data += 3.14;
        DataHL_Control.gaz2_data += 1.11;
        DataHL_Control.bmp280.humidity += 1.11;
        DataHL_Control.bmp280.pressure += 2.01;
        DataHL_Control.bmp280.temperature += 7.2;
        //DataHL_Control.cheksum = 777;

        bool rez_data = sendData2Iot(SPI_CHANNAL, Control, DataHL_Control); //
        //printf(" Send Data to Control... %d %f \n", countbed, (float)countbed / countgood);

        if (rez_data) // Если пришли хорошие данные то обрабатываем их и публикуем данные в ROS
        {
            digitalWrite(PIN_LED_RED, 0); // Гасим светодиод пришли хорошие данные
            //ROS_INFO("Data ok! ");
            dataProcessing_Control(); // Обрабатываем данные
            str_pub.publish(msg); //Публикация полученных данных
            ros::spinOnce(); // Обновление в данных в ядре ROS
        }

        if (!rez_data) // Если пришли плохие данные то выводим ошибку
        {
            digitalWrite(PIN_LED_RED, 1); // Включаем светодиод пришли плохие данные
            ROS_INFO(" Flag_bedData chek_sum BED %d %f \n");
        }
        //printData();
        r.sleep(); // Интеллектуальная задержка на указанную частоту
    }
    return 0;
}
