#include <ros/ros.h>
#include <my_msgs/Body.h>
#include <my_msgs/Command.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>

#include "MyKalman.h"
#include "config.h"
#include "code.h"

int main(int argc, char **argv)
{
    printf(" -------------------------------------------------\n");
    printf(" START SetUp Module HighLevel Raspberry Pi 4B !!! \n");
    printf(" -------------------------------------------------\n");

    ros::init(argc, argv, "body_node");
    ros::NodeHandle nh;
    ros::Publisher str_pub = nh.advertise<my_msgs::Body>("body_topic", 16);                    //Это мы публикуем структуру
    ros::Subscriber command_sub = nh.subscribe("command_topic", 16, message_callback_Command); // Это мы подписываемся на то что публигует Главная нода для Body
    ros::Rate r(RATE);                                                                         // Частота в Герцах - задержка

    wiringPiSetup();                  // Инициализация библиотеки
    set_PIN_Led();                    // Устанавливаем и обьявляем пины. для вывода анализатора светодиодов и всего прочего
    init_SPI(SPI_CHANNAL, SPI_SPEED); // Инициализация нужного канала SPI

   // laser.setParametr1 (1, 0.1);
    laser.setParametr2 (5, 0.1);
   // uzi.setParametr1 (0.5, 0.1);
    uzi.setParametr2 (5, 0.1);


    while (ros::ok())
    {
        Led_Blink(PIN_LED_GREEN, 1000); // Мигание светодиодом, что цикл работает
        ros::spinOnce();       // Обновление в данных в ядре ROS, по этой команде происходит вызов функции обратного вызова
        Collect_Command();     //Собираем рабочие данные в структуру для передачи считывая данный из топиков
        printCommand();
        bool rez_data = SendData_in_Body(SPI_CHANNAL, stru_body_receive, stru_body_send); //  Отправляем данные на нижний уровень
        //printf(" Send Data to Body... %d %f \n", countbed, (float)countbed / countgood);

        if (rez_data) // Если пришли хорошие данные то обрабатываем их и публикуем данные в ROS
        {
            digitalWrite(PIN_LED_BLUE, 0); // Гасим светодиод пришли хорошие данные
            printData();
            //ROS_INFO("Data ok! ");
            dataProcessing_Body();          // Обрабатываем данные
            str_pub.publish(msg_body_send); //Публикация полученных данных
            ros::spinOnce();                // Обновление в данных в ядре ROS
        }

        if (!rez_data) // Если пришли плохие данные то выводим ошибку
        {
            digitalWrite(PIN_LED_BLUE, 1); // Включаем светодиод пришли плохие данные
            printData();
            // ROS_INFO(" Flag_bedData_Body chek_sum BED %d %f \n");
            printf(" chek_sum BED \n");
        }
        r.sleep(); // Интеллектуальная задержка на указанную частоту
    }
    return 0;
}
