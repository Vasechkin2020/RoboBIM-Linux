
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>

#include <data/Struct_Joy.h>
//#include <data/point.h>
#include <data/Struct_Data2Driver.h>
#include <data/Struct_Driver2Data.h>
#include <data/PillarOut.h>

#include "head_code/config.h"
// pos_struct position; // Обьявляем переменную для позиции машинки

#include "head_code/car.h"

CCar car; // Обьявляем экземпляр класса в нем вся обработка и обсчет машинки как обьекта

#include "head_code/c_joy.h"
CJoy joy(MAX_SPEED, 0.5); // Обьявляем экземпляр класса в нем вся обработка джойстика

#include <data/topicPillar.h>
#include "head_code/pillar.h"
CPillar pillar; // Обьявляем экземпляр класса в нем вся обработка и обсчет столбов


data::topicPillar msg_pillar;               // Перемеенная в которую сохраняем данные лидара из сообщения
geometry_msgs::Pose2D msg_car;                        // Перемеенная в которую сохраняем данные о координатах машинки начальных
sensor_msgs::Joy msg_joy;                   // Переменная в которую записываем пришедшее сообщение а колбеке
sensor_msgs::LaserScan::ConstPtr msg_lidar; // Перемеенная в которую сохраняем данные лидара из сообщения
data::Struct_Driver2Data msg_Driver2Data;   // Сообщение которое считываем из топика

data::Struct_Joy joy2Head_msg;  // Структура в которую пишем обработанные данные от джойстика и потом публикуем в топик
data::Struct_Joy joy2Head_prev; // Структура в которую пишем обработанные данные от джойстика предыдущее состоние

data::PillarOut pillar_out_msg;               // Перемеенная в которую пишем данные для опубликования в топик

bool flag_msgJoy = false;    // Флаг что пришло сообщение в топик и можно его парсить
bool flag_msgPillar = false; // Флаг что пришло сообщение в топик и можно его парсить
bool flag_msgLidar = false;  // Флаг что пришло сообщение в топик и можно его парсить
bool flag_msgCar = false;    // Флаг что пришло сообщение в топик и можно его парсить

bool flag_dataPillar = false; // Флаг что разобрали данные по координатам столбов и можно обсчитывать дальше
bool flag_dataCar = false;    // Флаг что разобрали данные по координатам машины и можно обсчитывать дальше
bool flag_dataLidar = false;  // Флаг что разобрали данные по лидару и можно сопоставлять столбы

data::Struct_Data2Driver Data2Driver;      // Структура с командами которую публикуем и которую потом Driver исполняет
data::Struct_Data2Driver Data2Driver_prev; // Структура с командами которую публикуем и которую потом Driver исполняет предыдущее состоние

#include "head_code/code.h"

int main(int argc, char **argv)
{
    ROS_INFO("%s -------------------------------------------------------------", NN);
    ROS_WARN("%s        Main Module PrintBIM ROS 1.0 Raspberry Pi 4B  ver 1.12 ", NN);
    ROS_ERROR("%s ------------------ROS_ERROR----------------------------------", NN);

    ros::init(argc, argv, "head_node");

    ros::NodeHandle nh;

    ros::Subscriber subscriber_Lidar = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, callback_Lidar);
    ros::Subscriber subscriber_Pillar = nh.subscribe<data::topicPillar>("pbPillar", 1000, callback_Pillar);
    ros::Subscriber subscriber_Car = nh.subscribe<geometry_msgs::Pose2D>("pbCar", 1000, callback_Car);
    ros::Subscriber subscriber_Joy = nh.subscribe("joy", 16, callback_Joy); // Это мы подписываемся на то что публикует нода джойстика
    ros::Subscriber subscriber_Driver = nh.subscribe<data::Struct_Driver2Data>("Driver2Data", 1000, callback_Driver);

    ros::Publisher publisher_Joy2Head = nh.advertise<data::Struct_Joy>("Joy2Head", 16);             // Это мы публикуем структуру которую сформировали по данным с джойстика
    ros::Publisher publisher_Head2Driver = nh.advertise<data::Struct_Data2Driver>("Head2Data", 16); // Это мы публикуем структуру которую отправляем к исполнению на драйвер
    ros::Publisher publisher_PillarOut = nh.advertise<data::PillarOut>("pbPillarOut", 16); // Это мы публикуем итоговую информацию по столбам

    ros::Rate r(RATE);        // Частота в Герцах - задержка
    ros::Duration(1).sleep(); // Подождем пока все обьявится и инициализируется внутри ROS

    while (ros::ok())
    {
        ros::spinOnce(); // Опрашиваем ядро ROS и по этой команде наши срабатывают колбеки. Нужно только для подписки на топики
        printf("+ \n");
        if (flag_msgCar)
        {
            flag_msgCar = false;
            car.parsingTopicCar(msg_car);
            flag_dataCar = true;
            // pillar.parsingPillar(msg_car); // Разбираем пришедшие данные и ищем там столбы.
        }
        if (flag_msgPillar)
        {
            flag_msgPillar = false;
            pillar.parsingPillar(msg_pillar); // Разбираем пришедшие данные Заполняем массив правильных координат.
            flag_dataPillar = true;
        }

        if (flag_msgLidar && flag_dataCar) // Если пришло сообщение в топик от лидара и мы уже разобрали данные по координатам машинки
        {
            flag_msgLidar = false;
            pillar.parsingLidar(msg_lidar); // Разбираем пришедшие данные и ищем там столбы.
            flag_dataLidar = true;
        }

        if (flag_dataPillar && flag_dataLidar) // Если поступили данные и мы их разобрали по истинным координатоам столбов и есть данные по столюам с лидара то начинаем сопоставлять и публиковать сопоставленные столбы
        {
            flag_dataLidar = false;
            pillar.comparisonPillar(); // Сопоставляем столбы
            formationPillar(); // Формируем перемнную с собщением для публикации
            publisher_PillarOut.publish(pillar_out_msg); // Публикуем информацию по столбам
        }

        if (flag_msgJoy) // Если пришло новое сообшение и сработал колбек то разбираем что там пришло
        {
            joy2Head_msg = joy.parsingJoy(msg_joy); // Разбираем и формируем команды из полученного сообщения
            flag_msgJoy = false;
            publisher_Joy2Head.publish(joy2Head_msg);                                   // Публикация полученных данных для информации
            Data2Driver = joy.transform(joy2Head_msg, joy2Head_prev, Data2Driver_prev); // Преобразование кнопок джойстика в реальные команды
        }
        // Data2Driver = speedCorrect(Data2Driver);                                // Корректировка скорости движения в зависимости от растояния до преграды
        // publisher_Head2Driver.publish(Data2Driver); // Публикация сформированных данных структуры управления для исполнения драйвером

        // pillar.pillar[2].azimuth
        // collectCommand();             // Формируем команду на основе полученных данных
        // com_pub.publish(Command_msg); //Публикация данных команды

        // ros::spinOnce(); // Опрашиваем ядро ROS и по этой команде наши срабатывают колбеки. Нужно только для подписки на топики
        r.sleep(); // Интеллектуальная задержка на указанную частоту
    }

    return 0;
}