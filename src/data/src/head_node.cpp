
#include "head_code/config.h"
#include "head_code/c_joy.h"

// #include "head_code/car.h"
// CCar car; // Обьявляем экземпляр класса в нем вся обработка и обсчет машинки как обьекта

SPoseLidar g_poseLidar; // Позиции лидара по расчетам Центральная система координат

#include "head_code/laser.h"
#include "head_code/pillar.h"
#include "head_code/topic.h" // Файл для функций для формирования топиков в нужном виде и формате
#include "head_code/code.h"



int main(int argc, char **argv)
{
    ROS_INFO("%s -------------------------------------------------------------", NN);
    ROS_WARN("%s        Main Module PrintBIM ROS 1.0 Raspberry Pi 4B  ver 1.132 ", NN);
    ROS_ERROR("%s ------------------ROS_ERROR----------------------------------", NN);

    CJoy joy(MAX_SPEED, 0.5); // Обьявляем экземпляр класса в нем вся обработка джойстика
    CLaser laser;
    CPillar pillar; // Обьявляем экземпляр класса в нем вся обработка и обсчет столбов

    ros::init(argc, argv, "head_node");
    // topic.init(argc, argv);

    ros::NodeHandle nh;

    //----------------------------- ПОДПИСКИ НА ТОПИКИ -------НЕ УБИРАЮ В КЛАСС ТАК КАК НУЖНЫ ГЛОБАЛЬНЫЕ КОЛБЕКИ И ПРОЧАЯ ХЕРНЯ --------
    ros::Subscriber subscriber_Lidar = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, callback_Lidar);
    ros::Subscriber subscriber_Pillar = nh.subscribe<data::topicPillar>("pbPillar", 1000, callback_Pillar);
    ros::Subscriber subscriber_StartPose2D = nh.subscribe<geometry_msgs::Pose2D>("pbStartPose2D", 1000, callback_StartPose2D);
    ros::Subscriber subscriber_Joy = nh.subscribe("joy", 16, callback_Joy); // Это мы подписываемся на то что публикует нода джойстика
    ros::Subscriber subscriber_Driver = nh.subscribe<data::Struct_Driver2Data>("Driver2Data", 1000, callback_Driver);
    //---------------------------------------------------------------------------------------------------------------------------
    CTopic topic; // Экземпляр класса для всех публикуемых топиков

    ros::Rate r(RATE);        // Частота в Герцах - задержка
    ros::Duration(1).sleep(); // Подождем пока все обьявится и инициализируется внутри ROS

    while (ros::ok())
    {
        // testFunction();
        ros::spinOnce(); // Опрашиваем ядро ROS и по этой команде наши срабатывают колбеки. Нужно только для подписки на топики
        printf("+ \n");

        if (flag_msgCar) // Флаг что пришло сообщение о начальных координатах машинки
        {
            flag_msgCar = false;
            startPosition(msg_startPose2d); // Определяем начальное положение
            flag_dataCar = true;
        }
        if (flag_msgPillar) // Флаг что пришло сообщение о истинных координатах столбов
        {
            flag_msgPillar = false;
            pillar.parsingPillar(msg_pillar); // Разбираем пришедшие данные Заполняем массив правильных координат.
            flag_dataPillar = true;
        }

        if (flag_msgLidar && flag_dataCar) // Если пришло сообщение в топик от лидара и мы уже разобрали данные по координатам машинки, а значит можем грубо посчитать где стоят столбы.
        {
            flag_msgLidar = false;
            pillar.parsingLidar(msg_lidar); // Разбираем пришедшие данные и ищем там столбы.
            flag_dataLidar = true;
        }
        ROS_INFO("flag_dataPillar= %i flag_dataLidar= %i", flag_dataPillar, flag_dataLidar);

        if (flag_dataPillar && flag_dataLidar) // Если поступили данные и мы их разобрали по истинным координатоам столбов и есть данные по столюам с лидара то начинаем сопоставлять и публиковать сопоставленные столбы
        {
            flag_dataLidar = false;
            pillar.comparisonPillar();                                      // Сопоставляем столбы
            g_poseLidar.mode1 = pillar.getLocationMode1(g_poseLidar.mode1); // Считаем текущие координаты по столбам На вход старая позиция лидара, на выходе новая позиция лидара
            g_poseLidar.mode2 = pillar.getLocationMode2(g_poseLidar.mode2); // Считаем текущие координаты по столбам На вход старая позиция лидара, на выходе новая позиция лидара
            laser.calcAnglePillarForLaser(pillar.pillar, g_poseLidar);      // Расчет углов в локальной системе лазеров на столбы для передачи на нижний уровень для исполнения
            
            topic.transform(laser);         // Публикуем трансформации систем координат
            topic.visualPillarAll(pillar);   // Формируем перемнную с собщением для публикации
            topic.visualPillarPoint(pillar); // Формируем перемнную с собщением для публикации

            topic.visulStartPose(); // Формируем перемнную с собщением для публикации

            topic.visualPoseLidarAll(); // Формируем перемнную с собщением для публикации
            topic.visualPoseLidarMode();

            topic.visualAngleLaser(laser);     // Формируем перемнную с собщением для публикации
            topic.visualPoseAngleLaser(laser); // Формируем перемнную с собщением для публикации
        }

        // !!!!!!!!!!!! Сделать поиск столба если он приходится на нос, так чтобы перебирала снова пока не найдет конец столба. Искать только если стоб начался и не закончился

        // if (flag_msgJoy) // Если пришло новое сообшение и сработал колбек то разбираем что там пришло
        // {
        //     joy2Head_msg = joy.parsingJoy(msg_joy); // Разбираем и формируем команды из полученного сообщения
        //     flag_msgJoy = false;
        //     //publisher_Joy2Head.publish(joy2Head_msg);                                   // Публикация полученных данных для информации
        //     Data2Driver = joy.transform(joy2Head_msg, joy2Head_prev, Data2Driver_prev); // Преобразование кнопок джойстика в реальные команды
        // }
        // Data2Driver = speedCorrect(topic.msg_Driver2Data, Data2Driver);                                // Корректировка скорости движения в зависимости от растояния до преграды
        // publisher_Head2Driver.publish(Data2Driver); // Публикация сформированных данных структуры управления для исполнения драйвером

        // pillar.pillar[2].azimuth
        // collectCommand();             // Формируем команду на основе полученных данных
        // com_pub.publish(Command_msg); //Публикация данных команды

        // ros::spinOnce(); // Опрашиваем ядро ROS и по этой команде наши срабатывают колбеки. Нужно только для подписки на топики
        r.sleep(); // Интеллектуальная задержка на указанную частоту
    }

    return 0;
}