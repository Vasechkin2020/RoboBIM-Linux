
#include "head_code/config.h"
#include "head_code/low.h"

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

    CLaser laser;
    CPillar pillar; // Обьявляем экземпляр класса в нем вся обработка и обсчет столбов

    ros::init(argc, argv, "head_node");
    // topic.init(argc, argv);
    ros::NodeHandle nh;

    //----------------------------- ПОДПИСКИ НА ТОПИКИ -------НЕ УБИРАЮ В КЛАСС ТАК КАК НУЖНЫ ГЛОБАЛЬНЫЕ КОЛБЕКИ И ПРОЧАЯ ХЕРНЯ --------
    ros::Subscriber subscriber_Lidar = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, callback_Lidar);
    ros::Subscriber subscriber_Pillar = nh.subscribe<data::topicPillar>("pbPillar", 1000, callback_Pillar);
    ros::Subscriber subscriber_StartPose2D = nh.subscribe<geometry_msgs::Pose2D>("pbStartPose2D", 1000, callback_StartPose2D);
    ros::Subscriber subscriber_Driver = nh.subscribe<data::SDriver2Data>("pbData/Driver", 1000, callback_Driver);
    //---------------------------------------------------------------------------------------------------------------------------
    CTopic topic; // Экземпляр класса для всех публикуемых топиков
    CLow low; // Экземпляр класса для всех данных получаемых сноды Data  с нижнего уровня

    ros::Rate r(RATE);        // Частота в Герцах - задержка
    ros::Duration(1).sleep(); // Подождем пока все обьявится и инициализируется внутри ROS

    while (ros::ok())
    {
        // testFunction();
        ros::spinOnce(); // Опрашиваем ядро ROS и по этой команде наши срабатывают колбеки. Нужно только для подписки на топики
        printf("+ \n");

        if (flag_msgDriver) // Флаг что пришло сообщение от ноды Data по Driver
        {
            flag_msgDriver = false;
            low.parsingDriver(msg_Driver2Data);
        }

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
            pillar.parsingLidar(msg_lidar, g_poseLidar.mode1); // Разбираем пришедшие данные и ищем там столбы.
            flag_dataLidar = true;
        }
        ROS_INFO("flag_dataPillar= %i flag_dataLidar= %i", flag_dataPillar, flag_dataLidar);

        if (flag_dataPillar && flag_dataLidar) // Если поступили данные и мы их разобрали по истинным координатоам столбов и есть данные по столюам с лидара то начинаем сопоставлять и публиковать сопоставленные столбы
        {
            flag_dataLidar = false;
            pillar.comparisonPillar();                                      // Сопоставляем столбы
            g_poseLidar.mode1 = pillar.getLocationMode1(g_poseLidar.mode1); // Считаем текущие координаты по столбам На вход старая позиция лидара, на выходе новая позиция лидара
            g_poseLidar.mode2 = pillar.getLocationMode2(g_poseLidar.mode2); // Считаем текущие координаты по столбам На вход старая позиция лидара, на выходе новая позиция лидара

            // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Сделать определние начального угла Theta при запуске путем сканирования лазерами туда-сюда и нахождения минимальной точки и вычисления угла на основе локальных углов

            // topic.transform(laser, g_poseLidar.mode1); // Публикуем трансформации систем координат, задаем по какому расчету трансформировать "odom" в "base"
            // topic.visulStartPose();                    // Формируем перемнную с собщением для публикации
            //  topic.visualPillarAll(pillar);   // Формируем перемнную с собщением для публикации
            // topic.visualPillarPoint(pillar); // Формируем перемнную с собщением для публикации

            // topic.visualPoseLidarAll(); // Формируем перемнную с собщением для публикации
            // topic.visualPoseLidarMode();

            // laser.calcAnglePillarForLaser(pillar.pillar, g_poseLidar);      // Расчет углов в локальной системе лазеров на столбы для передачи на нижний уровень для исполнения
            // topic.visualAngleLaser(laser);     // Формируем перемнную с собщением для публикации
            // topic.visualPoseAngleLaser(laser); // Формируем перемнную с собщением для публикации
        }

        // collectCommand();             // Формируем команду на основе полученных данных
        // com_pub.publish(Command_msg); //Публикация данных команды

        // ros::spinOnce(); // Опрашиваем ядро ROS и по этой команде наши срабатывают колбеки. Нужно только для подписки на топики
        r.sleep(); // Интеллектуальная задержка на указанную частоту
    }

    return 0;
}