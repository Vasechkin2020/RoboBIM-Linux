
#include "genStruct.h" // Тут все общие структуры. Истользуются и Data и Main и Head

float g_angleMPU = 0;   // Глобальная перемнная угла получаемого с MPU куда смотрим
float g_angleLaser[4];  // Углы на столбы которые акпкдвкм нв нижний угол для управления
int g_numPillar[4];     // Номр столба до которого измеряем расстояние лазером

#include "head_code/laser.h"
CLaser laser;

#include "head_code/config.h"

SPoseLidar g_poseLidar; // Позиции лидара по расчетам Центральная система координат
// #include "head_code/car.h"
// CCar car; // Обьявляем экземпляр класса в нем вся обработка и обсчет машинки как обьекта


#include "head_code/pillar.h"
CPillar pillar; // Обьявляем экземпляр класса в нем вся обработка и обсчет столбов

#include "head_code/topic.h" // Файл для функций для формирования топиков в нужном виде и формате
CTopic topic;                // Экземпляр класса для всех публикуемых топиков

#include "head_code/dataNode.h"
CDataNode dataNode; // Экземпляр класса для всех данных получаемых с ноды Data  с нижнего уровня
#include "head_code/code.h"

int main(int argc, char **argv)
{
    ROS_INFO("%s -------------------------------------------------------------", NN);
    ROS_WARN("%s        Main Module PrintBIM ROS 1.0 Raspberry Pi 4B  ver 1.1366 ", NN);
    ROS_ERROR("%s ------------------ROS_ERROR----------------------------------", NN);

    ros::init(argc, argv, "head_node");
    // topic.init(argc, argv);
    ros::NodeHandle nh;

    //----------------------------- ПОДПИСКИ НА ТОПИКИ -------НЕ УБИРАЮ В КЛАСС ТАК КАК НУЖНЫ ГЛОБАЛЬНЫЕ КОЛБЕКИ И ПРОЧАЯ ХЕРНЯ --------
    ros::Subscriber subscriber_Lidar = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, callback_Lidar);
    ros::Subscriber subscriber_Pillar = nh.subscribe<pb_msgs::topicPillar>("pbStart/Pillar", 1000, callback_Pillar);
    ros::Subscriber subscriber_StartPose2D = nh.subscribe<geometry_msgs::Pose2D>("pbStart/Pose2D", 1000, callback_StartPose2D);

    ros::Subscriber subscriber_Driver = nh.subscribe<pb_msgs::Struct_Driver2Data>("pbData/Driver", 1000, callback_Driver);
    ros::Subscriber subscriber_Modul = nh.subscribe<pb_msgs::Struct_Modul2Data>("pbData/Modul", 1000, callback_Modul);
    ros::Subscriber subscriber_Speed = nh.subscribe<pb_msgs::SSetSpeed>("pbData/Speed", 1000, callback_Speed);

    //---------------------------------------------------------------------------------------------------------------------------
    // g_poseLidar.mode3 = pillar.getLocationMode3(g_poseLidar.mode3); // Считаем текущие координаты по столбам На вход старая позиция лидара, на выходе новая позиция лидара

    initArray();

    ros::Rate r(RATE);        // Частота в Герцах - задержка
    ros::Duration(2).sleep(); // Подождем пока все обьявится и инициализируется внутри ROS

    while (ros::ok())
    {
        // testFunction();
        ros::spinOnce(); // Опрашиваем ядро ROS и по этой команде наши срабатывают колбеки. Нужно только для подписки на топики

        printf("\n");
        printf("%u --- \n", millis());

        //  1 РАЗ как получили ***************************************************************************************************************************************************
        if (flag_msgStartPose) // Флаг что пришло сообщение о начальных координатах машинки
        {
            flag_msgStartPose = false;
            printf("startPosition in... \n");
            startPosition(msg_startPose2d); // Определяем начальное положение
            flag_startPose = true;
        }
        if (flag_msgPillar) // Флаг что пришло сообщение о истинных координатах столбов
        {
            flag_msgPillar = false;
            printf("parsingPillar in... \n");
            pillar.parsingPillar(msg_pillar); // Разбираем пришедшие данные Заполняем массив правильных координат.
            flag_dataPillar = true;
        }
        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Сделать определние начального угла Theta при запуске путем сканирования лазерами туда-сюда и нахождения минимальной точки и вычисления угла на основе локальных углов

        // 10 Hz как ЛИДАР ПРИШЛЕТ ***************************************************************************************************************************************************

        if (flag_msgLidar && flag_startPose && flag_dataPillar) // Если пришло сообщение в топик от лидара и мы уже разобрали данные по координатам машинки, а значит можем грубо посчитать где стоят столбы.  И знаем где истинные столбы
        {
            flag_msgLidar = false;

            pillar.parsingLidar(msg_lidar, g_poseLidar.mode1); // Разбираем пришедшие данные и ищем там столбы.
            pillar.comparisonPillar();                         // Сопоставляем столбы
            // topic.publicationPillarAll(pillar);                // Публикуем всю обобщенную информацию по столб

            pillar.getLocationMode1(g_poseLidar.mode1, g_poseLidar.mode1); // Считаем текущие координаты по столбам На вход старая позиция лидара, на выходе новая позиция лидара
            pillar.getLocationMode2(g_poseLidar.mode2, g_poseLidar.mode1); // Считаем текущие координаты по столбам На вход старая позиция лидара, на выходе новая позиция лидара

            // topic.visualPoseLidarMode_1_2();                                // Отобращение стрелкой где начало и куда смотрит в Mode0 1 2
            topic.visualPublishOdomMode_1(); // Отобращение стрелкой где начало и куда смотрит в Mode0 1 2
            topic.visualPublishOdomMode_2(); // Отобращение стрелкой где начало и куда смотрит в Mode0 1 2

            if (isnan(g_poseLidar.mode2.x) || isnan(g_poseLidar.mode2.y) || isnan(g_poseLidar.mode2.th))
            {
                exit(0);
            }
            poseComplementMode10(); // Комплеиентация Odom10// Комплементация положения
        }

        // 25 Hz ************************************************************ ОБРАБОТКА ДАННЫХ ИЗ ТОПИКОВ ЧТО ПОДПИСАНЫ  СРАБАТЫВАЕТ КАК ОТПРАВЛЯЕТ DATA_NODE  ********************************************
        if (flag_msgModul) // Флаг что пришло сообщение от ноды Data по Modul. Тут пишем какую-то обработку данных если нужно. Это будет MODE_3
        {
            flag_msgModul = false;
            // printf("++++++++++ data from msg_Modul2Data in ++++++++ \n");
            // for (int i = 0; i < 4; i++)
            //     printf("distance = % .3f angle = % .3f numPillar = %i \n", msg_Modul2Data.laser[i].distance, msg_Modul2Data.laser[i].angle, msg_Modul2Data.laser[i].numPillar);
            // printf("+++ \n");
            laser.calcPointPillarFromLaser(pillar.pillar);                 // Расчитываем Расстояние до столбов в /Odom/ системе
            pillar.getLocationMode3(g_poseLidar.mode3, g_poseLidar.mode1); // Считаем текущие координаты по столбам На вход старая позиция лидара, на выходе новая позиция лидара
            topic.visualPublishOdomMode_3();                               // Отобращение стрелкой где начало и куда смотрит в Mode3

            // topic.publicationAngleLaser(laser); // Формируем перемнную с собщением для публикации
            if (isnan(g_poseLidar.mode3.x) || isnan(g_poseLidar.mode3.y) || isnan(g_poseLidar.mode3.th))
            {
                printf("STOP MODE 3 \n");
                exit(0);
            }
        }

        if (flag_msgDriver) // Флаг что пришло сообщение от ноды Data по Driver. Тут пишем какую-то обработку данных если нужно.
        {
            flag_msgDriver = false;
            angleMPU(); // Расчет угла положения на основе данных с датчика MPU
        }
        //-------------------------------
        if (flag_msgSpeed) // Флаг что пришло сообщение от ноды Data по Speed. Это будет MODE_0
        {
            flag_msgSpeed = false;
            calculationOdometry();           // Расчет одометрии и применения ее для всех режимов
        }

        if (flag_startPose && flag_dataPillar) // Если уже разобрали данные по координатам машинки, а значит можем грубо посчитать направление на столбы.  И знаем где истинные столбы
        {
            laser.calcAnglePillarForLaser(pillar.pillar, g_poseLidar.mode10); // Расчет углов в локальной системе лазеров на столбы для передачи на нижний уровень для исполнения
            topic.publicationControlModul();                                  // Формируем и Публикуем команды для управления Modul

            // Публикуем тут так как если один раз опубликовать то они исчезают через некоторое время.
            topic.transformBase(g_poseLidar.mode10);  // Публикуем трансформации систем координат, задаем по какому расчету трансформировать
            topic.transformLidar(g_poseLidar.mode10); // Публикуем трансформации систем координат, задаем по какому расчету трансформировать
            topic.transformLaser(laser);              // Публикуем трансформации систем координат, задаем по какому расчету трансформировать
            topic.visulStartPose();                   // Отобращение стрелкой где начало стартовой позиции и куда направлен нос платформы
            topic.visualPillarPoint(pillar);          // Отображение места размещения столбов
            topic.visualPoseAngleLaser(laser);        // Отобращение стрелкой где начало и куда смотрят лазеры
            // topic.visualPublishOdomMode_11();         // Отобращение стрелкой где начало и куда смотрит в Mode11 Это по Калману
        }
        //-------------------------------------------------------------------------
        topic.publicationPoseLidarAll(); // Публикуем все варианты расчета позиций mode 0.1.2.3.4

        // topic.publicationControlDriver(); // Формируем и Публикуем команды для управления Driver
        topic.publicationControlPrint(); // Формируем и Публикуем команды для управления Print

        r.sleep(); // Интеллектуальная задержка на указанную частоту
        // ros::spinOnce(); // Опрашиваем ядро ROS и по этой команде наши срабатывают колбеки. Нужно только для подписки на топики
    }

    printf("Main Node STOP \n");
    return 0;
}