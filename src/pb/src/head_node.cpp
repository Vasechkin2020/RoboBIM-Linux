float g_angleLaser[4]; // Углы на столбы которые акпкдвкм нв нижний угол для управления
int g_numPillar[4];    // Номр столба до которого измеряем расстояние лазером

#include "genStruct.h" // Тут все общие структуры. Истользуются и Data и Main и Head
#include "head_code/laser.h"
CLaser laser;

#include "head_code/config.h"
#include "head_code/dataNode.h"

// #include "head_code/car.h"
// CCar car; // Обьявляем экземпляр класса в нем вся обработка и обсчет машинки как обьекта

SPoseLidar g_poseLidar; // Позиции лидара по расчетам Центральная система координат

#include "head_code/pillar.h"
CPillar pillar; // Обьявляем экземпляр класса в нем вся обработка и обсчет столбов

#include "head_code/topic.h" // Файл для функций для формирования топиков в нужном виде и формате
#include "head_code/code.h"

int main(int argc, char **argv)
{
    ROS_INFO("%s -------------------------------------------------------------", NN);
    ROS_WARN("%s        Main Module PrintBIM ROS 1.0 Raspberry Pi 4B  ver 1.1344 ", NN);
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
    CTopic topic;       // Экземпляр класса для всех публикуемых топиков
    CDataNode dataNode; // Экземпляр класса для всех данных получаемых с ноды Data  с нижнего уровня
    // g_poseLidar.mode3 = pillar.getLocationMode3(g_poseLidar.mode3); // Считаем текущие координаты по столбам На вход старая позиция лидара, на выходе новая позиция лидара

    initArray();

    ros::Rate r(RATE);        // Частота в Герцах - задержка
    ros::Duration(2).sleep(); // Подождем пока все обьявится и инициализируется внутри ROS

    while (ros::ok())
    {
        // testFunction();
        ros::spinOnce(); // Опрашиваем ядро ROS и по этой команде наши срабатывают колбеки. Нужно только для подписки на топики

        printf("%u --- \n\n\n", millis());

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
            printf("parsingLidar in... \n");
            pillar.parsingLidar(msg_lidar, g_poseLidar.mode1); // Разбираем пришедшие данные и ищем там столбы.
            pillar.comparisonPillar();                         // Сопоставляем столбы
            // topic.publicationPillarAll(pillar);                // Публикуем всю обобщенную информацию по столб

            g_poseLidar.mode1 = pillar.getLocationMode1(g_poseLidar.mode1); // Считаем текущие координаты по столбам На вход старая позиция лидара, на выходе новая позиция лидара
            // pillar.getLocationMode1(g_poseLidar.mode1); // Считаем текущие координаты по столбам На вход старая позиция лидара, на выходе новая позиция лидара
            g_poseLidar.mode2 = pillar.getLocationMode2(g_poseLidar.mode1); // Считаем текущие координаты по столбам На вход старая позиция лидара, на выходе новая позиция лидара
            // pillar.getLocationMode2(g_poseLidar.mode1); // Считаем текущие координаты по столбам На вход старая позиция лидара, на выходе новая позиция лидара
            // topic.visualPoseLidarMode_1_2();                                // Отобращение стрелкой где начало и куда смотрит в Mode0 1 2
            topic.visualPublishOdomMode_1(); // Отобращение стрелкой где начало и куда смотрит в Mode0 1 2
            topic.visualPublishOdomMode_2(); // Отобращение стрелкой где начало и куда смотрит в Mode0 1 2
        }

        // 25 Hz ************************************************************ ОБРАБОТКА ДАННЫХ ИЗ ТОПИКОВ ЧТО ПОДПИСАНЫ  СРАБАТЫВАЕТ КАК ОТПРАВЛЯЕТ DATA_NODE  ********************************************
        if (flag_msgModul) // Флаг что пришло сообщение от ноды Data по Modul. Тут пишем какую-то обработку данных если нужно. Это будет MODE_3
        {
            flag_msgModul = false;
            // printf("++++++++++ data from msg_Modul2Data in ++++++++ \n");
            // for (int i = 0; i < 4; i++)
            //     printf("distance = % .3f angle = % .3f numPillar = %i \n", msg_Modul2Data.laser[i].distance, msg_Modul2Data.laser[i].angle, msg_Modul2Data.laser[i].numPillar);
            // printf("+++ \n");

            // topic.publicationAngleLaser(laser); // Формируем перемнную с собщением для публикации

            // laser.calcPointPillarFromLaser(pillar.pillar); // Расчитываем кординаты столбов а лидарной системе по дистанции и углу с лазеров на Modul
            // g_poseLidar.mode3 = pillar.getLocationMode3(g_poseLidar.mode1); // Считаем текущие координаты по столбам На вход старая позиция лидара, на выходе новая позиция лидара
        }

        if (flag_msgDriver) // Флаг что пришло сообщение от ноды Data по Driver. Тут пишем какую-то обработку данных если нужно.
        {
            flag_msgDriver = false;
            // printf("msg_Driver2Data in... \n");
            //  dataNode.parsingDriver(msg_Driver2Data);
        }
        //-------------------------------
        if (flag_msgSpeed) // Флаг что пришло сообщение от ноды Data по Speed. Это будет MODE_0
        {
            printf("1 RAD2DEG(odomWheel.pose.th) = % .3f \n", RAD2DEG(odomWheel.pose.th));
            flag_msgSpeed = false;
            wheelTwistDt = calcTwistFromWheel(msg_Speed); // Обработка пришедших данных. По ним считаем линейные скорости по осям и угловую по углу. Запоминаем dt
            calcNewOdom(odomWheel, wheelTwistDt);         // На основе линейных скоростей считаем новую позицию и угол
            g_poseLidar.mode0.x = odomWheel.pose.x;
            g_poseLidar.mode0.y = odomWheel.pose.y;
            printf("2 RAD2DEG(odomWheel.pose.th) = % .3f \n", RAD2DEG(odomWheel.pose.th));
            g_poseLidar.mode0.th = RAD2DEG(odomWheel.pose.th);
            topic.visualPublishOdomMode_0(); // Публикация одометрии по моторам которая получается от начальной точки
            //---------------
            g_poseLidar.mode10.x = (g_poseLidar.mode0.x + g_poseLidar.mode1.x + g_poseLidar.mode2.x) / 3.0;
            g_poseLidar.mode10.y = (g_poseLidar.mode0.y + g_poseLidar.mode1.y + g_poseLidar.mode2.y) / 3.0;
            g_poseLidar.mode10.th = (g_poseLidar.mode0.th + g_poseLidar.mode1.th + g_poseLidar.mode2.th) / 3.0;
            // g_poseLidar.mode10.x = (g_poseLidar.mode0.x + g_poseLidar.mode1.x) / 2.0;
            // g_poseLidar.mode10.y = (g_poseLidar.mode0.y + g_poseLidar.mode1.y) / 2.0;
            // g_poseLidar.mode10.th = (g_poseLidar.mode0.th + g_poseLidar.mode1.th) / 2.0;

            // mpuTwistDt = calcTwistFromMpu(Driver2Data.bno055, 0.2); // Расчет и оформление в структуру ускорений по осям (линейных скоростей) и  разделить получение угловых скоростей и расчет сновой точки на основе этих скоростей
            // calcNewOdom(odomMpu, mpuTwistDt);                       // Обработка пришедших данных.Обсчитываем одометрию по датчику MPU BNO055
            // topic.publishOdomMpu();

            // // тут написать функцию комплементации данных угловых скоростей с разными условиями когда и в каком соотношении скомплементировать скорсти с двух источников
            // unitedTwistDt = calcTwistUnited(wheelTwistDt, mpuTwistDt);
            // calcNewOdom(odomUnited, unitedTwistDt); // // На основе линейных скоростей считаем новую позицию и угол
            // topic.publishOdomUnited();              // Публикация одометрии по моторам с корректировкой с верхнего уровня
            //-------------------------
        }

        if (flag_startPose && flag_dataPillar) // Если уже разобрали данные по координатам машинки, а значит можем грубо посчитать направление на столбы.  И знаем где истинные столбы
        {
            laser.calcAnglePillarForLaser(pillar.pillar, g_poseLidar.mode10); // Расчет углов в локальной системе лазеров на столбы для передачи на нижний уровень для исполнения
            topic.publicationControlModul();                                  // Формируем и Публикуем команды для управления Modul

            // Публикуем тут так как если один раз опубликовать то они исчезают через некоторое время.
            topic.transformBase(g_poseLidar.mode10); // Публикуем трансформации систем координат, задаем по какому расчету трансформировать
            topic.transformLidar(g_poseLidar.mode1); // Публикуем трансформации систем координат, задаем по какому расчету трансформировать
            topic.transformLaser(laser);             // Публикуем трансформации систем координат, задаем по какому расчету трансформировать
            topic.visulStartPose();                  // Отобращение стрелкой где начало стартовой позиции и куда направлен нос платформы
            topic.visualPillarPoint(pillar);         // Отображение места размещения столбов
            topic.visualPoseAngleLaser(laser);       // Отобращение стрелкой где начало и куда смотрят лазеры
        }
        //-------------------------------------------------------------------------
        topic.publicationPoseLidarAll(); // Публикуем все варианты расчета позиций mode 0.1.2.3.4

        // topic.publicationControlDriver(); // Формируем и Публикуем команды для управления Driver
        topic.publicationControlPrint(); // Формируем и Публикуем команды для управления Print

        r.sleep(); // Интеллектуальная задержка на указанную частоту
        // ros::spinOnce(); // Опрашиваем ядро ROS и по этой команде наши срабатывают колбеки. Нужно только для подписки на топики
    }

    return 0;
}