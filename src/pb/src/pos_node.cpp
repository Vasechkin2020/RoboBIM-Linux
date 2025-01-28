
#include "genStruct.h" // Тут все общие структуры. Истользуются и Data и Main и Head

float g_angleMPU = 0;  // Глобальная перемнная угла получаемого с MPU куда смотрим
float g_angleLaser[4]; // Углы на столбы которые передаем на нижний угол для управления
int g_numPillar[4];    // Номер столба до которого измеряем расстояние лазером

#include "pos_code/laser.h"
CLaser laser;

#include "pos_code/config.h"

SPoseLidar g_poseLidar; // Позиции лидара по расчетам Центральная система координат
// #include "pos_code/car.h"
// CCar car; // Обьявляем экземпляр класса в нем вся обработка и обсчет машинки как обьекта

#include "pos_code/pillar.h"
CPillar pillar; // Обьявляем экземпляр класса в нем вся обработка и обсчет столбов

#include "pos_code/topic.h" // Файл для функций для формирования топиков в нужном виде и формате

#include "pos_code/dataNode.h"
CDataNode dataNode; // Экземпляр класса для всех данных получаемых с ноды Data  с нижнего уровня
#include "pos_code/code.h"

#include "pos_code/kalman.h"
CKalman kalman11;
CKalman kalman12;
CKalman kalman13;

int main(int argc, char **argv)
{
    // ROS_INFO("%s -------------------------------------------------------------", NN);
    // ROS_WARN("%s        Main Module PrintBIM ROS 1.0 Raspberry Pi 4B  ver 1.55 ", NN);
    // ROS_ERROR("%s ------------------ROS_ERROR----------------------------------", NN);

    ROS_INFO("%s --------------------------------------------------------", NN);
    ROS_WARN("%s *** Pos_Node *** ver. 1.44 *** printBIM.ru *** 2025 ***", NN);
    ROS_INFO("%s --------------------------------------------------------", NN);

    ros::init(argc, argv, "pos_node");
    // topic.init(argc, argv);
    ros::NodeHandle nh;
    CTopic topic; // Экземпляр класса для всех публикуемых топиков

    //----------------------------- ПОДПИСКИ НА ТОПИКИ -------НЕ УБИРАЮ В КЛАСС ТАК КАК НУЖНЫ ГЛОБАЛЬНЫЕ КОЛБЕКИ И ПРОЧАЯ ХЕРНЯ --------
    ros::Subscriber subscriber_Lidar = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, callback_Lidar);
    ros::Subscriber subscriber_Modul = nh.subscribe<pb_msgs::Struct_Modul2Data>("pbData/Modul", 1000, callback_Modul);
    ros::Subscriber subscriber_Speed = nh.subscribe<pb_msgs::SSetSpeed>("pbData/Speed", 1000, callback_Speed);

    // ros::Subscriber subscriber_Pillar = nh.subscribe<pb_msgs::topicPillar>("pbStart/Pillar", 1000, callback_Pillar);
    // ros::Subscriber subscriber_StartPose2D = nh.subscribe<geometry_msgs::Pose2D>("pbStart/Pose2D", 1000, callback_StartPose2D);
    // ros::Subscriber subscriber_Driver = nh.subscribe<pb_msgs::Struct_Driver2Data>("pbData/Driver", 1000, callback_Driver);

    //---------------------------------------------------------------------------------------------------------------------------
    // g_poseLidar.mode3 = pillar.getLocationMode3(g_poseLidar.mode3); // Считаем текущие координаты по столбам На вход старая позиция лидара, на выходе новая позиция лидара

    initArray();

    initKalman(); // Задаем коэфициенты для Калмана

    readParam();                      // Считывание переменных параметров из лаунч файла при запуске. Там офсеты и режимы работы
    startPosition(msg_startPose2d);   // Определяем начальное положение
    pillar.parsingPillar(msg_pillar); // Разбираем пришедшие данные Заполняем массив правильных координат.

    ros::Rate r(RATE);        // Частота в Герцах - задержка
    ros::Duration(2).sleep(); // Подождем пока все обьявится и инициализируется внутри ROS

    while (ros::ok())
    {
        // testFunction();
        ros::spinOnce(); // Опрашиваем ядро ROS и по этой команде наши срабатывают колбеки. Нужно только для подписки на топики
                         // printf("\n");

        //  Выполняется 1 РАЗ как получили ***************************************************************************************************************************************************
        // if (flag_msgStartPose) // Флаг что пришло сообщение о начальных координатах машинки
        // {
        //     flag_msgStartPose = false;
        //     printf("startPosition in... \n");
        //     startPosition(msg_startPose2d); // Определяем начальное положение
        //     flag_startPose = true;
        // }

        // if (flag_msgPillar) // Флаг что пришло сообщение о истинных координатах столбов
        // {
        //     flag_msgPillar = false;
        //     printf("parsingPillar in... \n");
        //     pillar.parsingPillar(msg_pillar); // Разбираем пришедшие данные Заполняем массив правильных координат.
        //     flag_dataPillar = true;
        // }

        // if (flag_startPose && flag_dataPillar) //  Основной алгоритм только после того как загрузили начальную позицию и положение столбов
        // {

        if (flag_msgSpeed) // Флаг что пришло сообщение от ноды Data по Speed. Расчитываем линейную и угловую скорость и потом на нее основе расчитываем MODE0 MODE11 MODE12 MODE13 как первую часть Калмана по модели.
        {
            flag_msgSpeed = false;
            g_linAngVel.wheel = calcTwistFromWheel(msg_Speed); // Обработка пришедших данных. По ним считаем линейные скорости по осям и угловую по углу. Запоминаем dt

            calcMode0();   // Расчет одометрии Mode0
            // calcMode11();  // На основе линейных скоростей считаем новую позицию и угол для одометрии 100 Герц считаем и потом 10 Герц правим
            // calcMode12();  // На основе линейных скоростей считаем новую позицию и угол для одометрии 100 Герц считаем и потом 10 Герц правим
            // calcMode13();  // На основе линейных скоростей считаем новую позицию и угол для одометрии 100 Герц считаем и потом 10 Герц правим
            calcMode123(); // Комплементация Odom10// Комплементация положения и угла

            topic.visualPublishOdomMode_0();  // Публикация одометрии по моторам которая получается от начальной точки
            // topic.visualPublishOdomMode_11(); // Публикация одометрии по моторам которая получается от начальной точки
            // topic.visualPublishOdomMode_12(); // Публикация одометрии по моторам которая получается от начальной точки
            // topic.visualPublishOdomMode_13(); // Публикация одометрии по моторам которая получается от начальной точки

            // Этот расчет перенести в Speed? что бы 100 раз управляли моторами лазеров в не когда от них данные пришлт ??????
            // Скорсоть цикла увеличить до 200 - 1000 герц
            laser.calcAnglePillarForLaser(pillar.pillar, g_poseLidar.mode1); // Расчет углов в локальной системе лазеров на столбы для передачи на нижний уровень для исполнения
            static pb_msgs::Struct_Data2Modul dataControlModul2;
            dataControlModul2.controlLaser.mode = 1;
            dataControlModul2.controlMotor.mode = 1;
            // Поворачиваем на этот угол
            dataControlModul2.controlMotor.angle[0] = g_angleLaser[0];
            dataControlModul2.controlMotor.numPillar[0] = g_numPillar[0];
            dataControlModul2.controlMotor.angle[1] = g_angleLaser[1];
            dataControlModul2.controlMotor.numPillar[1] = g_numPillar[1];
            dataControlModul2.controlMotor.angle[2] = g_angleLaser[2];
            dataControlModul2.controlMotor.numPillar[2] = g_numPillar[2];
            dataControlModul2.controlMotor.angle[3] = g_angleLaser[3];
            dataControlModul2.controlMotor.numPillar[3] = g_numPillar[3];

            topic.publicationControlModul(dataControlModul2); // Формируем и Публикуем команды для управления Modul
        }
        // Считаем 200 Hz Каждый раз цикла

        // Выполняется 10 Hz как ЛИДАР ПРИШЛЕТ ***************************************************************************************************************************************************
        if (flag_msgLidar) // Если пришло сообщение в топик от лидара и мы уже разобрали данные по координатам машинки, а значит можем грубо посчитать где стоят столбы.  И знаем где истинные столбы
        {
            flag_msgLidar = false;
            printf("\n");
            pillar.parsingLidar(msg_lidar, g_poseLidar.mode1); // Разбираем пришедшие данные и ищем там столбы.
            pillar.comparisonPillar();                         // Сопоставляем столбы
            // topic.publicationPillarAll(pillar);                // Публикуем всю обобщенную информацию по столб

            pillar.getLocationMode1(g_poseLidar.mode1, g_poseLidar.mode1); // Считаем текущие координаты по столбам На вход старая позиция лидара, на выходе новая позиция лидара
            pillar.getLocationMode2(g_poseLidar.mode2, g_poseLidar.mode1); // Считаем текущие координаты по столбам На вход старая позиция лидара, на выходе новая позиция лидара

            odomMode11.pose.x = kalman11.calcX(g_poseLidar.mode1.x, odomMode11.pose.x); // Фильр Калмана для координаты Х. На вход подаем измеренное значение по MODE1  и вычисленное значение по модели ускорение на время плюс старое знаение
            odomMode11.pose.y = kalman11.calcY(g_poseLidar.mode1.y, odomMode11.pose.y); // Фильр Калмана для координаты Y. На вход подаем измеренное значение по MODE1  и вычисленное значение по модели ускорение на время плюс старое знаение

            odomMode12.pose.x = kalman12.calcX(g_poseLidar.mode2.x, odomMode12.pose.x); // Фильр Калмана для координаты Х. На вход подаем измеренное значение по MODE1  и вычисленное значение по модели ускорение на время плюс старое знаение
            odomMode12.pose.y = kalman12.calcY(g_poseLidar.mode2.y, odomMode12.pose.y); // Фильр Калмана для координаты Y. На вход подаем измеренное значение по MODE1  и вычисленное значение по модели ускорение на время плюс старое знаение

            odomMode13.pose.x = kalman13.calcX(g_poseLidar.mode3.x, odomMode13.pose.x); // Фильр Калмана для координаты Х. На вход подаем измеренное значение по MODE1  и вычисленное значение по модели ускорение на время плюс старое знаение
            odomMode13.pose.y = kalman13.calcY(g_poseLidar.mode3.y, odomMode13.pose.y); // Фильр Калмана для координаты Х. На вход подаем измеренное значение по MODE1  и вычисленное значение по модели ускорение на время плюс старое знаение

            // topic.visualPoseLidarMode_1_2();                                // Отобращение стрелкой где начало и куда смотрит в Mode0 1 2
            topic.visualPublishOdomMode_1(); // Отобращение стрелкой где начало и куда смотрит в Mode0 1 2
            topic.visualPublishOdomMode_2(); // Отобращение стрелкой где начало и куда смотрит в Mode0 1 2

            if (isnan(g_poseLidar.mode2.x) || isnan(g_poseLidar.mode2.y) || isnan(g_poseLidar.mode2.th))
            {
                printf("STOP MODE 1-2 \n");
                exit(0);
            }
        }

        if (!modeColibrovka) // Тут исполняется основной режим, не колибровка // ДОБАВИТЬ ФИЛЬТР КАЛМАНА для покааний С МОДЕЛЬЮ, так как знаем что окружность известного диаметра и расстояние до столюа
        {
            // 100 Hz ************************************************************ ОБРАБОТКА ДАННЫХ ИЗ ТОПИКОВ ЧТО ПОДПИСАНЫ  СРАБАТЫВАЕТ КАК ОТПРАВЛЯЕТ DATA_NODE  ********************************************
            if (flag_msgModul) // Флаг что пришло сообщение от ноды Data по Modul. Тут пишем какую-то обработку данных если нужно. Это будет MODE_3
            {
                flag_msgModul = false;
                // Тут вывести разницу в положении угла куда должны быть направлены лазеры и из какого угла данные
                //  !!!!!!!!!!!!!!! тут надо считать наверное когда данные с лазеров меняются, а то 100 раз приходят от модуль, а данные реже обновляются тут надо смотреть что данные пришли от измерения куда надо, а не куда попало любые данные тспользуем только правильные датчики, а те что не те или не дали сигнал в расчетах не учитывать
                //   printf("++++++++++ data from msg_Modul2Data in ++++++++ \n");
                //   for (int i = 0; i < 4; i++)
                //       printf("distance = % .3f angle = % .3f numPillar = %i \n", msg_Modul2Data.laser[i].distance, msg_Modul2Data.laser[i].angle, msg_Modul2Data.laser[i].numPillar);
                //   printf("+++ \n");

                laser.calcPointPillarFromLaser(pillar.pillar);                 // Расчитываем Расстояние до столбов в /Odom/ системе
                pillar.getLocationMode3(g_poseLidar.mode3, g_poseLidar.mode1); // Считаем текущие координаты по столбам На вход старая позиция лидара, на выходе новая позиция лидара
                topic.visualPublishOdomMode_3();                               // Отобращение стрелкой где начало и куда смотрит в Mode3

                // topic.publicationAngleLaser(laser); // Формируем перемнную с собщением для публикации
                if (isnan(g_poseLidar.mode3.x) || isnan(g_poseLidar.mode3.y) || isnan(g_poseLidar.mode3.th))
                {
                    printf("STOP MODE 3 \n");
                    exit(3);
                }
            }
        }
        else // Тут делаем колибровку лазерами туда-сюда
        {
            startColibrovka(topic); // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Сделать определние начального угла Theta при запуске путем сканирования лазерами туда-сюда и нахождения минимальной точки и вычисления угла на основе локальных углов
        }

        // if (flag_msgDriver) // Флаг что пришло сообщение от ноды Data по Driver. Тут пишем какую-то обработку данных если нужно.
        // {
        //     flag_msgDriver = false;
        //     // angleMPU(); // Расчет угла положения на основе данных с датчика MPU
        // }

        // Публикуем тут так как если один раз опубликовать то они исчезают через некоторое время.
        topic.transformBase(g_poseLidar.mode1);  // Публикуем трансформации систем координат, задаем по какому расчету трансформировать
        topic.transformLidar(g_poseLidar.mode1); // Публикуем трансформации систем координат , задаем по какому расчету трансформировать
        topic.transformLaser(laser);             // Публикуем трансформации систем координат, задаем по какому расчету трансформировать
        topic.visulStartPose();                  // Отобращение стрелкой где начало стартовой позиции и куда направлен нос платформы
        topic.visualPillarPoint(pillar);         // Отображение места размещения столбов
        topic.visualPoseAngleLaser(laser);       // Отобращение стрелкой где начало и куда смотрят лазеры
        topic.publicationPoseLidarAll();         // Публикуем все варианты расчета позиций mode 0.1.2.3.4
        // }

        static u_int64_t timeMil = millis();
        if (timeMil < millis())
        {
            printf("%u \n", millis());
            printf(" mode0.x= %.3f y= %.3f th= %.2f /", g_poseLidar.mode0.x, g_poseLidar.mode0.y, g_poseLidar.mode0.th);
            printf(" mode1.x= %.3f y= %.3f th= %.2f /", g_poseLidar.mode1.x, g_poseLidar.mode1.y, g_poseLidar.mode1.th);
            printf(" mode2.x= %.3f y= %.3f th= %.2f /", g_poseLidar.mode2.x, g_poseLidar.mode2.y, g_poseLidar.mode2.th);
            printf(" mode3.x= %.3f y= %.3f th= %.2f /", g_poseLidar.mode3.x, g_poseLidar.mode3.y, g_poseLidar.mode3.th);
            printf("\n");
            for (int i = 0; i < 4; i++)
            {
                printf(" angle= %.3f numPillar = %i |", g_angleLaser[i], g_numPillar[i]);
            }
            printf("\n");

            timeMil = millis() + 1000;
        }
        r.sleep(); // Интеллектуальная задержка на указанную частоту
        // ros::spinOnce(); // Опрашиваем ядро ROS и по этой команде наши срабатывают колбеки. Нужно только для подписки на топики
    }

    printf("Main Node STOP \n");
    return 0;
}