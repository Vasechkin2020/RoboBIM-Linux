
#include "genStruct.h" // Тут все общие структуры. Истользуются и Data и Main и Head
#include "pos_code/config.h"

float g_angleMPU = 0;  // Глобальная перемнная угла получаемого с MPU куда смотрим
float g_angleLaser[4]; // Углы на столбы которые передаем на нижний угол для управления
int g_numPillar[4];    // Номер столба до которого измеряем расстояние лазером

SPoseRotation g_poseRotation; // Глобальная позиция в точке вращения робота. Все считается к ней.Она основная во всем. Расчет для лидарной точки mode1.2.3 потом пересчитывается в g_PoseRotation
SPoseLidar g_poseLidar;       // Позиции лидара по расчетам Центральная система координат
SLinAngVel g_linAngVel;       // Линейная и угловая скорость полученная с колес и с датчика MPU IMU и может быть обьединенная как-то
SEuler g_angleEuler;          // Углы Эллера

#include "pos_code/laser.h"
CLaser laser;

#include "pos_code/pillar.h"
CPillar pillar; // Обьявляем экземпляр класса в нем вся обработка и обсчет столбов

#include "pos_code/topic.h" // Файл для функций для формирования топиков в нужном виде и формате

// #include "pos_code/dataNode.h"
// CDataNode dataNode; // Экземпляр класса для всех данных получаемых с ноды Data  с нижнего уровня

#include "pos_code/code.h"

#include "pos_code/kalman.h"
CKalman kalman11;
CKalman kalman12;
CKalman kalman13;

int main(int argc, char **argv)
{
    log4cxx::MDC::put("node", "|pos_node|"); // Установка дополнительных данных в контекст

    ROS_FATAL("\n");
    ROS_FATAL("***  pos_node *** ver. 1.55 *** printBIM.ru *** 2025 ***");
    ROS_FATAL("--------------------------------------------------------\n");

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

    initKalman(); // Задаем коэфициенты для Калмана

    readParam(); // Считывание переменных параметров из лаунч файла при запуске. Там офсеты и режимы работы

    startPosition(msg_startPose2d); // Определяем начальное положение

    pillar.parsingPillar(msg_pillar); // Разбираем пришедшие данные Заполняем массив правильных координат.

    ros::Rate r(RATE);        // Частота в Герцах - задержка
    ros::Duration(2).sleep(); // Подождем пока все обьявится и инициализируется внутри ROS

    ROS_WARN("++++ End Setup. Start loop.");
    while (ros::ok())
    {  
        // testFunction();
        ros::spinOnce(); // Опрашиваем ядро ROS и по этой команде наши срабатывают колбеки. Нужно только для подписки на топики
        ROS_INFO("-------------------------------------------------------------------------");

        // 100 Hz ************************************************************ ОБРАБОТКА ДАННЫХ ИЗ ТОПИКОВ ЧТО ПОДПИСАНЫ  СРАБАТЫВАЕТ КАК ОТПРАВЛЯЕТ DATA_NODE  ********************************************
        if (flag_msgSpeed) // Флаг что пришло сообщение от ноды Data по Speed. Расчитываем линейную и угловую скорость и потом на нее основе расчитываем остальное
        {
            ROS_INFO("--------------------------------------- flag_msgSpeed ***");
            flag_msgSpeed = false;
            calcEuler();         // Расчет угла yaw с датчика IMU
            // calcAngleAccelGyr(); // Не стал пока делать. Расчет самостоятельно угла на основании данных гироскопа и аксельрометра
            calcLinAngVel();     // РАсчет линейных и угловой скоростей на основаниие данных скоростей колес и скоростей с IMU 055 и их комплементация в united
            
            // MODE 0
            g_poseRotation.mode0 = calcNewOdom(g_poseRotation.mode0, g_linAngVel.wheel, "mode 0",1);     // На основе линейных скоростей считаем новую позицию и угол по колесам
            g_poseLidar.mode0 = convertRotation2Lidar(g_poseRotation.mode0, "mode 0"); // Эти данные mode10 используем как основную точку для расчета mode1.2.3
            
            // MODE 10
            g_poseRotation.mode10 = calcNewOdom(g_poseRotation.mode10, g_linAngVel.united, "mode10",1); // На основе линейных скоростей считаем новую позицию и угол по колесам
            g_poseRotation.mode10.th = DEG2RAD(g_angleEuler.yaw); // Напрямую присваиваем угол. Заменяем тот угол что насчитали внутри 
            // ROS_INFO("    g_poseRotation mode10 x = %.3f y = %.3f theta = %.3f (radian)", g_poseRotation.mode10.x, g_poseRotation.mode10.y, g_poseRotation.mode10.th);
            g_poseLidar.mode10 = convertRotation2Lidar(g_poseRotation.mode10, "mode10"); // Эти данные mode10 используем как основную точку для расчета mode1.2.3
            

            // g_poseRotation.mode99 = calcNewOdom(g_poseRotation.mode10, g_linAngVel.united, "mode99",5); // На основе линейных скоростей считаем новую позицию и угол по колесам
            // g_poseLidar.mode99 = convertRotation2Lidar(g_poseRotation.mode0, "mode99"); // Эти данные mode10 используем как основную точку для расчета mode1.2.3
            // ROS_INFO("    g_poseLidar    mode10 x = %.3f y = %.3f | theta = %.3f gradus %.4f rad ", g_poseLidar.mode10.x, g_poseLidar.mode10.y, g_poseLidar.mode10.th, DEG2RAD(g_poseLidar.mode10.th));

            // calcMode0(); // Расчет одометрии Mode0
            // calcMode11();  // На основе линейных скоростей считаем новую позицию и угол для одометрии 100 Герц считаем и потом 10 Герц правим            // calcMode12();  // На основе линейных скоростей считаем новую позицию и угол для одометрии 100 Герц считаем и потом 10 Герц правим
            // calcMode13();  // На основе линейных скоростей считаем новую позицию и угол для одометрии 100 Герц считаем и потом 10 Герц правим
            // calcMode123(); // Комплементация Odom10// Комплементация положения и угла
            
            // РАСЧЕТ НАПРАВЛЕНИЯ УГЛОВ ЛАЗЕРОВ
            // Тут поддумать как можно предсказывать угол поворота. Например учитывая угловую и линейные скорости считать вперед и поворачивать нс учетом будующей позиции переделать на вывод в переменную а не изменение имеющейся глобальной внутри
            laser.calcAnglePillarForLaser(pillar.pillar, g_poseLidar.mode0); // Расчет углов в локальной системе лазеров на столбы для передачи на нижний уровень для исполнения
            topic.publicationControlModul();                                  // Формируем и Публикуем команды для управления Modul

            // topic.visualPublishOdomMode_3();                                  // Отобращение стрелкой где начало и куда смотрит в Mode3
            // topic.publicationAngleLaser(laser);                               // Формируем перемнную с собщением для публикации

            // void angleMPU(); // Расчет угла положения на основе данных сдатчика MPU
            // topic.publishOdomMpu();
        }
        /*
       // 100 Hz ************************************************************ ОБРАБОТКА ДАННЫХ ИЗ ТОПИКОВ ЧТО ПОДПИСАНЫ  СРАБАТЫВАЕТ КАК ОТПРАВЛЯЕТ DATA_NODE  ********************************************
       if (flag_msgModul) // Флаг что пришло сообщение от ноды Data по Modul. Тут пишем какую-то обработку данных если нужно. Это будет MODE_3
       {
           flag_msgModul = false;
           ROS_INFO("--------------------------------------- flag_msgModul ***");
           rateLaserData(); // Расчет частоты изменения данных с лазеров
           laser.calcPointPillarFromLaser(pillar.pillar);                 // Расчитываем Расстояние до столбов в /Odom/ системе с учетом offset

           //  !!!!!!!!!!!!!!! тут надо смотреть что данные пришли от измерения куда надо, а не куда попало любые данные тспользуем только правильные датчики, а те что не те или не дали сигнал в расчетах не учитывать
           // Тут смотреть что в моменты переключения лазеров от столба к столбу значения не верные и не надо считать в эти моменты сравнивать угол откуда нужны данные и откуда пришли.
           // Менее 2 градусов разница должна быть и то это много...
           pillar.getLocationMode3(g_poseLidar.mode3, g_poseLidar.mode10); // Считаем текущие координаты по столбам На вход позиция лидара общая скомплементированная, на выходе новая позиция лидара по расчету

           // odomMode13.pose.x = kalman13.calcX(g_poseLidar.mode3.x, odomMode13.pose.x); // Фильр Калмана для координаты Х. На вход подаем измеренное значение по MODE1  и вычисленное значение по модели ускорение на время плюс старое знаение
           // odomMode13.pose.y = kalman13.calcY(g_poseLidar.mode3.y, odomMode13.pose.y); // Фильр Калмана для координаты Х. На вход подаем измеренное значение по MODE1  и вычисленное значение по модели ускорение на время плюс старое знаение

           if (isnan(g_poseLidar.mode3.x) || isnan(g_poseLidar.mode3.y) || isnan(g_poseLidar.mode3.th))
           {
               ROS_ERROR("STOP MODE 3");
               exit(3);
           }
       }
        */
        // if (!modeColibrovka) // Тут исполняется основной режим, не колибровка // ДОБАВИТЬ ФИЛЬТР КАЛМАНА для покааний С МОДЕЛЬЮ, так как знаем что окружность известного диаметра и расстояние до столюа
        // {
        // }
        // else // Тут делаем колибровку лазерами туда-сюда
        // {
        //     startColibrovka(topic); // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Сделать определние начального угла Theta при запуске путем сканирования лазерами туда-сюда и нахождения минимальной точки и вычисления угла на основе локальных углов
        // }

        /*
                // Выполняется 10 Hz как ЛИДАР ПРИШЛЕТ ***************************************************************************************************************************************************
                if (flag_msgLidar) // Если пришло сообщение в топик от лидара и мы уже разобрали данные по координатам машинки, а значит можем грубо посчитать где стоят столбы.  И знаем где истинные столбы
                {
                    ROS_INFO("--------------------------------------- flag_msgLidar ***");
                    flag_msgLidar = false;
                    pillar.parsingLidar(msg_lidar, g_poseLidar.mode0); // Разбираем пришедшие данные и ищем там столбы.
                    pillar.comparisonPillar();                         // Сопоставляем столбы
                    // topic.publicationPillarAll(pillar);                // Публикуем всю обобщенную информацию по столб

                    pillar.getLocationMode1(g_poseLidar.mode1, g_poseLidar.mode0); // Считаем текущие координаты по столбам На вход старая позиция лидара, на выходе новая позиция лидара
                    pillar.getLocationMode2(g_poseLidar.mode2, g_poseLidar.mode1); // Считаем текущие координаты по столбам На вход старая позиция лидара, на выходе новая позиция лидара

                    // odomMode11.pose.x = kalman11.calcX(g_poseLidar.mode1.x, odomMode11.pose.x); // Фильр Калмана для координаты Х. На вход подаем измеренное значение по MODE1  и вычисленное значение по модели ускорение на время плюс старое знаение
                    // odomMode11.pose.y = kalman11.calcY(g_poseLidar.mode1.y, odomMode11.pose.y); // Фильр Калмана для координаты Y. На вход подаем измеренное значение по MODE1  и вычисленное значение по модели ускорение на время плюс старое знаение

                    // odomMode12.pose.x = kalman12.calcX(g_poseLidar.mode2.x, odomMode12.pose.x); // Фильр Калмана для координаты Х. На вход подаем измеренное значение по MODE1  и вычисленное значение по модели ускорение на время плюс старое знаение
                    // odomMode12.pose.y = kalman12.calcY(g_poseLidar.mode2.y, odomMode12.pose.y); // Фильр Калмана для координаты Y. На вход подаем измеренное значение по MODE1  и вычисленное значение по модели ускорение на время плюс старое знаение

                    // topic.visualPoseLidarMode_1_2();                                // Отобращение стрелкой где начало и куда смотрит в Mode0 1 2
                    topic.visualPublishOdomMode_1(); // Отобращение стрелкой где начало и куда смотрит в Mode0 1 2
                    topic.visualPublishOdomMode_2(); // Отобращение стрелкой где начало и куда смотрит в Mode0 1 2

                    if (isnan(g_poseLidar.mode2.x) || isnan(g_poseLidar.mode2.y) || isnan(g_poseLidar.mode2.th))
                    {
                        ROS_ERROR("STOP MODE 1-2");
                        exit(0);
                    }
                }
                else // Если не пришло новых данных в этом цикле то значит просто корректируем предыдущий расчет на величину угловых и линейных скорстей полученых с одометрии и INU
                {
                }

        */
        // g_poseRotation.mode10 = g_poseRotation.mode0; // Времено. ПОТОМ ТУТ НАДО ИТОГОВУЮ КОМПЛЕМЕНТАЦИЮ СДЕЛАТЬ
        // Тут строка перевода в g_poseLidaк.mode10 для использовании  в расчетаз в следущей итерации

        // Публикуем тут так как если один раз опубликовать то они исчезают через некоторое время.
        topic.transformBase(g_poseLidar.mode0); // Публикуем трансформации систем координат, задаем по какому расчету трансформировать
        topic.transformLidar();                  // Публикуем трансформации систем координат , задаем по какому расчету трансформировать
        topic.transformLaser(laser);             // Публикуем трансформации систем координат, задаем по какому расчету трансформировать
        topic.transformRotation();               // Публикуем трансформации систем координат

        topic.visualStartPose();           // Отобращение стрелкой где начало стартовой позиции и куда направлен нос платформы
        topic.visualPillarPoint(pillar);   // Отображение места размещения столбов
        topic.visualPoseAngleLaser(laser); // Отобращение стрелкой где начало и куда смотрят лазеры

        topic.publicationPoseLidar();     // Публикуем все варианты расчета позиций mode 0.1.2.3.4
        topic.publicationPoseRotattion(); // Публикуем все варианты расчета позиций mode 0.1.2.3.4
        topic.publicationLinAngVel();     // Вывод в топик данных с данными угловой и линейной скоростью

        // topic.publishOdomUnited();              // Публикация одометрии по моторам с корректировкой с верхнего уровня

        // topic.visualPublishOdomMode_0(); // Публикация одометрии по моторам которая получается от начальной точки
        // topic.visualPublishOdomMode_11(); // Публикация одометрии по моторам которая получается от начальной точки
        // topic.visualPublishOdomMode_12(); // Публикация одометрии по моторам которая получается от начальной точки
        // topic.visualPublishOdomMode_13(); // Публикация одометрии по моторам которая получается от начальной точки
        // topic.visualPublishOdomMode_123(); // Публикация одометрии по моторам которая получается от начальной точки

        static u_int64_t timeMil = millis();
        if (timeMil <= millis())
        {
            // ROS_WARN("mode0.x= %.3f y= %.3f th= %.2f", g_poseLidar.mode0.x, g_poseLidar.mode0.y, g_poseLidar.mode0.th);
            // ROS_WARN("mode1.x= %.3f y= %.3f th= %.2f", g_poseLidar.mode1.x, g_poseLidar.mode1.y, g_poseLidar.mode1.th);
            // ROS_WARN("mode2.x= %.3f y= %.3f th= %.2f", g_poseLidar.mode2.x, g_poseLidar.mode2.y, g_poseLidar.mode2.th);
            // ROS_WARN("mode3.x= %.3f y= %.3f th= %.2f", g_poseLidar.mode3.x, g_poseLidar.mode3.y, g_poseLidar.mode3.th);

            // ROS_WARN("angle= %.3f numPillar = %i | angle= %.3f numPillar = %i | angle= %.3f numPillar = %i | angle= %.3f numPillar = %i",
            //          g_angleLaser[0], g_numPillar[0], g_angleLaser[1], g_numPillar[1], g_angleLaser[2], g_numPillar[2], g_angleLaser[3], g_numPillar[3]);
            timeMil = millis() + 1000;
        }
        r.sleep(); // Интеллектуальная задержка на указанную частоту
        // ros::spinOnce(); // Опрашиваем ядро ROS и по этой команде наши срабатывают колбеки. Нужно только для подписки на топики
    }

    printf("pos_node STOP \n");
    return 0;
}