
#include "genStruct.h" // Тут все общие структуры. Истользуются и Data и Main и Head
#include "pos_code/config.h"

float g_angleMPU = 0;  // Глобальная перемнная угла получаемого с MPU куда смотрим
float g_angleLaser[4]; // Углы на столбы которые передаем на нижний угол для управления
int g_numPillar[4];    // Номер столба до которого измеряем расстояние лазером

SPoseRotation g_poseRotation; // Глобальная позиция в точке вращения робота. Все считается к ней.Она основная во всем. Расчет для лидарной точки mode1.2.3 потом пересчитывается в g_PoseRotation
SPoseBase g_poseBase;         // Позиции лидара по расчетам Центральная система координат
SLinAngVel g_linAngVel;       // Линейная и угловая скорость полученная с колес и с датчика MPU IMU и может быть обьединенная как-то
SEuler g_angleEuler;          // Углы Эллера
SPose aver;                   // Усредненная позиция в моменты когда стоим на месте
uint32_t averCount = 0;       // Счетчик усреднений

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
    ros::NodeHandle nh;

    static ros::Time timeStart = ros::Time::now(); // Захватываем начальный момент времени
    static ros::Time timeNow = ros::Time::now();   // Захватываем конечный момент времени
    static ros::Time timeStop = ros::Time::now();  // Захватываем конечный момент времени

    CTopic topic; // Экземпляр класса для всех публикуемых топиков

    //----------------------------- ПОДПИСКИ НА ТОПИКИ -------НЕ УБИРАЮ В КЛАСС ТАК КАК НУЖНЫ ГЛОБАЛЬНЫЕ КОЛБЕКИ И ПРОЧАЯ ХЕРНЯ --------
    ros::Subscriber subscriber_Lidar = nh.subscribe<pb_msgs::Struct_PoseLidar>("pbLidar/PoseLidar", 1000, callback_Lidar);
    ros::Subscriber subscriber_Modul = nh.subscribe<pb_msgs::Struct_Modul2Data>("pbData/Modul", 1000, callback_Modul);
    ros::Subscriber subscriber_Speed = nh.subscribe<pb_msgs::SSetSpeed>("pbData/Speed", 1000, callback_Speed);
    //---------------------------------------------------------------------------------------------------------------------------

    initKalman(); // Задаем коэфициенты для Калмана

    readParam(); // Считывание переменных параметров из лаунч файла при запуске. Там офсеты и режимы работы

    startPosition(msg_startPose2d); // Определяем начальное положение

    pillar.parsingPillar(msg_pillar); // Разбираем пришедшие данные Заполняем массив правильных координат.

    ros::Rate r(200);         // Частота в Герцах - задержка
    ros::Duration(2).sleep(); // Подождем пока все обьявится и инициализируется внутри ROS
    static bool flagPublish = false;

    ROS_WARN("++++ End Setup. Start loop.");
    while (ros::ok())
    {
        timeNow = ros::Time::now(); // Захватываем текущий момент времени начала цикла
        // testFunction();
        ros::spinOnce(); // Опрашиваем ядро ROS и по этой команде наши срабатывают колбеки. Нужно только для подписки на топики
        // ROS_INFO_THROTTLE(RATE_OUTPUT,"----");

        // 100 Hz ************************************************************ ОБРАБОТКА ДАННЫХ ИЗ ТОПИКОВ ЧТО ПОДПИСАНЫ  СРАБАТЫВАЕТ КАК ОТПРАВЛЯЕТ DATA_NODE  ********************************************
        if (flag_msgSpeed) // Флаг что пришло сообщение от ноды Data по Speed. Расчитываем линейную и угловую скорость и потом на нее основе расчитываем остальное
        {
            ROS_INFO("--- flag_msgSpeed");
            flag_msgSpeed = false;
            flagPublish = true;
            timeStop = timeStopping(msg_Speed); // Расчет времени когда остановились. ЕСли движемся то выдаем текущее время. Если стоим то время когда остановились

            calcEuler(); // Расчет угла yaw с датчика IMU
            g_linAngVel.wheel = calcTwistFromWheel(msg_Speed);  // Обработка пришедших данных. По ним считаем линейные скорости по осям и угловую по углу. Запоминаем dt
            g_linAngVel.mpu = calcTwistFromMpu(msg_Modul2Data); // Обработка пришедших данных для расчета линейных и угловых скоростей

            // g_linAngVel.united = calcTwistUnited(g_linAngVel.wheel, g_linAngVel.mpu); // тут написать функцию комплементации данных угловых скоростей с разными условиями когда и в каком соотношении скомплементировать скорсти с двух источников
            // g_linAngVel.united = g_linAngVel.wheel; // Пока нет расчет по IMU и комплментации используем только по колесам


            g_poseRotation.mode0 = calcNewPose(g_poseRotation.mode0, g_linAngVel.wheel, "mode 0", 1);    // На основе линейных скоростей считаем новую позицию и угол по колесам
            g_poseRotation.mode10 = calcNewPose(g_poseRotation.mode10, g_linAngVel.wheel, "mode 10", 1); // На основе линейных скоростей считаем новую позицию и угол по колесам
            // g_poseRotation.mode10 = calcNewOdom2(g_poseRotation.mode10, g_linAngVel.united, "mode10"); // На основе линейных скоростей считаем новую позицию и угол по колесам
            // g_poseRotation.mode10.th = DEG2RAD(g_angleEuler.yaw); // Напрямую присваиваем угол. Заменяем тот угол что насчитали внутри
            // ROS_INFO("    g_poseRotation mode10 x = %.3f y = %.3f theta = %.3f (radian)", g_poseRotation.mode10.x, g_poseRotation.mode10.y, g_poseRotation.mode10.th);

            g_poseBase.mode0 = convertRotation2Base(g_poseRotation.mode0, "mode 0");    // Эти данные mode10 используем как основную точку для расчета mode1.2.3
            g_poseBase.mode10 = convertRotation2Base(g_poseRotation.mode10, "mode 10"); // Эти данные mode10 используем как основную точку для расчета mode1.2.3
            // g_poseBase.mode10 = convertRotation2Base(g_poseRotation.mode10, "mode10"); // Эти данные mode10 используем как основную точку для расчета mode1.2.3

            // calcMode0();   // Расчет одометрии Mode0
            // calcMode11();  // На основе линейных скоростей считаем новую позицию и угол для одометрии 100 Герц считаем и потом 10 Герц правим
            // calcMode12();  // На основе линейных скоростей считаем новую позицию и угол для одометрии 100 Герц считаем и потом 10 Герц правим
            // calcMode13();  // На основе линейных скоростей считаем новую позицию и угол для одометрии 100 Герц считаем и потом 10 Герц правим
            // calcMode123(); // Комплементация Odom10// Комплементация положения и угла

            // РАСЧЕТ НАПРАВЛЕНИЯ УГЛОВ ЛАЗЕРОВ
            laser.calcAnglePillarForLaser(pillar.pillar, g_poseBase.mode10); // Расчет углов в локальной системе лазеров на столбы для передачи на нижний уровень для исполнения
            topic.publicationControlModul();                                 // Формируем и Публикуем команды для управления Modul

            // topic.visualPublishOdomMode_3();                                  // Отобращение стрелкой где начало и куда смотрит в Mode3
            // topic.publicationAngleLaser(laser);                               // Формируем перемнную с собщением для публикации

            // angleMPU(); // Расчет угла положения на основе данных сдатчика MPU
            // topic.publishOdomMpu();
        }

        // Выполняется 10 Hz как ЛИДАР ПРИШЛЕТ ***************************************************************************************************************************************************
        if (flag_msgLidar) // Если пришло сообщение в топик от лидара и мы уже разобрали данные по координатам машинки, а значит можем грубо посчитать где стоят столбы.  И знаем где истинные столбы
        {
            //   ROS_INFO("--------------------------------------- flag_msgLidar ***");
            flag_msgLidar = false;
            ros::Duration durationStopping = ros::Time::now() - timeStop; // Находим разницу между началом и концом
            double dtStoping = durationStopping.toSec();                  // Получаем количество секунд

            ROS_INFO("---- IN Data PoseLidar x = %.3f y = %.3f th = %.3f | match = %i cross = %i | azimut %.3f %.3f %.3f %.3f | dtStoping = %f msec",
                     msg_lidar.mode.x, msg_lidar.mode.y, msg_lidar.mode.th,
                     msg_lidar.countMatchPillar, msg_lidar.countCrossCircle,
                     msg_lidar.azimut[0], msg_lidar.azimut[1], msg_lidar.azimut[2], msg_lidar.azimut[3], dtStoping * 1000);

            // ROS_INFO("    dtStoping = %f msec", dtStoping * 1000);

            if (dtStoping < 0.1) // Если меньше 0,2 секунды то возвращаем что есть
            {
                aver.x = msg_lidar.mode.x;
                aver.y = msg_lidar.mode.y;
                aver.th = msg_lidar.mode.th;
                averCount = 1;
            }
            else // Если прошло больше 0,2 секунды с момента остановки начинаем усреднять позицию
            {
                aver.x = aver.x + msg_lidar.mode.x;
                aver.y = aver.y + msg_lidar.mode.y;
                aver.th = aver.th + msg_lidar.mode.th;
                averCount++;
                // ROS_INFO("    averCount = %i aver.x = %.3f aver.x / averCount = %.3f", averCount, aver.x, aver.x / averCount);

                if (dtStoping > 0.6) // Если прошло больше 0,5 секунды с момента остановки тогда комплементируем позицию.
                {
                    aver.x = aver.x - (aver.x / averCount);
                    aver.y = aver.y - (aver.y / averCount);
                    aver.th = aver.th - (aver.th / averCount);
                    averCount--;
                    // ROS_INFO("    --- complementation Pose Average to Mode ---");
                    g_poseBase.mode10.x = aver.x / averCount;
                    g_poseBase.mode10.y = aver.y / averCount;
                    g_poseBase.mode10.th = aver.th / averCount;
                    g_poseRotation.mode10 = convertBase2Rotation(g_poseBase.mode10, "mode10");

                    g_poseBase.mode = g_poseBase.mode10;
                    ROS_INFO("    complementation g_poseBase.mode x= %.3f y= %.3f theta= %.3f grad", g_poseBase.mode.x, g_poseBase.mode.y, g_poseBase.mode.th);

                    // ROS_INFO("    complementation g_poseRotation.mode0 x= %.3f y= %.3f theta= %.3f grad", g_poseRotation.mode0.x, g_poseRotation.mode0.y, RAD2DEG(g_poseRotation.mode0.th));
                }
            }
            //     // odomMode11.pose.x = kalman11.calcX(g_poseBase.mode1.x, odomMode11.pose.x); // Фильр Калмана для координаты Х. На вход подаем измеренное значение по MODE1  и вычисленное значение по модели ускорение на время плюс старое знаение
            //     // odomMode11.pose.y = kalman11.calcY(g_poseBase.mode1.y, odomMode11.pose.y); // Фильр Калмана для координаты Y. На вход подаем измеренное значение по MODE1  и вычисленное значение по модели ускорение на время плюс старое знаение

            //     // odomMode12.pose.x = kalman12.calcX(g_poseBase.mode2.x, odomMode12.pose.x); // Фильр Калмана для координаты Х. На вход подаем измеренное значение по MODE1  и вычисленное значение по модели ускорение на время плюс старое знаение
            //     // odomMode12.pose.y = kalman12.calcY(g_poseBase.mode2.y, odomMode12.pose.y); // Фильр Калмана для координаты Y. На вход подаем измеренное значение по MODE1  и вычисленное значение по модели ускорение на время плюс старое знаение

            //     // topic.visualPoseLidarMode_1_2();                                // Отобращение стрелкой где начало и куда смотрит в Mode0 1 2
            //     topic.visualPublishOdomMode_1(); // Отобращение стрелкой где начало и куда смотрит в Mode0 1 2
            //     topic.visualPublishOdomMode_2(); // Отобращение стрелкой где начало и куда смотрит в Mode0 1 2

            //     if (isnan(g_poseBase.mode2.x) || isnan(g_poseBase.mode2.y) || isnan(g_poseBase.mode2.th))
            //     {
            //         ROS_ERROR("STOP MODE 1-2");
            //         exit(0);
            //     }
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
           pillar.getLocationMode3(g_poseBase.mode3, g_poseBase.mode10); // Считаем текущие координаты по столбам На вход позиция лидара общая скомплементированная, на выходе новая позиция лидара по расчету

           // odomMode13.pose.x = kalman13.calcX(g_poseBase.mode3.x, odomMode13.pose.x); // Фильр Калмана для координаты Х. На вход подаем измеренное значение по MODE1  и вычисленное значение по модели ускорение на время плюс старое знаение
           // odomMode13.pose.y = kalman13.calcY(g_poseBase.mode3.y, odomMode13.pose.y); // Фильр Калмана для координаты Х. На вход подаем измеренное значение по MODE1  и вычисленное значение по модели ускорение на время плюс старое знаение

           if (isnan(g_poseBase.mode3.x) || isnan(g_poseBase.mode3.y) || isnan(g_poseBase.mode3.th))
           {
               ROS_ERROR("STOP MODE 3");
               exit(3);
           }
       }
        //     startColibrovka(topic); // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Сделать определние начального угла Theta при запуске путем сканирования лазерами туда-сюда и нахождения минимальной точки и вычисления угла на основе локальных углов
        */
        // g_poseRotation.mode10 = g_poseRotation.mode0; // Времено. ПОТОМ ТУТ НАДО ИТОГОВУЮ КОМПЛЕМЕНТАЦИЮ СДЕЛАТЬ
        // Тут строка перевода в g_poseLidaк.mode10 для использовании  в расчетаз в следущей итерации

        if (flagPublish) //
        {
            flagPublish = false;
            topic.publicationPoseBase();      // Публикуем все варианты расчета позиций mode 0.1.2.3.4
            topic.publicationPoseRotattion(); // Публикуем все варианты расчета позиций mode 0.1.2.3.4
            topic.publicationLinAngVel();     // Вывод в топик данных с данными угловой и линейной скоростью
            // topic.publishOdomUnited();              // Публикация одометрии по моторам с корректировкой с верхнего уровня
        }

        static u_int64_t timeRviz = millis();
        if (timeRviz <= millis()) // 30 Hz
        {
            // Публикуем тут так как если один раз опубликовать то они исчезают через некоторое время.
            topic.transformBase(g_poseBase.mode0); // Публикуем трансформации систем координат, задаем по какому расчету трансформировать
            topic.transformLaser(laser);           // Публикуем трансформации систем координат, задаем по какому расчету трансформировать
            topic.transformRotation();             // Публикуем трансформации систем координат

            topic.visualStartPose();           // Отобращение стрелкой где начало стартовой позиции и куда направлен нос платформы
            topic.visualPillarPoint(pillar);   // Отображение места размещения столбов
            topic.visualPoseAngleLaser(laser); // Отобращение стрелкой где начало и куда смотрят лазеры

            // topic.visualPublishOdomMode_0(); // Публикация одометрии по моторам которая получается от начальной точки
            // topic.visualPublishOdomMode_11(); // Публикация одометрии по моторам которая получается от начальной точки
            // topic.visualPublishOdomMode_12(); // Публикация одометрии по моторам которая получается от начальной точки
            timeRviz = millis() + 33; // 30 Hz
        }

        static ros::Time timeMil = ros::Time::now();   // Захватываем начальный момент времени
        ros::Duration durationMil = timeNow - timeMil; // Находим разницу между началом и концом
        double dtMil = durationMil.toSec();            // Получаем количество секунд
        if (dtMil >= 1)
        {
            // ROS_INFO("%u --- %f ", millis(), dtMil);
            timeMil = ros::Time::now(); // Захватываем момент времени
        }
        timeCycle(timeStart, timeNow); // Выводим справочно время работы цикла и время с начала работы программы
        r.sleep();                     // Интеллектуальная задержка на указанную частоту
        // ros::spinOnce();            // Опрашиваем ядро ROS и по этой команде наши срабатывают колбеки. Нужно только для подписки на топики
    }

    printf("pos_node STOP \n");
    return 0;
}