
#include "genStruct.h" // Тут все общие структуры. Истользуются и Data и Main и Head

float g_angleMPU = 0;  // Глобальная перемнная угла получаемого с MPU куда смотрим
float g_angleLaser[4]; // Углы на столбы которые акпкдвкм нв нижний угол для управления
int g_numPillar[4];    // Номр столба до которого измеряем расстояние лазером

#include "head_code/laser.h"
CLaser laser;

#include "head_code/config.h"

SPoseLidar g_poseLidar; // Позиции лидара по расчетам Центральная система координат
// #include "head_code/car.h"
// CCar car; // Обьявляем экземпляр класса в нем вся обработка и обсчет машинки как обьекта

#include "head_code/pillar.h"
CPillar pillar; // Обьявляем экземпляр класса в нем вся обработка и обсчет столбов

#include "head_code/topic.h" // Файл для функций для формирования топиков в нужном виде и формате

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
    CTopic topic; // Экземпляр класса для всех публикуемых топиков

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

    u_int64_t time1Colibrovka = millis() + 5000;
    u_int64_t time2Colibrovka;
    u_int64_t time3Colibrovka;
    u_int64_t time4Colibrovka;
    u_int64_t time5Colibrovka;

    bool flag1Colibrovka = true;
    bool flag2Colibrovka = false;
    bool flag3Colibrovka = false;
    bool flag4Colibrovka = false;
    bool flag5Colibrovka = false;

    while (ros::ok())
    {
        // testFunction();
        ros::spinOnce(); // Опрашиваем ядро ROS и по этой команде наши срабатывают колбеки. Нужно только для подписки на топики

        // printf("\n");

        static u_int64_t timeMil = millis();
        if (timeMil < millis())
        {
            printf("%u --- \n", millis());
            timeMil = millis() + 1000;
        }

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
        if (flag_startPose && flag_dataPillar) //  Основной алгоритм только после того как загрузили начальную позицию и положение столбов
        {
            // 10 Hz как ЛИДАР ПРИШЛЕТ ***************************************************************************************************************************************************
            if (flag_msgLidar) // Если пришло сообщение в топик от лидара и мы уже разобрали данные по координатам машинки, а значит можем грубо посчитать где стоят столбы.  И знаем где истинные столбы
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
                    printf("STOP MODE 1-2 \n");
                    exit(0);
                }
                poseComplementMode10(); // Комплеиентация Odom10// Комплементация положения и угла
            }
            if (flag_msgDriver) // Флаг что пришло сообщение от ноды Data по Driver. Тут пишем какую-то обработку данных если нужно.
            {
                flag_msgDriver = false;
                // angleMPU(); // Расчет угла положения на основе данных с датчика MPU
            }
            //-------------------------------

            if (modeColibrovka) // Тут делаем колибровку лазерами туда-сюда // ДОБАВИТЬ ФИЛЬТР КАЛМАНА для покааний С МОДЕЛЬЮ, так как знаем что окружность известного диаметра и расстояние до столюа
            {
                startColibrovka();
                static pb_msgs::Struct_Data2Modul dataControlModul;
                struct SColibrData
                {
                    float angleL = 0;
                    float angleR = 0;
                    float distMin = 100;  // Дистанция что измерили
                    float angleMin = 0;   // Угол при котором измеряли
                    float distMin2 = 100; // Дистанция что измерили
                    float angleMin2 = 0;  // Угол при котором измеряли
                };

                static SColibrData colibrData[4]; // Массив со всеми переменными для расчета

                if (time1Colibrovka < millis() && flag1Colibrovka)
                {
                    //  Считаем угол на столбы и считаем на какой угол от него надо отклониться чтобы гарантированно отсканировать столб
                    laser.calcAnglePillarForLaser(pillar.pillar, g_poseLidar.mode10); // Расчет углов в локальной системе лазеров на столбы для передачи на нижний уровень для исполнения
                    // Включаем назерные датчики
                    dataControlModul.controlLaser.mode = 1;
                    dataControlModul.controlMotor.mode = 1;
                    // Поворачиваем на этот угол
                    dataControlModul.controlMotor.angle[0] = g_angleLaser[0];
                    dataControlModul.controlMotor.numPillar[0] = g_numPillar[0];
                    dataControlModul.controlMotor.angle[1] = g_angleLaser[1];
                    dataControlModul.controlMotor.numPillar[1] = g_numPillar[1];
                    dataControlModul.controlMotor.angle[2] = g_angleLaser[2];
                    dataControlModul.controlMotor.numPillar[2] = g_numPillar[2];
                    dataControlModul.controlMotor.angle[3] = g_angleLaser[3];
                    dataControlModul.controlMotor.numPillar[3] = g_numPillar[3];
                    topic.publicationControlModul(dataControlModul); // Формируем и Публикуем команды для управления Modul
                    flag1Colibrovka = false;
                    time2Colibrovka = millis() + 1000; // Начинаем 2 этап через 3 секунды
                    flag2Colibrovka = true;
                    printf("1 ETAP \n");
                }
                if (time2Colibrovka < millis() && flag2Colibrovka)
                {
                    printf("2 ETAP \n");
                    //==========================================================

                    for (int i = 0; i < 4; i++) // Перебираем моторы(лазеры)
                    {
                        SPoint a, b;
                        a.x = msg_startPose2d.x;
                        a.y = msg_startPose2d.y;
                        b.x = pillar.pillar[g_numPillar[i]].x_true;
                        b.y = pillar.pillar[g_numPillar[i]].y_true;
                        float len = vectorLen(a, b);
                        float angle = atan(PILLAR_RADIUS / len);
                        float angleL = g_angleLaser[i] - RAD2DEG(angle) - 0.1;
                        float angleR = g_angleLaser[i] + RAD2DEG(angle) + 0.1;
                        printf("len = % .3f angle= % .3f angleL= % .3f angleR= % .3f \n", len, RAD2DEG(angle), angleL, angleR);
                        colibrData[i].angleL = angleL;
                        colibrData[i].angleR = angleR;
                        dataControlModul.controlMotor.angle[i] = angleL;
                    }
                    topic.publicationControlModul(dataControlModul); // Формируем и Публикуем команды для управления Modul
                    printf("2 ETAP END\n");
                    flag2Colibrovka = false;
                    time3Colibrovka = millis() + 2500; // Начинаем 2 этап через 3 секунды
                    flag3Colibrovka = true;
                }
                //==========================================================
                if (time3Colibrovka < millis() && flag3Colibrovka)
                {
                    printf("3 ETAP START \n");
                    if (flag_msgModul) // Флаг что пришло сообщение от ноды Data по Modul. Тут пишем какую-то обработку данных если нужно.
                    {
                        flag_msgModul = false;
                        // Поворачиваемся на 0,1125 градус каждые 0,33 секунды и запоминаем показания в массив
                        for (int i = 0; i < 4; i++) // Перебираем моторы(лазеры)
                        {
                            printf("distance = % .3f angle = % .3f numPillar = %i \n", msg_Modul2Data.laser[i].distance, msg_Modul2Data.laser[i].angle, msg_Modul2Data.laser[i].numPillar);
                            if (msg_Modul2Data.laser[i].numPillar != -1) // Если пришли корректные значения от нужного столба. ПЕрвый раз почему-то приходят с -1
                            {
                                // Останавливаемся когда дойдем до другого угла
                                if (dataControlModul.controlMotor.angle[i] < colibrData[i].angleR)
                                {
                                    // dataControlModul.controlMotor.angle[i] += 0.1125; // 360 ГРАДУСОВ /400 шагов мотор /8 шагов драйвер
                                    // Могу измерять 3 Hz Могу поворачиваться на 0,11 значит за 1 секунду повернусь на 0,33 значит угол равен 0,33/Частоту работы RATE
                                    // dataControlModul.controlMotor.angle[i] += 0.0033; // 360 ГРАДУСОВ /400 шагов мотор /8 шагов драйвер
                                    float stepp = (RATE_LASER * STEP_LASER_MOTOR) / RATE;
                                    dataControlModul.controlMotor.angle[i] += (RATE_LASER * STEP_LASER_MOTOR) / RATE; // 360 ГРАДУСОВ /400 шагов мотор /16 шагов драйвер
                                    //  Смотри какое новое расстоние пришло, сравниваем со старым, ищем минимум и запоминаем угол с минимальным расстоянием
                                    if (colibrData[i].distMin > msg_Modul2Data.laser[i].distance)
                                    {
                                        colibrData[i].distMin = msg_Modul2Data.laser[i].distance; // Если новое измерение меньше то меняем раастояние
                                        colibrData[i].angleMin = msg_Modul2Data.laser[i].angle;
                                    }
                                    printf("distMin = %.3f angleMin = %.3f stepp= %f \n", colibrData[i].distMin, colibrData[i].angleMin, stepp);
                                }
                            }
                        }
                        topic.publicationControlModul(dataControlModul); // Формируем и Публикуем команды для управления Modul
                        printf("3 ETAP END \n");
                        if (dataControlModul.controlMotor.angle[0] >= colibrData[0].angleR && // ЕСли все углы отработали
                            dataControlModul.controlMotor.angle[1] >= colibrData[1].angleR &&
                            dataControlModul.controlMotor.angle[2] >= colibrData[2].angleR &&
                            dataControlModul.controlMotor.angle[3] >= colibrData[3].angleR)
                        {
                            flag3Colibrovka = false;
                            time4Colibrovka = millis() + 1000; // Начинаем 4 этап через 1 секунды
                            flag4Colibrovka = true;
                        }
                    }
                }
                //==========================================================
                if (time4Colibrovka < millis() && flag4Colibrovka)
                {
                    printf("4 ETAP START \n");
                    if (flag_msgModul) // Флаг что пришло сообщение от ноды Data по Modul. Тут пишем какую-то обработку данных если нужно.
                    {
                        flag_msgModul = false;
                        // Поворачиваемся на 0,1125 градус каждые 0,33 секунды и запоминаем показания в массив
                        for (int i = 0; i < 4; i++) // Перебираем моторы(лазеры)
                        {
                            printf("distance = % .3f angle = % .3f numPillar = %i \n", msg_Modul2Data.laser[i].distance, msg_Modul2Data.laser[i].angle, msg_Modul2Data.laser[i].numPillar);
                            if (msg_Modul2Data.laser[i].numPillar != -1) // Если пришли корректные значения от нужного столба. ПЕрвый раз почему-то приходят с -1
                            {
                                // Останавливаемся когда дойдем до другого угла
                                if (dataControlModul.controlMotor.angle[i] > colibrData[i].angleL)
                                {
                                    // dataControlModul.controlMotor.angle[i] += 0.1125; // 360 ГРАДУСОВ /400 шагов мотор /8 шагов драйвер
                                    // Могу измерять 3 Hz Могу поворачиваться на 0,11 значит за 1 секунду повернусь на 0,33 значит угол равен 0,33/Частоту работы RATE
                                    // dataControlModul.controlMotor.angle[i] += 0.0033; // 360 ГРАДУСОВ /400 шагов мотор /8 шагов драйвер
                                    float stepp = (RATE_LASER * STEP_LASER_MOTOR) / RATE;
                                    dataControlModul.controlMotor.angle[i] -= (RATE_LASER * STEP_LASER_MOTOR) / RATE; // 360 ГРАДУСОВ /400 шагов мотор /16 шагов драйвер
                                    //  Смотри какое новое расстоние пришло, сравниваем со старым, ищем минимум и запоминаем угол с минимальным расстоянием
                                    if (colibrData[i].distMin2 > msg_Modul2Data.laser[i].distance)
                                    {
                                        colibrData[i].distMin2 = msg_Modul2Data.laser[i].distance; // Если новое измерение меньше то меняем раастояние
                                        colibrData[i].angleMin2 = msg_Modul2Data.laser[i].angle;
                                    }
                                    printf("distMin2 = %.3f angleMin2 = %.3f stepp= %f \n", colibrData[i].distMin2, colibrData[i].angleMin2, stepp);
                                }
                            }
                        }
                        topic.publicationControlModul(dataControlModul); // Формируем и Публикуем команды для управления Modul
                        printf("4 ETAP END \n");
                        if (dataControlModul.controlMotor.angle[0] <= colibrData[0].angleL && // ЕСли все углы отработали
                            dataControlModul.controlMotor.angle[1] <= colibrData[1].angleL &&
                            dataControlModul.controlMotor.angle[2] <= colibrData[2].angleL &&
                            dataControlModul.controlMotor.angle[3] <= colibrData[3].angleL)
                        {
                            flag4Colibrovka = false;
                            time5Colibrovka = millis() + 1000; // Начинаем 4 этап через 1 секунды
                            flag5Colibrovka = true;
                        }
                    }
                }
                //==========================================================
                if (time5Colibrovka < millis() && flag5Colibrovka)
                {
                    // Устанавливаем лазер в полученный угол
                    for (int i = 0; i < 4; i++) // Перебираем моторы(лазеры)
                    {
                        dataControlModul.controlMotor.angle[i] = (colibrData[i].angleMin + colibrData[i].angleMin2) / 2.0;
                        printf(" numPillar = %i Teoria angle= %.3f |", g_numPillar[i], g_angleLaser[i]);
                        printf("angle1= %.3f angle2= %.3f  | distMin1 = %.3f distMin2 = %.3f \n", colibrData[i].angleMin, colibrData[i].angleMin2, colibrData[i].distMin, colibrData[i].distMin2);
                        printf("i= %i angle= %.3f distMin = %.3f \n", i, dataControlModul.controlMotor.angle[i], (colibrData[i].distMin + colibrData[i].distMin2) / 2.0);
                    }
                    topic.publicationControlModul(dataControlModul); // Формируем и Публикуем команды для управления Modul
                    flag5Colibrovka = false;
                    //==========================================================
                    // Устанавливаем и корректируем погрешность для расчета угла по лидару и корректируем угол начальной позиции
                    float correctAngle = 0;
                    for (int i = 0; i < 4; i++)
                    {
                        correctAngle += (g_angleLaser[i] - colibrData[i].angleMin); // Находим на сколько полученный угол не совпадает с расчетным по начальной позиции
                    }
                    correctAngle = correctAngle / 4.0; // Находим среднюю ошибку
                    printf(" correctAngle= %.3f\n\n", correctAngle);
                    exit(1);
                }
                /*
                                geometry_msgs::Pose2D startPose2ver;
                                startPose2ver.theta = correctAngle;
                                startPosition(startPose2ver); // Определяем начальное положение
                                offsetAngle = correctAngle; // Меняем поправочный коэффициент
                                //==========================================================
                                // меняем флаг что-бы сюда больше не попадать
                                //modeColibrovka = false;
                                */
            }
            else // Тут исполняется основной режим, не колибровка
            {
                if (flag_msgSpeed) // Флаг что пришло сообщение от ноды Data по Speed. Это будет MODE_0
                {
                    flag_msgSpeed = false;
                    calculationOdometry();           // Расчет одометрии и применения ее для всех режимов
                    topic.visualPublishOdomMode_0(); // Публикация одометрии по моторам которая получается от начальной точки
                }
                // 25 Hz ************************************************************ ОБРАБОТКА ДАННЫХ ИЗ ТОПИКОВ ЧТО ПОДПИСАНЫ  СРАБАТЫВАЕТ КАК ОТПРАВЛЯЕТ DATA_NODE  ********************************************
                if (flag_msgModul) // Флаг что пришло сообщение от ноды Data по Modul. Тут пишем какую-то обработку данных если нужно. Это будет MODE_3
                {
                    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Сделать определние начального угла Theta при запуске путем сканирования лазерами туда-сюда и нахождения минимальной точки и вычисления угла на основе локальных углов
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
                laser.calcAnglePillarForLaser(pillar.pillar, g_poseLidar.mode10); // Расчет углов в локальной системе лазеров на столбы для передачи на нижний уровень для исполнения
                //-------------------------------------------------------------------------
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

            // Публикуем тут так как если один раз опубликовать то они исчезают через некоторое время.
            topic.transformBase(g_poseLidar.mode10);  // Публикуем трансформации систем координат, задаем по какому расчету трансформировать
            topic.transformLidar(g_poseLidar.mode10); // Публикуем трансформации систем координат , задаем по какому расчету трансформировать
            topic.transformLaser(laser);              // Публикуем трансформации систем координат, задаем по какому расчету трансформировать
            topic.visulStartPose();                   // Отобращение стрелкой где начало стартовой позиции и куда направлен нос платформы
            topic.visualPillarPoint(pillar);          // Отображение места размещения столбов
            topic.visualPoseAngleLaser(laser);        // Отобращение стрелкой где начало и куда смотрят лазеры
            // topic.visualPublishOdomMode_11();         // Отобращение стрелкой где начало и куда смотрит в Mode11 Это по Калману
            topic.publicationPoseLidarAll(); // Публикуем все варианты расчета позиций mode 0.1.2.3.4

            // topic.publicationControlDriver(); // Формируем и Публикуем команды для управления Driver
            // topic.publicationControlPrint(); // Формируем и Публикуем команды для управления Print
        }

        r.sleep(); // Интеллектуальная задержка на указанную частоту
        // ros::spinOnce(); // Опрашиваем ядро ROS и по этой команде наши срабатывают колбеки. Нужно только для подписки на топики
    }

    printf("Main Node STOP \n");
    return 0;
}