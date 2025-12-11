
#include "genStruct.h" // Тут все общие структуры. Истользуются и Data и Est и Head

#include "logi.h" //Класс для моего формата логов
AsyncFileLogger logi("/home/pi/RoboBIM-Linux/src/pb/log/", "pose_node");

#include "pos_code/config.h" // Конфигурационный файл

#include "pos_code/localizer.h"
RateLimitedLocalizer rate_fuser; // Экземпляр Rate-Limited

float g_angleMPU = 0;  // Глобальная перемнная угла получаемого с MPU куда смотрим
float g_angleLaser[4]; // Углы на столбы которые передаем на нижний угол для управления
int g_numPillar[4];    // Номер столба до которого измеряем расстояние лазером

SPoseRotation g_poseRotation; // Глобальная позиция в точке вращения робота. Все считается к ней.Она основная во всем. Расчет для лидарной точки mode1.2.3 потом пересчитывается в g_PoseRotation
SPoseBase g_poseLidar;        // Позиции лидара по расчетам Центральная система координат
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

IMUManager imu;

int main(int argc, char **argv)
{
    log4cxx::MDC::put("node", "|pos_node|"); // Установка дополнительных данных в контекст

    ROS_FATAL("\n");
    logi.log("***  pos_node *** ver. 1.59 *** printBIM.ru 22/11/25 *** 2025 ***\n");
    logi.log("--------------------------------------------------------\n");

    logi.logf("Это сообщение попадёт ТОЛЬКО в файл.\n");                // 1) Только в файл
    logi.log("Обычный лог: скорость = %d, ошибка = %.2f\n", 42, 0.123); // 2) На экран + в файл (обычный белый)
    logi.log_g("Зелёный лог — всё хорошо!\n");                          // 3) Цветные логи + запись в файл
    logi.log_r("Красный лог — ошибка!\n");
    logi.log_w("Жёлтый лог — предупреждение!\n");
    logi.log_b("Синий лог — информационное сообщение.\n");

    ros::init(argc, argv, "pos_node");
    ros::NodeHandle nh;

    static ros::Time timeStart = ros::Time::now(); // Захватываем начальный момент времени
    static ros::Time timeNow = ros::Time::now();   // Захватываем конечный момент времени
    static ros::Time timeStop = ros::Time::now();  // Захватываем конечный момент времени

    CTopic topic; // Экземпляр класса для всех публикуемых топиков

    //----------------------------- ПОДПИСКИ НА ТОПИКИ -------НЕ УБИРАЮ В КЛАСС ТАК КАК НУЖНЫ ГЛОБАЛЬНЫЕ КОЛБЕКИ И ПРОЧАЯ ХЕРНЯ --------
    ros::Subscriber subscriber_Lidar = nh.subscribe<pb_msgs::Struct_PoseLidar>("pb/Lidar/Pose", 1, callback_Measurement, ros::TransportHints().tcpNoDelay(true)); // Измеренная позиция по лидару лазеру
    ros::Subscriber subscriber_Scan = nh.subscribe<pb_msgs::Struct_PoseScan>("/pb/Scan/PoseLidar", 1, callback_Scan, ros::TransportHints().tcpNoDelay(true));     // Измеренная позиция по лидару лазеру

    ros::Subscriber subscriber_Modul = nh.subscribe<pb_msgs::Struct_Modul2Data>("pb/Data/Modul", 1, callback_Modul, ros::TransportHints().tcpNoDelay(true));
    ros::Subscriber subscriber_Driver = nh.subscribe<pb_msgs::Struct_Driver2Data>("pb/Data/Driver", 1, callback_Driver, ros::TransportHints().tcpNoDelay(true));
    ros::Subscriber subscriber_Speed = nh.subscribe<pb_msgs::SSetSpeed>("pb/Data/Speed", 1, callback_Speed, ros::TransportHints().tcpNoDelay(true));
    //---------------------------------------------------------------------------------------------------------------------------

    read_Param_StartPose(); // Считывание переменных параметров из лаунч файла при запуске. Там офсеты и режимы работы

    pillar.parsingPillar(msg_pillar); // Разбираем пришедшие данные Заполняем массив правильных координат.

    ros::Rate rate(100);      // Частота в Герцах основного цикла
    ros::Duration(1).sleep(); // Подождем пока все обьявится и инициализируется внутри ROS
    static bool flagPublish = false;

    ROS_INFO("Waiting for the /startup_signal topic to become active...");                        // БЛОКИРУЮЩЕЕ ОЖИДАНИЕ ЗАПУСКА ДРУГОЙ НОДЫ // Ожидаем, пока топик сигнала запуска не станет активным
    ros::topic::waitForMessage<pb_msgs::Struct_Driver2Data>("pb/Data/Driver", ros::Duration(30)); // Блокирующий вызов: ждет первое сообщение из топика "/startup_signal" с таймаутом 30 секунд. Замените String на нужный тип сообщения.
    ROS_INFO("Topic received. Starting the calibration phase.");                                  // Топик получен. Начинаем фазу калибровки.

    calibr_accel_gyro(); // Калибровка гироскопа и акселерометра в момент запуска ноды
    initKalman();        // Задаем коэфициенты для Калмана

    rate_fuser.max_pos_step = 0.01; // Настройка параметров слияния Максимальный шаг коррекции позиции (0.01 м)

    ROS_WARN("++++ End Setup. Start loop.");
    ros::Duration(3).sleep(); // Подождем пока все обьявится и инициализируется внутри ROS

    while (ros::ok())
    {
        timeNow = ros::Time::now(); // Захватываем текущий момент времени начала цикла
        // testFunction();
        ros::spinOnce(); // Опрашиваем ядро ROS и по этой команде наши срабатывают колбеки. Нужно только для подписки на топики

        timeStop = timeStopping(msg_Speed);                           // Расчет времени когда остановились. ЕСли движемся то выдаем текущее время. Если стоим то время когда остановились
        ros::Duration durationStopping = ros::Time::now() - timeStop; // Находим разницу между началом и концом
        dtStoping = durationStopping.toSec();                         // Получаем количество секунд

        // 100 Hz ************************************************************ ОБРАБОТКА ДАННЫХ ИЗ ТОПИКОВ ЧТО ПОДПИСАНЫ  СРАБАТЫВАЕТ КАК ОТПРАВЛЯЕТ DATA_NODE  ********************************************
        if (flag_msgSpeed) // ЭТО РАСЧЕТ МОДЕЛИ Флаг что пришло сообщение от ноды Data по Speed. Расчитываем линейную и угловую скорость и потом на нее основе расчитываем остальное
        {
            logi.log_w("--- flag_msgSpeed \n");
            flag_msgSpeed = false;
            flagPublish = true;

            logi.log_b("odom \n");
            g_linAngVel.odom = calcTwistFromWheel(msg_Speed);                                 // Обработка пришедших данных. По ним считаем линейные скорости по осям и угловую по углу. Запоминаем dt
            g_poseRotation.odom = calcNewPose(g_poseRotation.odom, g_linAngVel.odom, "odom"); // На основе линейных скоростей считаем новую позицию и угол по колесам
            // logi.log("    100Hz OUT Pose Rotation odom  x = %+8.3f y = %+8.3f th = %+8.3f \n", g_poseRotation.odom.x, g_poseRotation.odom.y, g_poseRotation.odom.th);
            // g_poseLidar.odom = convertRotation2Lidar(g_poseRotation.odom, "odom"); // Позиция для отладки и сравнения по результатм только одометрии

            logi.log_b("imu \n");
            // Передаем скорость колес, которую посчитали строчкой выше
            g_linAngVel.imu = calcTwistFromImu(msg_Driver2Data, g_linAngVel.odom.vx);     // Обработка пришедших данных. По ним считаем линейные скорости по осям и угловую по углу. Запоминаем dt
            g_poseRotation.imu = calcNewPose(g_poseRotation.imu, g_linAngVel.imu, "imu"); // На основе линейных скоростей считаем новую позицию и угол по колесам

            logi.log_b("model \n");
            g_linAngVel.model = calcTwistFused(g_linAngVel.odom, g_linAngVel.imu);                // Комплементация данных используя фильтр (Комплементарный или Калмана) тут написать функцию комплементации данных угловых скоростей с разными условиями когда и в каком соотношении скомплементировать скорсти с двух источников
            g_poseRotation.model = calcNewPose(g_poseRotation.model, g_linAngVel.model, "model"); // На основе линейных скоростей считаем новую позицию и угол по колесам
            // logi.log("    100Hz OUT Pose Rotation model x = %+8.3f y = %+8.3f th = %+8.3f \n", g_poseRotation.model.x, g_poseRotation.model.y, g_poseRotation.model.th);
            g_poseLidar.model = convertRotation2Lidar(g_poseRotation.model, "model"); // Позиция по результатам слияния одометрии и imu считаем РАСЧЕТОМ

            g_poseRotation.est = calcNewPose(g_poseRotation.est, g_linAngVel.model, "est"); // На основе линейных скоростей считаем новую позицию и угол по колесам
            // logi.log("    100Hz OUT Pose Rotation est x = %+8.3f y = %+8.3f th = %+8.3f \n", g_poseRotation.est.x, g_poseRotation.est.y, g_poseRotation.est.th);
            g_poseLidar.est = convertRotation2Lidar(g_poseRotation.est, "model"); // Позиция по результатам слияния одометрии и imu считаем РАСЧЕТОМ

            // g_linAngVel.model = calcTwistFromMpu(g_linAngVel.model, msg_Modul2Data, g_linAngVel.odom); // Обработка пришедших данных для расчета линейных и угловых скоростей
            // g_poseRotation.mode10 = calcNewOdom2(g_poseRotation.mode10, g_linAngVel.model, "mode10"); // На основе линейных скоростей считаем новую позицию и угол по колесам

            // РАСЧЕТ НАПРАВЛЕНИЯ УГЛОВ ЛАЗЕРОВ
            // laser.calcAnglePillarForLaser(pillar.pillar, g_poseLidar.est); // Расчет углов в локальной системе лазеров на столбы для передачи на нижний уровень для исполнения
            // topic.publicationControlModul();                                // Формируем и Публикуем команды для управления Modul

            // angleMPU(); // Расчет угла положения на основе данных сдатчика MPU
            // calcEuler(); // Расчет угла yaw с датчика IMU
        }

        // Выполняется 10 -11 Hz как придут результаты ИЗМЕРЕНИЯ ***
        if (flag_msgScan) // Если пришло сообщение в топик от измерениям и мы уже разобрали данные по координатам машинки по расчету с колес и IMU
        {
            flag_msgScan = false;
            logi.log_b("--- flag_msgScan *** \n");

            bool is_data_valid = true; // Флаг для дальнейших проверок
            logi.log("    IN  Data Pose Est x = %+8.3f y = %+8.3f th = %+8.3f \n", g_poseLidar.est.x, g_poseLidar.est.y, g_poseLidar.est.th);

            g_poseLidar.meas.x = msg_Scan.x.fused; // Запоминаем данные ИЗМЕРЕНИЯ которые пришли
            g_poseLidar.meas.y = msg_Scan.y.fused;
            g_poseLidar.meas.th = msg_Scan.th.fused;

            g_poseRotation.meas = convertLidar2Rotation(g_poseLidar.meas, "meas"); // Обновляем

            logi.log("    IN  Data Pose Meas x = %+8.3f y = %+8.3f th = %+8.3f \n", g_poseLidar.meas.x, g_poseLidar.meas.y, g_poseLidar.meas.th);

            // --- ПРОВЕРКИ КАЧЕСТВА (Data Validation) ---
            // 1. Проверка RMSE (если приходит в сообщении) Допустим, 0.15 метра - это предел, дальше мусор.
            if (msg_Scan.rmse.fused > 0.20)
            {
                is_data_valid = false;
                logi.log_w("WARN: Bad RMSE %.3f. Skip measurement.\n", msg_Scan.rmse.fused);
            }

            // 2. Проверка скачка позиции (Innovation Gate) Считаем расстояние между тем, где мы ЕСТЬ (est), и ИЗМЕРЕНИЕМ (meas)
            double dist_diff = hypot(g_poseLidar.meas.x - g_poseLidar.est.x, g_poseLidar.meas.y - g_poseLidar.est.y);
            if (dist_diff > 0.5) // Если робот "телепортировался" на 0.5 метра - это ошибка распознавания
            {
                is_data_valid = false;
                logi.log_w("WARN: Large jump detected %.3f m. Skip.\n", dist_diff);
            }

            // 3. Проверка на NaN (на всякий случай)
            if (isnan(g_poseLidar.meas.x) || isnan(g_poseLidar.meas.y))
            {
                logi.log_r("ERROR: NaN. Skip.\n");
                is_data_valid = false;
            }

            static int firstData = 0;
            if (firstData == 0)
            {
                firstData = 1;                                                                                  // Флаг что начальные знаения поменяли на лидарные
                g_poseRotation.odom = convertLidar2Rotation(g_poseLidar.meas, "First data measurement odom");   // Обновляем уже уточненным значением
                g_poseRotation.imu = convertLidar2Rotation(g_poseLidar.meas, "First data measurement imu");     // Обновляем уже уточненным значением
                g_poseRotation.model = convertLidar2Rotation(g_poseLidar.meas, "First data measurement model"); // Обновляем уже уточненным значением
                g_poseRotation.meas = convertLidar2Rotation(g_poseLidar.meas, "First data measurement meas");   // Обновляем уже уточненным значением
                g_poseRotation.est = convertLidar2Rotation(g_poseLidar.meas, "First data measurement est");     // Обновляем уже уточненным значением
                logi.log_b("=== FIRST Data Pose Rotation Measurement x = %+8.3f y = %+8.3f th = %+8.3f \n", g_poseRotation.odom.x, g_poseRotation.odom.y, g_poseRotation.odom.th);
            }
            else
            {
                if (is_data_valid)// Вызываем СЛИЯНИЕ (Fusion) для Rate-Limited Fuser Используем скорости модели (плавные) для прогноза
                {                                                                                                                                                
                    const double lidar_latency_L = 0.35;                                                                                                         // Задержка лидара/SLAM:  мс Посчитано ИИ экспериментально
                    g_poseLidar.est = rate_fuser.fuse(g_poseLidar.est, g_poseLidar.meas, g_linAngVel.model.vx, RAD2DEG(g_linAngVel.model.vth), lidar_latency_L); // Состояние Модели (по ссылке) и Измерение (SPose)
                    logi.log("    is_data_valid  lidar_latency_L= %+8.3f model.vx= %+8.3f model.vth= %+8.3f | dtStoping = %+8.3f \n", lidar_latency_L, g_linAngVel.model.vx, g_linAngVel.model.vth, dtStoping);

                    // =========================================================================
                    // 2. СТАБИЛИЗАЦИЯ EST НА СТОЯНКЕ (МЕДИАННЫЙ ФИЛЬТР)
                    // =========================================================================
                    static std::vector<double> buf_x; // Статические буферы для накопления данных
                    static std::vector<double> buf_y;
                    static std::vector<double> buf_th;

                    // Если стоим дольше 0.5 сек, начинаем усреднять саму оценку, чтобы не дрожала
                    if (dtStoping >= 0.5) 
                    {
                        // ОГРАНИЧИТЕЛЬ: Копим до 100 измерений (хватит с головой)
                        if (buf_x.size() < 100) 
                        {
                            buf_x.push_back(g_poseLidar.est.x);
                            buf_y.push_back(g_poseLidar.est.y);
                            buf_th.push_back(g_poseLidar.est.th);
                        }
                        // Функция для поиска медианы (Лямбда-функция для компактности)
                        auto get_median = [](std::vector<double> v) -> double 
                        {
                            if (v.empty()) return 0.0;
                            size_t n = v.size() / 2;
                            std::sort(v.begin(), v.end()); // Частичная сортировка быстрее полной, но на 100 элементах разницы нет. Используем полную для простоты.
                            return v[n]; // Если четное число элементов, можно брать среднее двух центральных, но для наших целей достаточно просто взять центральный.
                        };

                        // Применяем медиану к текущей позиции  Теперь est не дрожит и игнорирует резкие скачки
                        if (!buf_x.empty()) 
                        {
                            g_poseLidar.est.x = get_median(buf_x);
                            g_poseLidar.est.y = get_median(buf_y);
                            g_poseLidar.est.th = get_median(buf_th);
                            normalizeAngle180(g_poseLidar.est.th);
                        }
                        // // Накапливаем сумму
                        // if (count_est_long < 100) // ОГРАНИЧИТЕЛЬ: Если накопили 100 замеров (10 сек), хватит считать, результат уже точный.
                        // {
                        //     count_est_long++;
                        //     // Накапливаем сумму
                        //     sum_est_long.x += g_poseLidar.est.x;
                        //     sum_est_long.y += g_poseLidar.est.y;
                        //     sum_est_long.th += g_poseLidar.est.th;  // С углами аккуратнее (сумма векторов), но для малых колебаний можно просто сумму (Для идеала лучше усреднять sin/cos, но пока так сойдет, если угол не скачет через 180)
                        // }

                        // // Делим всегда на count, который замер на 100.  est замрет в идеальном усредненном положении и перестанет меняться вообще.
                        // g_poseLidar.est.x = sum_est_long.x / count_est_long;
                        // g_poseLidar.est.y = sum_est_long.y / count_est_long;
                        // g_poseLidar.est.th = sum_est_long.th / count_est_long; 
                        // normalizeAngle180(g_poseLidar.est.th);
                    }
                    else 
                    {
                        // // Если движемся или только встали - сбрасываем усреднитель
                        // count_est_long = 0;
                        // sum_est_long = {0,0,0};

                        // Движемся - очищаем буферы Используем clear(), чтобы не перевыделять память лишний раз
                        if (!buf_x.empty()) 
                        {
                            buf_x.clear();
                            buf_y.clear();
                            buf_th.clear();
                        }
                    }

                    g_poseRotation.est = convertLidar2Rotation(g_poseLidar.est, "est"); // Обновляем уже уточненным значением

                    // logi.log("    OUT 10Hz Data Pose Rotation Est x = %+8.3f y = %+8.3f th = %+8.3f \n", g_poseRotation.est.x, g_poseRotation.est.y, g_poseRotation.est.th);

                    // =========================================================================
                    // ГИБРИДНАЯ СИНХРОНИЗАЦИЯ (Motion Soft Sync + Static Hard Sync)
                    // =========================================================================

                    static SPose sum_est = {0, 0, 0};        // Сумма для усреднения
                    static int count_est = 0;                // Счетчик измерений
                    static bool is_static_corrected = false; // Флаг, что мы уже скорректировались на стоянке

                    // 1. ЕСЛИ РОБОТ ДВИЖЕТСЯ (или только что остановился)
                    if (dtStoping < 0.2)
                    {
                        // Сбрасываем флаги статической коррекции
                        is_static_corrected = false;
                        sum_est = {0, 0, 0};
                        count_est = 0;

                        double k_motion = 0.025; // --- Мягкая синхронизация на ходу Коэффициент очень маленький 2,5% примерно на столько у меня одометрия ошибается от рулетки

                        g_poseLidar.model.x += (g_poseLidar.est.x - g_poseLidar.model.x) * k_motion;
                        g_poseLidar.model.y += (g_poseLidar.est.y - g_poseLidar.model.y) * k_motion;

                        double d_th = angle_diff_deg(g_poseLidar.est.th, g_poseLidar.model.th);
                        g_poseLidar.model.th += d_th * k_motion;
                        normalizeAngle180(g_poseLidar.model.th);
                    }
                    // 3. ПРИМЕНЯЕМ КОРРЕКЦИЮ (Один раз после накопления)
                    else if (dtStoping >= 1.9 && !is_static_corrected && buf_x.size() > 5)
                    {
                        // ПРОСТО БЕРЕМ ГОТОВУЮ EST Она к этому моменту уже идеально (она уже медианная!)
                        logi.log_w("STATIC MEDIAN SYNC. Drift fix: dist=%.3f angle=%.3f samples=%d", 
                                hypot(g_poseLidar.est.x - g_poseLidar.model.x, g_poseLidar.est.y - g_poseLidar.model.y), 
                                angle_diff_deg(g_poseLidar.est.th, g_poseLidar.model.th), (int)buf_x.size());

                        g_poseLidar.model = g_poseLidar.est; // <--- Простой и точный прыжок к красной линии
                        is_static_corrected = true; 
                    }
                    g_poseRotation.model = convertLidar2Rotation(g_poseLidar.model, "model"); // Обновляем уже уточненным значением
                }
            }
            logi.log("    OUT Data Pose Est x = %+8.3f y = %+8.3f th = %+8.3f \n", g_poseLidar.est.x, g_poseLidar.est.y, g_poseLidar.est.th);
        }

        // topic.visualPublishOdomMode_3();                                  // Отобращение стрелкой где начало и куда смотрит в Mode3
        // topic.publicationAngleLaser(laser);                               // Формируем перемнную с собщением для публикации
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
        //     startColibrovka(topic); // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Сделать определние начального угла Theta при запуске путем сканирования лазерами туда-сюда и нахождения минимальной точки и вычисления угла на основе локальных углов
        */
        // g_poseRotation.mode10 = g_poseRotation.mode0; // Времено. ПОТОМ ТУТ НАДО ИТОГОВУЮ КОМПЛЕМЕНТАЦИЮ СДЕЛАТЬ
        // Тут строка перевода в g_poseLidaк.mode10 для использовании  в расчетаз в следущей итерации

        if (flagPublish) //
        {
            flagPublish = false;
            topic.publicationLinAngVel();     // Вывод в топик данных с данными угловой и линейной скоростью
            topic.publicationPoseRotattion(); // Публикуем все варианты расчета позиций mode 0.1.2.3.4
            topic.publicationPoseBase();      // Публикуем все варианты расчета позиций mode 0.1.2.3.4
            // topic.publishOdomUnited();              // Публикация одометрии по моторам с корректировкой с верхнего уровня
        }

        static u_int64_t timeRviz = millis();
        if (timeRviz <= millis()) // 30 Hz
        {
            // Публикуем тут так как если один раз опубликовать то они исчезают через некоторое время.
            topic.transformBase(g_poseLidar.est); // Публикуем трансформации систем координат, задаем по какому расчету трансформировать
            topic.transformLaser(laser);          // Публикуем трансформации систем координат, задаем по какому расчету трансформировать
            topic.transformRotation();            // Публикуем трансформации систем координат

            topic.visualStartPose();           // Отобращение стрелкой где начало стартовой позиции и куда направлен нос платформы
            topic.visualPillarPoint(pillar);   // Отображение места размещения столбов
            topic.visualPoseAngleLaser(laser); // Отобращение стрелкой где начало и куда смотрят лазеры

            // topic.visualPublishOdomMode_0(); // Публикация одометрии по моторам которая получается от начальной точки
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
        rate.sleep();                  // Интеллектуальная задержка на указанную частоту
    }

    printf("pos_node STOP \n");
    return 0;
}