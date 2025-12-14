
#include "logi.h" //Класс для моего формата логов
AsyncFileLogger logi("/home/pi/RoboBIM-Linux/src/pb/log/", "control_node");

int g_controlMode; // Выбор режима управления 0- по одометрии 1- по слиянию est

#include "genStruct.h" // Тут все общие структуры. Истользуются и Data и Main и Control
#include "control_code/config.h"

#include "control_code/statistic.h"
SystemStatistics stats;
SPose g_coord_offset = {0, 0, 0};      // И оффсет тоже, если еще нет
SPose g_transition_offset = {0, 0, 0}; // Хранит разницу (Est - Model) в момент переключения

#include "control_code/topic.h" // Файл для функций для формирования топиков в нужном виде и формате
#include "control_code/code.h"
#include "control_code/gCodeParser.h"
#include "control_code/c_joy.h"

CJoy joy(0.5, 0.5); // Обьявляем экземпляр класса в нем вся обработка джойстика

float angleNow = 0; // Текущий угол из топика

int main(int argc, char **argv)
{
    ROS_FATAL("\n");
    logi.log_r("*** Control_node PrintBIM ROS 1.0 Raspberry Pi 4B 22/11/25 ver 1.03 \n");
    logi.log("--------------------------------------------------------\n");

    logi.logf("Это сообщение попадёт ТОЛЬКО в файл.\n");                // 1) Только в файл
    logi.log("Обычный лог: скорость = %d, ошибка = %.2f\n", 42, 0.123); // 2) На экран + в файл (обычный белый)
    logi.log_g("Зелёный лог — всё хорошо!\n");                          // 3) Цветные логи + запись в файл
    logi.log_r("Красный лог — ошибка!\n");
    logi.log_w("Жёлтый лог — предупреждение!\n");
    logi.log_b("Синий лог — информационное сообщение.\n");

    ros::init(argc, argv, "control_node");
    log4cxx::MDC::put("node", "|control_node|");
    ros::NodeHandle nh;

    stats.init(nh); // Инициализация топиков статистики (motion_lin, drift_lin и т.д.)
    CTopic topic;   // Экземпляр класса для всех публикуемых топиков

    //----------------------------- ПОДПИСКИ НА ТОПИКИ -------
    // ros::Subscriber subscriber_Driver = nh.subscribe<pb_msgs::Struct_Driver2Data>("pbData/Driver", 1, callback_Driver);
    ros::Subscriber subscriber_Speed = nh.subscribe<pb_msgs::SSetSpeed>("pb/Data/Speed", 1, callback_Speed, ros::TransportHints().tcpNoDelay(true));
    ros::Subscriber subscriber_Pose = nh.subscribe<pb_msgs::Struct_PoseRotation>("pb/Pos/PoseRotation", 1, callback_Pose, ros::TransportHints().tcpNoDelay(true));
    ros::Subscriber subscriber_Joy = nh.subscribe("joy", 1, callback_Joy, ros::TransportHints().tcpNoDelay(true)); // Это мы подписываемся на то что публикует нода джойстика

    ros::Rate r(100);         // Частота в Герцах - задержка
    ros::Duration(1).sleep(); // Подождем пока все обьявится и инициализируется внутри ROS

    logi.log_w("Start Setup.\n");
    readParam(); // Считывание переменных параметров из лаунч файла при запуске. Там офсеты и режимы работы
    // initCommandArray(verComand); // Заполнение маасива команд

    ros::spinOnce(); // Опрашиваем ядро ROS и по этой команде наши срабатывают колбеки. Нужно только для подписки на топики

    // if (g_controlMode) // В зависимости от режима из yaml файла заполняем перменную и далее все опирается на нее.
    // {
    //     g_poseC.x = msg_PoseRotation.x.est;
    //     g_poseC.y = msg_PoseRotation.y.est;
    //     g_poseC.th = msg_PoseRotation.th.est;
    // }
    // else
    // {
    //     g_poseC.x = msg_PoseRotation.x.odom;
    //     g_poseC.y = msg_PoseRotation.y.odom;
    //     g_poseC.th = msg_PoseRotation.th.odom;
    // }
    g_poseC = getPose_C(g_controlMode, false);

    GCodeParser parser; // Создание объекта парсера
    parser.run();       // Запуск обработки

    // static ros::Time time = ros::Time::now();      // Захватываем начальный момент времени
    static ros::Time timeStart = ros::Time::now(); // Захватываем начальный момент времени
    static ros::Time timeNow = ros::Time::now();   // Захватываем конечный момент времени

    u_int64_t time = millis();
    // u_int64_t timeStart = millis();
    // u_int64_t timeMil = millis();
    int i = 0;

    // std::list<int> numbers{1, 2, 3, 4, 5};
    // std::list<SCommand> listok;
    // list int listok;

    logi.log_w("End Setup. Start loop.\n");

    // g_controlMode = 0; // Выбор режима управления 0- по одометрии 1- по слиянию est
    if (g_controlMode == 0)
        logi.log("+++ Start Control ODOMETRY !!!\n");
    else if (g_controlMode == 1)
        logi.log("+++ Start Control MODEL !!!\n");
    else
        logi.log("+++ Start Control EST !!!\n");

    ros::Duration(3).sleep(); // Подождем пока

    while (ros::ok())
    {
        timeNow = ros::Time::now(); // Захватываем текущий момент времени начала цикла
        // ROS_INFO("loop \n");        // С новой строки в логе новый цикл
        ros::spinOnce(); // Опрашиваем ядро ROS и по этой команде наши срабатывают колбеки. Нужно только для подписки на топики

        stats.update(msg_PoseRotation); // ОБНОВЛЕНИЕ ДАННЫХ В КЛАССЕ СТАТИСТИКИ (1 раз за цикл!)

        // // Определяем, нужен ли плавный режим для текущей команды
        // bool current_use_smooth = false;
        // if (i < commandArray.size())
        //     current_use_smooth = commandArray[i].use_model_logic;

        // // g_poseC = getPose_C(g_controlMode);
        // g_poseC = getPose_C(g_controlMode, current_use_smooth);

        // 1. ОПРЕДЕЛЯЕМ ИСТОЧНИК КОМАНДЫ
        int current_cmd_source = 2; // По умолчанию считаем, что спец-режима нет
        if (i < commandArray.size())
            current_cmd_source = commandArray[i].control_source;

        // 2. ВЫЗЫВАЕМ ФУНКЦИЮ С ДВУМЯ ПАРАМЕТРАМИ
        // Передаем и глобальную настройку (g_controlMode), и требование команды
        g_poseC = getPose_C(g_controlMode, current_cmd_source);

        if (flagCommand)
        {
            flagCommand = false;
            // Сбрасываем все активные режимы движения
            flagAngle = false;
            flagVector = false;
            // Сбрасываем скорость на ноль
            controlSpeed.control.speedL = 0.0;
            controlSpeed.control.speedR = 0.0;

            if (i < commandArray.size()) // Проверка на выход за границы
            {
                // --- ЛОГИКА ВЫБОРА РЕЖИМА И РАСЧЕТ БЕСШОВНОГО СТЫКА ---
                int next_mode = commandArray[i].control_source; // 0=ODOM, 1=MODEL, 2=EST
                int cmd_type = commandArray[i].mode;            // G0, G1, G2...

                if (next_mode == 0) // === РЕЖИМ ODOM (Для микро-маневров L) ===
                {
                    // Считаем разницу: Offset = Est - Odom
                    g_transition_offset.x = msg_PoseRotation.x.est - msg_PoseRotation.x.odom;
                    g_transition_offset.y = msg_PoseRotation.y.est - msg_PoseRotation.y.odom;

                    double d_th = msg_PoseRotation.th.est - msg_PoseRotation.th.odom;
                    while (d_th > M_PI)
                        d_th -= 2 * M_PI;
                    while (d_th <= -M_PI)
                        d_th += 2 * M_PI;
                    g_transition_offset.th = d_th;

                    logi.log_w(">>> [CMD %d] EXECUTION MODE: ODOM (Blind / Local)\n", i);
                    logi.log("    Reason: Micro-maneuver without Lidar noise (G2 Length).\n");
                    logi.log("    Transition Offset (Est-Odom): X=%+.4f Y=%+.4f Th=%+.3f deg\n",
                             g_transition_offset.x, g_transition_offset.y, RAD2DEG(g_transition_offset.th));
                }
                else if (next_mode == 1) // === РЕЖИМ MODEL (Для плавных поворотов) ===
                {
                    // Считаем разницу: Offset = Est - Model
                    g_transition_offset.x = msg_PoseRotation.x.est - msg_PoseRotation.x.model;
                    g_transition_offset.y = msg_PoseRotation.y.est - msg_PoseRotation.y.model;

                    double d_th = msg_PoseRotation.th.est - msg_PoseRotation.th.model;
                    while (d_th > M_PI)
                        d_th -= 2 * M_PI;
                    while (d_th <= -M_PI)
                        d_th += 2 * M_PI;
                    g_transition_offset.th = d_th;

                    logi.log_w(">>> [CMD %d] EXECUTION MODE: MODEL (Smooth / Gyro)\n", i);
                    logi.log("    Reason: Precise rotation (G1 Angle).\n");
                    logi.log("    Transition Offset (Est-Model): X=%+.4f Y=%+.4f Th=%+.3f deg\n",
                             g_transition_offset.x, g_transition_offset.y, RAD2DEG(g_transition_offset.th));
                }
                else // === РЕЖИМ EST (По умолчанию) ===
                {
                    // Оффсет не нужен, едем по карте
                    logi.log_g(">>> [CMD %d] EXECUTION MODE: EST (Global / Map)\n", i);
                    logi.log("    Reason: Global navigation to coordinates (G2 X/Y).\n");
                    logi.log("    Lidar Correction: ACTIVE\n");
                }

                // --- ДОПОЛНИТЕЛЬНАЯ ИНФА О ЦЕЛИ (Твой старый блок, адаптированный) ---
                // Добавляем глобальный оффсет к координатам только если едем по EST (глобально)
                float display_offset_x = (next_mode == 2) ? g_coord_offset.x : 0;
                float display_offset_y = (next_mode == 2) ? g_coord_offset.y : 0;

                logi.log("    GLOBAL TARGET: X=%.3f Y=%.3f\n",
                         commandArray[i].point_B_x + display_offset_x,
                         commandArray[i].point_B_y + display_offset_y);

                logi.log("    RAW CMD TARGET: A(%.3f, %.3f) -> B(%.3f, %.3f)\n",
                         commandArray[i].point_A_x, commandArray[i].point_A_y,
                         commandArray[i].point_B_x, commandArray[i].point_B_y);

                logi.log("    TARGET ANGLE: %.3f deg (Speed: %.2f)\n", commandArray[i].angle, commandArray[i].velAngle);

                float signed_distance; // Для учета направления движения
                logi.log("    command Array i= %i Mode = %i \n", i, commandArray[i].mode);

                switch (commandArray[i].mode)
                {
                case 0:                                                 // Режим где управляем только скоростями колес отдельно каждым и временем сколько выполняется
                    controlSpeed.control.speedL = commandArray[i].velL; //
                    controlSpeed.control.speedR = commandArray[i].velR;
                    time = millis() + commandArray[i].duration;
                    logi.log_b("    Start Time \n");
                    break;
                case 1:                       // Режим где управляем только углом и добиваемся что в него повернули. Время не учитываем.
                    time = millis() + 999999; // Огромное время ставим
                    flagAngle = true;         // Флаг что теперь отслеживаем угол
                    flagAngleFirst = true;
                    checker.reset(); // СБРОС БУФЕРА

                    point_A.x = commandArray[i].point_A_x;
                    point_A.y = commandArray[i].point_A_y;
                    logi.log("    point A table  x = %+8.3f y = %+8.3f \n", commandArray[i].point_A_x, commandArray[i].point_A_y);

                    point_B.x = commandArray[i].point_B_x;
                    point_B.y = commandArray[i].point_B_y;
                    logi.log("    point B table  x = %+8.3f y = %+8.3f \n", commandArray[i].point_B_x, commandArray[i].point_B_y);

                    point_C.x = g_poseC.x; // Запоминаем те координаты которые были в момент начала движения
                    point_C.y = g_poseC.y;
                    logi.log("    'point C x = %+8.3f y = %+8.3f th = %+8.3f '\n", point_C.x, point_C.y, RAD2DEG(g_poseC.th));

                    logi.log_b("    Start Angle \n");

                    stats.begin_move(i); // Без аргументов! Только номер команды для лога // Передаем индекс 'i' для красоты логов
                    break;
                case 2: // Режим где движемся по координатам. даигаемся по длинне вектора.
                {
                    checker.reset(); // СБРОС БУФЕРА
                    // point_A.x = commandArray[i].point_A_x;
                    // point_A.y = commandArray[i].point_A_y;
                    // logi.log("    point A table  x = %+8.3f y = %+8.3f \n", commandArray[i].point_A_x, commandArray[i].point_A_y);

                    // point_B.x = commandArray[i].point_B_x;
                    // point_B.y = commandArray[i].point_B_y;
                    // logi.log("    point B table  x = %+8.3f y = %+8.3f \n", commandArray[i].point_B_x, commandArray[i].point_B_y);

                    // --- ЛОГИКА ОПРЕДЕЛЕНИЯ ЦЕЛЕЙ ---
                    // Проверяем: это спец-маневр (ODOM/MODEL) или глобальная езда (EST)?
                    int source = commandArray[i].control_source;

                    if (source == 0 || source == 1) // СПЕЦ-РЕЖИМ (L-команда)
                    {
                        // 1. Определяем базовую точку (откуда берем координаты)
                        // Если режим 0 (ODOM) - берем odom. Если 1 (MODEL) - берем model.
                        double base_x = (source == 0) ? msg_PoseRotation.x.odom : msg_PoseRotation.x.model;
                        double base_y = (source == 0) ? msg_PoseRotation.y.odom : msg_PoseRotation.y.model;
                        double base_th = (source == 0) ? msg_PoseRotation.th.odom : msg_PoseRotation.th.model;

                        // 2. Рассчитываем текущую "виртуальную" позицию (База + Оффсет)
                        // Это точка, где робот "думает", что он стоит
                        SPose curr_p;
                        curr_p.x = base_x + g_transition_offset.x;
                        curr_p.y = base_y + g_transition_offset.y;

                        double curr_th = base_th + g_transition_offset.th;
                        while (curr_th > M_PI)
                            curr_th -= 2 * M_PI;
                        while (curr_th <= -M_PI)
                            curr_th += 2 * M_PI;
                        curr_p.th = curr_th;

                        // 3. Точка А = Текущая позиция (Старт с места)
                        point_A.x = curr_p.x;
                        point_A.y = curr_p.y;

                        // 4. Точка В = Проекция длины L по ТЕКУЩЕМУ углу базы
                        float dir = (commandArray[i].velLen >= 0) ? 1.0 : -1.0;
                        float dist = commandArray[i].len;

                        point_B.x = point_A.x + (dist * dir * cos(curr_p.th));
                        point_B.y = point_A.y + (dist * dir * sin(curr_p.th));

                        if (source == 0)
                            logi.log_w("    [RECALC] ODOM TARGET (Blind). L=%.3f\n", dist);
                        else
                            logi.log_w("    [RECALC] MODEL TARGET (Gyro). L=%.3f\n", dist);

                        logi.log("    Real Start: x=%.3f y=%.3f th=%.3f\n", point_A.x, point_A.y, RAD2DEG(curr_p.th));
                        logi.log("    Real End  : x=%.3f y=%.3f\n", point_B.x, point_B.y);
                    }
                    else
                    {
                        // ОБЫЧНЫЙ РЕЖИМ (EST) - Едем по координатам из G-кода
                        point_A.x = commandArray[i].point_A_x + g_coord_offset.x;
                        point_A.y = commandArray[i].point_A_y + g_coord_offset.y;

                        point_B.x = commandArray[i].point_B_x + g_coord_offset.x;
                        point_B.y = commandArray[i].point_B_y + g_coord_offset.y;
                    }
                    // ---------------------------------

                    logi.log("    point A final  x = %+8.3f y = %+8.3f \n", point_A.x, point_A.y);
                    logi.log("    point B final  x = %+8.3f y = %+8.3f \n", point_B.x, point_B.y);

                    point_C.x = g_poseC.x; // Запоминаем те координаты которые были в момент начала движения
                    point_C.y = g_poseC.y;
                    logi.log("    'point C x = %+8.3f y = %+8.3f th = %+8.3f '\n", point_C.x, point_C.y, RAD2DEG(g_poseC.th));

                    time = millis() + 999999; // Огромное время ставим
                    flagVector = true;        // Флаг что теперь отслеживаем длину вектора

                    logi.log_b("    Start Vector. len = %f \n", commandArray[i].len);
                    stats.begin_move(i); // Без аргументов! Только номер команды для лога // Передаем индекс 'i' для красоты логов
                    break;
                }
                case 3: // Напечатать
                    controlPrint.id++;
                    controlPrint.controlPrint.mode = 0;     // 0 - работать по командам
                    controlPrint.controlPrint.status = 1;   // 1- печатать 0- не печатать
                    controlPrint.controlPrint.torque = 0.5; // Вручную задаю силу прижатия маркера к полу
                    time = millis() + commandArray[i].duration;
                    logi.log_b("    Print Start \n");
                    break;

                case 5: // Отменить печать
                    controlPrint.id++;
                    controlPrint.controlPrint.mode = 0;      // 0 - работать по командам
                    controlPrint.controlPrint.status = 0;    // 1- печатать 0- не печатать
                    controlPrint.controlPrint.torque = -2.0; // Вручную задаю силу с которой отводим маркер в течении 50 милисекунд
                    time = millis() + commandArray[i].duration;
                    logi.log_b("    Print Cancel \n");
                    break;

                case 6: // G10 - Установка системы координат

                    // Мы хотим, чтобы: Реальность = Карта + Оффсет. Значит: Оффсет = Реальность (g_poseC) - Карта (point_A из команды)
                    // g_coord_offset.x = g_poseC.x - commandArray[i].point_A_x;     // Рассчитываем смещение по X
                    // g_coord_offset.y = g_poseC.y - commandArray[i].point_A_y; // Рассчитываем смещение по Y

                    // g_coord_offset.th = normalizeAngle180(g_poseC.th - DEG2RAD(commandArray[i].point_A_a)); // Рассчитываем смещение по Углу (с нормализацией) commandArray хранит градусы, g_poseC хранит радианы (обычно). Приводим к радианам.

                    stats.start_session();  // Сбрасываем статистику, так как начинается "чистое" выполнение задания
                    time = millis() + 1000; // Пауза 1 секунду
                    logi.log_w(">>> G10 OFFSET APPLIED & STATS STARTED <<<\n");
                    break;
                }
            }
        }

        if (time < millis())
        {
            flagCommand = true;
            i++;
            logi.log_g("===============================================================================================================================");
            logi.log("    i = %i => mode = %i \n", i, commandArray[i].mode);

            if (commandArray[i].mode == 9)
            {
                logi.log_r("New loop\n");
                i = 0;
            }
            if (i >= commandArray.size())
            {

                stats.print_report(); // Печатаем финальный отчет
                logi.log_r("    commandArray.size shutdown.\n");
                ros::shutdown();
            }

            logi.log_r("    Start new step i = %i \n", i);
        }

        if (flagAngle) // Отслеживание угла
        {
            // СЧИТАЕМ ЧИСТУЮ ТЕОРИЮ ИЗ G-КОДА Берем конечный угол (point_B_a) и начальный (point_A_a) из текущей команды
            float theory_angle = abs(normalizeAngle180(commandArray[i].point_B_a - commandArray[i].point_A_a));

            // Передаем theory_angle в функцию
            workAngle(commandArray[i].angle, theory_angle, time, commandArray[i].velAngle); // Тут отрабатываем алгоритм отслеживания угла при повороте
        }

        if (flagVector) // Отслеживание длины вектора
        {
            // --- ВЫБОР ЗАПАСА ТОРМОЖЕНИЯ (MARGIN) ---
            float current_margin = 0.010; // По умолчанию 10 мм (для EST/Global)
            if (commandArray[i].control_source == 0)
                current_margin = 0.002; // Если едем по ODOM (L-команды, микро-маневры), уменьшаем запас до 2 мм,чтобы робот не полз пол-пути.
            // ----------------------------------------
            workVector(commandArray[i].len, point_A, point_B, time, commandArray[i].velLen, current_margin); // Передаем current_margin в функцию
        }

        topic.publicationControlDriver(); // Формируем и Публикуем команды для управления Driver
        topic.publicationControlPrint();  // Формируем и Публикуем команды для управления Print

        //============================================================================================================================

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
    }
    // Сбрасываем скорость на ноль
    controlSpeed.control.speedL = 0.0;
    controlSpeed.control.speedR = 0.0;
    logi.log_r("=== сontrol_node shutdown = STOP \n");
    return 0;
}

/*
    #include "data_code/jerk.h"
    JerkLimitedProfile left_wheel;  // Профиль для левого колеса
    JerkLimitedProfile right_wheel; // Профиль для правого колеса

    jlp_init(&left_wheel, "left", 0.0, 1.0, 1.0, 1.0);   // Инициализируем: начальная скорость 0 м/с, j_max=1.0, a_max=1.0, v_max = 1.0
    jlp_init(&right_wheel, "right", 0.0, 1.0, 1.0, 1.0); // Инициализируем: начальная скорость 0 м/с, j_max=1.0, a_max=1.0, v_max = 1.0

    left_wheel.enable_diagnostics = 1;  // Включаем диагностику для теста
    // right_wheel.enable_diagnostics = 1; // Включаем диагностику для теста

    ROS_INFO("p->enable_diagnostics L %i ", left_wheel.enable_diagnostics);
    ROS_INFO("p->enable_diagnostics R %i ", right_wheel.enable_diagnostics);

    // jlp_start_profile(&left_wheel, 0.0); // Запускаем первый профиль — в начальный момент скорость 0 м/с
    // jlp_start_profile(&right_wheel, 0.0); // Запускаем первый профиль — в начальный момент скорость 0 м/с

        //---------------- Тут какие-то постоянные данные вносятся в ручном режиме или алгоритмы корректировки данных перед передачей --------------------------------------------------------------------------------------

        // printf("g_desiredSpeed.L = %f g_desiredSpeed.R= %f \n",g_desiredSpeed.speedL,g_desiredSpeed.speedR);
        // controlAcc(g_desiredSpeed); // Функция контроля ускорения На вход скорость с которой хотим ехать. После будет скорость с которой поедем фактически с учетом возможностей по ускорению

        static double last_desired_speedL = 0.0; // Предыдущие целевые скорости — для сравнения
        static double last_desired_speedR = 0.0;

        const double EPSILON = 1e-3; // 0.001 м/с — настрой под свою систему// Порог для сравнения — чтобы не дергать профиль при мелких флуктуациях

        if (fabs(g_desiredSpeed.speedL - last_desired_speedL) > EPSILON) // Проверяем левое колесо
        {
            jlp_request_replan(&left_wheel, g_desiredSpeed.speedL);
            last_desired_speedL = g_desiredSpeed.speedL; // Обновляем предыдущее значение
        }

        if (fabs(g_desiredSpeed.speedR - last_desired_speedR) > EPSILON)// Проверяем правое колесо
        {
            jlp_request_replan(&right_wheel, g_desiredSpeed.speedR);
            last_desired_speedR = g_desiredSpeed.speedR; // Обновляем предыдущее значение
        }

        jlp_step(&left_wheel, dt); // 4. Выполняем ОДИН шаг для каждого колеса
        jlp_step(&right_wheel, dt);

        g_factSpeed.speedL = left_wheel.v_current;  // ← ПРАВИЛЬНО — это желаемая скорость для PID!
        g_factSpeed.speedR = right_wheel.v_current; // ← ПРАВИЛЬНО — это желаемая скорость для PID!


      // Функция контроля ускорения
void controlAcc(SControl dreamSpeed_)
{
    static unsigned long time = micros();		 // Время предыдущего расчета// Функция из WiringPi.// Замеряем интервалы по времени между запросами данных
    unsigned long time_now = micros();			 // Время в которое делаем расчет
    double dt = ((time_now - time) / 1000000.0); // Интервал расчета переводим сразу в секунды Находим интревал между текущим и предыдущим расчетом в секундах
    time = time_now;
    float accel = ACCELERATION * dt; // Ускорение
    // printf("dreamSpeed_ % .3f % .3f accel= % .5f dt= % .5f", dreamSpeed_.speedL, dreamSpeed_.speedR, accel, dt);
    if (dreamSpeed_.speedL != g_factSpeed.speedL) // Если скорость с которой хотим крутиться не равна тому что была ранее установлена, то меняем с учетом ускорения
    {
        // printf("dreamSpeed_ % f : % f : acc= % f | ", dreamSpeed_.speedL, dreamSpeed_.speedR, accel);
        if (g_factSpeed.speedL < dreamSpeed_.speedL) // Если меньше чем надо то прибавим оборотов
        {
            g_factSpeed.speedL = g_factSpeed.speedL + accel; // К старой скорости прибавляем ускорение за этот промежуток
            if (g_factSpeed.speedL > dreamSpeed_.speedL)	 // Если стала больше то ровняем
                g_factSpeed.speedL = dreamSpeed_.speedL;
        }
        if (g_factSpeed.speedL > dreamSpeed_.speedL) // Если меньше чем надо то прибавим оборотов
        {
            g_factSpeed.speedL = g_factSpeed.speedL - accel; // К старой скорости прибавляем ускорение за этот промежуток
            if (g_factSpeed.speedL < dreamSpeed_.speedL)	 // Если стала меньше нужной то далаем какая должна быть
                g_factSpeed.speedL = dreamSpeed_.speedL;
        }
    }
    if (dreamSpeed_.speedR != g_factSpeed.speedR) // Если скорость с которой хотим крутиться не равна тому что была ранее установлена, то меняем с учетом ускорения
    {
        if (g_factSpeed.speedR < dreamSpeed_.speedR) // Если меньше чем надо то прибавим оборотов
        {
            g_factSpeed.speedR = g_factSpeed.speedR + accel; // К старой скорости прибавляем ускорение за этот промежуток
            if (g_factSpeed.speedR > dreamSpeed_.speedR)	 // Если стала больше то ровняем
                g_factSpeed.speedR = dreamSpeed_.speedR;
        }
        if (g_factSpeed.speedR > dreamSpeed_.speedR) // Если меньше чем надо то прибавим оборотов
        {
            g_factSpeed.speedR = g_factSpeed.speedR - accel; // К старой скорости прибавляем ускорение за этот промежуток
            if (g_factSpeed.speedR < dreamSpeed_.speedR)	 // Если стала меньше нужной то далаем какая должна быть
                g_factSpeed.speedR = dreamSpeed_.speedR;
        }
    }
    // printf("g_factSpeed % f : % f : acc= % f \n ", g_factSpeed.speedL, g_factSpeed.speedR);
    // printf(" |g_factSpeed % .3f % .3f \n", g_factSpeed.speedL, g_factSpeed.speedR);
}



        //----------------------------- ТУТ ВНОСИМ ИЗМЕНЕНИЕ В УПРАВЛЕНИЕ ЕСЛИ ВРУЧНУЮ УПРАВЛЯЕМ ЧЕРЕЗ ДЖОЙСТИК ------------------------------------------------------------------

        if (flag_msgJoy) // Если пришло новое сообшение и сработал колбек то разбираем что там пришло
        {
            flag_msgJoy = false;                                       // Флаг сбратываем .Приоритет джойстику
            joy.parsingJoy(msg_joy);                                   // Разбираем и формируем команды из полученного сообщения
            joy.transform();                                           // Преобразование кнопок джойстика в реальные команды
            g_desiredSpeed.speedL = joy._ControlDriver.control.speedL; // Можно упростить и сделать без переменной g_desiredSpeed
            g_desiredSpeed.speedR = joy._ControlDriver.control.speedR;

            // Data2Print.controlPrint.status = joy._controlPrint.status; // Было раньше печать по джойстику
            // Data2Print.controlPrint.mode = joy._controlPrint.mode;

            // pub_JoyData.publish(joy._joy2Head); // Публикация данных разобранных из джойстика
            // topic.publicationControlDriver(joy._ControlDriver); // Публикация данных по управлению Driver (для отладки)
        }


*/