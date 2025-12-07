
#include "logi.h" //Класс для моего формата логов
AsyncFileLogger logi("/home/pi/RoboBIM-Linux/src/pb/log/", "control_node");

int g_controlMode; // Выбор режима управления 0- по одометрии 1- по слиянию main

#include "genStruct.h" // Тут все общие структуры. Истользуются и Data и Main и Control
#include "control_code/config.h"
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
    CTopic topic; // Экземпляр класса для всех публикуемых топиков

    //----------------------------- ПОДПИСКИ НА ТОПИКИ -------НЕ УБИРАЮ В КЛАСС ТАК КАК НУЖНЫ ГЛОБАЛЬНЫЕ КОЛБЕКИ И ПРОЧАЯ ХЕРНЯ --------
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

    if (g_controlMode) // В зависимости от режима из yaml файла заполняем перменную и далее все опирается на нее.
    {
        g_poseC.x = msg_PoseRotation.x.main;
        g_poseC.y = msg_PoseRotation.y.main;
        g_poseC.th = msg_PoseRotation.th.main;
    }
    else
    {
        g_poseC.x = msg_PoseRotation.x.odom;
        g_poseC.y = msg_PoseRotation.y.odom;
        g_poseC.th = msg_PoseRotation.th.odom;
    }

    GCodeParser parser; // Создание объекта парсера
    parser.run(); // Запуск обработки

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

    // g_controlMode = 0; // Выбор режима управления 0- по одометрии 1- по слиянию main
    if (g_controlMode)
        logi.log("+++ Start Control MAIN !!!\n");
    else
        logi.log("+++ Start Control ODOMETRY !!!\n");

    ros::Duration(3).sleep(); // Подождем пока

    while (ros::ok())
    {
        timeNow = ros::Time::now(); // Захватываем текущий момент времени начала цикла
        // ROS_INFO("loop \n");        // С новой строки в логе новый цикл
        ros::spinOnce(); // Опрашиваем ядро ROS и по этой команде наши срабатывают колбеки. Нужно только для подписки на топики

        if (g_controlMode) // В зависимости от режима из yaml файла заполняем перменную и далее все опирается на нее.
        {
            g_poseC.x = msg_PoseRotation.x.main;
            g_poseC.y = msg_PoseRotation.y.main;
            g_poseC.th = msg_PoseRotation.th.main;
        }
        else
        {
            g_poseC.x = msg_PoseRotation.x.odom;
            g_poseC.y = msg_PoseRotation.y.odom;
            g_poseC.th = msg_PoseRotation.th.odom;
        }

        if (flagCommand)
        {
            flagCommand = false;
            // Сбрасываем все активные режимы движения
            flagAngle = false;
            flagVector = false;
            // Сбрасываем скорость на ноль
            controlSpeed.control.speedL = 0.0;
            controlSpeed.control.speedR = 0.0;

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
                history.reset(); // СБРОС БУФЕРА
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
                break;
            case 2: // Режим где движемся по координатам. даигаемся по длинне вектора.
                // point_A.x = msg_PoseRotation.x.odom; // Запоминаем те координаты которые были в момент начала движения
                // point_A.y = msg_PoseRotation.y.odom;
                // logi.log("    point A odom    x = %+8.3f y = %+8.3f \n",point_A.x,point_A.y);
                point_A.x = commandArray[i].point_A_x;
                point_A.y = commandArray[i].point_A_y;
                logi.log("    point A table  x = %+8.3f y = %+8.3f \n", commandArray[i].point_A_x, commandArray[i].point_A_y);

                // signed_distance = commandArray[i].len;
                // if (commandArray[i].velLen < 0) // Если скорость орицательная то надо учитывать при расчетах
                //     signed_distance = -commandArray[i].len;
                // point_B = calculate_new_coordinates(point_A, msg_PoseRotation.th.odom, signed_distance); // Посчитали конечные координаты точки В
                // logi.log("    point B calc x = %+8.3f y = %+8.3f \n", point_B.x,point_B.y);

                point_B.x = commandArray[i].point_B_x;
                point_B.y = commandArray[i].point_B_y;
                logi.log("    point B table  x = %+8.3f y = %+8.3f \n", commandArray[i].point_B_x, commandArray[i].point_B_y);

                point_C.x = g_poseC.x; // Запоминаем те координаты которые были в момент начала движения
                point_C.y = g_poseC.y;
                logi.log("    'point C main  x = %+8.3f y = %+8.3f th = %+8.3f '\n", point_C.x, point_C.y, RAD2DEG(g_poseC.th));

                time = millis() + 999999; // Огромное время ставим
                flagVector = true;        // Флаг что теперь отслеживаем длину вектора
                logi.log_b("    Start Vector. len = %f \n", commandArray[i].len);
                break;
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
            }
        }

        if (time < millis())
        {
            flagCommand = true;
            i++;
            logi.log("    i = %i => mode = %i \n", i, commandArray[i].mode);

            if (commandArray[i].mode == 9)
            {
                logi.log_r("New loop\n");
                i = 0;
            }
            if (i >= commandArray.size())
            {
                logi.log_r("    commandArray.size shutdown.\n");
                ros::shutdown();
            }

            logi.log_r("    Start new step i = %i \n", i);
        }

        if (flagAngle) // Отслеживание угла
        {
            workAngle(commandArray[i].angle, time, commandArray[i].velAngle); // Тут отрабатываем алгоритм отслеживания угла при повороте
        }

        if (flagVector) // Отслеживание длины вектора
        {
            workVector(commandArray[i].len, point_A, point_B, time, commandArray[i].velLen); // Тут отрабатываем алгоритм отслеживания длины вектора при движении прямо
        }

        // if (flag_msgSpeed) // Флаг что пришло сообщение от ноды Data по Speed.
        // {
        //     flag_msgSpeed = false;
        // }
        // if (flag_msgPose) // Флаг что пришло сообщение от ноды Pose
        // {
        //     flag_msgPose = false;
        // }

        // if (flag_msgDriver) // Флаг что пришло сообщение от ноды Data по Driver. Тут пишем какую-то обработку данных если нужно.
        // {
        //     flag_msgDriver = false;
        // }

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