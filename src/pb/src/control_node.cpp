#include "genStruct.h" // Тут все общие структуры. Истользуются и Data и Main и Control
#include "control_code/config.h"
#include "control_code/topic.h" // Файл для функций для формирования топиков в нужном виде и формате
#include "control_code/code.h"
#include "control_code/gCodeParser.h"

float angleNow = 0; // Текущий угол из топика

int main(int argc, char **argv)
{
    ROS_WARN("%s -------------------------------------------------------------", NN);
    ROS_WARN("%s  Control Module PrintBIM ROS 1.0 Raspberry Pi 4B  ver 1.0001 ", NN);
    ROS_WARN("%s ------------------ROS_ERROR----------------------------------", NN);

    ros::init(argc, argv, "control_node");
    log4cxx::MDC::put("node", "|control_node|");
    ros::NodeHandle nh;
    CTopic topic; // Экземпляр класса для всех публикуемых топиков

    //----------------------------- ПОДПИСКИ НА ТОПИКИ -------НЕ УБИРАЮ В КЛАСС ТАК КАК НУЖНЫ ГЛОБАЛЬНЫЕ КОЛБЕКИ И ПРОЧАЯ ХЕРНЯ --------
    // ros::Subscriber subscriber_Driver = nh.subscribe<pb_msgs::Struct_Driver2Data>("pbData/Driver", 1000, callback_Driver);
    ros::Subscriber subscriber_Speed = nh.subscribe<pb_msgs::SSetSpeed>("pbData/Speed", 1000, callback_Speed);
    ros::Subscriber subscriber_Pose = nh.subscribe<pb_msgs::Struct_PoseRotation>("pbPos/PoseRotation", 1000, callback_Pose);

    ros::Rate r(100);         // Частота в Герцах - задержка
    ros::Duration(1).sleep(); // Подождем пока все обьявится и инициализируется внутри ROS

    ROS_WARN("Start Setup.");
    readParam(); // Считывание переменных параметров из лаунч файла при запуске. Там офсеты и режимы работы
    // initCommandArray(verComand); // Заполнение маасива команд

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

    ROS_WARN("End Setup. Start loop.\n");
    ros::Duration(1).sleep(); // Подождем пока

    while (ros::ok())
    {
        timeNow = ros::Time::now(); // Захватываем текущий момент времени начала цикла
        // ROS_INFO(""); // С новой строки в логе новый цикл
        ros::spinOnce(); // Опрашиваем ядро ROS и по этой команде наши срабатывают колбеки. Нужно только для подписки на топики

        if (flagCommand)
        {
            flagCommand = false;
            ROS_INFO("    command Array i= %i Mode = %i", i, commandArray[i].mode);
            switch (commandArray[i].mode)
            {
            case 0:                                                 // Режим где управляем только скоростями колес отдельно каждым и временем сколько выполняется
                controlSpeed.control.speedL = commandArray[i].velL; //
                controlSpeed.control.speedR = commandArray[i].velR;
                time = millis() + commandArray[i].duration;
                ROS_INFO("    Time Start");
                break;
            case 1:                       // Режим где управляем только углом и добиваемся что в него повернули. Время не учитываем.
                time = millis() + 999999; // Огромное время ставим
                flagAngle = true;         // Флаг что теперь отслеживаем угол
                ROS_INFO("    Angle Start");
                break;
            case 2:                               // Режим где движемся по координатам. даигаемся по длинне вектора.
                vectorStart.x = msg_Pose.x.odom; // Запоминаем те координаты которые были в момент начала движения
                vectorStart.y = msg_Pose.y.odom;
                time = millis() + 999999; // Огромное время ставим
                flagVector = true;        // Флаг что теперь отслеживаем длину вектора
                ROS_INFO("    Vector Start. len = %f", commandArray[i].len);
                break;
            }
        }

        if (flagAngle) // Отслеживание угла
        {
            workAngle(commandArray[i].angle, time, commandArray[i].velAngle); // Тут отрабатываем алгоритм отслеживания угла при повороте
        }

        if (flagVector) // Отслеживание длины вектора
        {
            workVector(commandArray[i].len, vectorStart, time, commandArray[i].velLen); // Тут отрабатываем алгоритм отслеживания длины вектора при движении прямо
        }

        if (time < millis())
        {
            flagCommand = true;
            i++;
            if (commandArray[i].mode == 5)
            {
                ROS_INFO("New loop");
                i = 0;
            }
            ROS_INFO("    Start new step i = %i ", i);
        }

        if (flag_msgSpeed) // Флаг что пришло сообщение от ноды Data по Speed.
        {
            flag_msgSpeed = false;
        }
        if (flag_msgPose) // Флаг что пришло сообщение от ноды Pose
        {
            flag_msgPose = false;
        }

        // if (flag_msgDriver) // Флаг что пришло сообщение от ноды Data по Driver. Тут пишем какую-то обработку данных если нужно.
        // {
        //     flag_msgDriver = false;
        // }

        topic.publicationControlDriver(); // Формируем и Публикуем команды для управления Driver

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
    controlSpeed.control.speedL = 0; // 
    controlSpeed.control.speedR = 0;
    topic.publicationControlDriver(); // Формируем и Публикуем команды для управления Driver
    printf("Control node STOP \n");
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

*/