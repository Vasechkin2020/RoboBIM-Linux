
// #include "bcm2835.h"

#include "genStruct.h" // Тут все общие структуры. Истользуются и Data и Main и Head
#include "data_code/config.h"
#include "data_code/c_joy.h"
CJoy joy(0.5, 0.5); // Обьявляем экземпляр класса в нем вся обработка джойстика
#include "data_code/data2driver.h"
#include "data_code/data2modul.h"
#include "data_code/data2print.h"
#include "data_code/jerk.h"
#include "data_code/topic.h" // Файл для функций для формирования топиков в нужном виде и формате
#include "data_code/code.h"

int main(int argc, char **argv)
{

    // ROS_WARN("%s --------------------------------------------------------", NN);
    ROS_FATAL("%s *** Data_Node *** ver. 1.5 22-09-25 *** printBIM.ru *** 2025 ***", NN);
    // ROS_WARN("%s --------------------------------------------------------", NN);

    ros::init(argc, argv, "data_node");
    log4cxx::MDC::put("node", "|data_node|");
    ros::NodeHandle nh;
    CTopic topic; // Экземпляр класса для всех публикуемых топиков
    ros::Rate r(RATE);

    ros::Subscriber sub_ControlModul = nh.subscribe("pbPos/ControlModul", 16, callback_ControlModul, ros::TransportHints().tcpNoDelay(true));        // Это мы подписываемся на то что публигует Main для Modul
    ros::Subscriber sub_ControlPrint = nh.subscribe("pbWrite/Write2Print", 16, callback_ControlPrint, ros::TransportHints().tcpNoDelay(true));       // Это мы подписываемся на то что публигует Main для Print
    ros::Subscriber sub_ControlDriver = nh.subscribe("pbControl/ControlDriver", 16, callback_ControlDriver, ros::TransportHints().tcpNoDelay(true)); // Это мы подписываемся на то что публигует Main для Data
    ros::Subscriber subscriber_Joy = nh.subscribe("joy", 16, callback_Joy);                                                                          // Это мы подписываемся на то что публикует нода джойстика

    JerkLimitedProfile left_wheel;  // Профиль для левого колеса
    JerkLimitedProfile right_wheel; // Профиль для правого колеса

    jlp_init(&left_wheel, "left", 0.0, 0.5, 0.5, 1.0);   // Инициализируем: начальная скорость 0 м/с, j_max=1.0, a_max=1.0, v_max = 1.0
    jlp_init(&right_wheel, "right", 0.0, 0.5, 0.5, 1.0); // Инициализируем: начальная скорость 0 м/с, j_max=1.0, a_max=1.0, v_max = 1.0

    left_wheel.enable_diagnostics = 1;  // Включаем диагностику для теста
    // right_wheel.enable_diagnostics = 1; // Включаем диагностику для теста

    ROS_INFO("p->enable_diagnostics L %i ", left_wheel.enable_diagnostics);
    ROS_INFO("p->enable_diagnostics R %i ", right_wheel.enable_diagnostics);

    // jlp_start_profile(&left_wheel, 0.0); // Запускаем первый профиль — в начальный момент скорость 0 м/с
    // jlp_start_profile(&right_wheel, 0.0); // Запускаем первый профиль — в начальный момент скорость 0 м/с

    // sub_low_state = _nh.subscribe("/low_state", 1, &IOInterface::_lowStateCallback, this, ros::TransportHints().tcpNoDelay(true)); // От Максима пример
    //*****************
    readParam(); // Считывание переменных параметров из лаунч файла при запуске. Там офсеты и режимы работы
    //*****************
    init_Gpio();                        // Настройка пинов по номерам wiringpi
    set_PIN_Led();                      // Устанавливаем и обьявляем пины. для вывода анализатора светодиодов и всего прочего
    init_SPI(SPI_CHANNAL_0, SPI_SPEED); // Инициализация нужного канала SPI
    // init_SPI(SPI_CHANNAL_1, SPI_SPEED); // Инициализация нужного канала SPI
    bool led_status = 0;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    // double dt = (current_time - last_time).toSec();

    ROS_INFO("+++ test laser... Waiting 7 sec...");
    ros::Duration(1).sleep(); // Подождем пока все обьявится и инициализируется внутри ROS

    // Data2Modul.id++;                                       //= 0x1F1F1F1F;
    // Data2Modul.controlMotor.mode = 0;                      // Ручной вариант проверка
    // Data2Modul.cheksum = measureCheksum(Data2Modul);       // Считаем контрольную сумму отправляемой структуры// тут нужно посчитать контрольную сумму структуры
    // sendData2Modul(SPI_CHANNAL_0, Modul2Data, Data2Modul); // Обмен данными с нижним уровнем
    // ros::Duration(1).sleep();                      // Подождем пока все обьявится и инициализируется внутри ROS

    Data2Modul.id++;                                       //= 0x1F1F1F1F;
    Data2Modul.controlMotor.mode = 9;                      // Ручной вариант проверка
    Data2Modul.controlLaser.mode = 0;                      // Ручной вариант проверка
    Data2Modul.cheksum = measureCheksum(Data2Modul);       // Считаем контрольную сумму отправляемой структуры// тут нужно посчитать контрольную сумму структуры
    sendData2Modul(SPI_CHANNAL_0, Modul2Data, Data2Modul); // Обмен данными с нижним уровнем
    ros::Duration(7).sleep();                              // Подождем пока все обьявится и инициализируется внутри ROS

    // Data2Modul.id++;                                       //= 0x1F1F1F1F;
    // Data2Modul.controlMotor.mode = 1;                      // Ручной вариант проверка
    // Data2Modul.cheksum = measureCheksum(Data2Modul);       // Считаем контрольную сумму отправляемой структуры// тут нужно посчитать контрольную сумму структуры
    // sendData2Modul(SPI_CHANNAL_0, Modul2Data, Data2Modul); // Обмен данными с нижним уровнем
    // ros::Duration(1).sleep();                              // Подождем пока все обьявится и инициализируется внутри ROS
    ROS_INFO("--- test laser");

    uint64_t timeWork = millis(); // Время работы ноды
    ROS_WARN("End Setup. Start loop.");

    while (ros::ok())
    {
        // ROS_INFO(""); // С новой строки в логе новый цикл
        led_status = 1 - led_status; // Мигаем с частотой работы цикла
        digitalWrite(PIN_LED_BLUE, led_status);

        static ros::Time start_time = ros::Time::now(); // Захватываем начальный момент времени
        ros::Time now_time = ros::Time::now();          // Захватываем текущий момент времени
        ros::Duration duration = now_time - start_time; // Находим разницу между началом и концом
        double dt = duration.toSec();                   // Получаем количество секунд
        start_time = now_time;                          // Запоминаем на будущий расчет

        ros::spinOnce(); // Обновление в данных в ядре ROS, по этой команде происходит вызов функции обратного вызова
        ROS_INFO_THROTTLE(THROTTLE_PERIOD_1, "%u msec. SPI Modul %u/%u %u/%u | Driver %u/%u %u/%u | Print %u/%u %u/%u |", millis(),
                          Modul2Data.spi.all, Modul2Data.spi.bed, data_modul_all, data_modul_bed,
                          Driver2Data.spi.all, Driver2Data.spi.bed, data_driver_all, data_driver_bed,
                          Print2Data.spi.all, Print2Data.spi.bed, data_print_all, data_print_bed);
        //-----------------------------------------------------------------------------------------------------------------------------------
        if (flag_msgControlModul) // Если пришло сообщение в топике и сработал колбек
        {
            flag_msgControlModul = false;
            collect_Data2Modul(1); // Обрабатываем пришедшие данные и пишем в перемнные для передачи на нижний уровень
        }
        if (flag_msgControlPrint) // Если пришло сообщение в топике и сработал колбек
        {
            flag_msgControlPrint = false;
            collect_Data2Print(1); // Обрабатываем пришедшие данные и пишем в переменные для передачи на нижний уровень
        }
        if (flag_msgControlDriver) // Если пришло сообщение в топике и сработал колбек
        {
            flag_msgControlDriver = false;
            collect_Data2Driver(1); // Обрабатываем пришедшие данные и пишем в переменные для передачи на нижний уровень
        }

        //----------------------------- ТУТ Смотрим по времени, если нет данных по топикам то отправляем значения по умолчанию ------------------------------------------------------------------
        if (millis() - timeSpiModul > 3000) // Если по топикам не пришли данные больше 3 секунд то отправляем значения по умолчанию
            collect_Data2Modul(0);
        if (millis() - timeSpiDriver > 3000) // Если по топикам не пришли данные больше 3 секунд то отправляем значения по умолчанию
            collect_Data2Driver(0);
        if (millis() - timeSpiPrint > 3000) // Если по топикам не пришли данные больше 3 секунд то отправляем значения по умолчанию
            collect_Data2Print(0);

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

        // printf("g_factSpeed.L = %f g_factSpeed.R= %f \n", g_factSpeed.speedL, g_factSpeed.speedR);

        Data2Driver.control = speedToRps(g_factSpeed); // Конвертация скорости из метров в секунду в обороты в секунду для передачи на нижний уровень
        // printf("Data2Driver.controlL = %f Data2Driver.controlR= %f \n \n",Data2Driver.control.speedL,Data2Driver.control.speedR);
        controlLed();   // Функция управления несколькими светодиодами которые отведены для прямого управления нодой data
        setModeModul(); // Установка режима работы - колибровки модуля на основании переменной из лаунч файла

        // --------------------------- ОТПРАВКА ДАННЫХ на нижний уровень и разборка и публикация данных полученных с нижнего уровня---------------------------------------------------------------------
        Data2Modul.id++;                                 //= 0x1F1F1F1F;
        Data2Modul.cheksum = measureCheksum(Data2Modul); // Считаем контрольную сумму отправляемой структуры// тут нужно посчитать контрольную сумму структуры
                                                         // printf("Отправляем: Id %i, чек= %i  ", Data2Modul.id, Data2Modul.cheksum);
                                                         // rezModul = sendData2Modul(SPI_CHANNAL_0, Modul2Data, Data2Modul); // Обмен данными с нижним уровнем

        // rezModul = sendData2Modul(SPI_CHANNAL_0, Modul2Test, Test2Modul); // Обмен данными с нижним уровнем
        rezModul = sendData2Modul(SPI_CHANNAL_0, Modul2Data, Data2Modul); // Обмен данными с нижним уровнем

        // uint8_t test[4]{0x01, 0x04, 0xFF, 0xAA};
        // uint8_t test[2]{0x01, 0x02};
        // digitalWrite(PIN_SPI_MODUL, 0);
        // delayMicroseconds(1);
        // printf("out- %#x %#x %#x %#x\n", test[0], test[1], test[2], test[3]);
        // rez = wiringPiSPIDataRW(SPI_CHANNAL_0, test, 4); // Передаем и одновременно получаем данные
        // printf(" in- %#X %#X %#X %#X\n\n", test[0], test[1], test[2], test[3]);
        // delayMicroseconds(1);
        // digitalWrite(PIN_SPI_MODUL, 1);

        if (rezModul) // Если пришли хорошие данные с нижнего уровня, то обрабатываем их и публикуем данные в ROS
        {
            // СДЕЛАТЬ СКОРОСТЬ ПУБЛИКАЦИИ ЕСЛИ БУДЕТ 100Герц то нафига так часто визуализацию публиковать
            topic.processing_Modul2Data(); // Обрабатываем данные
        }
        //----------------------------
        // Data2Print.id++;                                                  //= 0x1F1F1F1F;
        // Data2Print.cheksum = measureCheksum(Data2Print);                  // Считаем контрольную сумму отправляемой структуры// тут нужно посчитать контрольную сумму структуры
        // rezPrint = sendData2Print(SPI_CHANNAL_0, Print2Data, Data2Print); //  Отправляем данные на нижний уровень
        // if (rezPrint)                                                     // Если пришли хорошие данные то обрабатываем их и публикуем данные в ROS
        // {
        //     topic.processing_Print2Data(); // Обработанные данные записываем их в структуру для публикации в топике и публикуем
        // }
        //----------------------------
        Data2Driver.id++;                                                   //= 0x1F1F1F1F; Считаем каждый раз сколько отправляем, даже если не было изменений в данных ни от джойстика ни от топика от Head
        Data2Driver.cheksum = measureCheksum(Data2Driver);                  // Пересчитываем  контрольную сумму отправляемой структуры
        rezData = sendData2Driver(SPI_CHANNAL_0, Driver2Data, Data2Driver); ////  Отправляем данные на нижний уровень
        // ROS_INFO("id= %i speedL= %f speedR= %f cheksum = %i", Data2Driver.id, Data2Driver.control.speedL, Data2Driver.control.speedR, Data2Driver.cheksum);
        if (rezData) // Если пришли хорошие данные то обрабатываем их и публикуем данные в ROS
        {
            topic.processing_Driver2Data();         // Обработанные данные записываем их в структуру для публикации в топике и публикуем
            topic.processing_SetSpeed(g_factSpeed); // Заполнение и публикация заданной скорости Публикуем если только ее успешно отправили на нижний уровень
        }
        //-----------------------------------------------------------------------------------------------------------------------------------

        topic.processingSPI(); // Сбор и публикация статистики обмена по SPI
        r.sleep();             // Интеллектуальная задержка на указанную частоту
    }
    //! isnan(rez_data); // true если nan
    Data2Modul.id++;                                       //= 0x1F1F1F1F;
    Data2Modul.controlMotor.mode = 0;                      // Ручной вариант проверка
    Data2Modul.controlLaser.mode = 0;                      // Ручной вариант проверка
    Data2Modul.cheksum = measureCheksum(Data2Modul);       // Считаем контрольную сумму отправляемой структуры// тут нужно посчитать контрольную сумму структуры
    sendData2Modul(SPI_CHANNAL_0, Modul2Data, Data2Modul); // Обмен данными с нижним уровнем
    printf("data_node STOP \n");
    return 0;
}
