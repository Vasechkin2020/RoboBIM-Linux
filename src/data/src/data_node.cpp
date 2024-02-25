
#include "lib.h"
#include "data_code/config.h"
#include "data_code/c_joy.h"
#include "data_code/data2driver.h"
#include "data_code/data2modul.h"
#include "data_code/code.h"
#include "data_code/topic.h" // Файл для функций для формирования топиков в нужном виде и формате

int main(int argc, char **argv)
{
    ROS_INFO("%s -------------------------------------------------------", NN);
    ROS_WARN("%s Start Data_Node printBIM(c) 2024 printBIM.com ver. 1.0 ", NN);
    ROS_INFO("%s -------------------------------------------------------", NN);

    ros::init(argc, argv, "data_node");
    ros::NodeHandle nh;
    // ros::Time current_time = ros::Time::now(); // Время ROS
    // ros::Time last_time2 = ros::Time::now();

    // double dt = (current_time - last_time).toSec();
    ros::Rate r(RATE);

    ros::Subscriber sub_ControlDriver = nh.subscribe("pbData/ControlDriver", 16, callback_ControlDriver, ros::TransportHints().tcpNoDelay(true)); // Это мы подписываемся на то что публигует Main для Data
    ros::Subscriber sub_ControlModul = nh.subscribe("pbData/ControlModul", 16, callback_ControlModul, ros::TransportHints().tcpNoDelay(true));    // Это мы подписываемся на то что публигует Main для Modul
    ros::Subscriber subscriber_Joy = nh.subscribe("joy", 16, callback_Joy);                                                                       // Это мы подписываемся на то что публикует нода джойстика

    // sub_low_state = _nh.subscribe("/low_state", 1, &IOInterface::_lowStateCallback, this, ros::TransportHints().tcpNoDelay(true)); // От Максима пример

    CJoy joy(0.5, 0.5); // Обьявляем экземпляр класса в нем вся обработка джойстика
    CTopic topic;       // Экземпляр класса для всех публикуемых топиков

    wiringPiSetup();                    // Инициализация библиотеки
    set_PIN_Led();                      // Устанавливаем и обьявляем пины. для вывода анализатора светодиодов и всего прочего
    init_SPI(SPI_CHANNAL_0, SPI_SPEED); // Инициализация нужного канала SPI
    init_SPI(SPI_CHANNAL_1, SPI_SPEED); // Инициализация нужного канала SPI
    bool led_status = 0;
    ros::Duration(1).sleep(); // Подождем пока все обьявится и инициализируется внутри ROS

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    initArray();

    while (ros::ok())
    {
        // Сделать вывозтолько если пригли данные а не постоянно отправку поманд
        //  Led_Blink(PIN_LED_BLUE, 500); // Мигание светодиодом, что цикл работает
        led_status = 1 - led_status; // Мигаем с частотой работы цикла
        digitalWrite(PIN_LED_BLUE, led_status);

        ros::spinOnce();                                                  // Обновление в данных в ядре ROS, по этой команде происходит вызов функции обратного вызова
        topic.transform();                                                // Трансформация odom to map
                                                                          //-----------------------------------------------------------------------------------------------------------------------------------
        Collect_Data2Modul();                                             // Собираем рабочие данные в структуру для передачи считывая из топиков
        rez_data = sendData2Modul(SPI_CHANNAL_0, Modul2Data, Data2Modul); //
        data_modul_all++;
        if (rez_data) // Если пришли хорошие данные то обрабатываем их и публикуем данные в ROS
        {
            topic.dataProcessing_Modul(); // Обрабатываем данные
                                          // СДЕЛАТЬ СКОРОСТЬ ПУБЛИКАЦИИ ЕСЛИ БУДЕТ 100Герц то нафига так часто визуализацию публиковать
            // publish_ModulMotor.publish(modul_motor_msg);   // Публикация полученных данных
            // publish_ModulLidar.publish(modul_lidar_msg);   // Публикация полученных данных
            // publish_ModulMicric.publish(modul_micric_msg); // Публикация полученных данных
        }
        //-----------------------------------------------------------------------------------------------------------------------------------
        if (flag_msgControlDriver) // Если пришло сообщение в топике и сработал колбек
        {
            flag_msgControlDriver = false;
            collect_Data2Driver(); // Обрабатываем пришедшие данные и пишем в глобальные перемнные
        }

        if (flag_msgJoy) // Если пришло новое сообшение и сработал колбек то разбираем что там пришло
        {
            flag_msgJoy = false;     // Флаг сбратываем .Приоритет джойстику
            joy.parsingJoy(msg_joy); // Разбираем и формируем команды из полученного сообщения
            joy.transform();         // Преобразование кнопок джойстика в реальные команды
            g_dreamSpeed.speedL = joy._ControlDriver.control.speedL;
            g_dreamSpeed.speedR = joy._ControlDriver.control.speedR;
            // pub_JoyData.publish(joy._joy2Head); // Публикация данных разобранных из джойстика
            // topic.publicationControlDriver(joy._ControlDriver); // Публикация данных по управлению Driver (для отладки)
        }

        controlAcc(Data2Driver.control, g_dreamSpeed); // Функция контроля ускорения На вход скорость с которой хотим ехать. После будет скорость с которой поедем фактически с учетом возможностей по ускорению

        Data2Driver.id++;                                                    //= 0x1F1F1F1F; Считаем каждый раз сколько отправляем, даже если не было изменений в данных ни от джойстика ни от топика от Head
        Data2Driver.cheksum = measureCheksum(Data2Driver);                   // Пересчитываем  контрольную сумму отправляемой структуры
        rez_data = sendData2Driver(SPI_CHANNAL_1, Driver2Data, Data2Driver); ////  Отправляем данные на нижний уровень
        // ROS_INFO("id= %i speedL= %f speedR= %f cheksum = %i", Data2Driver.id, Data2Driver.control.speedL, Data2Driver.control.speedR, Data2Driver.cheksum);

        data_driver_all++;
        if (rez_data) // Если пришли хорошие данные то обрабатываем их и публикуем данные в ROS
        {
            topic.processing_Driver2Data(); // Обработанные данные записываем их в структуру для публикации в топике и публикуем

            wheelTwistDt = calcTwistFromWheel(Data2Driver.control); // Обработка пришедших данных. По ним считаем линейные скорости по осям и угловую по углу. Запоминаем dt
            calcNewOdom(odomWheel, wheelTwistDt);                   // На основе линейных скоростей считаем новую позицию и угол
            topic.publishOdomWheel();                               // Публикация одометрии по моторам которая получается от начальной точки

            mpuTwistDt = calcTwistFromMpu(Driver2Data.bno055, 0.2); // асчет и оформление в структуру ускорений по осям (линейных скоростей) и  разделить получение угловых скоростей и расчет сновой точки на основе этих скоростей
            calcNewOdom(odomMpu, mpuTwistDt);                       // Обработка пришедших данных.Обсчитываем одометрию по датчику MPU BNO055
            topic.publishOdomMpu();

            // тут написать функцию комплементации данных угловых скоростей с разными условиями когда и в каком соотношении скомплементировать скорсти с двух источников
            unitedTwistDt = calcTwistUnited(wheelTwistDt, mpuTwistDt);
            calcNewOdom(odomUnited, unitedTwistDt); // // На основе линейных скоростей считаем новую позицию и угол
            topic.publishOdomUnited(); // Публикация одометрии по моторам с корректировкой с верхнего уровня
        }
        //-----------------------------------------------------------------------------------------------------------------------------------

        topic.processingSPI(); // Сбор и публикация статистики обмена по SPI

        r.sleep(); // Интеллектуальная задержка на указанную частоту
    }
    return 0;
}
