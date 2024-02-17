
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

    ros::Subscriber sub_ControlDriver = nh.subscribe("pbData/ControlDriver", 16, callback_ControlDriver); // Это мы подписываемся на то что публигует Main для Data
    ros::Subscriber sub_ControlModul = nh.subscribe("pbData/ControlModul", 16, callback_ControlModul);    // Это мы подписываемся на то что публигует Main для Modul
    ros::Subscriber subscriber_Joy = nh.subscribe("joy", 16, callback_Joy);                               // Это мы подписываемся на то что публикует нода джойстика

    CJoy joy(0.5, 0.5); // Обьявляем экземпляр класса в нем вся обработка джойстика
    CTopic topic;       // Экземпляр класса для всех публикуемых топиков

    wiringPiSetup();                    // Инициализация библиотеки
    set_PIN_Led();                      // Устанавливаем и обьявляем пины. для вывода анализатора светодиодов и всего прочего
    init_SPI(SPI_CHANNAL_0, SPI_SPEED); // Инициализация нужного канала SPI
    init_SPI(SPI_CHANNAL_1, SPI_SPEED); // Инициализация нужного канала SPI
    bool led_status = 0;
    ros::Duration(1).sleep(); // Подождем пока все обьявится и инициализируется внутри ROS

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    double vx = 0.03;
    double vy = 0.3;
    double vth = 0.5;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    ros::Duration(1).sleep();
    while (ros::ok())
    {
        // current_time = ros::Time::now();

        // // compute odometry in a typical way given the velocities of the robot
        // double dt = (current_time - last_time).toSec();
        // double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        // double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        // double delta_th = vth * dt;

        // x += delta_x;
        // y += delta_y;
        // th += delta_th;

        // last_time = current_time;

        // printf(" x= %.6f y= %.6f \n ", x, y);

        
        // Сделать вывозтолько если пригли данные а не постоянно отправку поманд
        //  Led_Blink(PIN_LED_BLUE, 500); // Мигание светодиодом, что цикл работает
        led_status = 1 - led_status; // Мигаем с частотой работы цикла
        digitalWrite(PIN_LED_BLUE, led_status);

        ros::spinOnce();                                                  // Обновление в данных в ядре ROS, по этой команде происходит вызов функции обратного вызова
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
        if (!rez_data) // Если пришли плохие данные то выводим ошибку
        {
            // ROS_WARN("%s Flag_bedData chek_sum BED Modul", NN);
        }
        //-----------------------------------------------------------------------------------------------------------------------------------
        if (flag_msgControlDriver) // Если пришло сообщение в топике и сработал колбек
        {
            flag_msgControlDriver = false;
            collect_Data2Driver(); // Собираем рабочие данные в структуру для передачи считывая данные из топика ноды Head
        }

        if (flag_msgJoy) // Если пришло новое сообшение и сработал колбек то разбираем что там пришло
        {
            flag_msgJoy = false;     // Флаг сбратываем .Приоритет джойстику
            joy.parsingJoy(msg_joy); // Разбираем и формируем команды из полученного сообщения
            // pub_JoyData.publish(joy._joy2Head); // Публикация данных разобранных из джойстика
            joy.transform(); // Преобразование кнопок джойстика в реальные команды
            // Копирование данных из сообщения в топике в структуру для передачи по SPI
            Data2Driver.control.speedL = joy._ControlDriver.control.speedL;
            Data2Driver.control.speedR = joy._ControlDriver.control.speedR;
            // topic.publicationControlDriver(joy._ControlDriver); // Публикация данных по управлению Driver
        }

        Data2Driver.id++;                                  //= 0x1F1F1F1F; Считаем каждый раз сколько отправляем, даже если не было изменений в данных ни от джойстика ни от топика от Head
        Data2Driver.cheksum = measureCheksum(Data2Driver); // Пересчитываем  контрольную сумму отправляемой структуры
        // ROS_INFO("id= %i speedL= %f speedR= %f cheksum = %i", Data2Driver.id, Data2Driver.control.speedL, Data2Driver.control.speedR, Data2Driver.cheksum);
        rez_data = sendData2Driver(SPI_CHANNAL_1, Driver2Data, Data2Driver); ////  Отправляем данные на нижний уровень

        data_driver_all++;
        if (rez_data) // Если пришли хорошие данные то обрабатываем их и публикуем данные в ROS
        {
            calculateOdometryFromEncoder(Data2Driver.control); // Обработка пришедших данных.Обсчитываем одометрию по энкодеру
            topic.visualEncoderOdom();
            topic.transform(encoder.pose);

            // calculateOdometryFromMpu();                               // Обработка пришедших данных.Обсчитываем одометрию по датчику MPU BNO055
            // visualEncoderMpu();
            // publish_OdomMpu.publish(odomMpu_msg);         // Публикация полученных данных
            topic.processing_Driver2Data(); // Обработанные данные записываем их в структуру для публикации в топике
            // publish_Driver2Data.publish(Driver2Data_msg); // Публикация полученных данных
        }
        //-----------------------------------------------------------------------------------------------------------------------------------
        topic.processingSPI(); // Сбор и публикация статистики обмена по SPI

        r.sleep(); // Интеллектуальная задержка на указанную частоту
    }
    return 0;
}
