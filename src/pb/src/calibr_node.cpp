#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pb_msgs/Struct_Data2Driver.h>
#include <wiringPi.h>
#include <log4cxx/mdc.h>

#define RATE 10 // Частота шага

struct SCommand
{
    int mode = 0;     // Вид команды
    float velL = 0;   // Скорость колеса
    float velR = 0;   // Скорость колеса
    int duration = 0; // Длительность действия команды
};

SCommand commandArray[48]; // Массив команд
pb_msgs::Struct_Data2Driver controlSpeed;
void initCommandArray(); // Заполнение маасива команд

class CTopic /* Класс для функций для формирования топиков в нужном виде и формате и всех публикаций*/
{
public:
    void publicationControlDriver(); // Публикация данных для управления Driver
private:
    ros::NodeHandle _nh;
    ros::Time ros_time;                                                                                           // Время ROS
    ros::Publisher pub_ControlDriver = _nh.advertise<pb_msgs::Struct_Data2Driver>("pbControl/ControlDriver", 16); // Это мы публикуем структуру которую отправляем к исполнению на драйвер
};

// Публикация данных для управления Driver
void CTopic::publicationControlDriver()
{
    pb_msgs::Struct_Data2Driver data;
    static unsigned long led_time = 0;
    static int color = 0;

    if ((millis() - led_time) > 250)
    {
        color = 1 - color;
        led_time = millis();
    }
    for (int i = 0; i < 24; i++)
    {
        data.led.led[i] = color;
    }

    data.control = controlSpeed.control;
    pub_ControlDriver.publish(data);
}

// Заполнение маасива команд
void initCommandArray()
{
    commandArray[0].mode = 1;
    commandArray[0].duration = 30000;
    commandArray[0].velL = 0.1;
    commandArray[0].velR = -0.1;

    commandArray[1].mode = 1;
    commandArray[1].duration = 2000;
    commandArray[1].velL = 0.0;
    commandArray[1].velR = 0.0;

    commandArray[2].mode = 1;
    commandArray[2].duration = 30000;
    commandArray[2].velL = -0.1;
    commandArray[2].velR = 0.1;

    commandArray[3].mode = 1;
    commandArray[3].duration = 2000;
    commandArray[3].velL = 0.0;
    commandArray[3].velR = 0.0;

    commandArray[4].mode = 1;
    commandArray[4].duration = 30000;
    commandArray[4].velL = 0.05;
    commandArray[4].velR = -0.05;

    commandArray[5].mode = 1;
    commandArray[5].duration = 2000;
    commandArray[5].velL = 0.0;
    commandArray[5].velR = 0.0;

    commandArray[6].mode = 1;
    commandArray[6].duration = 30000;
    commandArray[6].velL = -0.05;
    commandArray[6].velR = 0.05;

    commandArray[7].mode = 1;
    commandArray[7].duration = 2000;
    commandArray[7].velL = 0.0;
    commandArray[7].velR = 0.0;

    commandArray[8].mode = 1;
    commandArray[8].duration = 30000;
    commandArray[8].velL = 0.03;
    commandArray[8].velR = -0.03;

    commandArray[9].mode = 1;
    commandArray[9].duration = 2000;
    commandArray[9].velL = 0.0;
    commandArray[9].velR = 0.0;

    commandArray[10].mode = 1;
    commandArray[10].duration = 30000;
    commandArray[10].velL = -0.03;
    commandArray[10].velR = 0.03;

    commandArray[11].mode = 1;
    commandArray[11].duration = 2000;
    commandArray[11].velL = 0.0;
    commandArray[11].velR = 0.0;
//----------------------------------------------------
    commandArray[12].mode = 1;
    commandArray[12].duration = 30000;
    commandArray[12].velL = 0.2;
    commandArray[12].velR = -0.2;

    commandArray[13].mode = 1;
    commandArray[13].duration = 2000;
    commandArray[13].velL = 0.0;
    commandArray[13].velR = 0.0;

    commandArray[14].mode = 1;
    commandArray[14].duration = 30000;
    commandArray[14].velL = -0.2;
    commandArray[14].velR = 0.2;

    commandArray[15].mode = 1;
    commandArray[15].duration = 2000;
    commandArray[15].velL = 0.0;
    commandArray[15].velR = 0.0;
//----------------------------------------------------

    commandArray[16].mode = 1;
    commandArray[16].duration = 2000;
    commandArray[16].velL = 0.03;
    commandArray[16].velR = -0.03;

    commandArray[17].mode = 1;
    commandArray[17].duration = 2000;
    commandArray[17].velL = 0.0;
    commandArray[17].velR = 0.0;

    commandArray[18].mode = 1;
    commandArray[18].duration = 2000;
    commandArray[18].velL = -0.03;
    commandArray[18].velR = 0.03;

    commandArray[19].mode = 1;
    commandArray[19].duration = 2000;
    commandArray[19].velL = 0.0;
    commandArray[19].velR = 0.0;

    commandArray[20].mode = 1;
    commandArray[20].duration = 2000;
    commandArray[20].velL = -0.03;
    commandArray[20].velR = 0.03;

    commandArray[21].mode = 1;
    commandArray[21].duration = 2000;
    commandArray[21].velL = 0.0;
    commandArray[21].velR = 0.0;

    commandArray[22].mode = 1;
    commandArray[22].duration = 2000;
    commandArray[22].velL = 0.03;
    commandArray[22].velR = -0.03;

    commandArray[23].mode = 1;
    commandArray[23].duration = 2000;
    commandArray[23].velL = 0.0;
    commandArray[23].velR = 0.0;

    commandArray[24].mode = 1;
    commandArray[24].duration = 2000;
    commandArray[24].velL = -0.03;
    commandArray[24].velR = 0.03;

    commandArray[25].mode = 1;
    commandArray[25].duration = 2000;
    commandArray[25].velL = 0.0;
    commandArray[25].velR = 0.0;

    //-----------------------
    commandArray[26].mode = 9;
}

int main(int argc, char **argv)
{
    ROS_FATAL("\n");
    ROS_FATAL("***  calibr_node *** ver. 1.0 *** printBIM.ru *** 2025 ***");
    ROS_FATAL("--------------------------------------------------------\n");

    ros::init(argc, argv, "calibr_node");
    log4cxx::MDC::put("node", "|calibr_node|");
    ros::NodeHandle nh;
    CTopic topic; // Экземпляр класса для всех публикуемых топиков

    ros::Rate r(RATE);        // Частота в Герцах - задержка
    ros::Duration(1).sleep(); // Подождем пока все обьявится и инициализируется внутри ROS

    initCommandArray(); // Заполнение маасива команд

    u_int64_t time = millis();
    u_int64_t timeStart = millis();
    u_int64_t timeMil = millis();
    bool flagCommand = true; // Флаг можно исполнять каманду
    int i = 0;

    ROS_WARN("End Setup. Start loop.\n");
    while (ros::ok())
    {
        // ROS_INFO(""); // С новой строки в логе новый цикл
        ros::spinOnce(); // Опрашиваем ядро ROS и по этой команде наши срабатывают колбеки. Нужно только для подписки на топики

        if (timeMil < millis())
        {
            printf("%u --- \n", millis());
            timeMil = millis() + 1000;
        }

        if (timeStart + 1000 < millis()) // Задаержка перед началом работы
        {
            if (flagCommand)
            {
                flagCommand = false;
                controlSpeed.control.speedL = commandArray[i].velL;
                controlSpeed.control.speedR = commandArray[i].velR;
                time = commandArray[i].duration + millis();
                ROS_INFO("commandArray i= %i", i);
            }

            if (time < millis())
            {
                flagCommand = true;
                i++;
                if (commandArray[i].mode == 9)
                {
                    ROS_INFO("exit");
                    i = 0;
                    exit(0);
                }
                printf("i = %i \n", i);
            }
        }

        topic.publicationControlDriver(); // Формируем и Публикуем команды для управления Driver
        r.sleep();                        // Интеллектуальная задержка на указанную частоту
    }
    controlSpeed.control.speedL = 0;
    controlSpeed.control.speedR = 0;
    topic.publicationControlDriver(); // Формируем и Публикуем команды для управления Driver
    printf("Control node STOP \n");
    return 0;
}