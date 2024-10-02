#include "genStruct.h" // Тут все общие структуры. Истользуются и Data и Main и Control
#include "control_code/config.h"
#include "control_code/topic.h" // Файл для функций для формирования топиков в нужном виде и формате
#include "control_code/code.h"

int main(int argc, char **argv)
{
    ROS_INFO("%s -------------------------------------------------------------", NN);
    ROS_WARN("%s  Control Module PrintBIM ROS 1.0 Raspberry Pi 4B  ver 1.0001 ", NN);
    ROS_ERROR("%s ------------------ROS_ERROR----------------------------------", NN);

    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;
    CTopic topic; // Экземпляр класса для всех публикуемых топиков

    //----------------------------- ПОДПИСКИ НА ТОПИКИ -------НЕ УБИРАЮ В КЛАСС ТАК КАК НУЖНЫ ГЛОБАЛЬНЫЕ КОЛБЕКИ И ПРОЧАЯ ХЕРНЯ --------
    ros::Subscriber subscriber_Driver = nh.subscribe<pb_msgs::Struct_Driver2Data>("pbData/Driver", 1000, callback_Driver);
    // ros::Subscriber subscriber_Speed = nh.subscribe<pb_msgs::SSetSpeed>("pbData/Speed", 1000, callback_Speed);

    ros::Rate r(RATE);        // Частота в Герцах - задержка
    ros::Duration(2).sleep(); // Подождем пока все обьявится и инициализируется внутри ROS

    u_int64_t time2 = 0;
    u_int64_t time3 = 0;
    u_int64_t time4 = 0;
    u_int64_t time5 = 0;
    u_int64_t time6 = 0;

    time2 = millis() + 5000;
    time3 = millis() + 10000;
    time4 = millis() + 15000;
    time5 = millis() + 20000;
    time6 = millis() + 25000;

    bool flagUp = false;    // Флаг что
    bool flagDown = false;  // Флаг
    bool flagStop = false;  // Флаг что
    bool flagStop2 = false;  // Флаг что
    bool flagSReset = false;  // Флаг что

    while (ros::ok())
    {
        ros::spinOnce(); // Опрашиваем ядро ROS и по этой команде наши срабатывают колбеки. Нужно только для подписки на топики

        printf("\n");
        printf("%u --- \n", millis());

        if (flag_msgDriver) // Флаг что пришло сообщение от ноды Data по Driver. Тут пишем какую-то обработку данных если нужно.
        {
            flag_msgDriver = false;
        }
        // if (flag_msgSpeed) // Флаг что пришло сообщение от ноды Data по Speed.
        // {
        //     flag_msgSpeed = false;
        // }
        if (time2 < millis() &&
            flagUp == false)
        {
            controlSpeed.control.speedL = 0.1;
            controlSpeed.control.speedR = 0.1;
            flagUp = true; // Взводим флаг что поехали
            printf("flagUp \n");
        }
        if (time3 < millis() &&
            flagStop == false)
        {
            controlSpeed.control.speedL = 0.0;
            controlSpeed.control.speedR = 0.0;
            flagStop = true; // Взводим флаг что поехали
            printf("flagStop \n");
        }
        if (time4 < millis() &&
            flagDown == false)
        {
            controlSpeed.control.speedL = -0.1;
            controlSpeed.control.speedR = -0.1;
            flagDown = true; // Взводим флаг что поехали
            printf("flagDown \n");
        }
        if (time5 < millis() &&
            flagStop2 == false)
        {
            controlSpeed.control.speedL = 0.0;
            controlSpeed.control.speedR = 0.0;
            flagStop2 = true; // Взводим флаг что поехали
            printf("flagStop2 \n");
        }
        if (time6 < millis())
        {
            time2 = millis();
            time3 = time2 + 5000;
            time4 = time3 + 5000;
            time5 = time4 + 5000;
            time6 = time5 + 5000;
            flagUp = false;
            flagDown = false;
            flagStop = false;
            flagStop2 = false;
            printf("flagSReset \n");
        }
        topic.publicationControlDriver(); // Формируем и Публикуем команды для управления Driver
        r.sleep(); // Интеллектуальная задержка на указанную частоту
    }

    printf("Control node STOP \n");
    return 0;
}


// ============================================================================
        // if (flagUp == false &&
        //     millis() - time > 5000) // Если прошло с запуска 3 секунды и еще не едем
        // {
        //     controlSpeed.control.speedL = 0.1;
        //     controlSpeed.control.speedR = 0.1;
        //     flagUp = true;     // Взводим флаг что поехали
        //     timeUp = millis(); // Запоминаем время старта
        //     printf("flagUp \n");
        // }

        // if ((flagUp == true) &&
        //     (flagStop == false) &&
        //     (millis() - timeUp) > 5000)
        // {
        //     controlSpeed.control.speedL = 0.0;
        //     controlSpeed.control.speedR = 0.0;
        //     flagStop = true;
        //     timeStop = millis(); // Запоминаем время Остановки
        //     printf("flagStop \n");
        // }

        // if (flagStop == true &&
        //     flagDown == false &&
        //     (millis() - timeStop) > 5000)
        // {
        //     controlSpeed.control.speedL = -0.1;
        //     controlSpeed.control.speedR = -0.1;
        //     flagDown = true;  // Флаг что поехади назад
        //     flagDown2 = true; // Флаг что поехади назад
        //     timeDown = millis();
        //     printf("flagDown \n");
        // }
        // if ((flagDown2 == true) &&
        //     (millis() - timeDown) > 5000)
        // {
        //     controlSpeed.control.speedL = 0.0;
        //     controlSpeed.control.speedR = 0.0;
        //     flagDown2 = false;
        //     printf("flagStop \n");
        // }
