#include "write_code/config.h"
#include "write_code/topic.h" // Файл для функций для формирования топиков в нужном виде и формате
#include "write_code/code.h"

int main(int argc, char **argv)
{
    ROS_INFO("%s ---------------------------------------------------------", NN);
    ROS_WARN("%s  Write Node PrintBIM ROS 1.0 Raspberry Pi 4B  ver 1.0001 ", NN);
    ROS_ERROR("%s ----------------ROS_ERROR-------------------------------", NN);

    ros::init(argc, argv, "write_node");
    ros::NodeHandle nh;
    CTopic topic; // Экземпляр класса для всех публикуемых топиков

    //----------------------------- ПОДПИСКИ НА ТОПИКИ -------НЕ УБИРАЮ В КЛАСС ТАК КАК НУЖНЫ ГЛОБАЛЬНЫЕ КОЛБЕКИ И ПРОЧАЯ ХЕРНЯ --------
    // ros::Subscriber subscriber_Driver = nh.subscribe<pb_msgs::Struct_Driver2Data>("pbData/Driver", 1000, callback_Driver);
    // ros::Subscriber subscriber_Speed = nh.subscribe<pb_msgs::SSetSpeed>("pbData/Speed", 1000, callback_Speed);

    u_int64_t timeMil = millis();

    ros::Rate r(RATE);        // Частота в Герцах - задержка
    ros::Duration(1).sleep(); // Подождем пока все обьявится и инициализируется внутри ROS

    while (ros::ok())
    {
        ros::spinOnce(); // Опрашиваем ядро ROS и по этой команде наши срабатывают колбеки. Нужно только для подписки на топики
        topic.publicationWrite2Data();

        if (timeMil < millis())
        {
            printf("%u --- \n", millis());
            timeMil = millis() + 1000;
        }
        r.sleep();                        // Интеллектуальная задержка на указанную частоту
    }
    printf("Write_node STOP \n");
    return 0;
}
