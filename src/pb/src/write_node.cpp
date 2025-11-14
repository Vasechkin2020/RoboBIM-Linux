
#include "genStruct.h" // Тут все общие структуры. Истользуются и Data и Main и Head
#include "write_code/config.h"
#include "write_code/topic.h" // Файл для функций для формирования топиков в нужном виде и формате
#include "write_code/code.h"

int main(int argc, char **argv)
{
    ROS_WARN("%s ---------------------------------------------------------", NN);
    ROS_WARN("%s  Write Node PrintBIM ROS 1.0 Raspberry Pi 4B  ver 1.0001 ", NN);
    ROS_WARN("%s ----------------ROS_ERROR-------------------------------", NN);

    ros::init(argc, argv, "write_node");
    log4cxx::MDC::put("node", "|write_node|"); 
    ros::NodeHandle nh;
    CTopic topic; // Экземпляр класса для всех публикуемых топиков

    //----------------------------- ПОДПИСКИ НА ТОПИКИ -------НЕ УБИРАЮ В КЛАСС ТАК КАК НУЖНЫ ГЛОБАЛЬНЫЕ КОЛБЕКИ И ПРОЧАЯ ХЕРНЯ --------
    // ros::Subscriber subscriber_Driver = nh.subscribe<pb_msgs::Struct_Driver2Data>("pbData/Driver", 1000, callback_Driver);
    // ros::Subscriber subscriber_Speed = nh.subscribe<pb_msgs::SSetSpeed>("pbData/Speed", 1000, callback_Speed);

	ros::Publisher subscriber_Print = nh.advertise<pb_msgs::Struct_Data2Print>("pb/Data/Print", 3); // Это мы создаем публикатор и определяем название топика в рос

    initCommandArray(); // Заполнение массива команд
    u_int64_t time = millis();
    u_int64_t timeStart = millis();
    u_int64_t timeMil = millis();
    bool flagCommand = true; // Флаг можно исполнять каманду
    int i = 0;

    ros::Rate r(RATE);        // Частота в Герцах - задержка
    ros::Duration(1).sleep(); // Подождем пока все обьявится и инициализируется внутри ROS
   
    ROS_WARN("End Setup. Start loop.\n");
    while (ros::ok())   
    {
        ROS_INFO(""); // С новой строки в логе новый цикл
        ros::spinOnce(); // Опрашиваем ядро ROS и по этой команде наши срабатывают колбеки. Нужно только для подписки на топики

        if (timeStart + 1000 < millis()) // Задаержка перед началом работы
        {
            if (flagCommand)
            {
                flagCommand = false;
                Write2Data.controlPrint.mode = commandArray[i].mode;
                Write2Data.controlPrint.status = commandArray[i].status;
                Write2Data.controlPrint.position = commandArray[i].position;
                Write2Data.controlPrint.velocity = commandArray[i].velocity;
                Write2Data.controlPrint.torque = commandArray[i].torque;

                time = commandArray[i].duration + millis();
                printf("commandArray i= %i \n", i);    
            }

            if (time < millis())
            {
                flagCommand = true;
                i++;
                if (commandArray[i].mode == 3)
                {
                    printf("New loop ");
                    i = 0;
                }
                printf("i = %i \n", i);
            }
        }

        topic.publicationWrite2Data();

        if (timeMil < millis())
        {
            printf("%u --- \n", millis());
            timeMil = millis() + 1000;
        }
        r.sleep(); // Интеллектуальная задержка на указанную частоту
    }
    printf("Write_node STOP \n");
    return 0;
}
