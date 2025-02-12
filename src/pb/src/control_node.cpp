#include "genStruct.h" // Тут все общие структуры. Истользуются и Data и Main и Control
#include "control_code/config.h"
#include "control_code/topic.h" // Файл для функций для формирования топиков в нужном виде и формате
#include "control_code/code.h"

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
    ros::Subscriber subscriber_Driver = nh.subscribe<pb_msgs::Struct_Driver2Data>("pbData/Driver", 1000, callback_Driver);
    // ros::Subscriber subscriber_Speed = nh.subscribe<pb_msgs::SSetSpeed>("pbData/Speed", 1000, callback_Speed);

    ros::Rate r(RATE);        // Частота в Герцах - задержка
    ros::Duration(1).sleep(); // Подождем пока все обьявится и инициализируется внутри ROS
    
    readParam(); // Считывание переменных параметров из лаунч файла при запуске. Там офсеты и режимы работы
    initCommandArray(verComand); // Заполнение маасива команд

    u_int64_t time = millis();
    u_int64_t timeStart = millis();
    u_int64_t timeMil = millis();
    bool flagCommand = true; // Флаг можно исполнять каманду
    int i = 0;

    std::list<int> numbers{1, 2, 3, 4, 5};
    std::list<SCommand> listok;
    // list int listok;
    
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
                    ROS_INFO("New loop");
                    i = 0;
                }
                printf("i = %i \n", i);
            }
        }

        // if (flag_msgDriver) // Флаг что пришло сообщение от ноды Data по Driver. Тут пишем какую-то обработку данных если нужно.
        // {
        //     flag_msgDriver = false;
        // }
        // if (flag_msgSpeed) // Флаг что пришло сообщение от ноды Data по Speed.
        // {
        //     flag_msgSpeed = false;
        // }

        topic.publicationControlDriver(); // Формируем и Публикуем команды для управления Driver
        r.sleep();                        // Интеллектуальная задержка на указанную частоту
    }
    controlSpeed.control.speedL = 0;
    controlSpeed.control.speedR = 0;
    topic.publicationControlDriver(); // Формируем и Публикуем команды для управления Driver
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
