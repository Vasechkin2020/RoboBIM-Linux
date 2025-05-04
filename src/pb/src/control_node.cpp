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

    ros::Rate r(200);         // Частота в Герцах - задержка
    ros::Duration(3).sleep(); // Подождем пока все обьявится и инициализируется внутри ROS

    readParam();                 // Считывание переменных параметров из лаунч файла при запуске. Там офсеты и режимы работы
    initCommandArray(verComand); // Заполнение маасива команд

    // static ros::Time time = ros::Time::now();      // Захватываем начальный момент времени
    static ros::Time timeStart = ros::Time::now(); // Захватываем начальный момент времени
    static ros::Time timeNow = ros::Time::now();   // Захватываем конечный момент времени

    u_int64_t time = millis();
    // u_int64_t timeStart = millis();
    // u_int64_t timeMil = millis();
    int i = 0;

    std::list<int> numbers{1, 2, 3, 4, 5};
    std::list<SCommand> listok;
    // list int listok;

    GCodeParser parser; // Создание объекта парсера
    parser.run(); // Запуск обработки

    ROS_WARN("End Setup. Start loop.\n");
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
            case 1:
                controlSpeed.control.speedL = commandArray[i].velL;
                controlSpeed.control.speedR = commandArray[i].velR;
                time = commandArray[i].duration + millis();
                ROS_INFO("    Time Start");
                break;
            case 2:
                time = millis() + 999999; // Огромное время ставим
                flagAngle = true;         // Флаг что теперь отслеживаем угол
                ROS_INFO("    Angle Start");
                break;
            case 3:
                vectorStart.x = msg_Pose.x.mode0; // Запоминаем те координаты которые были в момент начала движения
                vectorStart.y = msg_Pose.y.mode0;
                time = millis() + 999999; // Огромное время ставим
                flagVector = true;        // Флаг что теперь отслеживаем длину вектора
                ROS_INFO("    Vector Start. len = %f", commandArray[i].len);
                break;
            }
            // if (commandArray[i].mode == 1) // Если режим ездить по времени
            // {
            //     controlSpeed.control.speedL = commandArray[i].velL;
            //     controlSpeed.control.speedR = commandArray[i].velR;
            //     time = commandArray[i].duration + millis();
            // }
            // else if (commandArray[i].mode == 2) // Ездить не обращая внимание на время до результата по углу
            // {
            //     time = millis() + 999999; // Огромное время ставим
            //     flagAngle = true;         // Флаг что теперь отслеживаем угол
            //     ROS_INFO("    Angle Start");
            // }
            // else if (commandArray[i].mode == 3) // Ездить не обращая внимание на время до результата по Координатам
            // {
            //     time = millis() + 999999; // Огромное время ставим
            //     flagVector = true;         // Флаг что теперь отслеживаем длину вектора
            //     ROS_INFO("    Vector Start");
            // }
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
            if (commandArray[i].mode == 9)
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
    controlSpeed.control.speedL = 0;
    controlSpeed.control.speedR = 0;
    topic.publicationControlDriver(); // Формируем и Публикуем команды для управления Driver
    printf("Control node STOP \n");
    return 0;
}