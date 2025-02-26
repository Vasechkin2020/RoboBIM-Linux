#include "genStruct.h" // Тут все общие структуры. Истользуются и Data и Main и Control
#include "control_code/config.h"
#include "control_code/topic.h" // Файл для функций для формирования топиков в нужном виде и формате
#include "control_code/code.h"

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
    bool flagCommand = true; // Флаг можно исполнять каманду
    bool flagAngle = false;  // Флаг отслеживания угла из топика
    int i = 0;

    std::list<int> numbers{1, 2, 3, 4, 5};
    std::list<SCommand> listok;
    // list int listok;

    ROS_WARN("End Setup. Start loop.\n");
    while (ros::ok())
    {
        timeNow = ros::Time::now(); // Захватываем текущий момент времени начала цикла
        // ROS_INFO(""); // С новой строки в логе новый цикл
        ros::spinOnce(); // Опрашиваем ядро ROS и по этой команде наши срабатывают колбеки. Нужно только для подписки на топики

        if (flagCommand)
        {
            flagCommand = false;
            ROS_INFO("    command Array i= %i Mode i= %i", i, commandArray[i].mode);
            if (commandArray[i].mode == 1) // Если режим ездить по времени
            {
                controlSpeed.control.speedL = commandArray[i].velL;
                controlSpeed.control.speedR = commandArray[i].velR;
                time = commandArray[i].duration + millis();
            }
            else if (commandArray[i].mode == 2) // Ездить не обращая внимание на время до результата по углу
            {
                time = millis() + 999999; // Огромное время ставим
                flagAngle = true;         // Флаг что теперь отслеживаем угол
                ROS_INFO("    Angle Start");
            }
        }

        static float koef = 0.01;      // P коефициент пид регулятора
        static float minMistake = 0.02; // Минимальная ошибка по углу в Градусах
        static float mistake = 0;      // Текущая ошибка по углу в градусах

        if (flagAngle) // Отслеживание угла
        {
            float angleFact = msg_Pose.th.mode10; // Угол который отслеживаем
            mistake = commandArray[i].angle - RAD2DEG(angleFact); // Смотрим какой угол.// Смотрим куда нам надо Считаем ошибку по углу и включаем колеса в нужную сторону с учетом ошибки по углу и максимально заданой скорости на колесах
            ROS_INFO_THROTTLE(0.1 ,"    i= %i commandArray[i].angle = %6.2f angleFact = %6.2f mistake = %6.2f", i, commandArray[i].angle , RAD2DEG(angleFact), mistake);
            if (abs(mistake) <= minMistake)                       // Когда ошибка по углу будет меньше заданной считаем что приехали и включаем время что-бы выйти из данного этапа алгоритма
            {
                controlSpeed.control.speedL = 0;
                controlSpeed.control.speedR = 0;
                flagAngle = false;
                time = millis();
                ROS_INFO("    Angle OK. Final mistake = %f gradus", mistake);
            }
            else
            {
                float speed = abs(mistake * koef);
                ROS_INFO_THROTTLE(0.1 ,"    speed koef = %f", speed);
                if (speed > 0.2) // Максимальная скорость
                    speed = 0.2;
                if (speed < 0.0051) // Минимальная скорость
                    speed = 0.0051;
                ROS_INFO_THROTTLE(0.1 ,"    speed real = %f", speed);
                if (mistake > 0) // Если угол больше чем надо и положительный то вращается в одну сторону
                {
                    controlSpeed.control.speedL = -speed; // Скороть должна увеличивать до заданой или максимальной с учетом алогритма в data_node  а уменьшать будет по коефициету по ошибке по углу.
                    controlSpeed.control.speedR = speed;
                }
                else
                {
                    controlSpeed.control.speedL = speed;
                    controlSpeed.control.speedR = -speed;
                }
            }

            // Выводим различия в скорости что задали и что крутимся по обратной свзяи от data_node/Speed. Смотри как работает ускорение и замедление по углу.
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
