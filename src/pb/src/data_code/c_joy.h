#ifndef JOY_H
#define JOY_H
/* Класс где все методы и переменные по джойстику */
class CJoy
{
private:
    float speed_max = 0.3;       // Максимальная скорость
    pb_msgs::SJoy _joy2HeadPrev; // Структура в которую пишем обработанные данные от джойстика предыдущее состоние

    // float getRadius(float &offset); // Возвращает какой долpb_msgsжен быть радиус при указанном смещении
    //  pb_msgs::SJoy _joy2Headmsg;  // Структура в которую пишем обработанные данные от джойстика и потом публикуем в топик

public:
    CJoy(float speed_max_, float radius_min_);
    ~CJoy();
    pb_msgs::SJoy _joy2Head;                        // Структура в которую пишем обработанные данные от джойстика и выдаем наружу из класса
    pb_msgs::SControlDriver _ControlDriver;         // Тут формируем команды делаем
    SControlPrint _controlPrint;                    // Структура в которую пишем обработанные данные от джойстика и выдаем наружу из класса
    void parsingJoy(const sensor_msgs::Joy &data_); // Функция обработки и пробразований данных в мою понятную структуру с кнопками
    void transform();                               // Преобразование кнопок джойстика в реальные команды
};

CJoy::CJoy(float speed_max_, float radius_min_)
{
}

CJoy::~CJoy()
{
}
// Функция обработки и пробразований данных в мою понятную структуру с кнопками
void CJoy::parsingJoy(const sensor_msgs::Joy &data_)
{
    //--------------------------- BUTTON -----------------------------
    _joy2Head.button_cross = data_.buttons[0];    // 0 или 1, по умолчанию 0
    _joy2Head.button_circle = data_.buttons[1];   // 0 или 1, по умолчанию 0
    _joy2Head.button_triangle = data_.buttons[2]; // 0 или 1, по умолчанию 0
    _joy2Head.button_square = data_.buttons[3];   // 0 или 1, по умолчанию 0

    _joy2Head.button_left_right = data_.axes[6]; // !!!!!!!!! 1 или -1 по умолчанию 0
    _joy2Head.button_up_down = data_.axes[7];    // !!!!!!!!! 1 или -1 по умолчанию 0

    _joy2Head.button_share = data_.buttons[8];  // 0 или 1, по умолчанию 0
    _joy2Head.button_option = data_.buttons[9]; // 0 или 1, по умолчанию 0
    _joy2Head.button_ps4 = data_.buttons[10];   // 0 или 1, по умолчанию 0

    _joy2Head.button_L1 = data_.buttons[4]; // 0 или 1, по умолчанию 0
    _joy2Head.button_R1 = data_.buttons[5]; // 0 или 1, по умолчанию 0

    _joy2Head.button_L2 = data_.buttons[6]; // 0 или 1, по умолчанию 0
    _joy2Head.button_R2 = data_.buttons[7]; // 0 или 1, по умолчанию 0
                                            //---------------------------- AXES-----------------------------
    _joy2Head.axes_L_Hor = data_.axes[0];   // От 1 до -1 float, по умолчанию 0
    _joy2Head.axes_L_Ver = data_.axes[1];   // От 1 до -1 float, по умолчанию 0

    _joy2Head.axes_R_Hor = data_.axes[3]; // От 1 до -1 float, по умолчанию 0
    _joy2Head.axes_R_Ver = data_.axes[4]; // От 1 до -1 float, по умолчанию 0

    _joy2Head.axes_L2 = data_.axes[2]; // От 1 до -1 float с нулем по середине, по умолчанию 1 !!!!!!!!!
    _joy2Head.axes_R2 = data_.axes[5]; // От 1 до -1 float с нулем по середине, по умолчанию 1 !!!!!!!!!!!!!!
}

// Преобразование кнопок джойстика в реальные команды
void CJoy::transform()
{
    if (_joy2Head.button_ps4 != _joy2HeadPrev.button_ps4 && _joy2Head.button_ps4 == 1) // Если изменилась кнопка и стала единицей сейчас тогда делаем команду
    {
        _ControlDriver.control.speedL = -0.1; // Минимальная начальная скорость
        _ControlDriver.control.speedR = -0.1;
    }

    if (_joy2Head.button_L1 != _joy2HeadPrev.button_L1 && _joy2Head.button_L1 == 1) // Если изменилась кнопка и стала единицей сейчас тогда делаем команду
    {
        _ControlDriver.control.speedL = 0.0; // Вращение ВЛЕВО
        _ControlDriver.control.speedR = 0.1; //
    }
    if (_joy2Head.button_R1 != _joy2HeadPrev.button_R1 && _joy2Head.button_R1 == 1) // Если изменилась кнопка и стала единицей сейчас тогда делаем команду
    {
        _ControlDriver.control.speedL = 0.1; // Вращение ВПРАВО
        _ControlDriver.control.speedR = 0;   //
    }

    if (_joy2Head.button_option != _joy2HeadPrev.button_option && _joy2Head.button_option == 1) // Если изменилась кнопка и стала единицей сейчас тогда делаем команду
    {
        //_ControlDriver.led.num_program++; // Увеличиваем номер команды на 1
        _ControlDriver.control.speedL = 0.5;  // Скорость ноль
        _ControlDriver.control.speedR = -0.5; // Скорость ноль
    }
    if (_joy2Head.button_share != _joy2HeadPrev.button_share && _joy2Head.button_share == 1) // Если изменилась кнопка и стала единицей сейчас тогда делаем команду
    {
        //_ControlDriver.led.num_program = 0; // Выключаем свет, номер команды делаем 0
        _ControlDriver.control.speedL = -0.5; // Скорость ноль
        _ControlDriver.control.speedR = 0.5;  // Скорость ноль
    }

    // Скорость --------------------------
    if (_joy2Head.button_cross != _joy2HeadPrev.button_cross && _joy2Head.button_cross == 1) // Если изменилась кнопка и стала единицей сейчас тогда делаем команду
    {
        _ControlDriver.control.speedL = 0; // Скорость ноль
        _ControlDriver.control.speedR = 0; // Скорость ноль
    }
    if (_joy2Head.button_circle != _joy2HeadPrev.button_circle && _joy2Head.button_circle == 1) // Если изменилась кнопка и стала единицей сейчас тогда делаем команду
    {
        _ControlDriver.control.speedL = 0;   // Скорость уменьшаем на 0.1
        _ControlDriver.control.speedR = 0.5; // Скорость уменьшаем на 0.1

        // _ControlDriver.control.speedL += 0.05;          // Скорость увеличиваем на 0.1
        // _ControlDriver.control.speedR += 0.05;          // Скорость увеличиваем на 0.1
        // if (_ControlDriver.control.speedL >= speed_max) // Максимальная скорость ограничена
        // {
        //     _ControlDriver.control.speedL = speed_max;
        // }
        // if (_ControlDriver.control.speedR >= speed_max) // Максимальная скорость ограничена
        // {
        //     _ControlDriver.control.speedR = speed_max;
        // }
    }
    if (_joy2Head.button_square != _joy2HeadPrev.button_square && _joy2Head.button_square == 1) // Если изменилась кнопка и стала единицей сейчас тогда делаем команду
    {
        _ControlDriver.control.speedL = 0.5; // Скорость уменьшаем на 0.1
        _ControlDriver.control.speedR = 0;   // Скорость уменьшаем на 0.1
        // _ControlDriver.control.speedL -= 0.05; // Скорость уменьшаем на 0.1
        // _ControlDriver.control.speedR -= 0.05; // Скорость уменьшаем на 0.1
        // if (_ControlDriver.control.speedL < 0) // МИнимальная скорость ограничена 0
        // {
        //     _ControlDriver.control.speedL = 0;
        // }
        // if (_ControlDriver.control.speedR < 0) // МИнимальная скорость ограничена 0
        // {
        //     _ControlDriver.control.speedR = 0;
        // }
    }
    if (_joy2Head.button_triangle != _joy2HeadPrev.button_triangle && _joy2Head.button_triangle == 1) // Если изменилась кнопка и стала единицей сейчас тогда делаем команду
    {
        _ControlDriver.control.speedL = 0.5; //
        _ControlDriver.control.speedR = 0.5; //
    }

    if (_joy2Head.button_up_down != _joy2HeadPrev.button_up_down && _joy2Head.button_up_down == 1) // Если изменилась кнопка и стала единицей сейчас тогда делаем команду
    {
        _controlPrint.status = 1;
        _controlPrint.mode = 5;
    }

    if (_joy2Head.button_up_down != _joy2HeadPrev.button_up_down && _joy2Head.button_up_down == -1) // Если изменилась кнопка и стала минус единицей сейчас тогда делаем команду
    {
        _controlPrint.status = 1;
        _controlPrint.mode = 4;
    }
    if (_joy2Head.button_up_down != _joy2HeadPrev.button_up_down && _joy2Head.button_up_down == 0) // Если изменилась кнопка и стала нулем сейчас тогда делаем команду
    {
        _controlPrint.status = 0;
    }

    if (_joy2Head.button_left_right != _joy2HeadPrev.button_left_right && _joy2Head.button_left_right == -1) // Если изменилась кнопка и стала минус единицей сейчас тогда делаем команду
    {
    }
    if (_joy2Head.button_left_right != _joy2HeadPrev.button_left_right && _joy2Head.button_left_right == 1) // Если изменилась кнопка и стала единицей сейчас тогда делаем команду
    {
    }
    if (_joy2Head.button_left_right != _joy2HeadPrev.button_left_right && _joy2Head.button_left_right == 0) // Если изменилась кнопка и стала нулем сейчас тогда делаем команду
    {
    }

    // printf(" control.startStop %i ", _ControlDriver.control.startStop);
    // printf(" led.num_program %i \n", _ControlDriver.led.num_program);

    _joy2HeadPrev = _joy2Head; // Запоминаем состояние для следующего раза
}

#endif

// ВАРИАНТ КАК НАДО ОФОРМЛЯТЬ ВЫЗОВ ФУНКЦИИ ВНУТРИ КЛАССА!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// ros::NodeHandle nh_joy;
// ros::Publisher publisher_Joy2Head;
// ros::Subscriber subscriber_Joy = nh_joy.subscribe("joy", 16, &Joy::callback_Joy,this); // Это мы подписываемся на то что публикует нода джойстика

// publisher_Joy2Head = nh_joy.advertise<sensor_msgs::Joy>("Joy2Head", 16); // Это мы публикуем структуру которую получили с драйвера

// void callback_Joy(sensor_msgs::Joy msg)
// {
//     joy = msg;
// }
// // Обратный вызов при опросе топика Head2Data
// void callback_Joy(const sensor_msgs::Joy &msg)
// {
//     // msg_ControlDriver = msg; // Копируем структуру в глобальную переменную для дальнейшей работы с ней.
//     //  ROS_INFO("message_callback_Command.");
// }