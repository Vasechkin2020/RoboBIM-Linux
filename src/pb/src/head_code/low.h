#ifndef DATA_H
#define DATA_H

class CLow
{
private:
    // Структура сосдержит всю информацию по мотору на основании данных энкодера
    struct SEncoder
    {
        float rpsSet = 0;   // Текущая скорость вращения ( обороты в секунду)
        float rpsEncod = 0; // Текущая скорость вращения ( обороты в секунду)
    };

    struct SMpu // Структура с данными с mpu bno055
    {
        SPose pose;
        STwist twist;
        SEuler angleEuler;
    };

    // Структура состояния датчика расстония
    struct SSensor
    {
        int32_t status; // статус состояния
        float distance; // расстояние до препятствия
    };

    // Структура принимаемых данных от контроллера Driver в Data
    struct SDriver2Data
    {
        uint32_t id = 0; // id команды
        SEncoder motorLeft;
        SEncoder motorRight;
        double dtEncoder; // Время за которое данные с энкодера
        SMpu bno055;      // Данные с датчика BNO055
        SSensor laserL;
        SSensor laserR;
        SSensor uzi;
    };

public:
    CLow(/* args */);
    ~CLow();
    void parsingDriver(pb_msgs::Struct_Driver2Data data_); // Метод разбирает пришедшее сообщение из топика и копирует данные в переменную для дальнейшего использования
    SDriver2Data _driver;                         // Тут все переменные что получили от драйвера
    SOdom _encoderOdom;                           // Одометрия по Энкодеру
    SOdom _mpuOdom;                               // Одометрия по MPU BNO055
};


// Расчет линейных и угловой скорости из энкодера
// ОЧЕНЬ ХОРОШИЕ ЛЕКЦИИ И МНОГИ ФОРМУЛЫ ОТТУДА https://www.youtube.com/watch?v=xJYhNCifPj8&list=PL2PmRem6srUk-Jflnt3-RuzuICb7TN0FR&index=3

CLow::CLow(/* args */)
{
}

CLow::~CLow()
{
}
// Метод разбирает пришедшее сообщение из топика и копирует данные в переменную для дальнейшего использования
void CLow::parsingDriver(pb_msgs::Struct_Driver2Data data_)
{
    _driver.id = data_.id;
    //_driver.motorLeft.rpsEncod = data_.;

    _driver.laserL.status = data_.laserL.status;
    _driver.laserL.distance = data_.laserL.distance;
    _driver.laserR.status = data_.laserR.status;
    _driver.laserR.distance = data_.laserR.distance;
    _driver.uzi.status = data_.uzi.status;
    _driver.uzi.distance = data_.uzi.distance;
}

#endif