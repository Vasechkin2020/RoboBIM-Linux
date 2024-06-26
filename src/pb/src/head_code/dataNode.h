#ifndef DATANODE_H
#define DATANODE_H

#include "config.h"

class CDataNode
{
private:


public:
    CDataNode(/* args */);
    ~CDataNode();
    void parsingDriver(pb_msgs::Struct_Driver2Data data_); // Метод разбирает пришедшее сообщение из топика и копирует данные в переменную для дальнейшего использования
    Struct_Driver2Data _driver;                         // Тут все переменные что получили от драйвера
    SOdom _encoderOdom;                           // Одометрия по Энкодеру
    SOdom _mpuOdom;                               // Одометрия по MPU BNO055

    Struct_Data2Modul ControlModul;
};


// Расчет линейных и угловой скорости из энкодера
// ОЧЕНЬ ХОРОШИЕ ЛЕКЦИИ И МНОГИ ФОРМУЛЫ ОТТУДА https://www.youtube.com/watch?v=xJYhNCifPj8&list=PL2PmRem6srUk-Jflnt3-RuzuICb7TN0FR&index=3

CDataNode::CDataNode(/* args */)
{
}

CDataNode::~CDataNode()
{
}
// Метод разбирает пришедшее сообщение из топика и копирует данные в переменную для дальнейшего использования
void CDataNode::parsingDriver(pb_msgs::Struct_Driver2Data data_)
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