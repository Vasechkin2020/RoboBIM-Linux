#ifndef POSITION_H
#define POSITION_H


class Position
{
private:
        // Структура  данных где указаны координаты  текущей позиции
    struct pos_stru
    {
        double x = 0;         // Координата по Х
        double y = 0;         // Координата по У
        double th = 0;          // Направление 
    };
public:
    Position(/* args */);
    ~Position();
    pos_stru pos;
    //void setStartPos(double x_, double y_, double th_);
};

Position::Position(/* args */)
{
}

Position::~Position()
{
}
// void Position::setStartPos(double x_, double y_, double th_)
// {

//         // if (!nh_private.getParam("y", yy))
//         // yy = 1.11;
// }































#endif