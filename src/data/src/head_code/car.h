#ifndef CAR_H
#define CAR_H

class CCar
{
private:
    struct SPosition
    {
        float x = 0;
        float y = 0;
        float th = 0;
    };

public:
    CCar(/* args */);
    ~CCar();
    SPosition position; // Текущая позиция машинки
    void parsingTopicCar(data::point &car_);
};

CCar::CCar(/* args */)
{
}

CCar::~CCar()
{
}

void CCar::parsingTopicCar(data::point &car_)
{
    position.x = car_.x;
    position.y = car_.y;
}

#endif