#ifndef CAR_H
#define CAR_H

class CCar
{
private:
    struct SPosition
    {
        float x = 0;
        float y = 0;
        float theta = 0;
    };

public:
    CCar(/* args */);
    ~CCar();
    SPosition position; // Текущая позиция машинки
    void parsingTopicCar(geometry_msgs::Pose2D &car_);
};

CCar::CCar(/* args */)
{
}

CCar::~CCar()
{
}

void CCar::parsingTopicCar(geometry_msgs::Pose2D &car_)
{
    ROS_INFO("------------------------- parsingCar -------------------------------------");
    position.x = car_.x;
    position.y = car_.y;
    position.theta = car_.theta;
    ROS_INFO("parsingTopicCar  x= %.3f y= %.3f theta= %.3f ", position.x, position.y, position.theta);
    ROS_INFO("-------------------------            -------------------------------------");
}

#endif