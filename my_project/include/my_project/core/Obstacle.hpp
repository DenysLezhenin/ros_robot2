#pragma once

class Obstacle{
public:
    virtual ~Obstacle() = default;
};

class CircleObstacle : public Obstacle{
public:
    double x, y, radius;
};

class RectangleObstacle : public Obstacle{
public:
    double x, y, width, height;
};


