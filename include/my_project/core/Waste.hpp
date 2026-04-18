#pragma once
#include <string>

class Waste{
public:
    virtual ~Waste() = default;

    double x, y;
    double radius;

    virtual std::string getType() const = 0;
};

class PlasticWaste : public Waste{
public:
    std::string getType() const override {return "plastic";}
};

class PlasticPaper : public Waste{
public:
    std::string getType() const override {return "paper";}
};

class PlasticGlass : public Waste{
public:
    std::string getType() const override {return "glass";}
};