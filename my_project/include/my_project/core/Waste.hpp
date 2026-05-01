#pragma once

#include <string>
#include <memory>
#include <random>
#include <stdexcept>

// Custom exception
class WasteException : public std::exception {
private:
    std::string message_;
public:
    explicit WasteException(const std::string& msg) : message_(msg) {}
    const char* what() const noexcept override {
        return message_.c_str();
    }
};

// Waste base class
class Waste {
public:
    enum Type { PAPER, PLASTIC, GLASS };
    
    virtual ~Waste() = default;
    virtual Type getType() const = 0;
    virtual std::string getTypeName() const = 0;
    virtual std::string getId() const = 0;
    
    double x = 0.0;
    double y = 0.0;
    double radius = 0.1;

protected:
    std::string id_;
    Type type_;
};

// Derived waste classes
class PaperWaste : public Waste {
private:
    static int counter_;
public:
    PaperWaste(double x = 0.0, double y = 0.0, double radius = 0.1) {
        this->x = x;
        this->y = y;
        this->radius = radius;
        this->type_ = PAPER;
        id_ = "paper_" + std::to_string(counter_++);
    }
    
    Type getType() const override { return PAPER; }
    std::string getTypeName() const override { return "paper"; }
    std::string getId() const override { return id_; }
};

class PlasticWaste : public Waste {
private:
    static int counter_;
public:
    PlasticWaste(double x = 0.0, double y = 0.0, double radius = 0.1) {
        this->x = x;
        this->y = y;
        this->radius = radius;
        this->type_ = PLASTIC;
        id_ = "plastic_" + std::to_string(counter_++);
    }
    
    Type getType() const override { return PLASTIC; }
    std::string getTypeName() const override { return "plastic"; }
    std::string getId() const override { return id_; }
};

class GlassWaste : public Waste {
private:
    static int counter_;
public:
    GlassWaste(double x = 0.0, double y = 0.0, double radius = 0.1) {
        this->x = x;
        this->y = y;
        this->radius = radius;
        this->type_ = GLASS;
        id_ = "glass_" + std::to_string(counter_++);
    }
    
    Type getType() const override { return GLASS; }
    std::string getTypeName() const override { return "glass"; }
    std::string getId() const override { return id_; }
};

// Simple factory function
inline std::shared_ptr<Waste> createWaste(
    Waste::Type type, double x, double y, double radius) 
{
    switch (type) {
        case Waste::PAPER:
            return std::make_shared<PaperWaste>(x, y, radius);
        case Waste::PLASTIC:
            return std::make_shared<PlasticWaste>(x, y, radius);
        case Waste::GLASS:
            return std::make_shared<GlassWaste>(x, y, radius);
        default:
            throw WasteException("Unknown waste type");
    }
}

inline std::shared_ptr<Waste> createRandomWaste(
    double x, double y, double radius)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 2);
    
    Waste::Type type = static_cast<Waste::Type>(dis(gen));
    return createWaste(type, x, y, radius);
}