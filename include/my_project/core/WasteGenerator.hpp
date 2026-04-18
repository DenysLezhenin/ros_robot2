#pragma once

#include <memory>
#include <vector>

#include "my_project/core/Waste.hpp"
#include "my_project/core/WasteFactory.hpp"


class WasteGenerator{
public:
    std::vector<std::shared_ptr<Waste>> generateWastes(int count){
        std::vector<std::shared_ptr<Waste>> wastes;

        for(int i = 0; i < count; i++){
            std::string type = randomType();

            auto waste = FactoryWaste::createWaste(type);
            waste->x = random(0, 10);
            waste->y = random(0, 10);
            waste->radius = random(0.1, 0.5);
            wastes.push_back(waste);
        }
        return wastes;
    }

private:
    std::string randomType(){
        int r = rand() % 3;
        if(r == 0) return "plastic";
        else if(r == 1) return "paper";
        else return "glass";
    }

    double random(double min, double max){
        return min + static_cast<double>(rand()) / RAND_MAX * (max - min);
    }
};