#pragma once

#include <memory>
#include "my_project/core/Waste.hpp"

class FactoryWaste{
public:
    static std::shared_ptr<Waste> createWaste(const std::string& type){
        if(type == "plastic"){
            return std::make_shared<PlasticWaste>();
        } else if(type == "paper"){
            return std::make_shared<PlasticPaper>();
        } else if(type == "glass"){
            return std::make_shared<PlasticGlass>();
        }

        //throw std::runtime_error("Unknown waste type");
        return nullptr;
    }

};