/* 
 * File:   Obstacle.cpp
 * Author: geoso
 * 
 * Created on 17 de Fevereiro de 2021, 12:30
 */

#include "Obstacle.hpp"

Obstacle::Obstacle() { }

Obstacle::obstacle_t Obstacle::get_obstacle_type() {
    return this->__obstacle_type;
}

std::string Obstacle::get_streamof_obstacle_type() {
    switch(this->__obstacle_type) {
        case(Obstacle::fixed):
            return "fixed";
        case(Obstacle::dynamic):
            return "dynamic";
        default:
            std::cerr << "[" << __FILE__ <<":" << __LINE__ << "] Unknown obstacle type " << this->__obstacle_type << ". Exiting..." << std::endl;
            exit(0x83F);
    }
}

void Obstacle::set_obstacle_type(Obstacle::obstacle_t _o_type) {
    this->__obstacle_type = _o_type;
}

Obstacle::~Obstacle() { }