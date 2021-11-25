/* 
 * File:   Entity.cpp
 * Author: geoso
 * 
 * Created on 16 de Fevereiro de 2021, 21:34
 */

#include "Entity.hpp"

using namespace std;

Entity::Entity() { }

Entity::Entity(uint16_t _x, uint16_t _y, entity_t _entty_type) {
    this->__entity_type = _entty_type;
    this->__position = std::make_pair(_x, _y);
}

Entity::entity_t Entity::get_entity_type() {
    return this->__entity_type;
}

std::string Entity::get_streamof_entity_type() {
    switch(this->__entity_type) {
        case(Entity::product):
            return "product";
        case(Entity::task):
            return "task";
        case (Entity::robot):
            return ("robot");
        case (Entity::obstacle):
            return ("obstacle");
        default:
            std::cerr << "[" << __FILE__ <<":" << __LINE__ << "] Unknown entity type " << this->__entity_type << ". Exiting..." << std::endl;
            exit(0x78F);
    }
}

std::pair<uint16_t, uint16_t> Entity::get_position() {
    return this->__position;
}

uint16_t Entity::get_x() {
    return this->__position.first;
}

uint16_t Entity::get_y() {
    return this->__position.second;
}

void Entity::set_entity_type(Entity::entity_t _entity_type) {
    this->__entity_type = _entity_type;
}

void Entity::set_position(std::pair<uint16_t, uint16_t> _position) {
    this->__position = _position;
}

void Entity::set_x(uint16_t _x) {
    uint16_t y = this->__position.second;
    this->__position = std::make_pair(_x, y);
}

void Entity::set_y(uint16_t _y) {
    uint16_t x = this->__position.first;
    this->__position = std::make_pair(x, _y);
}

void Entity::show_me() { }

Entity::~Entity() { }

