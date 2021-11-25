/* 
 * File:   Entity.hpp
 * Author: geoso
 *
 * Created on 16 de Fevereiro de 2021, 21:34
 */

#ifndef ENTITY_HPP
#define	ENTITY_HPP

#include <iostream>
#include <cstdlib>
#include <memory>
#include <utility>
#include <cstdint>
#include <string>

class Entity {
public:
    typedef enum tentity { product = 0x301, task = 0x1B3, robot = 0x226, obstacle = 0x34D } entity_t;
    
    Entity();
    Entity(uint16_t, uint16_t, entity_t);
    Entity::entity_t get_entity_type();
    std::string get_streamof_entity_type();
    std::pair<uint16_t, uint16_t> get_position();
    uint16_t get_x();
    uint16_t get_y();
    void set_entity_type(Entity::entity_t);
    void set_position(std::pair<uint16_t, uint16_t>);
    void set_x(uint16_t);
    void set_y(uint16_t);
    virtual void show_me();
    virtual ~Entity();
protected:
    entity_t __entity_type;
    std::pair<uint16_t, uint16_t> __position;
};

#endif	/* ENTITY_HPP */

