/* 
 * File:   Obstacle.hpp
 * Author: geoso
 *
 * Created on 17 de Fevereiro de 2021, 12:30
 */

#ifndef OBSTACLE_HPP
#define	OBSTACLE_HPP

#include "Entity.hpp"

class Obstacle : public Entity {
public:
    typedef enum tobstacle { fixed = 0x210, dynamic = 0x2E5 } obstacle_t;
    
    Obstacle();
    Obstacle::obstacle_t get_obstacle_type();
    std::string get_streamof_obstacle_type();
    void set_obstacle_type(Obstacle::obstacle_t);
    ~Obstacle();
private:
    Obstacle::obstacle_t __obstacle_type;
};

#endif	/* OBSTACLE_HPP */

