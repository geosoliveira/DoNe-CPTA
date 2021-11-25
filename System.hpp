/* 
 * File:   System.hpp
 * Author: geoso
 *
 * Created on 16 de Fevereiro de 2021, 21:17
 */

#ifndef SYSTEM_HPP
#define	SYSTEM_HPP

#include <chrono>

class System {
public:
    typedef std::chrono::high_resolution_clock::time_point system_time_t;
    
    System();
    float get_proximity_coefficient();
    float get_weighting_coefficient();
    void set_proximity_coefficient(float);
    void set_weighting_coefficient(float);
    ~System();
private:
    float __proximity_coefficient;
    float __weighting_coefficient;
};

extern System ___system_parms;

#endif	/* SYSTEM_HPP */

