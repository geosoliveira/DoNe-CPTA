/* 
 * File:   RSE.hpp
 * Author: geoso
 *
 * Created on 18 de Março de 2021, 07:31
 */

#ifndef RSE_HPP
#define	RSE_HPP

//Robotic Simulation Environment
namespace RSE {
    extern int current_execution;
    extern int number_executions;
    extern float cost_tolerance;
    class Parser;
    template <typename T> void destroy_all_elements_from(T _a) {
        for (auto it = std::begin(_a); it != std::end(_a); ++it) {
            delete (*it);
        }
    }
    template <class T> void show(T _list) {
        for (auto it = std::begin(_list); it != std::end(_list); ++it) {
            (*it)->show_me();
        }
    }
    template <class T> void reset_all(T _list) {
        for (auto it = std::begin(_list); it != std::end(_list); ++it) {
            (*it).reset();
        }
    }
}

#endif	/* RSE_HPP */

