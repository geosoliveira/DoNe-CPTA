/* 
 * File:   Task.hpp
 * Author: geoso
 *
 * Created on 16 de Fevereiro de 2021, 23:26
 */

#ifndef TASK_HPP
#define	TASK_HPP

#include <vector>
#include <sstream>
#include "Entity.hpp"

class Task : public Entity {
public:
    typedef enum ttask { pick = 0x1A7, deliver = 0x2EB, recharge = 0x341 } task_t;
    
    Task();
    uint16_t get_id();
    Task::task_t get_task_type();
    std::string get_streamof_task_type();
    void set_id(uint16_t);
    void set_task_type(Task::task_t);
    void show_me();
    virtual std::string get_type_and_id();
    static void show_tasks(std::vector<Task*>);
    virtual ~Task();
protected:
    uint16_t __id;
    Task::task_t __task_type;
};

#endif	/* TASK_HPP */

