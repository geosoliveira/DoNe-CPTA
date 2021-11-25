/* 
 * File:   DeliverTask.hpp
 * Author: geoso
 *
 * Created on 17 de Fevereiro de 2021, 00:21
 */

#ifndef DELIVERTASK_HPP
#define	DELIVERTASK_HPP

#include <float.h>
#include <climits>
#include "Task.hpp"

class DeliverTask : public Task {
public:
    DeliverTask() = delete;
    DeliverTask(uint16_t, Entity::entity_t, Task::task_t);
    //DeliverTask(DeliverTask*);
    uint32_t get_demand();
    uint16_t get_height();
    void set_demand(uint32_t);
    void set_height(uint16_t);
    void show_me();
    std::string get_type_and_id();
    static void show_tasks(std::vector<DeliverTask*>);
    ~DeliverTask();
private:
    uint32_t __demand;
    uint16_t __height;
};

#endif	/* DELIVERTASK_HPP */

