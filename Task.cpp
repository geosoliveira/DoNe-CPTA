/* 
 * File:   Task.cpp
 * Author: geoso
 * 
 * Created on 16 de Fevereiro de 2021, 23:26
 */

#include "Task.hpp"

Task::Task() { }

uint16_t Task::get_id() {
    return this->__id;
}

Task::task_t Task::get_task_type() {
    return this->__task_type;
}

std::string Task::get_streamof_task_type() {
    switch(this->__task_type) {
        case(Task::pick):
            return "pick";
        case(Task::deliver):
            return "deliver";
        case (Task::recharge):
            return ("recharge");
        default:
            std::cerr << "[" << __FILE__ <<":" << __LINE__ << "] Unknown task type " << this->__task_type << ". Exiting..." << std::endl;
            exit(0x6A5);
    }
}

void Task::set_id(uint16_t _id) {
    this->__id = _id;
}

void Task::set_task_type(Task::task_t _task_type) {
    this->__task_type = _task_type;
}

void Task::show_me() { }

std::string Task::get_type_and_id() { return ""; }

void Task::show_tasks(std::vector<Task*> _tasks) {
    for (auto it = std::begin(_tasks); it != std::end(_tasks); ++it) {
        (*it)->show_me();
    }
    fprintf(stdout, "\n");
}

Task::~Task() { }

