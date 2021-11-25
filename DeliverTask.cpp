/* 
 * File:   DeliverTask.cpp
 * Author: geoso
 * 
 * Created on 17 de Fevereiro de 2021, 00:21
 */

#include "DeliverTask.hpp"

DeliverTask::DeliverTask(uint16_t _id, Entity::entity_t _etty_type, Task::task_t _t_type) {
    this->__id = _id;
    this->__entity_type = _etty_type;
    this->__task_type = _t_type;
    this->__demand = 0;
}

/*DeliverTask::DeliverTask(DeliverTask* _d_task) {
    this->set_entity_type(_d_task->get_entity_type());
    this->set_position(_d_task->get_position());
    this->set_id(_d_task->get_id());
    this->set_task_type(_d_task->get_task_type());
    this->__demand = _d_task->__demand;
    this->__height =_d_task->__height;
}*/

uint32_t DeliverTask::get_demand() {
    return this->__demand;
}

uint16_t DeliverTask::get_height() {
    return this->__height;
}

void DeliverTask::set_demand(uint32_t _demand) {
    this->__demand = _demand;
}

void DeliverTask::set_height(uint16_t _height) {
    this->__height = _height;
}

void DeliverTask::show_me() {
    std::cout << "{ id = " << this->__id << "; type = " << this->get_streamof_task_type() <<
            "; location = (" << this->__position.first << ", " << this->__position.second <<
            "); height = " << this->__height << "; demand = " << this->__demand << "}" << std::endl;
}

std::string DeliverTask::get_type_and_id() {
    std::string simple_stream;
    std::ostringstream os;
    os << "d" << this->__id;
    simple_stream = os.str();
    return simple_stream;
}

void DeliverTask::show_tasks(std::vector<DeliverTask*> _deliver_tasks) {
    for (auto it = std::begin(_deliver_tasks); it != std::end(_deliver_tasks); ++it) {
        (*it)->show_me();
    }
    fprintf(stdout, "\n");
}

DeliverTask::~DeliverTask() { }

