/* 
 * File:   RechargeTask.cpp
 * Author: geoso
 * 
 * Created on 17 de Fevereiro de 2021, 00:25
 */

#include "RechargeTask.hpp"

RechargeTask::RechargeTask() { }

/*RechargeTask::RechargeTask(RechargeTask* _r_task) {
    this->set_entity_type(_r_task->get_entity_type());
    this->set_position(_r_task->get_position());
    this->set_id(_r_task->get_id());
    this->set_task_type(_r_task->get_task_type());
    this->__charger_amperage = _r_task->__charger_amperage;
}*/

float RechargeTask::get_charger_amperage() {
    return this->__charger_amperage;
}

void RechargeTask::set_charger_amperage(float _ch_amperage) {
    this->__charger_amperage = _ch_amperage;
}

void RechargeTask::show_me() {
    std::cout << "{ id = " << this->__id << "; type = " << this->get_streamof_task_type() <<
            "; location = (" << this->__position.first << ", " << this->__position.second <<
            "); charger amperage = " << this->__charger_amperage << "}" << std::endl;
}

std::string RechargeTask::get_type_and_id() {
    std::string simple_stream;
    std::ostringstream os;
    os << "x" << this->__id;
    simple_stream = os.str();
    return simple_stream;
}

void RechargeTask::show_tasks(std::vector<RechargeTask*> _recharge_tasks) {
    for (auto it = std::begin(_recharge_tasks); it != std::end(_recharge_tasks); ++it) {
        (*it)->show_me();
    }
    fprintf(stdout, "\n");
}

RechargeTask::~RechargeTask() { }

