/* 
 * File:   Pick.cpp
 * Author: geoso
 * 
 * Created on 16 de Fevereiro de 2021, 23:52
 */

#include "PickTask.hpp"

bool PickTask::compare_tasks_by_cost(std::pair<uint16_t, float> _a1, std::pair<uint16_t, float> _a2) {
    return (_a1.second < _a2.second);
}

PickTask::PickTask(uint16_t _id, Entity::entity_t _etty_type, Task::task_t _t_type, PickTask::taskstat_t _state) {
    this->__id = _id;
    this->__entity_type = _etty_type;
    this->__task_type = _t_type;
    this->__current_state = _state;
}

PickTask::PickTask(uint16_t _id, std::pair<uint16_t, uint16_t> _position, Entity::entity_t _etty_type, Task::task_t _t_type, PickTask::taskstat_t _state) {
    this->__id = _id;
    this->__position = _position;
    this->__entity_type = _etty_type;
    this->__task_type = _t_type;
    this->__current_state = _state;
}

PickTask::PickTask(std::shared_ptr<PickTask> _p_task) {
    this->set_entity_type(_p_task->get_entity_type());
    this->set_position(_p_task->get_position());
    this->set_id(_p_task->get_id());
    this->set_task_type(_p_task->get_task_type());
    this->__current_state = _p_task->__current_state;
    this->__demand = _p_task->__demand;
    this->__height =_p_task->__height;
    for (auto it = std::begin(_p_task->__orders_ref); it != std::end(_p_task->__orders_ref); ++it) {
        this->__orders_ref.push_back(new Order(*it));
    }
}

PickTask::taskstat_t PickTask::get_current_state() {
    return this->__current_state;
}

std::string PickTask::get_streamof_current_state() {
    switch(this->__current_state) {
        case(PickTask::unused):
            return "unused";
        case(PickTask::waiting):
            return "waiting";
        case(PickTask::testing):
            return "testing";
        case (PickTask::assigned):
            return ("assigned");
        default:
            std::cerr << "[" << __FILE__ <<":" << __LINE__ << "] Unknown task status " << this->__current_state << ". Exiting..." << std::endl;
            exit(0x787);
    }
}

int32_t PickTask::get_demand() {
    return this->__demand;
}

uint16_t PickTask::get_height() {
    return this->__height;
}

std::vector<Order*> PickTask::get_orders_ref() {
    return this->__orders_ref;
}

Order* PickTask::get_order_ref(int _addr) {
    return this->__orders_ref[_addr];
}

void PickTask::set_current_state(PickTask::taskstat_t _curr_stat) {
    this->__current_state = _curr_stat;
}

void PickTask::set_demand(int32_t _demand) {
    this->__demand = _demand;
}

void PickTask::set_height(uint16_t _height) {
    this->__height = _height;
}

void PickTask::add_order_ref(Order* _o_ref) {
    this->__orders_ref.push_back(_o_ref);
}

bool PickTask::all_assigned(std::vector<std::shared_ptr<PickTask> > _pick_tasks) {
    for (auto it = std::begin(_pick_tasks); it != std::end(_pick_tasks); ++it) {
        if ((*it)->get_current_state() != PickTask::assigned && (*it)->get_current_state() != PickTask::unused) {
            return false;
        }
    }
    return true;
}

bool PickTask::is_it_here(std::vector<PickTask*> _pick_tasks, PickTask* _p_task) {
    for (auto it = std::begin(_pick_tasks); it != std::end(_pick_tasks); ++it) {
        if ((*it)->get_id() == _p_task->get_id()) {
            return true;
        }
    }
    return false;
}

void PickTask::show_me() {
    std::cout << "{ id = " << this->__id << "; type = " << this->get_streamof_task_type() <<
            "; location = (" << this->__position.first << ", " << this->__position.second <<
            "); height = " << this->__height << "; demand = " << this->__demand <<
            "; state = " << this->get_streamof_current_state() << "}" << std::endl;
}

std::string PickTask::get_type_and_id() {
    std::string simple_stream;
    std::ostringstream os;
    os << "t" << this->__id;
    simple_stream = os.str();
    return simple_stream;
}

std::vector<PickTask*>* PickTask::copy_picktasks(std::vector<PickTask*> _tasks) {
    std::vector<PickTask*>* cp_ref = new std::vector<PickTask*>[_tasks.size()];
    std::vector<PickTask*>& cp = *(cp_ref);
    for (int i = 0; i < _tasks.size(); i++)
        cp.push_back(_tasks[i]);
    return cp_ref;
}

std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > PickTask::copy_picktasks_v2(std::vector<std::shared_ptr<PickTask> > _tasks) {
    std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > cp_ref(new std::vector<std::shared_ptr<PickTask> >[_tasks.size()]);
    for (int i = 0; i < _tasks.size(); i++)
        cp_ref->push_back(_tasks[i]);
    return cp_ref;
}

std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > PickTask::clone_picktasks(std::vector<std::shared_ptr<PickTask> > _tasks) {
    std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > cp_ref(new std::vector<std::shared_ptr<PickTask> >[_tasks.size()]);
    for (int i = 0; i < _tasks.size(); i++) {
        std::shared_ptr<PickTask> task_cp(new PickTask(_tasks[i]));
        cp_ref->push_back(task_cp);
    }
    return cp_ref;
}

bool PickTask::delete_picktasks(std::vector<PickTask*>& _pick_tasks, PickTask* _task) {
    for (int i = 0; i < _pick_tasks.size(); i++) {
        if (_pick_tasks[i]->get_id() == _task->get_id()) {
            _pick_tasks.erase(_pick_tasks.begin() + i);
            return true;
        }
    }
    return false;
}

bool PickTask::delete_picktasks_v2(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >& _pick_tasks_ref, std::shared_ptr<PickTask> _task) {
    for (int i = 0; i < (*_pick_tasks_ref).size(); i++) {
        if ((*_pick_tasks_ref)[i]->get_id() == _task->get_id()) {
            _pick_tasks_ref->erase(_pick_tasks_ref->begin() + i);
            return true;
        }
    }
    return false;
}

std::vector<std::shared_ptr<PickTask> > PickTask::intersection(std::vector<std::shared_ptr<PickTask> > v1, std::vector<std::shared_ptr<PickTask> > v2) {
    std::vector<std::shared_ptr<PickTask> > v3;
    for (int i = 0; i < v1.size(); i++) {
        std::shared_ptr<PickTask> cur_task_v1 = v1[i];
        for (int j = 0; j < v2.size(); j++) {
            std::shared_ptr<PickTask> cur_task_v2 = v2[j];
            if (cur_task_v1->get_id() == cur_task_v2->get_id()) {
                v3.push_back(cur_task_v1);
                break;
            }
        }
    }
    return v3;
}

void PickTask::show_tasks(std::vector<PickTask*> _pick_tasks) {
    for (auto it = std::begin(_pick_tasks); it != std::end(_pick_tasks); ++it) {
        (*it)->show_me();
    }
}

PickTask::~PickTask() {
    for (auto it = std::begin(this->__orders_ref); it != std::end(this->__orders_ref); ++it) {
        delete (*it);
    }
}