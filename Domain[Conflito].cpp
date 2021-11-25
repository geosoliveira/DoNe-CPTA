/* 
 * File:   Domain.cpp
 * Author: geoso
 * 
 * Created on 17 de Fevereiro de 2021, 18:49
 */

#include "Domain.hpp"

Domain::Domain(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > _pick_tasks_ref, std::shared_ptr<std::vector<std::shared_ptr<Robot> > > _robots_ref, std::shared_ptr<CostEstimator> _ce) {
#ifdef DEBUG
    fprintf(stdout, "\nGETTING IN: Domain(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > _pick_tasks, std::shared_ptr<std::vector<std::shared_ptr<Robot> > > _robots, std::shared_ptr<CostEstimator> _ce) [%s:%d]\n", __FILE__, __LINE__);
#endif
    this->__ce_ref = _ce;
    this->__pick_tasks_ref = _pick_tasks_ref;
    this->__robots_ref = _robots_ref;
#ifdef DEBUG
    RSE::show<std::vector<std::shared_ptr<Robot> > >(*this->__robots_ref);
    RSE::show<std::vector<std::shared_ptr<PickTask> > >(*this->__pick_tasks_ref);
    this->__ce_ref->show_me();
#endif
#ifdef DEBUG
    fprintf(stdout, "LEAVING OF: Domain(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > _pick_tasks, std::shared_ptr<std::vector<std::shared_ptr<Robot> > > _robots, std::shared_ptr<CostEstimator> _ce) [%s:%d]\n", __FILE__, __LINE__);
#endif
}

void Domain::clear_domain_lists() {
    for (auto it = this->__domain_list.begin(); it != this->__domain_list.end(); it++) {
        auto tpl = it->second;
        std::vector<std::shared_ptr<Robot> > tpl_robots = std::get<1>(tpl);
        tpl_robots.clear();
    }
    this->__domain_list.clear();
    
    for (auto it = this->__dominant_list.begin(); it != this->__dominant_list.end(); it++) {
        auto tpl = it->second;
        std::vector<std::shared_ptr<PickTask> > tpl_tasks = std::get<1>(tpl);
        tpl_tasks.clear();
    }
    this->__dominant_list.clear();
}

void Domain::compute_domain_v1() {  
    std::vector<std::shared_ptr<Robot> > best_robots;
    int n_robots = this->__robots_ref->size();
    int n_tasks = this->__pick_tasks_ref->size();
    for (int i = 0; i < n_tasks; i++) {
        std::shared_ptr<PickTask> cur_task = (*this->__pick_tasks_ref)[i];
        float best_cost = FLT_MAX;
        if (cur_task->get_current_state() == PickTask::waiting) {
            for (int j = 0; j < n_robots; j++) {
                std::shared_ptr<Robot> cur_robot = (*this->__robots_ref)[j];
                float cur_cost;
                if (cur_robot->get_total_dataset_capacity() < cur_task->get_demand()) {
                    cur_cost = FLT_MAX;
                }
                else {
                    cur_cost = this->__ce_ref->get_cost_v2(cur_task, cur_robot, cur_robot);
                }
                if (cur_cost < best_cost) {
                    best_cost = cur_cost;
                    best_robots.clear();
                    best_robots.push_back(cur_robot);
                }
                else if (cur_cost == best_cost) {
                    best_robots.push_back(cur_robot);
                }
            }
            std::tuple<std::shared_ptr<PickTask>, std::vector<std::shared_ptr<Robot> > > tpl = std::make_tuple(cur_task, best_robots);
            // Push in primary domain list
            this->__domain_list.insert({cur_task->get_id(), tpl});
        }
    }
}

void Domain::compute_domain_v2() {
    std::vector<std::shared_ptr<Robot> > best_robots;
    int n_robots = this->__robots_ref->size();
    int n_tasks = this->__pick_tasks_ref->size();
    for (int i = 0; i < n_tasks; i++) {
        std::shared_ptr<PickTask> cur_task = (*this->__pick_tasks_ref)[i];
        float best_cost = FLT_MAX;
        if (cur_task->get_current_state() == PickTask::waiting) {
            for (int j = 0; j < n_robots; j++) {
                std::shared_ptr<Robot> cur_robot = (*this->__robots_ref)[j];
                float cur_cost;
                if (cur_robot->get_total_dataset_capacity() < cur_task->get_demand()) {
                    cur_cost = FLT_MAX;
                }
                else {
                    if (cur_robot->get_current_dataset_capacity() >= cur_task->get_demand()) {
                        // Pode pegar
                        cur_cost = this->__ce_ref->get_cost_v2(cur_task, cur_robot, cur_robot);
                    }
                    else {
                        // Não pode pegar -- o custo é o custo de ir à estação de entrega mais próxima e, de lá, voltar à cur_task
                        auto best_deliver_tuple = this->__ce_ref->closer_deliver_v2(cur_task, cur_robot);
                        cur_cost = best_deliver_tuple.second;
                        cur_cost += this->__ce_ref->get_cost_v2(best_deliver_tuple.first, cur_task, cur_robot);
                    }
                }
                if (cur_cost < best_cost) {
                    best_cost = cur_cost;
                    best_robots.clear();
                    best_robots.push_back(cur_robot);
                }
                else if (cur_cost == best_cost) {
                    best_robots.push_back(cur_robot);
                }
            }
            std::tuple<std::shared_ptr<PickTask>, std::vector<std::shared_ptr<Robot> > > tpl = std::make_tuple(cur_task, best_robots);
            // Push in primary domain list
            this->__domain_list.insert({cur_task->get_id(), tpl});
        }
    }
}

void Domain::compute_domain_v3() {  
    std::vector<std::shared_ptr<Robot> > best_robots;
    int n_robots = this->__robots_ref->size();
    int n_tasks = this->__pick_tasks_ref->size();
    for (int i = 0; i < n_tasks; i++) {
        std::shared_ptr<PickTask> cur_task = (*this->__pick_tasks_ref)[i];
        float best_cost = FLT_MAX;
        if (cur_task->get_current_state() == PickTask::waiting) {
            for (int j = 0; j < n_robots; j++) {
                std::shared_ptr<Robot> cur_robot = (*this->__robots_ref)[j];
                float cur_cost;
                if (cur_robot->get_total_dataset_capacity() < cur_task->get_demand()) {
                    cur_cost = FLT_MAX;
                }
                else {
                    cur_cost = this->__ce_ref->get_cost_v2(cur_task, cur_robot, cur_robot);
                    // Se ao pegar a próxima tarefa, o robô ficar impedido de pegar outra, então o custo é somado com o custo de voltar à estação de entrega
                    // Verificando se o robô ficará impedido de pegar outra tarefa:
                    bool can_exec_another_task = false;
                    int32_t capacity_after_pickup_i = cur_robot->get_current_dataset_capacity() - cur_task->get_demand();
                    for (int k = 0; k < n_tasks; k++) {
                        std::shared_ptr<PickTask> cur_task_of_all = (*this->__pick_tasks_ref)[k];
                        if (cur_task->get_id() != cur_task_of_all->get_id() &&
                                cur_task_of_all->get_current_state() == PickTask::waiting &&
                                capacity_after_pickup_i >= cur_task_of_all->get_demand()) {
                            can_exec_another_task = true;
                            break;
                        }
                    }
                    // Verificou. Agora, se 'can_exec_another_task == false', incrementa o custo
                    if (!can_exec_another_task) {
                        auto best_deliver_tuple = this->__ce_ref->closer_deliver_v2(cur_task, cur_robot);
                        cur_cost += best_deliver_tuple.second;
                    }
                }
                if (cur_cost < best_cost) {
                    best_cost = cur_cost;
                    best_robots.clear();
                    best_robots.push_back(cur_robot);
                }
                else if (cur_cost == best_cost) {
                    best_robots.push_back(cur_robot);
                }
            }
            std::tuple<std::shared_ptr<PickTask>, std::vector<std::shared_ptr<Robot> > > tpl = std::make_tuple(cur_task, best_robots);
            // Push in primary domain list
            this->__domain_list.insert({cur_task->get_id(), tpl});
        }
    }
}

void Domain::compute_domain_v4() {
    std::vector<std::shared_ptr<Robot> > best_robots;
    int n_robots = this->__robots_ref->size();
    int n_tasks = this->__pick_tasks_ref->size();
    for (int i = 0; i < n_tasks; i++) {
        std::shared_ptr<PickTask> cur_task = (*this->__pick_tasks_ref)[i];
        float best_cost = FLT_MAX;
        if (cur_task->get_current_state() == PickTask::waiting) {
            for (int j = 0; j < n_robots; j++) {
                std::shared_ptr<Robot> cur_robot = (*this->__robots_ref)[j];
                float cur_cost;
                int32_t capacity_after_pickup_i;
                if (cur_robot->get_total_dataset_capacity() < cur_task->get_demand()) {
                    cur_cost = FLT_MAX;
                }
                else {
                    if (cur_robot->get_current_dataset_capacity() >= cur_task->get_demand()) {
                        // Pode pegar
                        cur_cost = this->__ce_ref->get_cost_v2(cur_task, cur_robot, cur_robot);
                        capacity_after_pickup_i = cur_robot->get_current_dataset_capacity() - cur_task->get_demand();
                    }
                    else {
                        // Não pode pegar -- o custo é o custo de ir à estação de entrega mais próxima e, de lá, voltar à cur_task
                        auto best_deliver_tuple = this->__ce_ref->closer_deliver_v2(cur_robot, cur_robot);
                        cur_cost = best_deliver_tuple.second;
                        cur_cost += this->__ce_ref->get_cost_v2(best_deliver_tuple.first, cur_task, cur_robot);
                        capacity_after_pickup_i = cur_robot->get_total_dataset_capacity() - cur_task->get_demand();
                    }
                    // Se ao pegar a próxima tarefa, o robô ficar impedido de pegar outra, então o custo é somado com o custo de voltar à estação de entrega
                    // Verificando se o robô ficará impedido de pegar outra tarefa:
                    bool can_exec_another_task = false;
                    for (int k = 0; k < n_tasks; k++) {
                        std::shared_ptr<PickTask> cur_task_of_all = (*this->__pick_tasks_ref)[k];
                        if (cur_task->get_id() != cur_task_of_all->get_id() &&
                                cur_task_of_all->get_current_state() == PickTask::waiting &&
                                capacity_after_pickup_i >= cur_task_of_all->get_demand()) {
                            can_exec_another_task = true;
                            break;
                        }
                    }
                    // Verificou. Agora, se 'can_exec_another_task == false', incrementa o custo
                    if (!can_exec_another_task) {
                        auto best_deliver_tuple = this->__ce_ref->closer_deliver_v2(cur_task, cur_robot);
                        cur_cost += best_deliver_tuple.second;
                    }
                }
                if (cur_cost < best_cost) {
                    best_cost = cur_cost;
                    best_robots.clear();
                    best_robots.push_back(cur_robot);
                }
                else if (cur_cost == best_cost) {
                    best_robots.push_back(cur_robot);
                }
            }
            // Push in domain list
            std::tuple<std::shared_ptr<PickTask>, std::vector<std::shared_ptr<Robot> > > tpl_task = std::make_tuple(cur_task, best_robots);
            this->__domain_list.insert({cur_task->get_id(), tpl_task});
            /*for (int j = 0; j < best_robots.size(); j++) {
                std::vector<std::shared_ptr<PickTask> > *tasks_of_this_robot = &(std::get<1>(this->__dominant_list[best_robots[j]->get_id()]));
                std::vector<std::shared_ptr<PickTask> >& tasks_of_this_robot_deref = *(tasks_of_this_robot);
                tasks_of_this_robot_deref.push_back(cur_task);
                std::tuple<std::shared_ptr<Robot>, std::vector<std::shared_ptr<PickTask> > > tpl_robot = std::make_tuple(best_robots[j], tasks_of_this_robot_deref);
                this->__dominant_list.insert({best_robots[j]->get_id(), tpl_robot});
            }*/
        }
    }
}

void Domain::compute_domain_v5() {  
    std::vector<std::shared_ptr<Robot> > best_robots;
    int n_robots = this->__robots_ref->size();
    int n_tasks = this->__pick_tasks_ref->size();
    for (int i = 0; i < n_tasks; i++) {
        std::shared_ptr<PickTask> cur_task = (*this->__pick_tasks_ref)[i];
        float best_cost = FLT_MAX;
        if (cur_task->get_current_state() == PickTask::waiting) {
            for (int j = 0; j < n_robots; j++) {
                std::shared_ptr<Robot> cur_robot = (*this->__robots_ref)[j];
                float cur_cost = this->__ce_ref->get_cost_weighted_by_capacity(cur_robot, cur_task, cur_robot);
                if (cur_cost < best_cost) {
                    best_cost = cur_cost;
                    best_robots.clear();
                    best_robots.push_back(cur_robot);
                }
                else if (cur_cost == best_cost) {
                    best_robots.push_back(cur_robot);
                }
            }
            std::tuple<std::shared_ptr<PickTask>, std::vector<std::shared_ptr<Robot> > > tpl = std::make_tuple(cur_task, best_robots);
            // Push in primary domain list
            this->__domain_list.insert({cur_task->get_id(), tpl});
        }
    }
}

void Domain::compute_dominants() {
    int n_tasks = this->__pick_tasks_ref->size();
    int n_robots = this->__robots_ref->size();
    for (int i = 0; i < n_robots; i++) {
        std::vector<std::shared_ptr<PickTask> > temp_tasks;
        std::tuple<std::shared_ptr<Robot>, std::vector<std::shared_ptr<PickTask> > > tpl = std::make_tuple((*this->__robots_ref)[i], temp_tasks);
        this->__dominant_list.insert({(*this->__robots_ref)[i]->get_id(), tpl});
    }
    for (int i = 0; i < n_tasks; i++) {
        std::shared_ptr<PickTask> cur_task = (*this->__pick_tasks_ref)[i];
        if (cur_task->get_current_state() == PickTask::waiting) {
            std::tuple<std::shared_ptr<PickTask>, std::vector<std::shared_ptr<Robot> > >
                    tpl_list = this->__domain_list[cur_task->get_id()];
            std::vector<std::shared_ptr<Robot> > cur_robot_list = std::get<1>(tpl_list);
            for (int j = 0; j < cur_robot_list.size(); j++) {
                std::shared_ptr<Robot> cur_robot = cur_robot_list[j];
                std::tuple<std::shared_ptr<Robot>, std::vector<std::shared_ptr<PickTask> > >
                        tpl_trasnp_list = this->__dominant_list[cur_robot->get_id()];
                std::vector<std::shared_ptr<PickTask> > cur_task_list = std::get<1>(tpl_trasnp_list);
                cur_task_list.push_back(cur_task);
                std::tuple<std::shared_ptr<Robot>, std::vector<std::shared_ptr<PickTask> > >
                        tpl = std::make_tuple(cur_robot, cur_task_list);
                this->__dominant_list[cur_robot->get_id()] = tpl;
            }
        }
    }
}

std::vector<std::shared_ptr<PickTask> > Domain::get_domain_of_robot(std::shared_ptr<Robot> _robot) {
    std::tuple<std::shared_ptr<Robot>, std::vector<std::shared_ptr<PickTask> > >
            tpl = this->__dominant_list[_robot->get_id()];
    std::vector<std::shared_ptr<PickTask> > all_tasks = std::get<1>(tpl);
    //Retuning waiting tasks
    std::vector<std::shared_ptr<PickTask> > waiting_tasks;
    for (int i = 0; i < all_tasks.size(); i++) {
        if (all_tasks[i]->get_current_state() == PickTask::waiting) {
            waiting_tasks.push_back(all_tasks[i]);
        }
    }
    return waiting_tasks;
}

std::vector<std::shared_ptr<Robot> > Domain::get_dominants_of_task(std::shared_ptr<PickTask> _p_task) {
    std::tuple<std::shared_ptr<PickTask>, std::vector<std::shared_ptr<Robot> > >
            tpl = this->__domain_list[_p_task->get_id()];
    return std::get<1>(tpl);
}

int Domain::get_n_primary_dominants_by_r_v2(std::shared_ptr<Robot> _robot) {
    std::tuple<std::shared_ptr<Robot>, std::vector<std::shared_ptr<PickTask> > >
            tpl = this->__dominant_list[_robot->get_id()];
    std::vector<std::shared_ptr<PickTask> > cur_tasks_ref = std::get<1>(tpl);
    return cur_tasks_ref.size();
}

std::map<uint16_t, std::tuple<std::shared_ptr<Robot>, std::vector<std::shared_ptr<PickTask> > > >
Domain::get_dominants() {
    return this->__dominant_list;
}

std::tuple<std::shared_ptr<Robot>, std::vector<std::shared_ptr<PickTask> > >
Domain::get_tuple_of_robot(uint16_t _id) {
    return this->__dominant_list[_id];
}

std::tuple<std::shared_ptr<PickTask>, std::vector<std::shared_ptr<Robot> > >
Domain::get_tuple_of_task(uint16_t _id) {
    return this->__domain_list[_id];
}

void Domain::show_domain_of_robot(std::shared_ptr<Robot> _robot) {
    std::tuple<std::shared_ptr<Robot>, std::vector<std::shared_ptr<PickTask> > >
            tpl = this->__dominant_list[_robot->get_id()];
    std::vector<std::shared_ptr<PickTask> > cur_tasks = std::get<1>(tpl);
    std::cout << "Robot " << _robot->get_id() << " (x = " << _robot->get_x() <<
            ", y = " << _robot->get_y() << ", capacity = " << _robot->get_current_dataset_capacity() <<
            ") domains over tasks:" << std::endl;
    for (int j = 0; j < cur_tasks.size(); j++) {
        std::cout << "\tt" << cur_tasks[j]->get_id() << ". Cost = " << this->__ce_ref->get_cost_v2(cur_tasks[j], _robot, _robot) << std::endl;
    }
}

void Domain::show_domains() {
    std::cout << "Domain list:" << std::endl;
    for (int i = 0; i < this->__pick_tasks_ref->size(); i++) {
        std::shared_ptr<PickTask> cur_task_at_domain_list = (*this->__pick_tasks_ref)[i];
        if (cur_task_at_domain_list->get_current_state() == PickTask::waiting) {
            std::tuple<std::shared_ptr<PickTask>, std::vector<std::shared_ptr<Robot> > >
                    tpl = this->__domain_list[cur_task_at_domain_list->get_id()];
            std::vector<std::shared_ptr<Robot> > cur_robots_at_domain_list = std::get<1>(tpl);
            std::cout << "Task " << cur_task_at_domain_list->get_id() << " (x = " << cur_task_at_domain_list->get_x() <<
                ", y = " << cur_task_at_domain_list->get_y() << ", demand = " << cur_task_at_domain_list->get_demand() <<
                ") dominated by robots:" << std::endl;
            for (int j = 0; j < cur_robots_at_domain_list.size(); j++) {
                std::cout << "\tr" << cur_robots_at_domain_list[j]->get_id() << ". Cost = " << this->__ce_ref->get_cost_v2(cur_task_at_domain_list, cur_robots_at_domain_list[j], cur_robots_at_domain_list[j]) << std::endl;
            }
        }
    }
    std::cout << std::endl;
}

void Domain::show_dominants() {
    std::cout << "Dominant list:" << std::endl;
    for (int i = 0; i < this->__robots_ref->size(); i++) {
        std::shared_ptr<Robot> cur_robot_at_transposed_list = (*this->__robots_ref)[i];
        std::tuple<std::shared_ptr<Robot>, std::vector<std::shared_ptr<PickTask> > >
                tpl = this->__dominant_list[cur_robot_at_transposed_list->get_id()];
        std::vector<std::shared_ptr<PickTask> > cur_tasks_at_transposed_list = std::get<1>(tpl);            
        std::cout << "Robot " << cur_robot_at_transposed_list->get_id() << " (x = " << cur_robot_at_transposed_list->get_x() <<
                ", y = " << cur_robot_at_transposed_list->get_y() << ", capacity = " << cur_robot_at_transposed_list->get_current_dataset_capacity() <<
                ") domains over tasks:" << std::endl;
        for (int j = 0; j < cur_tasks_at_transposed_list.size(); j++) {
            std::cout << "\tt" << cur_tasks_at_transposed_list[j]->get_id() << ". Cost = " << this->__ce_ref->get_cost_v2(cur_tasks_at_transposed_list[j], cur_robot_at_transposed_list, cur_robot_at_transposed_list) << std::endl;
        }
    }
    std::cout << std::endl;
}

Domain::~Domain() {
    for (auto it = this->__domain_list.begin(); it != this->__domain_list.end(); it++) {
        auto tpl = it->second;
        std::vector<std::shared_ptr<Robot> > tpl_robots = std::get<1>(tpl);
        tpl_robots.clear();
    }
    this->__domain_list.clear();
    
    for (auto it = this->__dominant_list.begin(); it != this->__dominant_list.end(); it++) {
        auto tpl = it->second;
        std::vector<std::shared_ptr<PickTask> > tpl_tasks = std::get<1>(tpl);
        tpl_tasks.clear();
    }
    this->__dominant_list.clear();
}

std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >
Domain::assignment_variety(std::shared_ptr<Robot> _robot, std::vector<std::shared_ptr<PickTask> > _tasks) {
    std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >
            best_tasks_ref(new std::vector<std::shared_ptr<PickTask> >[_tasks.size()]);
    
    int best_assignment_variety = 0;
    for (int i = 0; i < _tasks.size(); i++) {
#ifdef DEBUG
        std::cout << "\tIf r" << _robot->get_id() << " pick up t" << _tasks[i]->get_id() << ":" << std::endl;
#endif
        int cur_assignment_variety = 0;
        for (int j = 0; j < _tasks.size(); j++) {
            if (i != j) {
#ifdef DEBUG
                std::cout << "\t\tt" << _tasks[j]->get_id() << ": ";
#endif
                for (int k = 0; k < this->__robots_ref->size(); k++) {
                    int32_t cur_capacity = (_robot->get_id() == (*this->__robots_ref)[k]->get_id()) ?
                        _robot->get_current_dataset_capacity() - _tasks[i]->get_demand() :
                        (*this->__robots_ref)[k]->get_current_dataset_capacity();
                    if (cur_capacity >= _tasks[j]->get_demand()) {
#ifdef DEBUG
                        std::cout << "r" << (*this->__robots_ref)[k]->get_id() << " ";
#endif
                        cur_assignment_variety++;
                    }
                }
#ifdef DEBUG
                        std::cout << std::endl;
#endif
            }
        }
        if (cur_assignment_variety > best_assignment_variety) {
            best_assignment_variety = cur_assignment_variety;
            best_tasks_ref->clear();
            best_tasks_ref->push_back(_tasks[i]);
        }
        else if (cur_assignment_variety == best_assignment_variety)
            best_tasks_ref->push_back(_tasks[i]);
    }
    
#ifdef DEBUG
    std::cout << "\tWinner Tasks: ";
    for (auto it = std::begin(*best_tasks_ref); it != std::end(*best_tasks_ref); ++it)
        std::cout << "t" << (*it)->get_id() << " ";
    std::cout << std::endl;
#endif
    
    return best_tasks_ref;
}

std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >
Domain::cost_comparison(std::shared_ptr<Robot> _robot, std::vector<std::shared_ptr<PickTask> > _tasks) {
    std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >
            best_tasks_ref(new std::vector<std::shared_ptr<PickTask> >[_tasks.size()]);
    
    float best_cost = floor(FLT_MAX / (1 + RSE::cost_tolerance));
    for (int i = 0; i < _tasks.size(); i++) {
        std::shared_ptr<PickTask> cur_task_of_main = _tasks[i];
#ifdef DEBUG
        std::cout << "\tIf r" << _robot->get_id() << " pick up t" << cur_task_of_main->get_id() << ": ";
#endif
        float cur_cost = this->__ce_ref->get_cost_v2(cur_task_of_main, _robot, _robot);
#ifdef DEBUG
        std::cout << cur_cost;
#endif
        // Se ao pegar a próxima tarefa, o robô ficar impedido de pegar outra, então o custo é somado com o custo de voltar à estação de entrega
        // Verificando se o robô ficará impedido de pegar outra tarefa:
        bool can_exec_another_task = false;
        int32_t capacity_after_pickup_i = _robot->get_current_dataset_capacity() - cur_task_of_main->get_demand();
        for (int j = 0; j < this->__pick_tasks_ref->size(); j++) {
            std::shared_ptr<PickTask> cur_task_of_all = (*this->__pick_tasks_ref)[j];
            if (cur_task_of_main->get_id() != cur_task_of_all->get_id() &&
                    cur_task_of_all->get_current_state() == PickTask::waiting &&
                    capacity_after_pickup_i >= cur_task_of_all->get_demand()) {
                can_exec_another_task = true;
                break;
            }
        }
        // Verificou. Agora, se 'can_exec_another_task == false', incrementa o custo
        if (!can_exec_another_task) {
            auto best_deliver_tuple = this->__ce_ref->closer_deliver_v2(cur_task_of_main, _robot);
            cur_cost += best_deliver_tuple.second;
#ifdef DEBUG
        std::cout << " + " << best_deliver_tuple.second << std::endl;
#endif
        }
#ifdef DEBUG
        else {
            std::cout << std::endl;
        }
#endif
        
        // Comparando se o custo calculado é menor que o melhor custo conhecido
        if (cur_cost < best_cost - (best_cost * RSE::cost_tolerance)) {
            best_cost = cur_cost;
            best_tasks_ref->clear();
            best_tasks_ref->push_back(cur_task_of_main);
        }
        else if ((cur_cost >= best_cost - (best_cost * RSE::cost_tolerance)) &&
                (cur_cost <= best_cost + (best_cost * RSE::cost_tolerance)))
            best_tasks_ref->push_back(cur_task_of_main);
    }
    
#ifdef DEBUG
    std::cout << "\tWinner Tasks: ";
    for (auto it = std::begin(*best_tasks_ref); it != std::end(*best_tasks_ref); ++it)
        std::cout << "t" << (*it)->get_id() << " ";
    std::cout << std::endl;
#endif
    return best_tasks_ref;
}

std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >
Domain::euler_test(std::shared_ptr<Robot> _robot, std::vector<std::shared_ptr<PickTask> > _tasks) {
    std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >
            tasks_best_cost_ref(new std::vector<std::shared_ptr<PickTask> >[_tasks.size()]);
    
#ifdef DEBUG
        std::cout << "All tasks:" << std::endl;
        RSE::show<std::vector<std::shared_ptr<PickTask> > >(_tasks);
#endif
    
    float smaller_cost = FLT_MAX;
    for (int i = 0; i < _tasks.size(); i++) {
        std::shared_ptr<PickTask> cur_task = _tasks[i];
#ifdef DEBUG
        std::cout << "Computing Euler cycle without task t" << cur_task->get_id() << std::endl << std::endl;
#endif
        std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > tasks_copy(new std::vector<std::shared_ptr<PickTask> >(_tasks.begin(), _tasks.end()));
#ifdef DEBUG
        std::cout << "tasks_copy.size() = " << tasks_copy.size() << std::endl;
#endif
        tasks_copy->erase(tasks_copy->begin()+i);
#ifdef DEBUG
        std::cout << "tasks_copy.size() = " << tasks_copy.size() << std::endl;
        std::cout << "Input tasks for Euler cycle:" << std::endl;
        RSE::show<std::vector<std::shared_ptr<PickTask> > >(tasks_copy);
        std::cout << "Creating graph... ";
#endif
        std::shared_ptr<Graph> graph(new Graph(tasks_copy, this->__ce_ref));
#ifdef DEBUG
        std::cout << "done!" << std::endl;
#endif
        // Criando cópia do robô
        std::shared_ptr<Robot> robot_cp(new Robot(_robot));
        // Alterando sua posição para a posição da tarefa removida
        robot_cp->set_position(cur_task->get_position());
        
        auto ec_tpl = graph->euler_tour_v3(robot_cp);
        float cur_cost = std::get<1>(ec_tpl);
        if (cur_cost < smaller_cost) {
            smaller_cost = cur_cost;
            tasks_best_cost_ref->clear();
            tasks_best_cost_ref->push_back(cur_task);
        }
        else if (cur_cost == smaller_cost)
            tasks_best_cost_ref->push_back(cur_task);
        
        robot_cp.reset();
        graph.reset();
        tasks_copy.reset();
    }
#ifdef DEBUG
    std::cout << "\tWinner Tasks: ";
    for (auto it = std::begin(*tasks_best_cost_ref); it != std::end(*tasks_best_cost_ref); ++it)
        std::cout << "t" << (*it)->get_id() << " ";
    std::cout << std::endl;
#endif
    
    return tasks_best_cost_ref;
}

std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >
Domain::tsp_test(std::shared_ptr<Robot> _robot, std::vector<std::shared_ptr<PickTask> > _tasks) {
    std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >
            tasks_best_cost_ref(new std::vector<std::shared_ptr<PickTask> >[_tasks.size()]);
    
#ifdef DEBUG
        std::cout << "All tasks:" << std::endl;
        RSE::show<std::vector<std::shared_ptr<PickTask> > >(_tasks);
#endif
    
    float smaller_cost = FLT_MAX;
    for (int i = 0; i < _tasks.size(); i++) {
        std::shared_ptr<PickTask> cur_task = _tasks[i];
#ifdef DEBUG
        std::cout << "Computing TSP without task t" << cur_task->get_id() << std::endl << std::endl;
#endif
        std::vector<std::shared_ptr<PickTask> > tasks_copy(_tasks.begin(), _tasks.end());
#ifdef DEBUG
        std::cout << "tasks_copy.size() = " << tasks_copy.size() << std::endl;
#endif
        tasks_copy.erase(tasks_copy.begin()+i);
#ifdef DEBUG
        std::cout << "tasks_copy.size() = " << tasks_copy.size() << std::endl;
        std::cout << "Input tasks for TSP:" << std::endl;
        RSE::show<std::vector<std::shared_ptr<PickTask> > >(tasks_copy);
        std::cout << "Creating object... ";
#endif
        std::shared_ptr<TSP> tsp(new TSP(tasks_copy, _robot, this->__ce_ref));
#ifdef DEBUG
        std::cout << "done!" << std::endl;
        tsp->printCities();
#endif
        int tsp_size = tsp->get_size();
#ifdef DEBUG
        std::cout << "Graph (before filling matrix)... " << std::endl;
        tsp->printGraph();
        std::cout << "Filling matrix... ";
#endif
        tsp->fillMatrix();
#ifdef DEBUG
        std::cout << "done!" << std::endl;
        std::cout << "Graph (after filling matrix)... " << std::endl;
        tsp->printGraph();
#endif
        tsp->findMST();
#ifdef DEBUG
        std::cout << "MST created" << std::endl;
#endif
        tsp->perfectMatching();
#ifdef DEBUG
        std::cout << "Matching completed" << std::endl;
        std::cout << "Loop through each index and find shortest path... ";
#endif
        int best = INT_MAX;
        int bestIndex;
        for (long t = 0; t < tsp_size; t++) {
                int result = tsp->findBestPath(t);

                tsp->path_vals[t][0] = t; // set start
                tsp->path_vals[t][1] = result; // set end

                if (tsp->path_vals[t][1] < best) {
                        bestIndex = tsp->path_vals[t][0];
                        best = tsp->path_vals[t][1];
                }
        }
#ifdef DEBUG
        std::cout << "done!" << std::endl;
        std::cout << "Create path for best tour... ";
#endif
        tsp->euler_tour(bestIndex,tsp->circuit);
        tsp->make_hamiltonian(tsp->circuit,tsp->pathLength);
#ifdef DEBUG
        std::cout << "done!" << std::endl;
        std::cout << "Final length: " << tsp->pathLength << std::endl;
        tsp->printPath();
#endif
        float cur_cost = tsp->pathLength;
        if (cur_cost < smaller_cost) {
            smaller_cost = cur_cost;
            tasks_best_cost_ref->clear();
            tasks_best_cost_ref->push_back(cur_task);
        }
        else if (cur_cost == smaller_cost)
            tasks_best_cost_ref->push_back(cur_task);
        
        tsp.reset();
    }
#ifdef DEBUG
    std::cout << "\tWinner Tasks: ";
    for (auto it = std::begin(*tasks_best_cost_ref); it != std::end(*tasks_best_cost_ref); ++it)
        std::cout << "t" << (*it)->get_id() << " ";
    std::cout << std::endl;
#endif
    
    return tasks_best_cost_ref;
}

std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >
Domain::utility_test(std::shared_ptr<Robot> _robot, std::vector<std::shared_ptr<PickTask> > _tasks) {
    std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >
            tasks_max_utility_ref(new std::vector<std::shared_ptr<PickTask> >[_tasks.size()]);
    
    int max_utility = 0;
    for (int i = 0; i < _tasks.size(); i++) {
        std::shared_ptr<PickTask> cur_task_of_main = _tasks[i];
        int32_t capacity_after_pickup_i = _robot->get_current_dataset_capacity() - cur_task_of_main->get_demand();
        int cur_utility = 0;
        for (int j = 0; j < this->__pick_tasks_ref->size(); j++) {
            std::shared_ptr<PickTask> cur_task_of_all = (*this->__pick_tasks_ref)[j];
            if (cur_task_of_main->get_id() != cur_task_of_all->get_id() &&
                    cur_task_of_all->get_current_state() == PickTask::waiting &&
                    capacity_after_pickup_i >= cur_task_of_all->get_demand()) {
                cur_utility++;
            }
        }
#ifdef DEBUG
        std::cout << "\tIf r" << _robot->get_id() << " pick up t" << cur_task_of_main->get_id() <<
                ": capacity = " << capacity_after_pickup_i << " (utility = " << cur_utility << ")" << std::endl;
#endif
        if (cur_utility > max_utility) {
            max_utility = cur_utility;
            tasks_max_utility_ref->clear();
            tasks_max_utility_ref->push_back(cur_task_of_main);
        }
        else if (cur_utility == max_utility)
            tasks_max_utility_ref->push_back(cur_task_of_main);
    }
    
    // Verificando se deu empate
    if (tasks_max_utility_ref->size() > 1) { //Deu empate
        // Monta a lista de tarefas de maior demanda
#ifdef DEBUG
        std::cout << "\tIn case of a tie, r" << _robot->get_id() << " must take the task(s) with the highest demand: " << std::endl;
#endif
        int32_t larger_demand = 0;
        std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >
                tasks_best_demand_ref(new std::vector<std::shared_ptr<PickTask> >[tasks_max_utility_ref->size()]);
        for (int i = 0; i < tasks_max_utility_ref->size(); i++) {
            std::shared_ptr<PickTask> cur_task = (*tasks_max_utility_ref)[i];
            if (cur_task->get_demand() > larger_demand) {
                larger_demand = cur_task->get_demand();
                tasks_best_demand_ref->clear();
                tasks_best_demand_ref->push_back(cur_task);
            }
            else if (cur_task->get_demand() == larger_demand)
                tasks_best_demand_ref->push_back(cur_task);
        }
#ifdef DEBUG
        std::cout << "\t\tTask(s) with the highest demand: ";
        for (auto it = std::begin(*tasks_best_demand_ref); it != std::end(*tasks_best_demand_ref); ++it) {
            std::cout << "t" << (*it)->get_id() << " ";
        }
        std::cout << std::endl;
#endif
        tasks_max_utility_ref->clear();
        return tasks_best_demand_ref;
    }
    else { // Não deu empate
#ifdef DEBUG
        std::cout << "Winner task: t" << (*tasks_max_utility_ref)[0] << std::endl;
#endif
        return tasks_max_utility_ref;
    }
}