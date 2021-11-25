/* 
 * File:   Scheduler.cpp
 * Author: geoso
 * 
 * Created on 10 de Março de 2021, 11:29
 */

#include "Scheduler.hpp"
#include "Graph.hpp"

bool Scheduler::__compare_arrivals(std::tuple<std::shared_ptr<Robot>, float, int, bool> _a1, std::tuple<std::shared_ptr<Robot>, float, int, bool> _a2) {
    std::shared_ptr<Robot> robot_1 = std::get<0>(_a1);
    std::shared_ptr<Robot> robot_2 = std::get<0>(_a2);
    float time_robot_1 = std::get<1>(_a1);
    float time_robot_2 = std::get<1>(_a2);
    int n_dominants_robot_1 = std::get<2>(_a1);
    int n_dominants_robot_2 = std::get<2>(_a2);
    if (time_robot_1 == time_robot_2) {
        if (n_dominants_robot_1 == n_dominants_robot_2)
            return (robot_1->get_id() < robot_2->get_id());
        return (n_dominants_robot_1 > n_dominants_robot_2);
    }
    return (time_robot_1 < time_robot_2);
}

Scheduler::Scheduler(std::shared_ptr<std::vector<std::shared_ptr<Robot> > > _robots_ref) {
    int n_robots = _robots_ref->size();    
    for (int i = 0; i < n_robots; i++) {
        this->__arrivals.push_back(std::make_tuple((*_robots_ref)[i], 0.0, 0, false));
    }
#ifdef DEBUG
    std::cout << "Arrivals list (after Constructor):" << std::endl;
    this->show_arrivals();
    std::cout << std::endl;
#endif
}

void Scheduler::decrease_time_of_all(std::shared_ptr<Robot> _robot) {
    //Primeiro, procura o endereço do robô em arrivals
    int addr;
    for (int i = 0; i < this->__arrivals.size(); i++) {
        std::shared_ptr<Robot> rbt = std::get<0>(this->__arrivals[i]);
        if (rbt->get_id() == _robot->get_id()) {
            addr = i;
            break;
        }
    }
    
    std::shared_ptr<Robot> found_robot = std::get<0>(this->__arrivals[addr]);
    float tm = std::get<1>(this->__arrivals[addr]);
    
    // Atualiza
    for (int i = 0; i < this->__arrivals.size(); i++) {
        std::shared_ptr<Robot> rbt = std::get<0>(this->__arrivals[i]);
        float cur_tm = std::get<1>(this->__arrivals[i]);
        int n_dominants = std::get<2>(this->__arrivals[i]);
        float new_tm = (cur_tm - tm < 0.0) ? 0.0 : cur_tm - tm;
        bool must_compute_domain;
        if (new_tm == 0 && (cur_tm != 0.0 || tm != 0.0) && rbt->get_id() != found_robot->get_id())
            must_compute_domain = true;
        else
            must_compute_domain = std::get<3>(this->__arrivals[i]);
        this->__arrivals[i] = std::make_tuple(rbt, new_tm, n_dominants, must_compute_domain);
    }
}

std::tuple<int, int, float, int, int> Scheduler::nCAR_v2(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > _pick_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > > _deliver_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > > _recharge_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<Robot> > >& _robots_ref,
        CostEstimator::costing_t _costing_type) {
    std::vector<std::shared_ptr<PickTask> > pick_tasks = *(_pick_tasks_ref);
    std::vector<std::shared_ptr<Robot> > robots = *(_robots_ref);
    
    std::shared_ptr<CostEstimator> ce(new CostEstimator(_pick_tasks_ref, _deliver_tasks_ref, _recharge_tasks_ref, _robots_ref));
    ce->set_costing_type(_costing_type);
    int n_visits_to_depot = 0;
    float c_total = 0.0;
    float n_assigned = 0.0, total_tasks = (float)(_pick_tasks_ref->size() - 1);
    std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > t;
    std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > not_t = _pick_tasks_ref;
    while (n_assigned < total_tasks) {
    //while(!PickTask::all_assigned(_pick_tasks)) {
        for (int i = 0; i < robots.size(); i++) {
            auto tpl = Scheduler::__feasible_route_v2(not_t, robots[i], ce);
            t = std::get<0>(tpl);
            not_t = std::get<1>(tpl);
            std::shared_ptr<Graph> g(new Graph(t, ce));
            auto circuit = g->tsp_christofides_v2(robots[i]);
            std::vector<std::shared_ptr<Task> > cur_path = std::get<0>(circuit);
            c_total += std::get<1>(circuit);
            robots[i]->add_scheduling_cost(std::get<1>(circuit));
            for (int j = 0; j < cur_path.size(); j++) {
                robots[i]->add_task_ref(cur_path[j].get());
            }
            // Setting position
            std::shared_ptr<Task> last_task = cur_path[cur_path.size()-1];
            robots[i]->set_position(last_task->get_position());
            for (int j = 0; j < (*t).size(); j++) {
                (*t)[j]->set_current_state(PickTask::assigned);
            }
#ifdef DEBUG
            std::cout << "Returned circuit: ";
            for (int i = 0; i < cur_path.size(); i++) {
                std::shared_ptr<Task> cur_task = cur_path[i];
                if (cur_task->get_task_type() == Task::pick)
                    std::cout << "t" << cur_task->get_id() << " ";
                else if (cur_task->get_task_type() == Task::deliver)
                    std::cout << "d" << cur_task->get_id() << " ";
            }
            std::cout << std::endl;
            RSE::show<std::vector<std::shared_ptr<PickTask> > >(*not_t);
#endif
            n_visits_to_depot++;
            n_assigned += (float)(cur_path.size() - 1);
            std::cerr << "[" << RSE::current_execution << " of " << RSE::number_executions << "]: " << 
                    (n_assigned / total_tasks)*100 << "% Completed...\r";
            if (n_assigned == total_tasks) break;
        }
    }
    int n_sch_robots = 0;
    for (int i = 0; i < _robots_ref->size(); i++) {
        if ((*_robots_ref)[i]->get_n_tasks() > 0)
            n_sch_robots++;
    }
    return std::make_tuple(n_sch_robots, n_visits_to_depot, c_total, -1, 0);
}

std::tuple<int, int, float, int, int> Scheduler::fifo(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > _pick_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > > _deliver_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > > _recharge_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<Robot> > >& _robots_ref,
        CostEstimator::costing_t _costing_type) {
    std::shared_ptr<CostEstimator> ce(new CostEstimator(_pick_tasks_ref, _deliver_tasks_ref, _recharge_tasks_ref, _robots_ref));
    ce->set_costing_type(_costing_type);
    float c_total = 0.0;
    int n_visits_to_depot = 0;
    int i = 0, j = 0;
    float n_assigned = 0.0, total_tasks = (float)(_pick_tasks_ref->size() - 1);
    while (j < _pick_tasks_ref->size()) {
        std::shared_ptr<Robot> cur_robot = (*_robots_ref)[i];
        std::shared_ptr<PickTask> cur_task = (*_pick_tasks_ref)[j];
        if (cur_robot->get_total_dataset_capacity() < cur_task->get_demand()) {
            i = (i + 1 == _robots_ref->size()) ? 0 : i + 1;
            continue;
        }
        if (cur_robot->get_current_dataset_capacity() >= cur_task->get_demand()) {
            float cost_from_robot_to_task = ce->get_cost_v2(cur_task, cur_robot, cur_robot);
            c_total += cost_from_robot_to_task;
            cur_robot->add_scheduling_cost(cost_from_robot_to_task);
            cur_robot->add_task_ref(cur_task.get());
            cur_robot->decrease_from_current_capacity(cur_task->get_demand());
            cur_task->set_current_state(PickTask::assigned);
            j++;
            n_assigned += 1.0;
            std::cerr << "[" << RSE::current_execution << " of " << RSE::number_executions << "]: " << 
                    (n_assigned / total_tasks)*100 << "% Completed...\r";
        }
        else {
            auto best_deliver_tuple = ce->closer_deliver_v2(cur_robot, cur_robot);
            std::shared_ptr<DeliverTask> best_deliver = best_deliver_tuple.first;
            float cost_to_best_deliver = best_deliver_tuple.second;
            c_total += cost_to_best_deliver;
            cur_robot->add_scheduling_cost(cost_to_best_deliver);
            cur_robot->add_task_ref(best_deliver.get());
            cur_robot->reset_current_capacity();
            i = (i + 1 == _robots_ref->size()) ? 0 : i + 1;
            n_visits_to_depot++;
        }
    }
    for (int i = 0; i < _robots_ref->size(); i++) {
        if ((*_robots_ref)[i]->get_n_tasks()) {
            Task* last_task_ref = (*_robots_ref)[i]->get_task_ref((*_robots_ref)[i]->get_n_tasks() - 1);
            if (last_task_ref->get_task_type() != Task::deliver) {
                auto best_deliver_tuple = ce->closer_deliver_v2((*_robots_ref)[i], (*_robots_ref)[i]);
                float cost_to_best_deliver = best_deliver_tuple.second;
                //float cost_to_best_deliver = ce->get_manhattan_cost(_robots[i], best_deliver_tuple.first);
                c_total += cost_to_best_deliver;
                (*_robots_ref)[i]->add_scheduling_cost(cost_to_best_deliver);
                (*_robots_ref)[i]->add_task_ref(best_deliver_tuple.first.get());
                n_visits_to_depot++;
            }
        }
    }
    int n_sch_robots = 0;
    for (int i = 0; i < _robots_ref->size(); i++) {
        if ((*_robots_ref)[i]->get_n_tasks() > 0)
            n_sch_robots++;
    }
    return std::make_tuple(n_sch_robots, n_visits_to_depot, c_total, -1, 0);
}

std::tuple<int, int, float, int, int> Scheduler::greedy(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > _pick_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > > _deliver_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > > _recharge_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<Robot> > >& _robots_ref,
        CostEstimator::costing_t _costing_type) {
    CostEstimator *ce = new CostEstimator(_pick_tasks_ref, _deliver_tasks_ref, _recharge_tasks_ref, _robots_ref);
    ce->set_costing_type(_costing_type);
    float c_total = 0.0;
    int n_visits_to_depot = 0;
    float n_assigned = 0.0, total_tasks = (float)(_pick_tasks_ref->size() - 1);
    while (n_assigned < total_tasks) {
    //while(!PickTask::all_assigned(_pick_tasks)) {
        for (int i = 0; i < _robots_ref->size(); i++) {
            std::shared_ptr<Robot> cur_robot = (*_robots_ref)[i];
#ifdef DEBUG
            fprintf(stdout, "Robot %d: position = (%d, %d), capacity = %d\n", cur_robot->get_id(), cur_robot->get_x(), cur_robot->get_y(), cur_robot->get_current_dataset_capacity());
#endif
            float min_cost = FLT_MAX;
            std::vector<std::shared_ptr<PickTask> > tasks_min_cost;
            for (int j = 0; j < _pick_tasks_ref->size(); j++) {
                std::shared_ptr<PickTask> cur_task = (*_pick_tasks_ref)[j];
#ifdef DEBUG
                fprintf(stdout, "\tTask %d: position = (%d, %d), demand = %d\n", cur_task->get_id(), cur_task->get_x(), cur_task->get_y(), cur_task->get_demand());
#endif
                if (cur_task->get_current_state() == PickTask::waiting) {
                    float cur_cost =
                            (cur_robot->get_total_dataset_capacity() < cur_task->get_demand()) ? FLT_MAX : ce->get_cost_v2(cur_task, cur_robot, cur_robot);
#ifdef DEBUG
                    std::cout << "\t\tCost = " << cur_cost << std::endl;
#endif
                    if (cur_cost < min_cost) {
                        min_cost = cur_cost;
                        tasks_min_cost.clear();
                        tasks_min_cost.push_back(cur_task);
                    }
                    else if (cur_cost == min_cost) {
                        tasks_min_cost.push_back(cur_task);
                    }
                }
            }
            std::shared_ptr<PickTask> best_task = NULL;
            // Se não der empate, pega a primeira (e única) tarefa:
#ifdef DEBUG
            std::cout << "tasks_min_cost.size() = " << tasks_min_cost.size() << std::endl;
#endif
            if (tasks_min_cost.size() == 0) break;
            best_task = tasks_min_cost[0];
            /*if (tasks_min_cost.size() == 1) {
                best_task = tasks_min_cost[0];
            }
            // Se der empate, pega a tarefa de maior demanda que o robô consegue carregar
            else {
                int32_t larger_demand = 0;
                for (int i = 0; i < tasks_min_cost.size(); i++) {
                    std::shared_ptr<PickTask> cur_task = tasks_min_cost[i];
                    if (cur_task->get_demand() > larger_demand &&
                            cur_robot->get_current_dataset_capacity() >= cur_task->get_demand()) {
                        larger_demand = cur_task->get_demand();
                        best_task = cur_task;
                    }
                }
                // Se best_task é vazio, quer dizer que o robô não tem capacidade para carregar nenhuma tarefa.
                // Sendo assim, pega a tarefa de maior demanda
                if (!best_task) {
                    int32_t larger_demand = 0;
                    for (int i = 0; i < tasks_min_cost.size(); i++) {
                        std::shared_ptr<PickTask> cur_task = tasks_min_cost[i];
                        if (cur_task->get_demand() > larger_demand) {
                            larger_demand = cur_task->get_demand();
                            best_task = cur_task;
                        }
                    }
                }
            }
            // Se mesmo assim best_task ficar vazio, isso significa que todas as tarefas foram atribuídas
            // antes que o laço sobre o vetor de robôs terminasse. Se é assim, é só quebrar o laço
            if (!best_task) break;*/
            // Atribuindo a tarefa ao robô
            if (cur_robot->get_current_dataset_capacity() >= best_task->get_demand()) {
                float cost_from_robot_to_task = ce->get_cost_v2(best_task, cur_robot, cur_robot);
                c_total += cost_from_robot_to_task;
                cur_robot->add_scheduling_cost(cost_from_robot_to_task);
                cur_robot->add_task_ref(best_task.get());
                cur_robot->decrease_from_current_capacity(best_task->get_demand());
                cur_robot->set_x(best_task->get_x());
                cur_robot->set_y(best_task->get_y());
                best_task->set_current_state(PickTask::assigned);
                n_assigned += 1.0;
                std::cerr << "[" << RSE::current_execution << " of " << RSE::number_executions << "]: " << 
                    (n_assigned / total_tasks)*100 << "% Completed...\r";
            }
            else {
                auto best_deliver_tuple = ce->closer_deliver_v2(cur_robot, cur_robot);
                std::shared_ptr<DeliverTask> best_deliver = best_deliver_tuple.first;
                float cost_to_best_deliver = best_deliver_tuple.second;
                c_total += cost_to_best_deliver;
                cur_robot->add_scheduling_cost(cost_to_best_deliver);
                cur_robot->add_task_ref(best_deliver.get());
                cur_robot->reset_current_capacity();
                cur_robot->set_x(best_deliver->get_x());
                cur_robot->set_y(best_deliver->get_y());
                n_visits_to_depot++;
            }
        }
    }
    // Inserindo o melhor deliver_task no final e somando custos
    for (int i = 0; i < _robots_ref->size(); i++) {
        if ((*_robots_ref)[i]->get_n_tasks()) {
            Task* last_task_ref = (*_robots_ref)[i]->get_task_ref((*_robots_ref)[i]->get_n_tasks() - 1);
            if (last_task_ref->get_task_type() != Task::deliver) {
                auto best_deliver_tuple = ce->closer_deliver_v2((*_robots_ref)[i], (*_robots_ref)[i]);
                float cost_to_best_deliver = best_deliver_tuple.second;
                c_total += cost_to_best_deliver;
                (*_robots_ref)[i]->add_scheduling_cost(cost_to_best_deliver);
                (*_robots_ref)[i]->add_task_ref(best_deliver_tuple.first.get());
                n_visits_to_depot++;
            }
        }
    }
    int n_sch_robots = 0;
    for (int i = 0; i < _robots_ref->size(); i++) {
        if ((*_robots_ref)[i]->get_n_tasks() > 0)
            n_sch_robots++;
    }
    return std::make_tuple(n_sch_robots, n_visits_to_depot, c_total, -1, 0);
}

std::tuple<int, int, float, int, int> Scheduler::greedy_v2(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > _pick_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > > _deliver_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > > _recharge_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<Robot> > >& _robots_ref,
        CostEstimator::costing_t _costing_type) {
    CostEstimator *ce = new CostEstimator(_pick_tasks_ref, _deliver_tasks_ref, _recharge_tasks_ref, _robots_ref);
    ce->set_costing_type(_costing_type);
    float c_total = 0.0;
    int n_visits_to_depot = 0;
    float n_assigned = 0.0, total_tasks = (float)(_pick_tasks_ref->size() - 1);
    while (n_assigned < total_tasks) {
    //while(!PickTask::all_assigned(_pick_tasks)) {
        for (int i = 0; i < _robots_ref->size(); i++) {
            std::shared_ptr<Robot> cur_robot = (*_robots_ref)[i];
#ifdef DEBUG
            fprintf(stdout, "Robot %d: position = (%d, %d), capacity = %d\n", cur_robot->get_id(), cur_robot->get_x(), cur_robot->get_y(), cur_robot->get_current_dataset_capacity());
#endif
            float min_cost = FLT_MAX;
            std::vector<std::shared_ptr<PickTask> > tasks_min_cost;
            for (int j = 0; j < _pick_tasks_ref->size(); j++) {
                std::shared_ptr<PickTask> cur_task = (*_pick_tasks_ref)[j];
#ifdef DEBUG
                fprintf(stdout, "\tTask %d: position = (%d, %d), demand = %d\n", cur_task->get_id(), cur_task->get_x(), cur_task->get_y(), cur_task->get_demand());
#endif
                if (cur_task->get_current_state() == PickTask::waiting) {
                    float cur_cost =
                            (cur_robot->get_total_dataset_capacity() < cur_task->get_demand()) ? FLT_MAX : ce->get_cost_v2(cur_task, cur_robot, cur_robot);
#ifdef DEBUG
                    std::cout << "\t\tCost = " << cur_cost << std::endl;
#endif
                    if (cur_cost < min_cost) {
                        min_cost = cur_cost;
                        tasks_min_cost.clear();
                        tasks_min_cost.push_back(cur_task);
                    }
                    else if (cur_cost == min_cost) {
                        tasks_min_cost.push_back(cur_task);
                    }
                }
            }
            std::shared_ptr<PickTask> best_task = NULL;
            // Se não der empate, pega a primeira (e única) tarefa:
#ifdef DEBUG
            std::cout << "tasks_min_cost.size() = " << tasks_min_cost.size() << std::endl;
#endif
            if (tasks_min_cost.size() == 1) {
                best_task = tasks_min_cost[0];
            }
            // Se der empate, pega a tarefa de maior demanda que o robô consegue carregar
            else {
                int32_t larger_demand = 0;
                for (int i = 0; i < tasks_min_cost.size(); i++) {
                    std::shared_ptr<PickTask> cur_task = tasks_min_cost[i];
                    if (cur_task->get_demand() > larger_demand &&
                            cur_robot->get_current_dataset_capacity() >= cur_task->get_demand()) {
                        larger_demand = cur_task->get_demand();
                        best_task = cur_task;
                    }
                }
                // Se best_task é vazio, quer dizer que o robô não tem capacidade para carregar nenhuma tarefa.
                // Sendo assim, pega a tarefa de maior demanda
                if (!best_task) {
                    int32_t larger_demand = 0;
                    for (int i = 0; i < tasks_min_cost.size(); i++) {
                        std::shared_ptr<PickTask> cur_task = tasks_min_cost[i];
                        if (cur_task->get_demand() > larger_demand) {
                            larger_demand = cur_task->get_demand();
                            best_task = cur_task;
                        }
                    }
                }
            }
            // Se mesmo assim best_task ficar vazio, isso significa que todas as tarefas foram atribuídas
            // antes que o laço sobre o vetor de robôs terminasse. Se é assim, é só quebrar o laço
            if (!best_task) break;
            // Atribuindo a tarefa ao robô
            if (cur_robot->get_current_dataset_capacity() >= best_task->get_demand()) {
                float cost_from_robot_to_task = ce->get_cost_v2(best_task, cur_robot, cur_robot);
                c_total += cost_from_robot_to_task;
                cur_robot->add_scheduling_cost(cost_from_robot_to_task);
                cur_robot->add_task_ref(best_task.get());
                cur_robot->decrease_from_current_capacity(best_task->get_demand());
                cur_robot->set_x(best_task->get_x());
                cur_robot->set_y(best_task->get_y());
                best_task->set_current_state(PickTask::assigned);
                n_assigned += 1.0;
                std::cerr << "[" << RSE::current_execution << " of " << RSE::number_executions << "]: " << 
                    (n_assigned / total_tasks)*100 << "% Completed...\r";
            }
            else {
                auto best_deliver_tuple = ce->closer_deliver_v2(cur_robot, cur_robot);
                std::shared_ptr<DeliverTask> best_deliver = best_deliver_tuple.first;
                float cost_to_best_deliver = best_deliver_tuple.second;
                c_total += cost_to_best_deliver;
                cur_robot->add_scheduling_cost(cost_to_best_deliver);
                cur_robot->add_task_ref(best_deliver.get());
                cur_robot->reset_current_capacity();
                cur_robot->set_x(best_deliver->get_x());
                cur_robot->set_y(best_deliver->get_y());
                n_visits_to_depot++;
            }
        }
    }
    // Inserindo o melhor deliver_task no final e somando custos
    for (int i = 0; i < _robots_ref->size(); i++) {
        if ((*_robots_ref)[i]->get_n_tasks()) {
            Task* last_task_ref = (*_robots_ref)[i]->get_task_ref((*_robots_ref)[i]->get_n_tasks() - 1);
            if (last_task_ref->get_task_type() != Task::deliver) {
                auto best_deliver_tuple = ce->closer_deliver_v2((*_robots_ref)[i], (*_robots_ref)[i]);
                float cost_to_best_deliver = best_deliver_tuple.second;
                c_total += cost_to_best_deliver;
                (*_robots_ref)[i]->add_scheduling_cost(cost_to_best_deliver);
                (*_robots_ref)[i]->add_task_ref(best_deliver_tuple.first.get());
                n_visits_to_depot++;
            }
        }
    }
    int n_sch_robots = 0;
    for (int i = 0; i < _robots_ref->size(); i++) {
        if ((*_robots_ref)[i]->get_n_tasks() > 0)
            n_sch_robots++;
    }
    return std::make_tuple(n_sch_robots, n_visits_to_depot, c_total, -1, 0);
}

bool Scheduler::must_compute_domain() {
#ifdef DEBUG
    std::cout << "Arrivals list:" << std::endl;
    this->show_arrivals();
    std::cout << std::endl;
#endif
    
    // arrivals_list é ordenado em prepare() e set_time(), assim, varre a lista e retorna o booleano com dominância maior que zero
    bool must_compute_domain;
    for (int i = 0; i < this->__arrivals.size(); i++) {
        int n_dominants = std::get<2>(this->__arrivals[i]);
        if (n_dominants) {
            must_compute_domain = std::get<3>(this->__arrivals[i]);
#ifdef DEBUG
            std::cout << "Returning value: " << must_compute_domain << std::endl;
#endif
            break;
        }
    }
    return must_compute_domain;
}

std::shared_ptr<Robot> Scheduler::next_robot_v2() {
#ifdef DEBUG
    std::cout << "Arrivals list:" << std::endl;
    this->show_arrivals();
    std::cout << std::endl;
#endif
    
     // arrivals_list é ordenado em prepare() e set_time(), assim, varre a lista e retorna o melhor robô com dominância maior que zero
    std::shared_ptr<Robot> best_robot = NULL;
    for (int i = 0; i < this->__arrivals.size(); i++) {
        int n_dominants = std::get<2>(this->__arrivals[i]);
        if (n_dominants) {
            best_robot = std::get<0>(this->__arrivals[i]);
#ifdef DEBUG
            std::cout << "Returning robot: r" << best_robot->get_id() << std::endl;
#endif
            break;
        }
    }
    return best_robot;
}

void Scheduler::prepare_v2(std::shared_ptr<Domain> _dom) {
    for (int i = 0; i < this->__arrivals.size(); i++) {
        std::shared_ptr<Robot> rbt = std::get<0>(this->__arrivals[i]);
        float tm = std::get<1>(this->__arrivals[i]); 
        this->__arrivals[i] = std::make_tuple(rbt, tm, _dom->get_n_primary_dominants_by_r_v2(rbt), false);
    }
    
#ifdef DEBUG
    std::cout << "Arrivals list (before sorting):" << std::endl;
    this->show_arrivals();
    std::cout << std::endl;
#endif
    //Ordena arrivals_deref
    //[]( float acc, std::pair<uint16_t, float> p ) { return ( acc + p.second ); }
    std::sort(this->__arrivals.begin(), this->__arrivals.end(), Scheduler::__compare_arrivals);
    
#ifdef DEBUG
    std::cout << "Arrivals list (after sorting):" << std::endl;
    this->show_arrivals();
    std::cout << std::endl;
#endif
}

std::tuple<int, int, float, int, int> Scheduler::done(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > _pick_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > > _deliver_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > > _recharge_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<Robot> > >& _robots_ref,
        CostEstimator::costing_t _costing_type) {
    std::shared_ptr<CostEstimator> ce(new CostEstimator(_pick_tasks_ref, _deliver_tasks_ref, _recharge_tasks_ref, _robots_ref));
    ce->set_costing_type(_costing_type);
    std::shared_ptr<Domain> dom(new Domain(_pick_tasks_ref, _robots_ref, ce));
    float c_total = 0.0;
    int n_visits_to_depot = 0;
    int n_domain_comput = 0;
    int n_domain_with_unique_robot_per_task = 0;
    std::vector<std::shared_ptr<PickTask> > dominated_tasks;
    std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > best_costed_tasks_ref, greater_utility_tasks_ref, best_assignment_tasks_ref, best_tsp_tasks_ref;
    float n_assigned = 0.0, total_tasks = (float)(_pick_tasks_ref->size() - 1);
    while (n_assigned < total_tasks) {
    //while(!PickTask::all_assigned(*_pick_tasks_ref)) {
#ifdef DEBUG
        std::cout << "Computing domains" << std::endl;
#endif
        bool unique_robots = dom->compute_domain_v1();
        n_domain_comput++;
        if (unique_robots) n_domain_with_unique_robot_per_task++;
#ifdef DEBUG
        dom->show_domains();
#endif
#ifdef DEBUG
        std::cout << "Computing dominants" << std::endl;
#endif
        dom->compute_dominants();
#ifdef DEBUG
        dom->show_dominants();
        std::cout << "Copying robot attributes" << std::endl;
#endif
        std::shared_ptr<std::vector<std::shared_ptr<Robot> > >
                backup_of_robots_ref(new std::vector<std::shared_ptr<Robot> >[_robots_ref->size()]);
        for (int i = 0; i < _robots_ref->size(); i++) {
            std::shared_ptr<Robot> temp_cp_robot(new Robot((*_robots_ref)[i]));
            backup_of_robots_ref->push_back(temp_cp_robot);
        }
        for (int i = 0; i < _robots_ref->size(); i++) {
#ifdef DEBUG
            std::cout << "Getting next robot from queue" << std::endl;
#endif
            std::shared_ptr<Robot> cur_robot = (*_robots_ref)[i];
            std::shared_ptr<PickTask> cur_task;
#ifdef DEBUG
            std::cout << "\tGot robot r" << cur_robot->get_id() << " (x = " << cur_robot->get_x() <<
                ", y = " << cur_robot->get_y() << ", capacity = " << cur_robot->get_current_dataset_capacity() <<
                ")" << std::endl;
#endif
            dominated_tasks = dom->get_domain_of_robot(cur_robot);
#ifdef DEBUG
            std::cout << "\t\tDominated tasks:" << std::endl;
            RSE::show<std::vector<std::shared_ptr<PickTask> > >(dominated_tasks);
#endif
            uint32_t n_dominants = dominated_tasks.size();
            if (n_dominants == 0) continue;
            if (n_dominants == 1)
                cur_task = dominated_tasks[0];
            else {
                best_costed_tasks_ref = dom->cost_comparison(cur_robot, dominated_tasks);
#ifdef DEBUG
                std::cout << "\t\tBest costed tasks:" << std::endl;
                RSE::show<std::vector<std::shared_ptr<PickTask> > >(*best_costed_tasks_ref);
#endif
                //if (best_costed_tasks_ref->size() == 1) {
                cur_task = (*best_costed_tasks_ref)[0];
                /*float min_cost = FLT_MAX;
                for (int j = 0; j < best_costed_tasks_ref->size(); j++) {
                    float cur_cost = ce->get_cost_v2((*best_costed_tasks_ref)[j], cur_robot, cur_robot);
                    if (cur_cost < min_cost) {
                        min_cost = cur_cost;
                        cur_task = (*best_costed_tasks_ref)[j];
                    }
                }*/
                /*}
                else {
                    greater_utility_tasks_ref = dom->utility_test(cur_robot, *best_costed_tasks_ref);
#ifdef DEBUG
                    std::cout << "\t\tGreater utility tasks:" << std::endl;
                    RSE::show<std::vector<std::shared_ptr<PickTask> > >(*greater_utility_tasks_ref);
#endif
                    if (greater_utility_tasks_ref->size() == 1)
                        cur_task = (*greater_utility_tasks_ref)[0];
                    else {
                        best_assignment_tasks_ref = dom->assignment_variety(cur_robot, *greater_utility_tasks_ref);
#ifdef DEBUG
                        std::cout << "\t\tBest assignment tasks:" << std::endl;
                        RSE::show<std::vector<std::shared_ptr<PickTask> > >(*best_assignment_tasks_ref);
#endif
                        if (best_assignment_tasks_ref->size() == 1)
                            cur_task = (*best_assignment_tasks_ref)[0];
                        else {
                            best_tsp_tasks_ref = dom->tsp_test(cur_robot, *best_assignment_tasks_ref);
#ifdef DEBUG
                            std::cout << "\t\tBest tsp tasks:" << std::endl;
                            RSE::show<std::vector<std::shared_ptr<PickTask> > >(*best_tsp_tasks_ref);
#endif
                            //if (best_tsp_tasks_ref->size() == 1)
                                cur_task = (*best_tsp_tasks_ref)[0];
                            /*else {
                                std::cerr << "[" << __FILE__ <<":" << __LINE__ << "] Got many tasks here. Exiting..." << std::endl;
                                exit(1);
                            }*
                            best_tsp_tasks_ref->clear();
                            best_tsp_tasks_ref.reset();
                        }
                        best_assignment_tasks_ref->clear();
                        best_assignment_tasks_ref.reset();
                    }
                    greater_utility_tasks_ref->clear();
                    greater_utility_tasks_ref.reset();
                }*/
                best_costed_tasks_ref->clear();
                best_costed_tasks_ref.reset();
            }
            if (cur_task) {
#ifdef DEBUG
                fprintf(stdout, "Robot %d: position = (%d, %d), capacity = %d\n", cur_robot->get_id(), cur_robot->get_x(), cur_robot->get_y(), cur_robot->get_current_dataset_capacity());
#endif
                if (cur_robot->get_current_dataset_capacity() - cur_task->get_demand() >= 0) {
#ifdef DEBUG
                    fprintf(stdout, "\tAssigning task %d: position = (%d, %d), demand = %d\n", cur_task->get_id(), cur_task->get_x(), cur_task->get_y(), cur_task->get_demand());
#endif
                    float cost_to_task = ce->get_cost_v2(cur_robot, cur_task, cur_robot);
                    //float cost_to_task = ce->get_manhattan_cost(cur_robot, cur_task);
                    c_total += cost_to_task;
                    cur_robot->add_scheduling_cost(cost_to_task);
                    cur_robot->add_task_ref(cur_task.get());
                    cur_robot->decrease_from_current_capacity(cur_task->get_demand());
                    cur_robot->set_x(cur_task->get_x());
                    cur_robot->set_y(cur_task->get_y());
                    cur_task->set_current_state(PickTask::assigned);
                    n_assigned += 1.0;
                    std::cerr << "[" << RSE::current_execution << " of " << RSE::number_executions << "]: " << 
                            (n_assigned / total_tasks)*100 << "% Completed...\r";
#ifdef DEBUG
                    fprintf(stdout, "\t\tRobot %d: updated_position = (%d, %d), updated_capacity = %d\n", cur_robot->get_id(), cur_robot->get_x(), cur_robot->get_y(), cur_robot->get_current_dataset_capacity());
                    fprintf(stdout, "\t\tCost: %.f\n\n", cur_robot->get_scheduling_cost());
#endif
                }
                else {
#ifdef DEBUG
                    fprintf(stdout, "\tAssigning task %d: position = (%d, %d), demand = %d\n", cur_task->get_id(), cur_task->get_x(), cur_task->get_y(), cur_task->get_demand());
                    fprintf(stdout, "\t\tRobot %d must go to deliver first\n", cur_robot->get_id());
#endif
                    cur_robot->reset_current_capacity();
#ifdef DEBUG
                    fprintf(stdout, "\t\tRobot %d: capacity after restart = %d\n", cur_robot->get_id(), cur_robot->get_current_dataset_capacity());
#endif
                    auto best_deliver_tuple = ce->closer_deliver_v2(cur_robot, cur_robot);
                    std::shared_ptr<DeliverTask> best_deliver = best_deliver_tuple.first;
                    float cost_to_best_deliver = best_deliver_tuple.second;
                    //float cost_to_best_deliver = ce->get_manhattan_cost(cur_robot, best_deliver);
                    c_total += cost_to_best_deliver;
                    cur_robot->add_scheduling_cost(cost_to_best_deliver);
                    cur_robot->add_task_ref(best_deliver.get());
                    n_visits_to_depot++;
#ifdef DEBUG
                    fprintf(stdout, "\t\tAssigning delivery %d. Cost = %.2f\n", best_deliver->get_id(), cost_to_best_deliver);
#endif
                    float cost_from_best_deliver_to_task = ce->get_cost_v2(best_deliver, cur_task, cur_robot);
                    c_total += cost_from_best_deliver_to_task;
                    cur_robot->add_scheduling_cost(cost_from_best_deliver_to_task);
                    cur_robot->add_task_ref(cur_task.get());
                    cur_robot->decrease_from_current_capacity(cur_task->get_demand());
                    cur_robot->set_x(cur_task->get_x());
                    cur_robot->set_y(cur_task->get_y());
                    cur_task->set_current_state(PickTask::assigned);
                    n_assigned += 1.0;
                    std::cerr << "[" << RSE::current_execution << " of " << RSE::number_executions << "]: " << 
                            (n_assigned / total_tasks)*100 << "% Completed...\r";
#ifdef DEBUG
                    fprintf(stdout, "\t\tAssigning task %d\n", cur_task->get_id());
                    fprintf(stdout, "\t\tRobot %d: updated_position = (%d, %d), updated_capacity = %d\n", cur_robot->get_id(), cur_robot->get_x(), cur_robot->get_y(), cur_robot->get_current_dataset_capacity());
                    fprintf(stdout, "\t\tCost: %.f\n\n", cur_robot->get_scheduling_cost());
#endif
                }
            }
#ifdef DEBUG
            else {
                fprintf(stdout, "\tNo tasks to assign!\n\n");
            }
#endif
        }
        int n_backup_of_robots = backup_of_robots_ref->size();
        for (int i = 0; i < n_backup_of_robots; i++)
            (*backup_of_robots_ref)[i].reset();
        backup_of_robots_ref.reset();
        dom->clear_domain_lists();
        //dom->reset_domain_lists();
    }
    for (int i = 0; i < _robots_ref->size(); i++) {
        if ((*_robots_ref)[i]->get_n_tasks()) {
            Task* last_task_ref = (*_robots_ref)[i]->get_task_ref((*_robots_ref)[i]->get_n_tasks() - 1);
            if (last_task_ref->get_task_type() != Task::deliver) {
                auto best_deliver_tuple = ce->closer_deliver_v2((*_robots_ref)[i], (*_robots_ref)[i]);
                float cost_to_best_deliver = best_deliver_tuple.second;
                //float cost_to_best_deliver = ce->get_manhattan_cost(_robots[i], best_deliver_tuple.first);
                c_total += cost_to_best_deliver;
                (*_robots_ref)[i]->add_scheduling_cost(cost_to_best_deliver);
                (*_robots_ref)[i]->add_task_ref(best_deliver_tuple.first.get());
                n_visits_to_depot++;
            }
        }
    }
    int n_sch_robots = 0;
    for (int i = 0; i < _robots_ref->size(); i++) {
        if ((*_robots_ref)[i]->get_n_tasks() > 0)
            n_sch_robots++;
    }
    return std::make_tuple(n_sch_robots, n_visits_to_depot, c_total, n_domain_comput, n_domain_with_unique_robot_per_task);
}

//domain zone based capacity-constrained task allocator (DoNe-CTA)
std::tuple<int, int, float, int, int> Scheduler::done_cta(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > _pick_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > > _deliver_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > > _recharge_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<Robot> > >& _robots_ref,
        CostEstimator::costing_t _costing_type) {  
    std::shared_ptr<CostEstimator> ce(new CostEstimator(_pick_tasks_ref, _deliver_tasks_ref, _recharge_tasks_ref, _robots_ref));
    ce->set_costing_type(_costing_type);
    std::shared_ptr<Domain> dom(new Domain(_pick_tasks_ref, _robots_ref, ce));
    float c_total = 0.0;
    int n_visits_to_depot = 0;
    int n_domain_comput = 0;
    int n_domain_with_unique_robot_per_task = 0;
    std::vector<std::shared_ptr<PickTask> > dominated_tasks;
    std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > best_costed_tasks_ref, greater_utility_tasks_ref, best_assignment_tasks_ref, best_tsp_tasks_ref;
    float n_assigned = 0.0, total_tasks = (float)(_pick_tasks_ref->size() - 1);
    while (n_assigned < total_tasks) {
    //while(!PickTask::all_assigned(*_pick_tasks_ref)) {
#ifdef DEBUG
        std::cout << "Computing domains" << std::endl;
#endif
        bool unique_robots = dom->compute_domain_v4();
        n_domain_comput++;
        if (unique_robots) n_domain_with_unique_robot_per_task++;
#ifdef DEBUG
        dom->show_domains();
#endif
#ifdef DEBUG
        std::cout << "Computing dominants" << std::endl;
#endif
        dom->compute_dominants();
#ifdef DEBUG
        dom->show_dominants();
        std::cout << "Copying robot attributes" << std::endl;
#endif
        std::shared_ptr<std::vector<std::shared_ptr<Robot> > >
                backup_of_robots_ref(new std::vector<std::shared_ptr<Robot> >[_robots_ref->size()]);
        for (int i = 0; i < _robots_ref->size(); i++) {
            std::shared_ptr<Robot> temp_cp_robot(new Robot((*_robots_ref)[i]));
            backup_of_robots_ref->push_back(temp_cp_robot);
        }
        for (int i = 0; i < _robots_ref->size(); i++) {
#ifdef DEBUG
            std::cout << "Getting next robot from queue" << std::endl;
#endif
            std::shared_ptr<Robot> cur_robot = (*_robots_ref)[i];
            std::shared_ptr<PickTask> cur_task;
#ifdef DEBUG
            std::cout << "\tGot robot r" << cur_robot->get_id() << " (x = " << cur_robot->get_x() <<
                ", y = " << cur_robot->get_y() << ", capacity = " << cur_robot->get_current_dataset_capacity() <<
                ")" << std::endl;
#endif
            dominated_tasks = dom->get_domain_of_robot(cur_robot);
#ifdef DEBUG
            std::cout << "\t\tDominated tasks:" << std::endl;
            RSE::show<std::vector<std::shared_ptr<PickTask> > >(dominated_tasks);
#endif
            uint32_t n_dominants = dominated_tasks.size();
            if (n_dominants == 0) continue;
            if (n_dominants == 1)
                cur_task = dominated_tasks[0];
            else {
                best_costed_tasks_ref = dom->cost_comparison(cur_robot, dominated_tasks);
#ifdef DEBUG
                std::cout << "\t\tBest costed tasks:" << std::endl;
                RSE::show<std::vector<std::shared_ptr<PickTask> > >(*best_costed_tasks_ref);
#endif
                //if (best_costed_tasks_ref->size() == 1) {
                cur_task = (*best_costed_tasks_ref)[0];
                /*float min_cost = FLT_MAX;
                for (int j = 0; j < best_costed_tasks_ref->size(); j++) {
                    float cur_cost = ce->get_cost_v2((*best_costed_tasks_ref)[j], cur_robot, cur_robot);
                    if (cur_cost < min_cost) {
                        min_cost = cur_cost;
                        cur_task = (*best_costed_tasks_ref)[j];
                    }
                }*/
                /*}
                else {
                    greater_utility_tasks_ref = dom->utility_test(cur_robot, *best_costed_tasks_ref);
#ifdef DEBUG
                    std::cout << "\t\tGreater utility tasks:" << std::endl;
                    RSE::show<std::vector<std::shared_ptr<PickTask> > >(*greater_utility_tasks_ref);
#endif
                    if (greater_utility_tasks_ref->size() == 1)
                        cur_task = (*greater_utility_tasks_ref)[0];
                    else {
                        best_assignment_tasks_ref = dom->assignment_variety(cur_robot, *greater_utility_tasks_ref);
#ifdef DEBUG
                        std::cout << "\t\tBest assignment tasks:" << std::endl;
                        RSE::show<std::vector<std::shared_ptr<PickTask> > >(*best_assignment_tasks_ref);
#endif
                        if (best_assignment_tasks_ref->size() == 1)
                            cur_task = (*best_assignment_tasks_ref)[0];
                        else {
                            best_tsp_tasks_ref = dom->tsp_test(cur_robot, *best_assignment_tasks_ref);
#ifdef DEBUG
                            std::cout << "\t\tBest tsp tasks:" << std::endl;
                            RSE::show<std::vector<std::shared_ptr<PickTask> > >(*best_tsp_tasks_ref);
#endif
                            //if (best_tsp_tasks_ref->size() == 1)
                                cur_task = (*best_tsp_tasks_ref)[0];
                            /*else {
                                std::cerr << "[" << __FILE__ <<":" << __LINE__ << "] Got many tasks here. Exiting..." << std::endl;
                                exit(1);
                            }*
                            best_tsp_tasks_ref->clear();
                            best_tsp_tasks_ref.reset();
                        }
                        best_assignment_tasks_ref->clear();
                        best_assignment_tasks_ref.reset();
                    }
                    greater_utility_tasks_ref->clear();
                    greater_utility_tasks_ref.reset();
                }*/
                best_costed_tasks_ref->clear();
                best_costed_tasks_ref.reset();
            }
            if (cur_task) {
#ifdef DEBUG
                fprintf(stdout, "Robot %d: position = (%d, %d), capacity = %d\n", cur_robot->get_id(), cur_robot->get_x(), cur_robot->get_y(), cur_robot->get_current_dataset_capacity());
#endif
                if (cur_robot->get_current_dataset_capacity() - cur_task->get_demand() >= 0) {
#ifdef DEBUG
                    fprintf(stdout, "\tAssigning task %d: position = (%d, %d), demand = %d\n", cur_task->get_id(), cur_task->get_x(), cur_task->get_y(), cur_task->get_demand());
#endif
                    float cost_to_task = ce->get_cost_v2(cur_robot, cur_task, cur_robot);
                    //float cost_to_task = ce->get_manhattan_cost(cur_robot, cur_task);
                    c_total += cost_to_task;
                    cur_robot->add_scheduling_cost(cost_to_task);
                    cur_robot->add_task_ref(cur_task.get());
                    cur_robot->decrease_from_current_capacity(cur_task->get_demand());
                    cur_robot->set_x(cur_task->get_x());
                    cur_robot->set_y(cur_task->get_y());
                    cur_task->set_current_state(PickTask::assigned);
                    n_assigned += 1.0;
                    std::cerr << "[" << RSE::current_execution << " of " << RSE::number_executions << "]: " << 
                            (n_assigned / total_tasks)*100 << "% Completed...\r";
#ifdef DEBUG
                    fprintf(stdout, "\t\tRobot %d: updated_position = (%d, %d), updated_capacity = %d\n", cur_robot->get_id(), cur_robot->get_x(), cur_robot->get_y(), cur_robot->get_current_dataset_capacity());
                    fprintf(stdout, "\t\tCost: %.f\n\n", cur_robot->get_scheduling_cost());
#endif
                }
                else {
#ifdef DEBUG
                    fprintf(stdout, "\tAssigning task %d: position = (%d, %d), demand = %d\n", cur_task->get_id(), cur_task->get_x(), cur_task->get_y(), cur_task->get_demand());
                    fprintf(stdout, "\t\tRobot %d must go to deliver first\n", cur_robot->get_id());
#endif
                    cur_robot->reset_current_capacity();
#ifdef DEBUG
                    fprintf(stdout, "\t\tRobot %d: capacity after restart = %d\n", cur_robot->get_id(), cur_robot->get_current_dataset_capacity());
#endif
                    auto best_deliver_tuple = ce->closer_deliver_v2(cur_robot, cur_robot);
                    std::shared_ptr<DeliverTask> best_deliver = best_deliver_tuple.first;
                    float cost_to_best_deliver = best_deliver_tuple.second;
                    //float cost_to_best_deliver = ce->get_manhattan_cost(cur_robot, best_deliver);
                    c_total += cost_to_best_deliver;
                    cur_robot->add_scheduling_cost(cost_to_best_deliver);
                    cur_robot->add_task_ref(best_deliver.get());
                    cur_robot->set_x(best_deliver->get_x());
                    cur_robot->set_y(best_deliver->get_y());
                    n_visits_to_depot++;
#ifdef DEBUG
                    fprintf(stdout, "\t\tAssigning delivery %d. Cost = %.2f\n", best_deliver->get_id(), cost_to_best_deliver);
#endif
                }
            }
#ifdef DEBUG
            else {
                fprintf(stdout, "\tNo tasks to assign!\n\n");
            }
#endif
        }
        int n_backup_of_robots = backup_of_robots_ref->size();
        for (int i = 0; i < n_backup_of_robots; i++)
            (*backup_of_robots_ref)[i].reset();
        backup_of_robots_ref.reset();
        dom->clear_domain_lists();
        //dom->reset_domain_lists();
    }
    for (int i = 0; i < _robots_ref->size(); i++) {
        if ((*_robots_ref)[i]->get_n_tasks()) {
            Task* last_task_ref = (*_robots_ref)[i]->get_task_ref((*_robots_ref)[i]->get_n_tasks() - 1);
            if (last_task_ref->get_task_type() != Task::deliver) {
                auto best_deliver_tuple = ce->closer_deliver_v2((*_robots_ref)[i], (*_robots_ref)[i]);
                float cost_to_best_deliver = best_deliver_tuple.second;
                //float cost_to_best_deliver = ce->get_manhattan_cost(_robots[i], best_deliver_tuple.first);
                c_total += cost_to_best_deliver;
                (*_robots_ref)[i]->add_scheduling_cost(cost_to_best_deliver);
                (*_robots_ref)[i]->add_task_ref(best_deliver_tuple.first.get());
                n_visits_to_depot++;
            }
        }
    }
    int n_sch_robots = 0;
    for (int i = 0; i < _robots_ref->size(); i++) {
        if ((*_robots_ref)[i]->get_n_tasks() > 0)
            n_sch_robots++;
    }
    return std::make_tuple(n_sch_robots, n_visits_to_depot, c_total, n_domain_comput, n_domain_with_unique_robot_per_task);
}

//domain zone based priority constrained task allocator (DoNe-PTA)
std::tuple<int, int, float, int, int> Scheduler::done_pta(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > _pick_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > > _deliver_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > > _recharge_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<Robot> > >& _robots_ref,
        CostEstimator::costing_t _costing_type) {  
    std::shared_ptr<CostEstimator> ce(new CostEstimator(_pick_tasks_ref, _deliver_tasks_ref, _recharge_tasks_ref, _robots_ref));
    ce->set_costing_type(_costing_type);
    std::shared_ptr<Domain> dom(new Domain(_pick_tasks_ref, _robots_ref, ce));
    //dom->init_domain_lists();
    std::shared_ptr<Scheduler> sch(new Scheduler(_robots_ref));
    float c_total = 0.0;
    int n_visits_to_depot = 0;
    int n_domain_comput = 0;
    int n_domain_with_unique_robot_per_task = 0;
    std::vector<std::shared_ptr<PickTask> > dominated_tasks;
    std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > best_costed_tasks_ref, greater_utility_tasks_ref, best_assignment_tasks_ref, best_tsp_tasks_ref;
    float n_assigned = 0.0, total_tasks = (float)(_pick_tasks_ref->size() - 1);
    while (n_assigned < total_tasks) {
    //while(!PickTask::all_assigned(_pick_tasks)) {
#ifdef DEBUG
        std::cout << "Computing domains" << std::endl;
#endif
        bool unique_robots = dom->compute_domain_v1();
        n_domain_comput++;
        if (unique_robots) n_domain_with_unique_robot_per_task++;
#ifdef DEBUG
        dom->show_domains();
#endif
#ifdef DEBUG
        std::cout << "Computing dominants" << std::endl;
#endif
        dom->compute_dominants();
#ifdef DEBUG
        dom->show_dominants();
#endif
        sch->prepare_v2(dom);
#ifdef DEBUG
        //std::cout << "Copying robot attributes" << std::endl;
#endif
        std::shared_ptr<std::vector<std::shared_ptr<Robot> > >
                backup_of_robots_ref(new std::vector<std::shared_ptr<Robot> >[_robots_ref->size()]);
        for (int i = 0; i < _robots_ref->size(); i++) {
            std::shared_ptr<Robot> temp_cp_robot(new Robot((*_robots_ref)[i]));
            backup_of_robots_ref->push_back(temp_cp_robot);
        }
        while(true) {
#ifdef DEBUG
            std::cout << "Testing if I must have compute domain" << std::endl;
#endif
            if (sch->must_compute_domain()) {
#ifdef DEBUG
                std::cout << "\tYeah! Computing domain..." << std::endl;
#endif
                break;
            }
#ifdef DEBUG
            std::cout << "Getting next robot from queue" << std::endl;
#endif
            std::shared_ptr<Robot> cur_robot = sch->next_robot_v2();
            std::shared_ptr<PickTask> cur_task;
#ifdef DEBUG
            std::cout << "\tGot robot r" << cur_robot->get_id() << " (x = " << cur_robot->get_x() <<
                ", y = " << cur_robot->get_y() << ", capacity = " << cur_robot->get_current_dataset_capacity() <<
                ")" << std::endl;
#endif
            sch->decrease_time_of_all(cur_robot);
#ifdef DEBUG
            std::cout << "Arrivals list (after decreasing time of robot r" << cur_robot->get_id() << "):" << std::endl;
            sch->show_arrivals();
            std::cout << std::endl;
#endif
            dominated_tasks = dom->get_domain_of_robot(cur_robot);
#ifdef DEBUG
            std::cout << "\t\tDominated tasks:" << std::endl;
            RSE::show<std::vector<std::shared_ptr<PickTask> > >(dominated_tasks);
#endif
            uint32_t n_dominants = dominated_tasks.size();
            if (n_dominants == 0) continue;
            if (n_dominants == 1)
                cur_task = dominated_tasks[0];
            else {
                best_costed_tasks_ref = dom->cost_comparison(cur_robot, dominated_tasks);
#ifdef DEBUG
                std::cout << "\t\tBest costed tasks:" << std::endl;
                RSE::show<std::vector<std::shared_ptr<PickTask> > >(*best_costed_tasks_ref);
#endif
                //if (best_costed_tasks_ref->size() == 1) {
                cur_task = (*best_costed_tasks_ref)[0];
                /*float min_cost = FLT_MAX;
                for (int j = 0; j < best_costed_tasks_ref->size(); j++) {
                    float cur_cost = ce->get_cost_v2((*best_costed_tasks_ref)[j], cur_robot, cur_robot);
                    if (cur_cost < min_cost) {
                        min_cost = cur_cost;
                        cur_task = (*best_costed_tasks_ref)[j];
                    }
                }*/
                /*}
                else {
                    greater_utility_tasks_ref = dom->tsp_test(cur_robot, *best_costed_tasks_ref);
#ifdef DEBUG
                    std::cout << "\t\tGreater utility tasks:" << std::endl;
                    RSE::show<std::vector<std::shared_ptr<PickTask> > >(*greater_utility_tasks_ref);
#endif
                    //if (greater_utility_tasks_ref->size() == 1)
                        cur_task = (*greater_utility_tasks_ref)[0];
                    /*else {
                        best_assignment_tasks_ref = dom->assignment_variety(cur_robot, *greater_utility_tasks_ref);
#ifdef DEBUG
                        std::cout << "\t\tBest assignment tasks:" << std::endl;
                        RSE::show<std::vector<std::shared_ptr<PickTask> > >(*best_assignment_tasks_ref);
#endif
                        if (best_assignment_tasks_ref->size() == 1)
                            cur_task = (*best_assignment_tasks_ref)[0];
                        else {
                            best_tsp_tasks_ref = dom->tsp_test(cur_robot, *best_assignment_tasks_ref);
#ifdef DEBUG
                            std::cout << "\t\tBest tsp tasks:" << std::endl;
                            RSE::show<std::vector<std::shared_ptr<PickTask> > >(*best_tsp_tasks_ref);
#endif
                            //if (best_tsp_tasks_ref->size() == 1)
                                cur_task = (*best_tsp_tasks_ref)[0];
                            /*else {
                                std::cerr << "[" << __FILE__ <<":" << __LINE__ << "] Got many tasks here. Exiting..." << std::endl;
                                exit(1);
                            }*
                            best_tsp_tasks_ref->clear();
                            best_tsp_tasks_ref.reset();
                        }
                        best_assignment_tasks_ref->clear();
                        best_assignment_tasks_ref.reset();
                    }*
                    greater_utility_tasks_ref->clear();
                    greater_utility_tasks_ref.reset();
                }*/
                best_costed_tasks_ref->clear();
                best_costed_tasks_ref.reset();
            }
            if (cur_task) {
                float arrival_time = sch->set_time_v2(cur_robot, cur_task, dom, ce, _costing_type);
#ifdef DEBUG
                std::cout << "Arrivals list (after setting new time " << arrival_time << " of robot r" << cur_robot->get_id() << "):" << std::endl;
                sch->show_arrivals();
                std::cout << std::endl;
#endif
#ifdef DEBUG
                fprintf(stdout, "Robot %d: position = (%d, %d), capacity = %d\n", cur_robot->get_id(), cur_robot->get_x(), cur_robot->get_y(), cur_robot->get_current_dataset_capacity());
#endif
                if (cur_robot->get_current_dataset_capacity() - cur_task->get_demand() >= 0) {
#ifdef DEBUG
                    fprintf(stdout, "\tAssigning task %d: position = (%d, %d), demand = %d\n", cur_task->get_id(), cur_task->get_x(), cur_task->get_y(), cur_task->get_demand());
#endif
                    float cost_to_task = ce->get_cost_v2(cur_robot, cur_task, cur_robot);
                    //float cost_to_task = ce->get_manhattan_cost(cur_robot, cur_task);
                    c_total += cost_to_task;
                    cur_robot->add_scheduling_cost(cost_to_task);
                    cur_robot->add_task_ref(cur_task.get());
                    cur_robot->decrease_from_current_capacity(cur_task->get_demand());
                    cur_robot->set_x(cur_task->get_x());
                    cur_robot->set_y(cur_task->get_y());
                    cur_task->set_current_state(PickTask::assigned);
                    n_assigned += 1.0;
                    std::cerr << "[" << RSE::current_execution << " of " << RSE::number_executions << "]: " << 
                            (n_assigned / total_tasks)*100 << "% Completed...\r";
#ifdef DEBUG
                    fprintf(stdout, "\t\tRobot %d: updated_position = (%d, %d), updated_capacity = %d\n", cur_robot->get_id(), cur_robot->get_x(), cur_robot->get_y(), cur_robot->get_current_dataset_capacity());
                    fprintf(stdout, "\t\tCost: %.f\n\n", cur_robot->get_scheduling_cost());
#endif
                }
                else {
#ifdef DEBUG
                    fprintf(stdout, "\tAssigning task %d: position = (%d, %d), demand = %d\n", cur_task->get_id(), cur_task->get_x(), cur_task->get_y(), cur_task->get_demand());
                    fprintf(stdout, "\t\tRobot %d must go to deliver first\n", cur_robot->get_id());
#endif
                    cur_robot->reset_current_capacity();
#ifdef DEBUG
                    fprintf(stdout, "\t\tRobot %d: capacity after restart = %d\n", cur_robot->get_id(), cur_robot->get_current_dataset_capacity());
#endif
                    auto best_deliver_tuple = ce->closer_deliver_v2(cur_robot, cur_robot);
                    std::shared_ptr<DeliverTask> best_deliver = best_deliver_tuple.first;
                    float cost_to_best_deliver = best_deliver_tuple.second;
                    //float cost_to_best_deliver = ce->get_manhattan_cost(cur_robot, best_deliver);
                    c_total += cost_to_best_deliver;
                    cur_robot->add_scheduling_cost(cost_to_best_deliver);
                    cur_robot->add_task_ref(best_deliver.get());
                    n_visits_to_depot++;
#ifdef DEBUG
                    fprintf(stdout, "\t\tAssigning delivery %d. Cost = %.2f\n", best_deliver->get_id(), cost_to_best_deliver);
#endif
                    float cost_from_best_deliver_to_task = ce->get_cost_v2(best_deliver, cur_task, cur_robot);
                    c_total += cost_from_best_deliver_to_task;
                    cur_robot->add_scheduling_cost(cost_from_best_deliver_to_task);
                    cur_robot->add_task_ref(cur_task.get());
                    cur_robot->decrease_from_current_capacity(cur_task->get_demand());
                    cur_robot->set_x(cur_task->get_x());
                    cur_robot->set_y(cur_task->get_y());
                    cur_task->set_current_state(PickTask::assigned);
                    n_assigned += 1.0;
                    std::cerr << "[" << RSE::current_execution << " of " << RSE::number_executions << "]: " << 
                            (n_assigned / total_tasks)*100 << "% Completed...\r";
#ifdef DEBUG
                    fprintf(stdout, "\t\tAssigning task %d\n", cur_task->get_id());
                    fprintf(stdout, "\t\tRobot %d: updated_position = (%d, %d), updated_capacity = %d\n", cur_robot->get_id(), cur_robot->get_x(), cur_robot->get_y(), cur_robot->get_current_dataset_capacity());
                    fprintf(stdout, "\t\tCost: %.f\n\n", cur_robot->get_scheduling_cost());
#endif
                }
            }
#ifdef DEBUG
            else {
                fprintf(stdout, "\tNo tasks to assign!\n\n");
            }
#endif
        }
        int n_backup_of_robots = backup_of_robots_ref->size();
        for (int i = 0; i < n_backup_of_robots; i++)
            (*backup_of_robots_ref)[i].reset();
        backup_of_robots_ref.reset();
        dom->clear_domain_lists();
        //dom->reset_domain_lists();
    }
    for (int i = 0; i < _robots_ref->size(); i++) {
        if ((*_robots_ref)[i]->get_n_tasks()) {
            Task* last_task_ref = (*_robots_ref)[i]->get_task_ref((*_robots_ref)[i]->get_n_tasks() - 1);
            if (last_task_ref->get_task_type() != Task::deliver) {
                auto best_deliver_tuple = ce->closer_deliver_v2((*_robots_ref)[i], (*_robots_ref)[i]);
                float cost_to_best_deliver = best_deliver_tuple.second;
                //float cost_to_best_deliver = ce->get_manhattan_cost(_robots[i], best_deliver_tuple.first);
                c_total += cost_to_best_deliver;
                (*_robots_ref)[i]->add_scheduling_cost(cost_to_best_deliver);
                (*_robots_ref)[i]->add_task_ref(best_deliver_tuple.first.get());
                n_visits_to_depot++;
            }
        }
    }
    int n_sch_robots = 0;
    for (int i = 0; i < _robots_ref->size(); i++) {
        if ((*_robots_ref)[i]->get_n_tasks() > 0)
            n_sch_robots++;
    }
    return std::make_tuple(n_sch_robots, n_visits_to_depot, c_total, n_domain_comput, n_domain_with_unique_robot_per_task);
}

//domain zone based capacity and priority constrained task allocator (DoNe-CPTA)
std::tuple<int, int, float, int, int> Scheduler::done_cpta(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > _pick_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > > _deliver_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > > _recharge_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<Robot> > >& _robots_ref,
        CostEstimator::costing_t _costing_type) {  
    std::shared_ptr<CostEstimator> ce(new CostEstimator(_pick_tasks_ref, _deliver_tasks_ref, _recharge_tasks_ref, _robots_ref));
    ce->set_costing_type(_costing_type);
    std::shared_ptr<Domain> dom(new Domain(_pick_tasks_ref, _robots_ref, ce));
    //dom->init_domain_lists();
    std::shared_ptr<Scheduler> sch(new Scheduler(_robots_ref));
    float c_total = 0.0;
    int n_visits_to_depot = 0;
    int n_domain_comput = 0;
    int n_domain_with_unique_robot_per_task = 0;
    std::vector<std::shared_ptr<PickTask> > dominated_tasks;
    std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > best_costed_tasks_ref, greater_utility_tasks_ref, best_assignment_tasks_ref, best_tsp_tasks_ref;
    float n_assigned = 0.0, total_tasks = (float)(_pick_tasks_ref->size() - 1);
    while (n_assigned < total_tasks) {
    //while(!PickTask::all_assigned(_pick_tasks)) {
#ifdef DEBUG
        std::cout << "Computing domains" << std::endl;
#endif
        bool unique_robots = dom->compute_domain_v4();
        n_domain_comput++;
        if (unique_robots) n_domain_with_unique_robot_per_task++;
#ifdef DEBUG
        dom->show_domains();
#endif
#ifdef DEBUG
        std::cout << "Computing dominants" << std::endl;
#endif
        dom->compute_dominants();
#ifdef DEBUG
        dom->show_dominants();
#endif
        sch->prepare_v2(dom);
#ifdef DEBUG
        //std::cout << "Copying robot attributes" << std::endl;
#endif
        std::shared_ptr<std::vector<std::shared_ptr<Robot> > >
                backup_of_robots_ref(new std::vector<std::shared_ptr<Robot> >[_robots_ref->size()]);
        for (int i = 0; i < _robots_ref->size(); i++) {
            std::shared_ptr<Robot> temp_cp_robot(new Robot((*_robots_ref)[i]));
            backup_of_robots_ref->push_back(temp_cp_robot);
        }
        while(true) {
#ifdef DEBUG
            std::cout << "Testing if I must have compute domain" << std::endl;
#endif
            if (sch->must_compute_domain()) {
#ifdef DEBUG
                std::cout << "\tYeah! Computing domain..." << std::endl;
#endif
                break;
            }
#ifdef DEBUG
            std::cout << "Getting next robot from queue" << std::endl;
#endif
            std::shared_ptr<Robot> cur_robot = sch->next_robot_v2();
            std::shared_ptr<PickTask> cur_task;
#ifdef DEBUG
            std::cout << "\tGot robot r" << cur_robot->get_id() << " (x = " << cur_robot->get_x() <<
                ", y = " << cur_robot->get_y() << ", capacity = " << cur_robot->get_current_dataset_capacity() <<
                ")" << std::endl;
#endif
            sch->decrease_time_of_all(cur_robot);
#ifdef DEBUG
            std::cout << "Arrivals list (after decreasing time of robot r" << cur_robot->get_id() << "):" << std::endl;
            sch->show_arrivals();
            std::cout << std::endl;
#endif
            dominated_tasks = dom->get_domain_of_robot(cur_robot);
#ifdef DEBUG
            std::cout << "\t\tDominated tasks:" << std::endl;
            RSE::show<std::vector<std::shared_ptr<PickTask> > >(dominated_tasks);
#endif
            uint32_t n_dominants = dominated_tasks.size();
            if (n_dominants == 0) continue;
            if (n_dominants == 1)
                cur_task = dominated_tasks[0];
            else {
                best_costed_tasks_ref = dom->cost_comparison(cur_robot, dominated_tasks);
#ifdef DEBUG
                std::cout << "\t\tBest costed tasks:" << std::endl;
                RSE::show<std::vector<std::shared_ptr<PickTask> > >(*best_costed_tasks_ref);
#endif
                //if (best_costed_tasks_ref->size() == 1) {
                cur_task = (*best_costed_tasks_ref)[0];
                /*float min_cost = FLT_MAX;
                for (int j = 0; j < best_costed_tasks_ref->size(); j++) {
                    float cur_cost = ce->get_cost_v2((*best_costed_tasks_ref)[j], cur_robot, cur_robot);
                    if (cur_cost < min_cost) {
                        min_cost = cur_cost;
                        cur_task = (*best_costed_tasks_ref)[j];
                    }
                }*/
                /*}
                else {
                    greater_utility_tasks_ref = dom->tsp_test(cur_robot, *best_costed_tasks_ref);
#ifdef DEBUG
                    std::cout << "\t\tGreater utility tasks:" << std::endl;
                    RSE::show<std::vector<std::shared_ptr<PickTask> > >(*greater_utility_tasks_ref);
#endif
                    //if (greater_utility_tasks_ref->size() == 1)
                        cur_task = (*greater_utility_tasks_ref)[0];
                    /*else {
                        best_assignment_tasks_ref = dom->assignment_variety(cur_robot, *greater_utility_tasks_ref);
#ifdef DEBUG
                        std::cout << "\t\tBest assignment tasks:" << std::endl;
                        RSE::show<std::vector<std::shared_ptr<PickTask> > >(*best_assignment_tasks_ref);
#endif
                        if (best_assignment_tasks_ref->size() == 1)
                            cur_task = (*best_assignment_tasks_ref)[0];
                        else {
                            best_tsp_tasks_ref = dom->tsp_test(cur_robot, *best_assignment_tasks_ref);
#ifdef DEBUG
                            std::cout << "\t\tBest tsp tasks:" << std::endl;
                            RSE::show<std::vector<std::shared_ptr<PickTask> > >(*best_tsp_tasks_ref);
#endif
                            //if (best_tsp_tasks_ref->size() == 1)
                                cur_task = (*best_tsp_tasks_ref)[0];
                            /*else {
                                std::cerr << "[" << __FILE__ <<":" << __LINE__ << "] Got many tasks here. Exiting..." << std::endl;
                                exit(1);
                            }*
                            best_tsp_tasks_ref->clear();
                            best_tsp_tasks_ref.reset();
                        }
                        best_assignment_tasks_ref->clear();
                        best_assignment_tasks_ref.reset();
                    }*
                    greater_utility_tasks_ref->clear();
                    greater_utility_tasks_ref.reset();
                }*/
                best_costed_tasks_ref->clear();
                best_costed_tasks_ref.reset();
            }
            if (cur_task) {
                float arrival_time = sch->set_time_v2(cur_robot, cur_task, dom, ce, _costing_type);
#ifdef DEBUG
                std::cout << "Arrivals list (after setting new time " << arrival_time << " of robot r" << cur_robot->get_id() << "):" << std::endl;
                sch->show_arrivals();
                std::cout << std::endl;
#endif
#ifdef DEBUG
                fprintf(stdout, "Robot %d: position = (%d, %d), capacity = %d\n", cur_robot->get_id(), cur_robot->get_x(), cur_robot->get_y(), cur_robot->get_current_dataset_capacity());
#endif
                if (cur_robot->get_current_dataset_capacity() - cur_task->get_demand() >= 0) {
#ifdef DEBUG
                    fprintf(stdout, "\tAssigning task %d: position = (%d, %d), demand = %d\n", cur_task->get_id(), cur_task->get_x(), cur_task->get_y(), cur_task->get_demand());
#endif
                    float cost_to_task = ce->get_cost_v2(cur_robot, cur_task, cur_robot);
                    //float cost_to_task = ce->get_manhattan_cost(cur_robot, cur_task);
                    c_total += cost_to_task;
                    cur_robot->add_scheduling_cost(cost_to_task);
                    cur_robot->add_task_ref(cur_task.get());
                    cur_robot->decrease_from_current_capacity(cur_task->get_demand());
                    cur_robot->set_x(cur_task->get_x());
                    cur_robot->set_y(cur_task->get_y());
                    cur_task->set_current_state(PickTask::assigned);
                    n_assigned += 1.0;
                    std::cerr << "[" << RSE::current_execution << " of " << RSE::number_executions << "]: " << 
                            (n_assigned / total_tasks)*100 << "% Completed...\r";
#ifdef DEBUG
                    fprintf(stdout, "\t\tRobot %d: updated_position = (%d, %d), updated_capacity = %d\n", cur_robot->get_id(), cur_robot->get_x(), cur_robot->get_y(), cur_robot->get_current_dataset_capacity());
                    fprintf(stdout, "\t\tCost: %.f\n\n", cur_robot->get_scheduling_cost());
#endif
                }
                else {
#ifdef DEBUG
                    fprintf(stdout, "\tAssigning task %d: position = (%d, %d), demand = %d\n", cur_task->get_id(), cur_task->get_x(), cur_task->get_y(), cur_task->get_demand());
                    fprintf(stdout, "\t\tRobot %d must go to deliver first\n", cur_robot->get_id());
#endif
                    cur_robot->reset_current_capacity();
#ifdef DEBUG
                    fprintf(stdout, "\t\tRobot %d: capacity after restart = %d\n", cur_robot->get_id(), cur_robot->get_current_dataset_capacity());
#endif
                    auto best_deliver_tuple = ce->closer_deliver_v2(cur_robot, cur_robot);
                    std::shared_ptr<DeliverTask> best_deliver = best_deliver_tuple.first;
                    float cost_to_best_deliver = best_deliver_tuple.second;
                    //float cost_to_best_deliver = ce->get_manhattan_cost(cur_robot, best_deliver);
                    c_total += cost_to_best_deliver;
                    cur_robot->add_scheduling_cost(cost_to_best_deliver);
                    cur_robot->add_task_ref(best_deliver.get());
                    cur_robot->set_x(best_deliver->get_x());
                    cur_robot->set_y(best_deliver->get_y());
                    n_visits_to_depot++;
#ifdef DEBUG
                    fprintf(stdout, "\t\tAssigning delivery %d. Cost = %.2f\n", best_deliver->get_id(), cost_to_best_deliver);
#endif
                }
            }
#ifdef DEBUG
            else {
                fprintf(stdout, "\tNo tasks to assign!\n\n");
            }
#endif
        }
        int n_backup_of_robots = backup_of_robots_ref->size();
        for (int i = 0; i < n_backup_of_robots; i++)
            (*backup_of_robots_ref)[i].reset();
        backup_of_robots_ref.reset();
        dom->clear_domain_lists();
        //dom->reset_domain_lists();
    }
    for (int i = 0; i < _robots_ref->size(); i++) {
        if ((*_robots_ref)[i]->get_n_tasks()) {
            Task* last_task_ref = (*_robots_ref)[i]->get_task_ref((*_robots_ref)[i]->get_n_tasks() - 1);
            if (last_task_ref->get_task_type() != Task::deliver) {
                auto best_deliver_tuple = ce->closer_deliver_v2((*_robots_ref)[i], (*_robots_ref)[i]);
                float cost_to_best_deliver = best_deliver_tuple.second;
                //float cost_to_best_deliver = ce->get_manhattan_cost(_robots[i], best_deliver_tuple.first);
                c_total += cost_to_best_deliver;
                (*_robots_ref)[i]->add_scheduling_cost(cost_to_best_deliver);
                (*_robots_ref)[i]->add_task_ref(best_deliver_tuple.first.get());
                n_visits_to_depot++;
            }
        }
    }
    int n_sch_robots = 0;
    for (int i = 0; i < _robots_ref->size(); i++) {
        if ((*_robots_ref)[i]->get_n_tasks() > 0)
            n_sch_robots++;
    }
    return std::make_tuple(n_sch_robots, n_visits_to_depot, c_total, n_domain_comput, n_domain_with_unique_robot_per_task);
}

//domain zone based capacity and priority constrained task allocator (DoNe-CPTA)
std::tuple<int, int, float, int, int> Scheduler::done_cpta_v2(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > _pick_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > > _deliver_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > > _recharge_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<Robot> > >& _robots_ref,
        CostEstimator::costing_t _costing_type) {  
    std::shared_ptr<CostEstimator> ce(new CostEstimator(_pick_tasks_ref, _deliver_tasks_ref, _recharge_tasks_ref, _robots_ref));
    ce->set_costing_type(_costing_type);
    std::shared_ptr<Domain> dom(new Domain(_pick_tasks_ref, _robots_ref, ce));
    //dom->init_domain_lists();
    std::shared_ptr<Scheduler> sch(new Scheduler(_robots_ref));
    float c_total = 0.0;
    int n_visits_to_depot = 0;
    int n_domain_comput = 0;
    int n_domain_with_unique_robot_per_task = 0;
    std::vector<std::shared_ptr<PickTask> > dominated_tasks;
    std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > best_costed_tasks_ref, greater_utility_tasks_ref, best_assignment_tasks_ref, best_tsp_tasks_ref;
    float n_assigned = 0.0, total_tasks = (float)(_pick_tasks_ref->size() - 1);
    while (n_assigned < total_tasks) {
    //while(!PickTask::all_assigned(_pick_tasks)) {
#ifdef DEBUG
        std::cout << "Computing domains" << std::endl;
#endif
        bool unique_robots = dom->compute_domain_v4();
        n_domain_comput++;
        if (unique_robots) n_domain_with_unique_robot_per_task++;
#ifdef DEBUG
        dom->show_domains();
#endif
#ifdef DEBUG
        std::cout << "Computing dominants" << std::endl;
#endif
        dom->compute_dominants();
#ifdef DEBUG
        dom->show_dominants();
#endif
        sch->prepare_v2(dom);
#ifdef DEBUG
        //std::cout << "Copying robot attributes" << std::endl;
#endif
        std::shared_ptr<std::vector<std::shared_ptr<Robot> > >
                backup_of_robots_ref(new std::vector<std::shared_ptr<Robot> >[_robots_ref->size()]);
        for (int i = 0; i < _robots_ref->size(); i++) {
            std::shared_ptr<Robot> temp_cp_robot(new Robot((*_robots_ref)[i]));
            backup_of_robots_ref->push_back(temp_cp_robot);
        }
        while(true) {
#ifdef DEBUG
            std::cout << "Testing if I must have compute domain" << std::endl;
#endif
            if (sch->must_compute_domain()) {
#ifdef DEBUG
                std::cout << "\tYeah! Computing domain..." << std::endl;
#endif
                break;
            }
#ifdef DEBUG
            std::cout << "Getting next robot from queue" << std::endl;
#endif
            std::shared_ptr<Robot> cur_robot = sch->next_robot_v2();
            std::shared_ptr<PickTask> cur_task;
#ifdef DEBUG
            std::cout << "\tGot robot r" << cur_robot->get_id() << " (x = " << cur_robot->get_x() <<
                ", y = " << cur_robot->get_y() << ", capacity = " << cur_robot->get_current_dataset_capacity() <<
                ")" << std::endl;
#endif
            sch->decrease_time_of_all(cur_robot);
#ifdef DEBUG
            std::cout << "Arrivals list (after decreasing time of robot r" << cur_robot->get_id() << "):" << std::endl;
            sch->show_arrivals();
            std::cout << std::endl;
#endif
            dominated_tasks = dom->get_domain_of_robot(cur_robot);
#ifdef DEBUG
            std::cout << "\t\tDominated tasks:" << std::endl;
            RSE::show<std::vector<std::shared_ptr<PickTask> > >(dominated_tasks);
#endif
            uint32_t n_dominants = dominated_tasks.size();
            if (n_dominants == 0) continue;
            if (n_dominants == 1)
                cur_task = dominated_tasks[0];
            else {
                best_costed_tasks_ref = dom->cost_comparison(cur_robot, dominated_tasks);
#ifdef DEBUG
                std::cout << "\t\tBest costed tasks:" << std::endl;
                RSE::show<std::vector<std::shared_ptr<PickTask> > >(*best_costed_tasks_ref);
#endif
                if (best_costed_tasks_ref->size() == 1) {
                    cur_task = (*best_costed_tasks_ref)[0];
                }
                else {
                    greater_utility_tasks_ref = dom->euler_test(cur_robot, *best_costed_tasks_ref);
#ifdef DEBUG
                    std::cout << "\t\tGreater utility tasks:" << std::endl;
                    RSE::show<std::vector<std::shared_ptr<PickTask> > >(*greater_utility_tasks_ref);
#endif
                    //if (greater_utility_tasks_ref->size() == 1)
                        cur_task = (*greater_utility_tasks_ref)[0];
                    /*else {
                        best_assignment_tasks_ref = dom->assignment_variety(cur_robot, *greater_utility_tasks_ref);
#ifdef DEBUG
                        std::cout << "\t\tBest assignment tasks:" << std::endl;
                        RSE::show<std::vector<std::shared_ptr<PickTask> > >(*best_assignment_tasks_ref);
#endif
                        if (best_assignment_tasks_ref->size() == 1)
                            cur_task = (*best_assignment_tasks_ref)[0];
                        else {
                            best_tsp_tasks_ref = dom->tsp_test(cur_robot, *best_assignment_tasks_ref);
#ifdef DEBUG
                            std::cout << "\t\tBest tsp tasks:" << std::endl;
                            RSE::show<std::vector<std::shared_ptr<PickTask> > >(*best_tsp_tasks_ref);
#endif
                            //if (best_tsp_tasks_ref->size() == 1)
                                cur_task = (*best_tsp_tasks_ref)[0];
                            /*else {
                                std::cerr << "[" << __FILE__ <<":" << __LINE__ << "] Got many tasks here. Exiting..." << std::endl;
                                exit(1);
                            }*
                            best_tsp_tasks_ref->clear();
                            best_tsp_tasks_ref.reset();
                        }
                        best_assignment_tasks_ref->clear();
                        best_assignment_tasks_ref.reset();
                    }*/
                    greater_utility_tasks_ref->clear();
                    greater_utility_tasks_ref.reset();
                }
                best_costed_tasks_ref->clear();
                best_costed_tasks_ref.reset();
            }
            if (cur_task) {
                float arrival_time = sch->set_time_v2(cur_robot, cur_task, dom, ce, _costing_type);
#ifdef DEBUG
                std::cout << "Arrivals list (after setting new time " << arrival_time << " of robot r" << cur_robot->get_id() << "):" << std::endl;
                sch->show_arrivals();
                std::cout << std::endl;
#endif
#ifdef DEBUG
                fprintf(stdout, "Robot %d: position = (%d, %d), capacity = %d\n", cur_robot->get_id(), cur_robot->get_x(), cur_robot->get_y(), cur_robot->get_current_dataset_capacity());
#endif
                if (cur_robot->get_current_dataset_capacity() - cur_task->get_demand() >= 0) {
#ifdef DEBUG
                    fprintf(stdout, "\tAssigning task %d: position = (%d, %d), demand = %d\n", cur_task->get_id(), cur_task->get_x(), cur_task->get_y(), cur_task->get_demand());
#endif
                    float cost_to_task = ce->get_cost_v2(cur_robot, cur_task, cur_robot);
                    //float cost_to_task = ce->get_manhattan_cost(cur_robot, cur_task);
                    c_total += cost_to_task;
                    cur_robot->add_scheduling_cost(cost_to_task);
                    cur_robot->add_task_ref(cur_task.get());
                    cur_robot->decrease_from_current_capacity(cur_task->get_demand());
                    cur_robot->set_x(cur_task->get_x());
                    cur_robot->set_y(cur_task->get_y());
                    cur_task->set_current_state(PickTask::assigned);
                    n_assigned += 1.0;
                    std::cerr << "[" << RSE::current_execution << " of " << RSE::number_executions << "]: " << 
                            (n_assigned / total_tasks)*100 << "% Completed...\r";
#ifdef DEBUG
                    fprintf(stdout, "\t\tRobot %d: updated_position = (%d, %d), updated_capacity = %d\n", cur_robot->get_id(), cur_robot->get_x(), cur_robot->get_y(), cur_robot->get_current_dataset_capacity());
                    fprintf(stdout, "\t\tCost: %.f\n\n", cur_robot->get_scheduling_cost());
#endif
                }
                else {
#ifdef DEBUG
                    fprintf(stdout, "\tAssigning task %d: position = (%d, %d), demand = %d\n", cur_task->get_id(), cur_task->get_x(), cur_task->get_y(), cur_task->get_demand());
                    fprintf(stdout, "\t\tRobot %d must go to deliver first\n", cur_robot->get_id());
#endif
                    cur_robot->reset_current_capacity();
#ifdef DEBUG
                    fprintf(stdout, "\t\tRobot %d: capacity after restart = %d\n", cur_robot->get_id(), cur_robot->get_current_dataset_capacity());
#endif
                    auto best_deliver_tuple = ce->closer_deliver_v2(cur_robot, cur_robot);
                    std::shared_ptr<DeliverTask> best_deliver = best_deliver_tuple.first;
                    float cost_to_best_deliver = best_deliver_tuple.second;
                    //float cost_to_best_deliver = ce->get_manhattan_cost(cur_robot, best_deliver);
                    c_total += cost_to_best_deliver;
                    cur_robot->add_scheduling_cost(cost_to_best_deliver);
                    cur_robot->add_task_ref(best_deliver.get());
                    cur_robot->set_x(best_deliver->get_x());
                    cur_robot->set_y(best_deliver->get_y());
                    n_visits_to_depot++;
#ifdef DEBUG
                    fprintf(stdout, "\t\tAssigning delivery %d. Cost = %.2f\n", best_deliver->get_id(), cost_to_best_deliver);
#endif
                }
            }
#ifdef DEBUG
            else {
                fprintf(stdout, "\tNo tasks to assign!\n\n");
            }
#endif
        }
        int n_backup_of_robots = backup_of_robots_ref->size();
        for (int i = 0; i < n_backup_of_robots; i++)
            (*backup_of_robots_ref)[i].reset();
        backup_of_robots_ref.reset();
        dom->clear_domain_lists();
        //dom->reset_domain_lists();
    }
    for (int i = 0; i < _robots_ref->size(); i++) {
        if ((*_robots_ref)[i]->get_n_tasks()) {
            Task* last_task_ref = (*_robots_ref)[i]->get_task_ref((*_robots_ref)[i]->get_n_tasks() - 1);
            if (last_task_ref->get_task_type() != Task::deliver) {
                auto best_deliver_tuple = ce->closer_deliver_v2((*_robots_ref)[i], (*_robots_ref)[i]);
                float cost_to_best_deliver = best_deliver_tuple.second;
                //float cost_to_best_deliver = ce->get_manhattan_cost(_robots[i], best_deliver_tuple.first);
                c_total += cost_to_best_deliver;
                (*_robots_ref)[i]->add_scheduling_cost(cost_to_best_deliver);
                (*_robots_ref)[i]->add_task_ref(best_deliver_tuple.first.get());
                n_visits_to_depot++;
            }
        }
    }
    int n_sch_robots = 0;
    for (int i = 0; i < _robots_ref->size(); i++) {
        if ((*_robots_ref)[i]->get_n_tasks() > 0)
            n_sch_robots++;
    }
    return std::make_tuple(n_sch_robots, n_visits_to_depot, c_total, n_domain_comput, n_domain_with_unique_robot_per_task);
}

//domain zone based capacity and priority constrained task allocator (DoNe-CPTA)
std::tuple<int, int, float, int, int> Scheduler::done_cpta_v3(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > _pick_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > > _deliver_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > > _recharge_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<Robot> > >& _robots_ref,
        CostEstimator::costing_t _costing_type) {  
    std::shared_ptr<CostEstimator> ce(new CostEstimator(_pick_tasks_ref, _deliver_tasks_ref, _recharge_tasks_ref, _robots_ref));
    ce->set_costing_type(_costing_type);
    std::shared_ptr<Domain> dom(new Domain(_pick_tasks_ref, _robots_ref, ce));
    //dom->init_domain_lists();
    std::shared_ptr<Scheduler> sch(new Scheduler(_robots_ref));
    float c_total = 0.0;
    int n_visits_to_depot = 0;
    int n_domain_comput = 0;
    int n_domain_with_unique_robot_per_task = 0;
    std::vector<std::shared_ptr<PickTask> > dominated_tasks;
    std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > best_costed_tasks_ref, greater_utility_tasks_ref, best_assignment_tasks_ref, best_tsp_tasks_ref, best_euler_tasks_ref;
    float n_assigned = 0.0, total_tasks = (float)(_pick_tasks_ref->size() - 1);
    while (n_assigned < total_tasks) {
    //while(!PickTask::all_assigned(_pick_tasks)) {
#ifdef DEBUG
        std::cout << "Computing domains" << std::endl;
#endif
        bool unique_robots = dom->compute_domain_v4();
        n_domain_comput++;
        if (unique_robots) n_domain_with_unique_robot_per_task++;
#ifdef DEBUG
        dom->show_domains();
#endif
#ifdef DEBUG
        std::cout << "Computing dominants" << std::endl;
#endif
        dom->compute_dominants();
#ifdef DEBUG
        dom->show_dominants();
#endif
        sch->prepare_v2(dom);
#ifdef DEBUG
        //std::cout << "Copying robot attributes" << std::endl;
#endif
        std::shared_ptr<std::vector<std::shared_ptr<Robot> > >
                backup_of_robots_ref(new std::vector<std::shared_ptr<Robot> >[_robots_ref->size()]);
        for (int i = 0; i < _robots_ref->size(); i++) {
            std::shared_ptr<Robot> temp_cp_robot(new Robot((*_robots_ref)[i]));
            backup_of_robots_ref->push_back(temp_cp_robot);
        }
        while(true) {
#ifdef DEBUG
            std::cout << "Testing if I must have compute domain" << std::endl;
#endif
            if (sch->must_compute_domain()) {
#ifdef DEBUG
                std::cout << "\tYeah! Computing domain..." << std::endl;
#endif
                break;
            }
#ifdef DEBUG
            std::cout << "Getting next robot from queue" << std::endl;
#endif
            std::shared_ptr<Robot> cur_robot = sch->next_robot_v2();
            std::shared_ptr<PickTask> cur_task;
#ifdef DEBUG
            std::cout << "\tGot robot r" << cur_robot->get_id() << " (x = " << cur_robot->get_x() <<
                ", y = " << cur_robot->get_y() << ", capacity = " << cur_robot->get_current_dataset_capacity() <<
                ")" << std::endl;
#endif
            sch->decrease_time_of_all(cur_robot);
#ifdef DEBUG
            std::cout << "Arrivals list (after decreasing time of robot r" << cur_robot->get_id() << "):" << std::endl;
            sch->show_arrivals();
            std::cout << std::endl;
#endif
            dominated_tasks = dom->get_domain_of_robot(cur_robot);
#ifdef DEBUG
            std::cout << "\t\tDominated tasks:" << std::endl;
            RSE::show<std::vector<std::shared_ptr<PickTask> > >(dominated_tasks);
#endif
            uint32_t n_dominants = dominated_tasks.size();
            if (n_dominants == 0) continue;
            if (n_dominants == 1)
                cur_task = dominated_tasks[0];
            else {
                best_costed_tasks_ref = dom->cost_comparison(cur_robot, dominated_tasks);
#ifdef DEBUG
                std::cout << "\t\tBest costed tasks:" << std::endl;
                RSE::show<std::vector<std::shared_ptr<PickTask> > >(*best_costed_tasks_ref);
#endif
                if (best_costed_tasks_ref->size() == 1) {
                    cur_task = (*best_costed_tasks_ref)[0];
                }
                else {
                    greater_utility_tasks_ref = dom->euler_test(cur_robot, *best_costed_tasks_ref);
#ifdef DEBUG
                    std::cout << "\t\tGreater utility tasks:" << std::endl;
                    RSE::show<std::vector<std::shared_ptr<PickTask> > >(*greater_utility_tasks_ref);
#endif
                    if (greater_utility_tasks_ref->size() == 1)
                        cur_task = (*greater_utility_tasks_ref)[0];
                    else {
                        best_euler_tasks_ref = dom->utility_test(cur_robot, *greater_utility_tasks_ref);
#ifdef DEBUG
                        std::cout << "\t\tBest euler tasks:" << std::endl;
                        RSE::show<std::vector<std::shared_ptr<PickTask> > >(*best_euler_tasks_ref);
#endif
                        cur_task = (*best_euler_tasks_ref)[0];
                        best_euler_tasks_ref->clear();
                        best_euler_tasks_ref.reset();
                    }
                    greater_utility_tasks_ref->clear();
                    greater_utility_tasks_ref.reset();
                }
                best_costed_tasks_ref->clear();
                best_costed_tasks_ref.reset();
            }
            if (cur_task) {
                float arrival_time = sch->set_time_v2(cur_robot, cur_task, dom, ce, _costing_type);
#ifdef DEBUG
                std::cout << "Arrivals list (after setting new time " << arrival_time << " of robot r" << cur_robot->get_id() << "):" << std::endl;
                sch->show_arrivals();
                std::cout << std::endl;
#endif
#ifdef DEBUG
                fprintf(stdout, "Robot %d: position = (%d, %d), capacity = %d\n", cur_robot->get_id(), cur_robot->get_x(), cur_robot->get_y(), cur_robot->get_current_dataset_capacity());
#endif
                if (cur_robot->get_current_dataset_capacity() - cur_task->get_demand() >= 0) {
#ifdef DEBUG
                    fprintf(stdout, "\tAssigning task %d: position = (%d, %d), demand = %d\n", cur_task->get_id(), cur_task->get_x(), cur_task->get_y(), cur_task->get_demand());
#endif
                    float cost_to_task = ce->get_cost_v2(cur_robot, cur_task, cur_robot);
                    //float cost_to_task = ce->get_manhattan_cost(cur_robot, cur_task);
                    c_total += cost_to_task;
                    cur_robot->add_scheduling_cost(cost_to_task);
                    cur_robot->add_task_ref(cur_task.get());
                    cur_robot->decrease_from_current_capacity(cur_task->get_demand());
                    cur_robot->set_x(cur_task->get_x());
                    cur_robot->set_y(cur_task->get_y());
                    cur_task->set_current_state(PickTask::assigned);
                    n_assigned += 1.0;
                    std::cerr << "[" << RSE::current_execution << " of " << RSE::number_executions << "]: " << 
                            (n_assigned / total_tasks)*100 << "% Completed...\r";
#ifdef DEBUG
                    fprintf(stdout, "\t\tRobot %d: updated_position = (%d, %d), updated_capacity = %d\n", cur_robot->get_id(), cur_robot->get_x(), cur_robot->get_y(), cur_robot->get_current_dataset_capacity());
                    fprintf(stdout, "\t\tCost: %.f\n\n", cur_robot->get_scheduling_cost());
#endif
                }
                else {
#ifdef DEBUG
                    fprintf(stdout, "\tAssigning task %d: position = (%d, %d), demand = %d\n", cur_task->get_id(), cur_task->get_x(), cur_task->get_y(), cur_task->get_demand());
                    fprintf(stdout, "\t\tRobot %d must go to deliver first\n", cur_robot->get_id());
#endif
                    cur_robot->reset_current_capacity();
#ifdef DEBUG
                    fprintf(stdout, "\t\tRobot %d: capacity after restart = %d\n", cur_robot->get_id(), cur_robot->get_current_dataset_capacity());
#endif
                    auto best_deliver_tuple = ce->closer_deliver_v2(cur_robot, cur_robot);
                    std::shared_ptr<DeliverTask> best_deliver = best_deliver_tuple.first;
                    float cost_to_best_deliver = best_deliver_tuple.second;
                    //float cost_to_best_deliver = ce->get_manhattan_cost(cur_robot, best_deliver);
                    c_total += cost_to_best_deliver;
                    cur_robot->add_scheduling_cost(cost_to_best_deliver);
                    cur_robot->add_task_ref(best_deliver.get());
                    cur_robot->set_x(best_deliver->get_x());
                    cur_robot->set_y(best_deliver->get_y());
                    n_visits_to_depot++;
#ifdef DEBUG
                    fprintf(stdout, "\t\tAssigning delivery %d. Cost = %.2f\n", best_deliver->get_id(), cost_to_best_deliver);
#endif
                }
            }
#ifdef DEBUG
            else {
                fprintf(stdout, "\tNo tasks to assign!\n\n");
            }
#endif
        }
        int n_backup_of_robots = backup_of_robots_ref->size();
        for (int i = 0; i < n_backup_of_robots; i++)
            (*backup_of_robots_ref)[i].reset();
        backup_of_robots_ref.reset();
        dom->clear_domain_lists();
        //dom->reset_domain_lists();
    }
    for (int i = 0; i < _robots_ref->size(); i++) {
        if ((*_robots_ref)[i]->get_n_tasks()) {
            Task* last_task_ref = (*_robots_ref)[i]->get_task_ref((*_robots_ref)[i]->get_n_tasks() - 1);
            if (last_task_ref->get_task_type() != Task::deliver) {
                auto best_deliver_tuple = ce->closer_deliver_v2((*_robots_ref)[i], (*_robots_ref)[i]);
                float cost_to_best_deliver = best_deliver_tuple.second;
                //float cost_to_best_deliver = ce->get_manhattan_cost(_robots[i], best_deliver_tuple.first);
                c_total += cost_to_best_deliver;
                (*_robots_ref)[i]->add_scheduling_cost(cost_to_best_deliver);
                (*_robots_ref)[i]->add_task_ref(best_deliver_tuple.first.get());
                n_visits_to_depot++;
            }
        }
    }
    int n_sch_robots = 0;
    for (int i = 0; i < _robots_ref->size(); i++) {
        if ((*_robots_ref)[i]->get_n_tasks() > 0)
            n_sch_robots++;
    }
    return std::make_tuple(n_sch_robots, n_visits_to_depot, c_total, n_domain_comput, n_domain_with_unique_robot_per_task);
}

//domain zone based capacity and priority constrained task allocator (DoNe-CPTA)
std::tuple<int, int, float, int, int> Scheduler::done_cpta_v4(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > _pick_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > > _deliver_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > > _recharge_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<Robot> > >& _robots_ref,
        CostEstimator::costing_t _costing_type) {  
    std::shared_ptr<CostEstimator> ce(new CostEstimator(_pick_tasks_ref, _deliver_tasks_ref, _recharge_tasks_ref, _robots_ref));
    ce->set_costing_type(_costing_type);
    std::shared_ptr<Domain> dom(new Domain(_pick_tasks_ref, _robots_ref, ce));
    //dom->init_domain_lists();
    std::shared_ptr<Scheduler> sch(new Scheduler(_robots_ref));
    float c_total = 0.0;
    int n_visits_to_depot = 0;
    int n_domain_comput = 0;
    int n_domain_with_unique_robot_per_task = 0;
    std::vector<std::shared_ptr<PickTask> > dominated_tasks;
    std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > best_costed_tasks_ref, greater_utility_tasks_ref, best_assignment_tasks_ref, best_tsp_tasks_ref;
    float n_assigned = 0.0, total_tasks = (float)(_pick_tasks_ref->size() - 1);
    while (n_assigned < total_tasks) {
    //while(!PickTask::all_assigned(_pick_tasks)) {
#ifdef DEBUG
        std::cout << "Computing domains" << std::endl;
#endif
        bool unique_robots = dom->compute_domain_v1();
        n_domain_comput++;
        if (unique_robots) n_domain_with_unique_robot_per_task++;
#ifdef DEBUG
        dom->show_domains();
#endif
#ifdef DEBUG
        std::cout << "Computing dominants" << std::endl;
#endif
        dom->compute_dominants();
#ifdef DEBUG
        dom->show_dominants();
#endif
        sch->prepare_v2(dom);
#ifdef DEBUG
        //std::cout << "Copying robot attributes" << std::endl;
#endif
        std::shared_ptr<std::vector<std::shared_ptr<Robot> > >
                backup_of_robots_ref(new std::vector<std::shared_ptr<Robot> >[_robots_ref->size()]);
        for (int i = 0; i < _robots_ref->size(); i++) {
            std::shared_ptr<Robot> temp_cp_robot(new Robot((*_robots_ref)[i]));
            backup_of_robots_ref->push_back(temp_cp_robot);
        }
        while(true) {
#ifdef DEBUG
            std::cout << "Testing if I must have compute domain" << std::endl;
#endif
            if (sch->must_compute_domain()) {
#ifdef DEBUG
                std::cout << "\tYeah! Computing domain..." << std::endl;
#endif
                break;
            }
#ifdef DEBUG
            std::cout << "Getting next robot from queue" << std::endl;
#endif
            std::shared_ptr<Robot> cur_robot = sch->next_robot_v2();
            std::shared_ptr<PickTask> cur_task;
#ifdef DEBUG
            std::cout << "\tGot robot r" << cur_robot->get_id() << " (x = " << cur_robot->get_x() <<
                ", y = " << cur_robot->get_y() << ", capacity = " << cur_robot->get_current_dataset_capacity() <<
                ")" << std::endl;
#endif
            sch->decrease_time_of_all(cur_robot);
#ifdef DEBUG
            std::cout << "Arrivals list (after decreasing time of robot r" << cur_robot->get_id() << "):" << std::endl;
            sch->show_arrivals();
            std::cout << std::endl;
#endif
            dominated_tasks = dom->get_domain_of_robot(cur_robot);
#ifdef DEBUG
            std::cout << "\t\tDominated tasks:" << std::endl;
            RSE::show<std::vector<std::shared_ptr<PickTask> > >(dominated_tasks);
#endif
            uint32_t n_dominants = dominated_tasks.size();
            if (n_dominants == 0) continue;
            if (n_dominants == 1)
                cur_task = dominated_tasks[0];
            else {
                best_costed_tasks_ref = dom->cost_comparison(cur_robot, dominated_tasks);
#ifdef DEBUG
                std::cout << "\t\tBest costed tasks:" << std::endl;
                RSE::show<std::vector<std::shared_ptr<PickTask> > >(*best_costed_tasks_ref);
#endif
                //if (best_costed_tasks_ref->size() == 1) {
                cur_task = (*best_costed_tasks_ref)[0];
                /*float min_cost = FLT_MAX;
                for (int j = 0; j < best_costed_tasks_ref->size(); j++) {
                    float cur_cost = ce->get_cost_v2((*best_costed_tasks_ref)[j], cur_robot, cur_robot);
                    if (cur_cost < min_cost) {
                        min_cost = cur_cost;
                        cur_task = (*best_costed_tasks_ref)[j];
                    }
                }*/
                /*}
                else {
                    greater_utility_tasks_ref = dom->tsp_test(cur_robot, *best_costed_tasks_ref);
#ifdef DEBUG
                    std::cout << "\t\tGreater utility tasks:" << std::endl;
                    RSE::show<std::vector<std::shared_ptr<PickTask> > >(*greater_utility_tasks_ref);
#endif
                    //if (greater_utility_tasks_ref->size() == 1)
                        cur_task = (*greater_utility_tasks_ref)[0];
                    /*else {
                        best_assignment_tasks_ref = dom->assignment_variety(cur_robot, *greater_utility_tasks_ref);
#ifdef DEBUG
                        std::cout << "\t\tBest assignment tasks:" << std::endl;
                        RSE::show<std::vector<std::shared_ptr<PickTask> > >(*best_assignment_tasks_ref);
#endif
                        if (best_assignment_tasks_ref->size() == 1)
                            cur_task = (*best_assignment_tasks_ref)[0];
                        else {
                            best_tsp_tasks_ref = dom->tsp_test(cur_robot, *best_assignment_tasks_ref);
#ifdef DEBUG
                            std::cout << "\t\tBest tsp tasks:" << std::endl;
                            RSE::show<std::vector<std::shared_ptr<PickTask> > >(*best_tsp_tasks_ref);
#endif
                            //if (best_tsp_tasks_ref->size() == 1)
                                cur_task = (*best_tsp_tasks_ref)[0];
                            /*else {
                                std::cerr << "[" << __FILE__ <<":" << __LINE__ << "] Got many tasks here. Exiting..." << std::endl;
                                exit(1);
                            }*
                            best_tsp_tasks_ref->clear();
                            best_tsp_tasks_ref.reset();
                        }
                        best_assignment_tasks_ref->clear();
                        best_assignment_tasks_ref.reset();
                    }*
                    greater_utility_tasks_ref->clear();
                    greater_utility_tasks_ref.reset();
                }*/
                best_costed_tasks_ref->clear();
                best_costed_tasks_ref.reset();
            }
            if (cur_task) {
                float arrival_time = sch->set_time_v2(cur_robot, cur_task, dom, ce, _costing_type);
#ifdef DEBUG
                std::cout << "Arrivals list (after setting new time " << arrival_time << " of robot r" << cur_robot->get_id() << "):" << std::endl;
                sch->show_arrivals();
                std::cout << std::endl;
#endif
#ifdef DEBUG
                fprintf(stdout, "Robot %d: position = (%d, %d), capacity = %d\n", cur_robot->get_id(), cur_robot->get_x(), cur_robot->get_y(), cur_robot->get_current_dataset_capacity());
#endif
                if (cur_robot->get_current_dataset_capacity() - cur_task->get_demand() >= 0) {
#ifdef DEBUG
                    fprintf(stdout, "\tAssigning task %d: position = (%d, %d), demand = %d\n", cur_task->get_id(), cur_task->get_x(), cur_task->get_y(), cur_task->get_demand());
#endif
                    float cost_to_task = ce->get_cost_v2(cur_robot, cur_task, cur_robot);
                    //float cost_to_task = ce->get_manhattan_cost(cur_robot, cur_task);
                    c_total += cost_to_task;
                    cur_robot->add_scheduling_cost(cost_to_task);
                    cur_robot->add_task_ref(cur_task.get());
                    cur_robot->decrease_from_current_capacity(cur_task->get_demand());
                    cur_robot->set_x(cur_task->get_x());
                    cur_robot->set_y(cur_task->get_y());
                    cur_task->set_current_state(PickTask::assigned);
                    n_assigned += 1.0;
                    /*std::cerr << "[" << RSE::current_execution << " of " << RSE::number_executions << "]: " << 
                            (n_assigned / total_tasks)*100 << "% Completed...\r";*/
#ifdef DEBUG
                    fprintf(stdout, "\t\tRobot %d: updated_position = (%d, %d), updated_capacity = %d\n", cur_robot->get_id(), cur_robot->get_x(), cur_robot->get_y(), cur_robot->get_current_dataset_capacity());
                    fprintf(stdout, "\t\tCost: %.f\n\n", cur_robot->get_scheduling_cost());
#endif
                }
                else {
#ifdef DEBUG
                    fprintf(stdout, "\tAssigning task %d: position = (%d, %d), demand = %d\n", cur_task->get_id(), cur_task->get_x(), cur_task->get_y(), cur_task->get_demand());
                    fprintf(stdout, "\t\tRobot %d must go to deliver first\n", cur_robot->get_id());
#endif
                    cur_robot->reset_current_capacity();
#ifdef DEBUG
                    fprintf(stdout, "\t\tRobot %d: capacity after restart = %d\n", cur_robot->get_id(), cur_robot->get_current_dataset_capacity());
#endif
                    auto best_deliver_tuple = ce->closer_deliver_v2(cur_robot, cur_robot);
                    std::shared_ptr<DeliverTask> best_deliver = best_deliver_tuple.first;
                    float cost_to_best_deliver = best_deliver_tuple.second;
                    //float cost_to_best_deliver = ce->get_manhattan_cost(cur_robot, best_deliver);
                    c_total += cost_to_best_deliver;
                    cur_robot->add_scheduling_cost(cost_to_best_deliver);
                    cur_robot->add_task_ref(best_deliver.get());
                    cur_robot->set_x(best_deliver->get_x());
                    cur_robot->set_y(best_deliver->get_y());
                    n_visits_to_depot++;
#ifdef DEBUG
                    fprintf(stdout, "\t\tAssigning delivery %d. Cost = %.2f\n", best_deliver->get_id(), cost_to_best_deliver);
#endif
                }
            }
#ifdef DEBUG
            else {
                fprintf(stdout, "\tNo tasks to assign!\n\n");
            }
#endif
        }
        int n_backup_of_robots = backup_of_robots_ref->size();
        for (int i = 0; i < n_backup_of_robots; i++)
            (*backup_of_robots_ref)[i].reset();
        backup_of_robots_ref.reset();
        dom->clear_domain_lists();
        //dom->reset_domain_lists();
    }
    for (int i = 0; i < _robots_ref->size(); i++) {
        if ((*_robots_ref)[i]->get_n_tasks()) {
            Task* last_task_ref = (*_robots_ref)[i]->get_task_ref((*_robots_ref)[i]->get_n_tasks() - 1);
            if (last_task_ref->get_task_type() != Task::deliver) {
                auto best_deliver_tuple = ce->closer_deliver_v2((*_robots_ref)[i], (*_robots_ref)[i]);
                float cost_to_best_deliver = best_deliver_tuple.second;
                //float cost_to_best_deliver = ce->get_manhattan_cost(_robots[i], best_deliver_tuple.first);
                c_total += cost_to_best_deliver;
                (*_robots_ref)[i]->add_scheduling_cost(cost_to_best_deliver);
                (*_robots_ref)[i]->add_task_ref(best_deliver_tuple.first.get());
                n_visits_to_depot++;
            }
        }
    }
    int n_sch_robots = 0;
    for (int i = 0; i < _robots_ref->size(); i++) {
        if ((*_robots_ref)[i]->get_n_tasks() > 0)
            n_sch_robots++;
    }
    return std::make_tuple(n_sch_robots, n_visits_to_depot, c_total, n_domain_comput, n_domain_with_unique_robot_per_task);
}

float Scheduler::set_time_v2(std::shared_ptr<Robot> _robot, std::shared_ptr<DeliverTask> _task, std::shared_ptr<Domain> _dom, std::shared_ptr<CostEstimator> _ce, CostEstimator::costing_t _costing_type) {
    float arrival_time;
    /*if (_costing_type == CostEstimator::euc_2d || _costing_type == CostEstimator::euc_energy ||
            _costing_type == CostEstimator::euc_time || _costing_type == CostEstimator::euc_time_energy)
        arrival_time = _ce->get_euclidean_time_cost_v2(_robot, _task, _robot);
    else 
        arrival_time = _ce->get_manhattan_time_cost_v2(_robot, _task, _robot);*/
    arrival_time = _ce->get_cost_v2(_robot, _task, _robot);
                
    //Primeiro, procura o endereço do robô em arrivals
    int addr;
    for (int i = 0; i < this->__arrivals.size(); i++) {
        std::shared_ptr<Robot> rbt = std::get<0>(this->__arrivals[i]);
        if (rbt->get_id() == _robot->get_id()) {
            addr = i;
            break;
        }
    }
    
    // Atualiza e diz que precisa calcular o domínio
    std::shared_ptr<Robot> rbt = std::get<0>(this->__arrivals[addr]);
    int n_dominants = std::get<2>(this->__arrivals[addr]);
    this->__arrivals[addr] = std::make_tuple(rbt, arrival_time, n_dominants, true);
    
#ifdef DEBUG
    std::cout << "Arrivals list (before sorting):" << std::endl;
    this->show_arrivals();
    std::cout << std::endl;
#endif
    //Ordena arrivals_deref
    std::sort(this->__arrivals.begin(), this->__arrivals.end(), Scheduler::__compare_arrivals);
    
#ifdef DEBUG
    std::cout << "Arrivals list (after sorting):" << std::endl;
    this->show_arrivals();
    std::cout << std::endl;
#endif
    return arrival_time;
}

float Scheduler::set_time_v2(std::shared_ptr<Robot> _robot, std::shared_ptr<PickTask> _task, std::shared_ptr<Domain> _dom, std::shared_ptr<CostEstimator> _ce, CostEstimator::costing_t _costing_type) {
    float arrival_time;
    /*if (_costing_type == CostEstimator::euc_2d || _costing_type == CostEstimator::euc_energy ||
            _costing_type == CostEstimator::euc_time || _costing_type == CostEstimator::euc_time_energy)
        arrival_time = _ce->get_euclidean_time_cost_v2(_robot, _task, _robot);
    else 
        arrival_time = _ce->get_manhattan_time_cost_v2(_robot, _task, _robot);*/
    arrival_time = _ce->get_cost_v2(_robot, _task, _robot);
                
    //Primeiro, procura o endereço do robô em arrivals
    int addr;
    for (int i = 0; i < this->__arrivals.size(); i++) {
        std::shared_ptr<Robot> rbt = std::get<0>(this->__arrivals[i]);
        if (rbt->get_id() == _robot->get_id()) {
            addr = i;
            break;
        }
    }
    
    // Atualiza e diz que precisa calcular o domínio
    std::shared_ptr<Robot> rbt = std::get<0>(this->__arrivals[addr]);
    int n_dominants = std::get<2>(this->__arrivals[addr]);
    this->__arrivals[addr] = std::make_tuple(rbt, arrival_time, n_dominants, true);
    
    // Se a tarefa _task estiver no domínio de outro robô s, então diz que, ao chegar em s, precisa calcular o domínio
    std::vector<std::shared_ptr<Robot> > dominants = _dom->get_dominants_of_task(_task);
    for (int i = 0; i < dominants.size(); i++) {
        std::shared_ptr<Robot> robot_in_dominants = dominants[i];
        if (robot_in_dominants->get_id() != _robot->get_id()) {
            //Procura o endereço de robot_in_dominants em arrivals
            for (int j = 0; j < this->__arrivals.size(); j++) {
                std::shared_ptr<Robot> rbt = std::get<0>(this->__arrivals[j]);
                if (rbt->get_id() == robot_in_dominants->get_id()) { //Achou
                    float tm = std::get<1>(this->__arrivals[j]);
                    int n_dominants = std::get<2>(this->__arrivals[j]);
                    this->__arrivals[j] = std::make_tuple(robot_in_dominants, tm, n_dominants, true);
                    break;
                }
            }
        }
    }
    
#ifdef DEBUG
    std::cout << "Arrivals list (before sorting):" << std::endl;
    this->show_arrivals();
    std::cout << std::endl;
#endif
    //Ordena arrivals_deref
    std::sort(this->__arrivals.begin(), this->__arrivals.end(), Scheduler::__compare_arrivals);
    
#ifdef DEBUG
    std::cout << "Arrivals list (after sorting):" << std::endl;
    this->show_arrivals();
    std::cout << std::endl;
#endif
    return arrival_time;
}

Scheduler::~Scheduler() {
    this->__arrivals.clear();
}

void Scheduler::show_arrivals() {
    for (int i = 0; i < this->__arrivals.size(); i++) {
        std::shared_ptr<Robot> rbt = std::get<0>(this->__arrivals[i]);
        float tm = std::get<1>(this->__arrivals[i]); 
        int n_dominants = std::get<2>(this->__arrivals[i]);
        bool lock = std::get<3>(this->__arrivals[i]);
        std::cout << "\t[Robot: r" << rbt->get_id() << ", Time: " << tm << ", Dominants: " << n_dominants << ", Must calc. domain: " << lock << "]" << std::endl;
    }
}

std::tuple<std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >, std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > >
Scheduler::__feasible_route_v2(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > _pick_tasks_ref, std::shared_ptr<Robot> _robot, std::shared_ptr<CostEstimator> _ce) {
    std::vector<std::shared_ptr<PickTask> > pick_tasks = *(_pick_tasks_ref);
    
    std::shared_ptr<Graph> g_main(new Graph(_pick_tasks_ref, _ce));
    auto snc_tpl_main = g_main->single_node_cycle_v2(_robot);
    //float p_min = std::get<1>(snc_tpl_main);
    float p_min = FLT_MAX;
    //float best_cost = FLT_MAX;
    std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > t_min, not_t_min;
    bool changed = false;
    for (int i = 0; i < pick_tasks.size(); i++) {
        for (int j = 0; j < pick_tasks.size(); j++) {
            if (pick_tasks[j]->get_current_state() == PickTask::testing)
                pick_tasks[j]->set_current_state(PickTask::waiting);

        }
        std::shared_ptr<PickTask> cur_task = pick_tasks[i];
        if (cur_task->get_current_state() != PickTask::waiting) continue;
        std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > nT(new std::vector<std::shared_ptr<PickTask> >());
        std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > not_nT = PickTask::copy_picktasks_v2(*_pick_tasks_ref);
        nT->push_back(cur_task);
        std::shared_ptr<Graph> g_nT(new Graph(nT, _ce));
        PickTask::delete_picktasks_v2(not_nT, cur_task);
        std::shared_ptr<Graph> g_not_nT(new Graph(not_nT, _ce));
        int32_t dT = cur_task->get_demand();
        cur_task->set_current_state(PickTask::testing);
        //std::cout << i <<"_robot->get_current_dataset_capacity() = " << _robot->get_current_dataset_capacity() << std::endl;
        while(dT < _robot->get_current_dataset_capacity()) {
            //std::cout << "\tdT = " << dT << std::endl;
            std::shared_ptr<PickTask> best_feasible_task = g_not_nT->restricted_nearest_neighbor_v2(cur_task, dT, _robot);
            if (!best_feasible_task)
                break;
            best_feasible_task->set_current_state(PickTask::testing);
            dT += best_feasible_task->get_demand();
            nT->push_back(best_feasible_task);
            PickTask::delete_picktasks_v2(not_nT, best_feasible_task);
        }
        g_not_nT.reset();
        g_nT.reset();
        g_nT = std::make_shared<Graph>(nT, _ce);
        g_not_nT = std::make_shared<Graph>(not_nT, _ce);
        //std::cout << "euler tour" << std::endl;
        auto ec_tpl = g_nT->euler_tour_v3(_robot);
        //std::cout << "single node cycle" << std::endl;
        auto snc_tpl = g_not_nT->single_node_cycle_v2(_robot);
        //std::cout << "ok" << std::endl;
        float p = std::get<1>(ec_tpl) + std::get<1>(snc_tpl);
        if (p < p_min) {
        //if (std::get<1>(snc_tpl) < p_min && std::get<1>(ec_tpl) < best_cost) {
            //best_cost = std::get<1>(ec_tpl);
            p_min = p;
            t_min = nT;
            not_t_min = not_nT;
            changed = true;
        }
        g_not_nT.reset();
        g_nT.reset();
    }
    if (!changed) {
        fprintf(stderr, "Cannot find a route...\n");
        exit(1);
    }
    g_main.reset();
#ifdef DEBUG
    std::cout << "Returned t_min: ";
    for (int i = 0; i < t_min->size(); i++) {
        std::shared_ptr<Task> cur_task = (*t_min)[i];
        std::cout << "t" << cur_task->get_id() << " ";
    }
    std::cout << std::endl;
    std::cout << "Returned not_t_min: ";
    for (int i = 0; i < not_t_min->size(); i++) {
        std::shared_ptr<Task> cur_task = (*not_t_min)[i];
        std::cout << "t" << cur_task->get_id() << " ";
    }
    std::cout << std::endl;
#endif
    return std::make_tuple(t_min, not_t_min);
}
