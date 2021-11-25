/* 
 * File:   Match.cpp
 * Author: geoso
 * 
 * Created on 9 de Março de 2021, 15:44
 */

#include "Match.hpp"

bool Match::__compare_tasks_by_cost(std::pair<PickTask*, float> _a1, std::pair<PickTask*, float> _a2) {
    return (_a1.second < _a2.second);
}

Match::Match(std::vector<PickTask*> _pick_tasks, std::vector<Robot*> _robots, Domain* _dom, CostEstimator *_ce) {
#ifdef DEBUG
    fprintf(stdout, "\nGETTING IN: Match::Match(Robot* _main_r, std::vector<PickTask*> _pick_tasks, std::vector<Robot*> _robots, Domain* _dom) [%s:%d]\n", __FILE__, __LINE__);
#endif
    this->__ce_ref = _ce;
    
    this->__dom_ref = _dom;
    
    for (auto it = std::begin(_pick_tasks); it != std::end(_pick_tasks); ++it) {
        this->__all_pick_tasks_ref.push_back(*it);
    }
    for (auto it = std::begin(_robots); it != std::end(_robots); ++it) {
        this->__all_robots_ref.push_back(*it);
    }
    
    this->__main_robot = NULL;
    this->__scoreboard = NULL;
#ifdef DEBUG
    fprintf(stdout, "LEAVING OF: Match::Match(Robot* _main_r, std::vector<PickTask*> _pick_tasks, std::vector<Robot*> _robots, Domain* _dom) [%s:%d]\n", __FILE__, __LINE__);
#endif
}

void Match::game1_assignment_variety() {
#ifdef DEBUG
    std::cout << "Step 1 : Robot " << this->__main_robot->get_id() <<
            " in position (" << this->__main_robot->get_x() << ", " << this->__main_robot->get_y() <<
            ") with capacity " << this->__main_robot->get_current_dataset_capacity() << std::endl;
#endif
    std::vector<PickTask*> main_tasks = (this->__main_tasks);
    std::vector<Robot*> all_robots = (this->__all_robots_ref);
    int n_all_robots = all_robots.size();
    int n_main_tasks = main_tasks.size();
    int best_assignment_variety = 0;
    std::vector<PickTask*> best_tasks;
    for (int i = 0; i < n_main_tasks; i++) {
#ifdef DEBUG
        std::cout << "\tIf r" << this->__main_robot->get_id() << " pick up t" << main_tasks[i]->get_id() << ":" << std::endl;
#endif
        int cur_assignment_variety = 0;
        for (int j = 0; j < n_main_tasks; j++) {
            if (i != j) {
#ifdef DEBUG
                std::cout << "\t\tt" << main_tasks[j]->get_id() << ": ";
#endif
                for (int k = 0; k < n_all_robots; k++) {
                    int32_t cur_capacity = (this->__main_robot->get_id() == all_robots[k]->get_id()) ?
                        this->__main_robot->get_current_dataset_capacity() - main_tasks[i]->get_demand() :
                        all_robots[k]->get_current_dataset_capacity();
                    if (cur_capacity >= main_tasks[j]->get_demand()) {
#ifdef DEBUG
                        std::cout << "r" << all_robots[k]->get_id() << " ";
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
            best_tasks.clear();
            best_tasks.push_back(main_tasks[i]);
        }
        else if (cur_assignment_variety == best_assignment_variety)
            best_tasks.push_back(main_tasks[i]);
    }
    
#ifdef DEBUG
    std::cout << "\tWinner Tasks: ";
    for (auto it = std::begin(best_tasks); it != std::end(best_tasks); ++it)
        std::cout << "t" << (*it)->get_id() << " ";
    std::cout << std::endl;
#endif
    
    std::map<uint16_t, int8_t>& scoreboard_deref = *(this->__scoreboard);
    for (auto it = std::begin(best_tasks); it != std::end(best_tasks); ++it)
        scoreboard_deref[(*it)->get_id()] += 1;
    
#ifdef DEBUG
    std::cout << "Scoreboard:" << std::endl;
    this->show_scoreboard();
    std::cout << std::endl;
#endif
}

void Match::game2_utility_test() {
#ifdef DEBUG
    std::cout << "Step 2 : Robot " << this->__main_robot->get_id() <<
            " in position (" << this->__main_robot->get_x() << ", " << this->__main_robot->get_y() <<
            ") with capacity " << this->__main_robot->get_current_dataset_capacity() << std::endl;
#endif
    std::vector<PickTask*> main_tasks = (this->__main_tasks);
    std::vector<PickTask*> all_tasks = (this->__all_pick_tasks_ref);
    int n_all_tasks = all_tasks.size();
    int n_main_tasks = main_tasks.size();
    
    int max_utility = 0;
    std::vector<PickTask*> tasks_max_utility;
    for (int i = 0; i < n_main_tasks; i++) {
        PickTask* cur_task_of_main = main_tasks[i];
        int32_t capacity_after_pickup_i = this->__main_robot->get_current_dataset_capacity() - cur_task_of_main->get_demand();
        int cur_utility = 0;
        for (int j = 0; j < n_all_tasks; j++) {
            PickTask* cur_task_of_all = all_tasks[j];
            if (cur_task_of_main->get_id() != cur_task_of_all->get_id() &&
                    cur_task_of_all->get_current_state() == PickTask::waiting &&
                    capacity_after_pickup_i >= cur_task_of_all->get_demand()) {
                cur_utility++;
            }
        }
#ifdef DEBUG
        std::cout << "\tIf r" << this->__main_robot->get_id() << " pick up t" << cur_task_of_main->get_id() <<
                ": capacity = " << capacity_after_pickup_i << " (utility = " << cur_utility << ")" << std::endl;
#endif
        if (cur_utility > max_utility) {
            max_utility = cur_utility;
            tasks_max_utility.clear();
            tasks_max_utility.push_back(cur_task_of_main);
        }
        else if (cur_utility == max_utility)
            tasks_max_utility.push_back(cur_task_of_main);
    }
    
    std::map<uint16_t, int8_t>& scoreboard_deref = *(this->__scoreboard);
    // Verificando se deu empate
    if (tasks_max_utility.size() == 0) { // Primeiro, se não deu erro
        std::cerr << "[" << __FILE__ <<":" << __LINE__ << "] Unknown Error! Exiting..." << std::endl;
        exit(0x53A);
    }
    if (tasks_max_utility.size() > 1) { //Deu empate
        // Monta a lista de tarefas de maior demanda
#ifdef DEBUG
        std::cout << "\tIn case of a tie, r" << this->__main_robot->get_id() << " must take the task(s) with the highest demand: " << std::endl;
#endif
        int32_t larger_demand = 0;
        std::vector<PickTask*> tasks_best_demand;
        for (int i = 0; i < tasks_max_utility.size(); i++) {
            PickTask* cur_task = tasks_max_utility[i];
            if (cur_task->get_demand() > larger_demand) {
                larger_demand = cur_task->get_demand();
                tasks_best_demand.clear();
                tasks_best_demand.push_back(cur_task);
            }
            else if (cur_task->get_demand() == larger_demand)
                tasks_best_demand.push_back(cur_task);
        }
        
        // Pontuando os vencedores (mesmo empatados)
#ifdef DEBUG
        std::cout << "\t\tTask(s) with the highest demand: ";
#endif
        for (auto it = std::begin(tasks_best_demand); it != std::end(tasks_best_demand); ++it) {
#ifdef DEBUG
            std::cout << "t" << (*it)->get_id() << " ";
#endif
            scoreboard_deref[(*it)->get_id()] += 1;
        }
#ifdef DEBUG
        std::cout << std::endl;
#endif
        
        // Verificando se deu empate (duas ou mais tarefas com a mesma demanda)
        if (tasks_best_demand.size() == 0) { // Primeiro, se não deu erro
            std::cerr << "[" << __FILE__ <<":" << __LINE__ << "] Unknown Error! Exiting..." << std::endl;
            exit(0x53A);
        }
        if (tasks_best_demand.size() > 1) { //Deu empate (há no mínimo duas tarefas com a mesma demanda)
            // Monta a lista de tarefas de menor custo
#ifdef DEBUG
            std::cout << "\tIf tied again, r" << this->__main_robot->get_id() << " must take the lowest cost task: " << std::endl;
#endif
            float smaller_cost = FLT_MAX;
            std::vector<PickTask*> tasks_best_cost;
            for (int i = 0; i < tasks_best_demand.size(); i++) {
                PickTask* cur_task = tasks_best_demand[i];
                float cur_cost = this->__ce_ref->get_cost(cur_task, this->__main_robot, this->__main_robot);
                if (cur_cost < smaller_cost) {
                    smaller_cost = cur_cost;
                    tasks_best_cost.clear();
                    tasks_best_cost.push_back(cur_task);
                }
                else if (cur_cost == smaller_cost)
                    tasks_best_cost.push_back(cur_task);
            }
            
            // Pontuando os vencedores (mesmo empatados)
#ifdef DEBUG
            std::cout << "\t\tTask(s) with the lowest cost: ";
#endif
            for (auto it = std::begin(tasks_best_cost); it != std::end(tasks_best_cost); ++it) {
#ifdef DEBUG
                std::cout << "t" << (*it)->get_id() << " ";
#endif
                scoreboard_deref[(*it)->get_id()] += 1;
            }
#ifdef DEBUG
            std::cout << std::endl;
#endif
        }
    }
    else { // Não deu empate
#ifdef DEBUG
    std::cout << "\tWinner Task: t" << tasks_max_utility[0]->get_id() << std::endl;
#endif
        uint16_t id_winner_task = tasks_max_utility[0]->get_id();
        scoreboard_deref[id_winner_task] += 1;
    }
#ifdef DEBUG
    std::cout << "Scoreboard:" << std::endl;
    this->show_scoreboard();
    std::cout << std::endl;
#endif
}

void Match::game2_utility_test_v2() {
#ifdef DEBUG
    std::cout << "Step 2 : Robot " << this->__main_robot->get_id() <<
            " in position (" << this->__main_robot->get_x() << ", " << this->__main_robot->get_y() <<
            ") with capacity " << this->__main_robot->get_current_dataset_capacity() << std::endl;
#endif
    std::vector<PickTask*> main_tasks = (this->__main_tasks);
    std::vector<PickTask*> all_tasks = (this->__all_pick_tasks_ref);
    int n_all_tasks = all_tasks.size();
    int n_main_tasks = main_tasks.size();
    
    int max_utility = 0;
    std::vector<PickTask*> tasks_max_utility;
    for (int i = 0; i < n_main_tasks; i++) {
        PickTask* cur_task_of_main = main_tasks[i];
        int32_t capacity_after_pickup_i = this->__main_robot->get_current_dataset_capacity() - cur_task_of_main->get_demand();
        int cur_utility = 0;
        for (int j = 0; j < n_all_tasks; j++) {
            PickTask* cur_task_of_all = all_tasks[j];
            if (cur_task_of_main->get_id() != cur_task_of_all->get_id() &&
                    cur_task_of_all->get_current_state() == PickTask::waiting &&
                    capacity_after_pickup_i >= cur_task_of_all->get_demand()) {
                cur_utility++;
            }
        }
#ifdef DEBUG
        std::cout << "\tIf r" << this->__main_robot->get_id() << " pick up t" << cur_task_of_main->get_id() <<
                ": capacity = " << capacity_after_pickup_i << " (utility = " << cur_utility << ")" << std::endl;
#endif
        if (cur_utility > max_utility) {
            max_utility = cur_utility;
            tasks_max_utility.clear();
            tasks_max_utility.push_back(cur_task_of_main);
        }
        else if (cur_utility == max_utility)
            tasks_max_utility.push_back(cur_task_of_main);
    }
#ifdef DEBUG
    std::cout << "\tWinner Tasks: ";
    for (auto it = std::begin(tasks_max_utility); it != std::end(tasks_max_utility); ++it)
        std::cout << "t" << (*it)->get_id() << " ";
    std::cout << std::endl;
#endif
    
    std::map<uint16_t, int8_t>& scoreboard_deref = *(this->__scoreboard);
    for (auto it = std::begin(tasks_max_utility); it != std::end(tasks_max_utility); ++it)
        scoreboard_deref[(*it)->get_id()] += 1;
    
#ifdef DEBUG
    std::cout << "Scoreboard:" << std::endl;
    this->show_scoreboard();
    std::cout << std::endl;
#endif
}

void Match::game2_utility_test_v3() {
//#ifdef DEBUG
    std::cout << "Step 2 : Robot " << this->__main_robot->get_id() <<
            " in position (" << this->__main_robot->get_x() << ", " << this->__main_robot->get_y() <<
            ") with capacity " << this->__main_robot->get_current_dataset_capacity() << std::endl;
    this->__dom_ref->show_primary_domain_v2();
    this->__dom_ref->show_primary_transposed_list();
//#endif
        
    std::vector<PickTask*> main_tasks = (this->__main_tasks);
    std::vector<PickTask*> all_tasks = (this->__all_pick_tasks_ref);
    int n_all_tasks = all_tasks.size();
    int n_main_tasks = main_tasks.size();
    
    int max_utility = 0;
    std::vector<PickTask*> tasks_max_utility;
    for (int i = 0; i < n_main_tasks; i++) {
        PickTask* cur_task_of_main = main_tasks[i];
        int32_t capacity_after_pickup_i = this->__main_robot->get_current_dataset_capacity() - cur_task_of_main->get_demand();
        int cur_utility = 0;
        for (int j = 0; j < n_all_tasks; j++) {
            PickTask* cur_task_of_all = all_tasks[j];
            if (cur_task_of_main->get_id() != cur_task_of_all->get_id() &&
                    cur_task_of_all->get_current_state() == PickTask::waiting &&
                    capacity_after_pickup_i >= cur_task_of_all->get_demand()) {
                cur_utility++;
            }
        }
//#ifdef DEBUG
        std::cout << "\tIf r" << this->__main_robot->get_id() << " pick up t" << cur_task_of_main->get_id() <<
                ": capacity = " << capacity_after_pickup_i << " (utility = " << cur_utility << ")" << std::endl;
//#endif
        if (cur_utility > max_utility) {
            max_utility = cur_utility;
            tasks_max_utility.clear();
            tasks_max_utility.push_back(cur_task_of_main);
        }
        else if (cur_utility == max_utility)
            tasks_max_utility.push_back(cur_task_of_main);
    }
    
    std::map<uint16_t, int8_t>& scoreboard_deref = *(this->__scoreboard);
    // Verificando se deu empate
    if (tasks_max_utility.size() == 0) { // Primeiro, se não deu erro
        std::cerr << "[" << __FILE__ <<":" << __LINE__ << "] Unknown Error! Exiting..." << std::endl;
        exit(0x53A);
    }
    if (tasks_max_utility.size() > 1) { //Deu empate
        // Monta a lista de tarefas de maior demanda
//#ifdef DEBUG
        std::cout << "\tIn case of a tie, r" << this->__main_robot->get_id() << " must take the task(s) with the highest demand: " << std::endl;
//#endif
        int32_t larger_demand = 0;
        std::vector<PickTask*> tasks_best_demand;
        for (int i = 0; i < tasks_max_utility.size(); i++) {
            PickTask* cur_task = tasks_max_utility[i];
            if (cur_task->get_demand() > larger_demand) {
                larger_demand = cur_task->get_demand();
                tasks_best_demand.clear();
                tasks_best_demand.push_back(cur_task);
            }
            else if (cur_task->get_demand() == larger_demand)
                tasks_best_demand.push_back(cur_task);
        }
        
        // Pontuando os vencedores (mesmo empatados)
//#ifdef DEBUG
        std::cout << "\t\tTask(s) with the highest demand: ";
//#endif
        for (auto it = std::begin(tasks_best_demand); it != std::end(tasks_best_demand); ++it) {
//#ifdef DEBUG
            std::cout << "t" << (*it)->get_id() << " ";
//#endif
            scoreboard_deref[(*it)->get_id()] += 1;
        }
//#ifdef DEBUG
        std::cout << std::endl;
//#endif
    }
    else { // Não deu empate
//#ifdef DEBUG
    std::cout << "\tWinner Task: t" << tasks_max_utility[0]->get_id() << std::endl;
//#endif
        uint16_t id_winner_task = tasks_max_utility[0]->get_id();
        scoreboard_deref[id_winner_task] += 1;
    }
//#ifdef DEBUG
    std::cout << "Scoreboard:" << std::endl;
    this->show_scoreboard();
    std::cout << std::endl;
//#endif
}

void Match::game3_cost_comparison() {
//#ifdef DEBUG
    std::cout << "Step 3 : Robot " << this->__main_robot->get_id() <<
            " in position (" << this->__main_robot->get_x() << ", " << this->__main_robot->get_y() <<
            ") with capacity " << this->__main_robot->get_current_dataset_capacity() << std::endl;
//#endif
    std::vector<PickTask*> main_tasks = (this->__main_tasks);
    std::vector<PickTask*> all_tasks = (this->__all_pick_tasks_ref);
    int n_all_tasks = all_tasks.size();
    int n_main_tasks = main_tasks.size();
    float best_cost = FLT_MAX;
    std::vector<PickTask*> best_tasks;
    for (int i = 0; i < n_main_tasks; i++) {
        PickTask* cur_task_of_main = main_tasks[i];
#ifdef DEBUG
        std::cout << "\tIf r" << this->__main_robot->get_id() << " pick up t" << cur_task_of_main->get_id() << ": ";
#endif
        float cur_cost = this->__ce_ref->get_cost(cur_task_of_main, this->__main_robot, this->__main_robot);
#ifdef DEBUG
        std::cout << cur_cost;
#endif
        // Se ao pegar a próxima tarefa, o robô ficar impedido de pegar outra, então o custo é somado com o custo de voltar à estação de entrega
        // Verificando se o robô ficará impedido de pegar outra tarefa:
        bool can_exec_another_task = false;
        int32_t capacity_after_pickup_i = this->__main_robot->get_current_dataset_capacity() - cur_task_of_main->get_demand();
        for (int j = 0; j < n_all_tasks; j++) {
            PickTask* cur_task_of_all = all_tasks[j];
            if (cur_task_of_main->get_id() != cur_task_of_all->get_id() &&
                    cur_task_of_all->get_current_state() == PickTask::waiting &&
                    capacity_after_pickup_i >= cur_task_of_all->get_demand()) {
                can_exec_another_task = true;
                break;
            }
        }
        // Verificou. Agora, se 'can_exec_another_task == false', incrementa o custo
        if (!can_exec_another_task) {
            auto best_deliver_tuple = this->__ce_ref->closer_deliver(cur_task_of_main, this->__main_robot);
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
        if (cur_cost < best_cost) {
            best_cost = cur_cost;
            best_tasks.clear();
            best_tasks.push_back(cur_task_of_main);
        }
        else if (cur_cost == best_cost)
            best_tasks.push_back(cur_task_of_main);
    }
    
//#ifdef DEBUG
    std::cout << "\tWinner Tasks: ";
    for (auto it = std::begin(best_tasks); it != std::end(best_tasks); ++it)
        std::cout << "t" << (*it)->get_id() << " ";
    std::cout << std::endl;
//#endif
    
    // Pontuando as tarefas de menores custos
    std::map<uint16_t, int8_t>& scoreboard_deref = *(this->__scoreboard);
    for (auto it = std::begin(best_tasks); it != std::end(best_tasks); ++it)
        scoreboard_deref[(*it)->get_id()] += 1;
    
#ifdef DEBUG
    std::cout << "Scoreboard:" << std::endl;
    this->show_scoreboard();
    std::cout << std::endl;
#endif
}

/*void Match::game4_secondary_domain_extent() {
#ifdef DEBUG
    std::cout << "Step 4 : Robot " << this->__main_robot->get_id() <<
            " in position (" << this->__main_robot->get_x() << ", " << this->__main_robot->get_y() <<
            ") with capacity " << this->__main_robot->get_current_dataset_capacity() << std::endl;
    std::cout << "Scoreboard:" << std::endl;
    this->show_scoreboard();
    std::cout << std::endl;
#endif
    
    std::vector<Robot*> all_robots = (this->__all_robots_ref);
    std::vector<PickTask*> main_tasks = (this->__main_tasks);
    int n_main_tasks = main_tasks.size();
    // Busca no domínio secundário os robôs que dominam as tarefas principais, armazena em extent:
    // Primeiro incicializa a estrutura
    std::map<uint16_t, std::vector<PickTask*>* > extent;
    for (int i = 0; i < all_robots.size(); i++) {
        std::vector<PickTask*>* attached_tasks_ref = new std::vector<PickTask*>[n_main_tasks];
        extent.insert({all_robots[i]->get_id(), attached_tasks_ref});
    }
    // Agora busca e anexa as tarefas
    for (int i = 0; i < main_tasks.size(); i++) {
        PickTask* cur_task = main_tasks[i];
        Robot* cur_robot = this->__dom_ref->get_pair_of_secondary_domain(cur_task->get_id()).second;
        std::vector<PickTask*>* attached_tasks_ref = extent[cur_robot->get_id()];
        std::vector<PickTask*>& attached_tasks = *(attached_tasks_ref);
        attached_tasks.push_back(cur_task);
        extent[cur_robot->get_id()] = attached_tasks_ref;
    }
    // Verifica as maiores repetições
    int32_t highest_repetition = 0;
    std::vector<int32_t> addrs_robot_highest_repetition;
    for (int i = 0; i < all_robots.size(); i++) {
        Robot* cur_robot = all_robots[i];
        std::vector<PickTask*>* attached_tasks_ref = extent[cur_robot->get_id()];
        std::vector<PickTask*>& attached_tasks = *(attached_tasks_ref);
        uint16_t cur_repetition = attached_tasks.size();
        if (cur_repetition > highest_repetition) {
            highest_repetition = cur_repetition;
            addrs_robot_highest_repetition.clear();
            addrs_robot_highest_repetition.push_back(cur_robot->get_id());
        }
        else if (cur_repetition == highest_repetition)
            addrs_robot_highest_repetition.push_back(cur_robot->get_id());
    }
    
#ifdef DEBUG
    std::cout << "\tWinner Tasks: ";
    for (int i = 0; i < addrs_robot_highest_repetition.size(); i++) {
        uint16_t id_robot = addrs_robot_highest_repetition[i];
        std::vector<PickTask*>* attached_tasks_ref = extent[id_robot];
        std::vector<PickTask*>& attached_tasks = *(attached_tasks_ref);
        for (auto it = std::begin(attached_tasks); it != std::end(attached_tasks); ++it)
            std::cout << "t" << (*it)->get_id() << " ";
    }  
    std::cout << std::endl;
#endif
    
    //Pontua as tarefas anexadas
    std::map<uint16_t, int8_t>& scoreboard_deref = *(this->__scoreboard);
    for (int i = 0; i < addrs_robot_highest_repetition.size(); i++) {
        uint16_t id_robot = addrs_robot_highest_repetition[i];
        std::vector<PickTask*>* attached_tasks_ref = extent[id_robot];
        std::vector<PickTask*>& attached_tasks = *(attached_tasks_ref);
        for (auto it = std::begin(attached_tasks); it != std::end(attached_tasks); ++it)
            scoreboard_deref[(*it)->get_id()] += 1;
    }
    
#ifdef DEBUG
    std::cout << "Scoreboard:" << std::endl;
    this->show_scoreboard();
    std::cout << std::endl;
#endif
    
    //Deleta extent
    for (int i = 0; i < all_robots.size(); i++) {
        std::vector<PickTask*>* attached_tasks_ref = extent[all_robots[i]->get_id()];
        delete attached_tasks_ref;
    }
}*/

/*void Match::game5_christofides_test(CostEstimator *_ce) {
#ifdef DEBUG
    std::cout << "Step 5 : Robot " << this->__main_robot->get_id() <<
            " in position (" << this->__main_robot->get_x() << ", " << this->__main_robot->get_y() <<
            ") with capacity " << this->__main_robot->get_current_dataset_capacity() << std::endl;
    std::cout << "Main tasks:" << std::endl;
    PickTask::show_tasks(this->__main_tasks);
#endif
    std::vector<PickTask*> main_tasks = (this->__main_tasks);
    int n_main_tasks = main_tasks.size();
    float smaller_cost = FLT_MAX;
    std::vector<PickTask*> tasks_best_cost;
    for (int i = 0; i < n_main_tasks; i++) {
        PickTask* cur_task = main_tasks[i];
#ifdef DEBUG
        std::cout << "Computing TSP without task t" << cur_task->get_id() << std::endl << std::endl;
#endif
        std::vector<PickTask*> main_tasks_copy(main_tasks.begin(), main_tasks.end());
        main_tasks_copy.erase(main_tasks_copy.begin()+i);
#ifdef DEBUG
        std::cout << "Input tasks for TSP:" << std::endl;
        PickTask::show_tasks(main_tasks_copy);
        std::cout << "Creating object... ";
#endif
        TSP *tsp = new TSP(main_tasks_copy, this->__main_robot, _ce);
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
            tasks_best_cost.clear();
            tasks_best_cost.push_back(cur_task);
        }
        else if (cur_cost == smaller_cost)
            tasks_best_cost.push_back(cur_task);
        
        delete tsp;
    }
#ifdef DEBUG
    std::cout << "\tWinner Tasks: ";
    for (auto it = std::begin(tasks_best_cost); it != std::end(tasks_best_cost); ++it)
        std::cout << "t" << (*it)->get_id() << " ";
    std::cout << std::endl;
#endif
    
    // Pontuando as tarefas de menores custos
    std::map<uint16_t, int8_t>& scoreboard_deref = *(this->__scoreboard);
    for (auto it = std::begin(tasks_best_cost); it != std::end(tasks_best_cost); ++it)
        scoreboard_deref[(*it)->get_id()] += 1;
    
#ifdef DEBUG
    std::cout << "Scoreboard:" << std::endl;
    this->show_scoreboard();
    std::cout << std::endl;
#endif
}*/

/*std::pair<PickTask*, float> Match::get_alternative_task_for(Robot* _robot) {
    std::map<uint16_t, std::tuple<Robot*, std::vector<PickTask*>* > >* secondary_dominance_list_ref = this->__dom_ref->get_secondary_transposed_list();
    std::map<uint16_t, std::tuple<Robot*, std::vector<PickTask*>* > >& secondary_dominance_list = *(secondary_dominance_list_ref);
    std::tuple<Robot*, std::vector<PickTask*>* > secondary_dominance_tuple = secondary_dominance_list[_robot->get_id()];
    std::vector<PickTask*>* secondary_dominance_tasks_ref = std::get<1>(secondary_dominance_tuple);
    std::vector<PickTask*>& secondary_dominance_tasks = *(secondary_dominance_tasks_ref);
    
    // Criando lista de tarefas temporária (para ordenação)), e calculando distância para _robot
    std::vector<std::pair<PickTask*, float> > temp_task_list;
    for (int i = 0; i < secondary_dominance_tasks.size(); i++) {
        float cost = this->__ce_ref->get_cost(secondary_dominance_tasks[i], _robot, _robot);
        temp_task_list.push_back(std::make_pair(secondary_dominance_tasks[i], cost));
    }
    
    // Ordeno temp_task_list pelo custo
    std::sort(temp_task_list.begin(), temp_task_list.end(), Match::__compare_tasks_by_cost);
    
    // Itero sobre temp_task_list ordenado e retorno a primeira tarefa com demanda menor ou igual a capacidade
    for (int i = 0; i < temp_task_list.size(); i++) {
        PickTask *cur_task = temp_task_list[i].first;
        if (cur_task->get_demand() <= _robot->get_current_dataset_capacity())
            return temp_task_list[i];
    }
    PickTask* nil = NULL;
    return std::make_pair(nil, 0.0);
}*/

/*PickTask* Match::get_best_scoring_task() {           
#ifdef DEBUG
    std::cout << "Last step : Returning best scoring task " << std::endl;
    std::cout << "Scoreboard:" << std::endl;
    this->show_scoreboard();
    std::cout << std::endl;
#endif
    // Buscando tarefa (ou lista de tarefas) com o maior score:
    std::map<uint16_t, int8_t>& scoreboard_deref = *(this->__scoreboard);
    std::vector<PickTask*> main_tasks = (this->__main_tasks);
    int n_main_tasks = main_tasks.size();
    int8_t max_score = 0;
    std::vector<PickTask*> best_tasks;
    for (int i = 0; i < n_main_tasks; i++) {
        PickTask* cur_task = main_tasks[i];
        int8_t cur_score = scoreboard_deref.find(cur_task->get_id())->second;
        if (cur_score > max_score) {
            max_score = cur_score;
            best_tasks.clear();
            best_tasks.push_back(cur_task);
        }
        else if (cur_score == max_score)
            best_tasks.push_back(cur_task);
    }
    
    //Se der empate, precisa desempatar
    if (best_tasks.size() == 1) { // Não deu empate. Apenas retorna
#ifdef DEBUG
        std::cout << "\tReturning task t" << best_tasks[0]->get_id() << std::endl;
#endif
        return best_tasks[0];
    }
    // Deu empate
#ifdef DEBUG
    std::cout << "\tThese tasks have the same highscore:" << std::endl;
    for (auto it = std::begin(best_tasks); it != std::end(best_tasks); ++it)
        std::cout << "t" << (*it)->get_id() << " ";
    std::cout << std::endl;
    this->__dom_ref->show_secondary_domain();
#endif
    // Para desempatar, seleciona a tarefa cujo robô do domínio secundário possui o menor custo ponderado
    int n_all_tasks = this->__all_pick_tasks_ref.size();
    float min_cost = FLT_MAX;
    PickTask* best_scoring_task;
    for (int i = 0; i < n_all_tasks; i++) {
        if (this->__all_pick_tasks_ref[i]->get_current_state() == PickTask::waiting) {
            auto tuple = this->__dom_ref->get_pair_of_secondary_domain(this->__all_pick_tasks_ref[i]->get_id());
            PickTask* cur_task_of_sec_domain = tuple.first;
            Robot* cur_robot_of_sec_domain = tuple.second;

            //Verifica se a tarefa retirada do secondary domain está na lista de melhores tarefas
            if (PickTask::is_it_here(best_tasks, cur_task_of_sec_domain)) {
                // A tarefa retirada está na lista de melhores tarefas
                // Obtém o custo ponderado desse par
                float cur_cost = this->__ce_ref->get_cost_weighted_by_capacity(cur_task_of_sec_domain, cur_robot_of_sec_domain);
                if (cur_cost < min_cost) {
                    min_cost = cur_cost;
                    best_scoring_task = cur_task_of_sec_domain;
                }
            }
        }
    }
#ifdef DEBUG
    std::cout << "\tReturning task t" << best_scoring_task->get_id() << std::endl;
#endif
    return best_scoring_task;
}*/

std::vector<PickTask*> Match::get_best_scoring_task_v2() {           
#ifdef DEBUG
    std::cout << "Last step : Returning best scoring task " << std::endl;
    std::cout << "Scoreboard:" << std::endl;
    this->show_scoreboard();
    std::cout << std::endl;
#endif
    // Buscando tarefa (ou lista de tarefas) com o maior score:
    std::map<uint16_t, int8_t>& scoreboard_deref = *(this->__scoreboard);
    std::vector<PickTask*> main_tasks = (this->__main_tasks);
    int n_main_tasks = main_tasks.size();
    int8_t max_score = 0;
    std::vector<PickTask*> best_tasks;
    for (int i = 0; i < n_main_tasks; i++) {
        PickTask* cur_task = main_tasks[i];
        int8_t cur_score = scoreboard_deref.find(cur_task->get_id())->second;
        if (cur_score > max_score) {
            max_score = cur_score;
            best_tasks.clear();
            best_tasks.push_back(cur_task);
        }
        else if (cur_score == max_score)
            best_tasks.push_back(cur_task);
    }
    
    return best_tasks;
}

PickTask* Match::get_unique_task() {
    return this->__main_tasks[0];
}

/*uint32_t Match::prepare(Robot* _main_r) {
    try {
        if (this->__scoreboard)
            delete this->__scoreboard;
    }
    catch (std::exception& e) {
        std::cerr << "[" << __FILE__ <<":" << __LINE__ << "] An exception was caught while trying to destroy Match::__scoreboard, with message: '" << e.what() << "'. Exiting..." << std::endl;
        exit(0x1E38);
    
    }
    this->__main_tasks.clear();
    
    this->__main_robot = _main_r;
    
    this->__scoreboard = new std::map<uint16_t, int8_t>;
    std::map<uint16_t, int8_t>& scoreboard_deref = *(this->__scoreboard);
    int n_all_tasks = this->__all_pick_tasks_ref.size();
    for (int i = 0; i < n_all_tasks; i++) {
        if (this->__all_pick_tasks_ref[i]->get_current_state() == PickTask::waiting) {
            auto tuple = this->__dom_ref->get_pair_of_primary_domain(this->__all_pick_tasks_ref[i]->get_id());
            PickTask* cur_task = tuple.first;
            Robot* cur_robot = tuple.second;
            if (cur_robot->get_id() == this->__main_robot->get_id()) {
                this->__main_tasks.push_back(cur_task);
                scoreboard_deref.insert({cur_task->get_id(), 0}); // task on address i receives score 0
            }
        }
    }
#ifdef DEBUG
    std::cout << "Scoreboard (after preparing):" << std::endl;
    this->show_scoreboard();
#endif
    
    return scoreboard_deref.size();
}*/

uint32_t Match::prepare_v2(Robot* _main_r) {
    try {
        if (this->__scoreboard)
            delete this->__scoreboard;
    }
    catch (std::exception& e) {
        std::cerr << "[" << __FILE__ <<":" << __LINE__ << "] An exception was caught while trying to destroy Match::__scoreboard, with message: '" << e.what() << "'. Exiting..." << std::endl;
        exit(0x1E38);
    
    }
    this->__main_tasks.clear();
    
    this->__main_robot = _main_r;
    
    this->__scoreboard = new std::map<uint16_t, int8_t>;
    std::map<uint16_t, int8_t>& scoreboard_deref = *(this->__scoreboard);
    int n_all_tasks = this->__all_pick_tasks_ref.size();
    for (int i = 0; i < n_all_tasks; i++) {
        if (this->__all_pick_tasks_ref[i]->get_current_state() == PickTask::waiting) {
            auto tuple = this->__dom_ref->get_tuple_of_task(this->__all_pick_tasks_ref[i]->get_id());
            PickTask* cur_task = std::get<0>(tuple);
            std::vector<Robot*>* cur_robots_ref = std::get<1>(tuple);
            std::vector<Robot*>& cur_robots = *(cur_robots_ref);
            for (int j = 0; j < cur_robots.size(); j++) {
                if (cur_robots[j]->get_id() == this->__main_robot->get_id()) {
                    this->__main_tasks.push_back(cur_task);
                    scoreboard_deref.insert({cur_task->get_id(), 0}); // task on address i receives score 0
                    break;
                }
            }
        }
    }
#ifdef DEBUG
    std::cout << "Scoreboard (after preparing):" << std::endl;
    this->show_scoreboard();
#endif
    
    return scoreboard_deref.size();
}

Match::~Match() {
    try {
        delete this->__scoreboard;
    }
    catch (std::exception& e) {
        std::cerr << "[" << __FILE__ <<":" << __LINE__ << "] An exception was caught while trying to destroy Match::__scoreboard, with message: '" << e.what() << "'. Exiting..." << std::endl;
        exit(0x1E38);
    }
    this->__main_tasks.clear();
    this->__main_robot = NULL;
    this->__dom_ref = NULL;
    this->__all_pick_tasks_ref.clear();
    this->__all_robots_ref.clear();
}

void Match::show_scoreboard() {
    std::map<uint16_t, int8_t>& scoreboard_deref = *(this->__scoreboard);
    for (auto it = std::begin(this->__main_tasks); it != std::end(this->__main_tasks); ++it)
        fprintf(stdout, "t%d: %d points\n", (*it)->get_id(), scoreboard_deref[(*it)->get_id()]);
        //std::cout << "t" << (*it)->get_id() << ": " << scoreboard_deref[(*it)->get_id()] << " points" << std::endl;
}