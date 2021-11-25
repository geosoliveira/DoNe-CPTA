/* 
 * File:   Graph.cpp
 * Author: geoso
 * 
 * Created on 15 de Março de 2021, 21:46
 */

#include <map>
#include "Graph.hpp"

/*Graph::Graph(std::vector<PickTask*> _pick_tasks, CostEstimator* _ce) {
    this->__ce_ref = _ce;
    for (auto it = std::begin(_pick_tasks); it != std::end(_pick_tasks); ++it) {
        this->__pick_tasks_ref.push_back(*it);
        this->__tasks_map.insert({(*it)->get_id(), (*it)});
    }
}*/

Graph::Graph(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > _tasks_ref, std::shared_ptr<CostEstimator> _ce) {
    this->__ce_ref_v2 = _ce;
    this->__pick_tasks_ref_v2 = _tasks_ref;
    for (auto it = std::begin(*this->__pick_tasks_ref_v2); it != std::end(*this->__pick_tasks_ref_v2); ++it) {
        this->__tasks_map_v2.insert({(*it)->get_id(), (*it)});
    }
}

/*std::tuple<uint16_t, uint16_t, float> Graph::__get_farther_neighbor_cost_of(std::map<uint16_t, std::map<uint16_t, float> > _neighborhood, uint16_t _id) {
#ifdef DEBUG
    fprintf(stdout, "\nGETTING IN: std::tuple<int, int, float> Graph::__get_farther_neighbor_cost_of(std::map<uint16_t, std::map<uint16_t, float> > _neighborhood, uint16_t _id) [%s:%d]\n", __FILE__, __LINE__);
#endif
    std::vector<PickTask*> pick_tasks = (this->__pick_tasks_ref);
    
    uint16_t farther_u = 0, me = 0;
    float farther_cost = 0.0;
    for (int i = 0; i < _neighborhood[_id].size(); i++) {
        PickTask* cur_task = pick_tasks[i];
        if (_neighborhood[_id][cur_task->get_id()] > farther_cost) {
            farther_cost = _neighborhood[_id][cur_task->get_id()];
            farther_u = cur_task->get_id();
            me = _id;
        }
    }
    for (int j = 0; j < _neighborhood[_id].size(); j++) {
        PickTask* cur_task = pick_tasks[j];
        if (_neighborhood[cur_task->get_id()][_id] > farther_cost) {
            farther_cost = _neighborhood[cur_task->get_id()][_id];
            farther_u = _id;
            me = cur_task->get_id();
        }
    }
#ifdef DEBUG
    fprintf(stdout, "\tReturning tuple:\n\t\t[i = %d, j = %d, cost = %d]\n", me, farther_u, farther_cost);
#endif
#ifdef DEBUG
    fprintf(stdout, "LEAVING OF: std::tuple<uint16_t, uint16_t, float> Graph::__get_farther_neighbor_cost_of(std::map<uint16_t, std::map<uint16_t, float> > _neighborhood, uint16_t _id) [%s:%d]\n", __FILE__, __LINE__);
#endif
    return std::make_tuple(me, farther_u, farther_cost);
}

std::tuple<std::vector<Task*>, float> Graph::euler_tour(Robot* _robot) {
#ifdef DEBUG
    fprintf(stdout, "\nGETTING IN: std::tuple<std::vector<PickTask*>, float> Graph::euler_tour(Robot* _robot) [%s:%d]\n", __FILE__, __LINE__);
#endif
    std::vector<PickTask*> pick_tasks = (this->__pick_tasks_ref);
    
    std::map<uint16_t, std::map<uint16_t, float> >* all_neighbors_ref = new std::map<uint16_t, std::map<uint16_t, float> >[pick_tasks.size()];
    std::map<uint16_t, std::map<uint16_t, float> >& all_neighbors = *(all_neighbors_ref);
    
#ifdef DEBUG
    PickTask::show_tasks(pick_tasks);
#endif
    
    for (int i = 0; i < pick_tasks.size(); i++) {
        if (pick_tasks[i]->get_current_state() == PickTask::testing) {
            std::map<uint16_t, float>* neighbors_of_i = this->neighbors_of(pick_tasks[i], _robot, PickTask::testing);
            all_neighbors.insert({pick_tasks[i]->get_id(), *(neighbors_of_i)});
        }
    }
    
#ifdef DEBUG
    std::cout << "\tNeighborhood:" << std::endl;
    for (int i = 0; i < pick_tasks.size(); i++) {
        PickTask* cur_task_i = pick_tasks[i];
        std::cout << "\t[ ";
        for (int j = 0; j < pick_tasks.size(); j++) {
            PickTask* cur_task_j = pick_tasks[j];
            std::cout << all_neighbors[cur_task_i->get_id()][cur_task_j->get_id()] << " ";
        }
        std::cout << "]" << std::endl;
    }
#endif
    
    // Encontrando a tarefa mais próxima do robô e definindo-a como a tarefa inicial
    float min_cost = FLT_MAX;
    PickTask* start_task = NULL;
    for (int i = 0; i < pick_tasks.size(); i++) {
        PickTask* cur_task = pick_tasks[i];
        float cur_cost = this->__ce_ref->get_cost(cur_task, _robot, _robot);
        if (cur_cost < min_cost) {
            min_cost = cur_cost;
            start_task = cur_task;
        }
    }
    float cost = min_cost;
    
#ifdef DEBUG
    std::cout << "\tStart task: t" << start_task->get_id() << std::endl;
#endif
    
    // Euler Tour
    std::vector<Task*> path;
    std::stack<PickTask*> stack;
    uint16_t cur_id = start_task->get_id();
    path.push_back(start_task);
    while(!stack.empty() || all_neighbors[cur_id].size() > 0){
        //Current node has no neighbors
        /*float sum = std::accumulate( all_neighbors[cur_id].begin(), all_neighbors[cur_id].end(), 0.0,
                      []( float acc, std::pair<uint16_t, float> p ) { return ( acc + p.second ); } );
        if (!sum) {*
        if(all_neighbors[cur_id].empty()) {
            //add to circuit
            path.push_back(this->__tasks_map[cur_id]);
            //remove last vertex from stack and set it to current
            cur_id = stack.top()->get_id();
            stack.pop();
        }
        //If current node has neighbors
        else{
            for (int i = 0; i < pick_tasks.size(); i++) {
                PickTask* cur_task = pick_tasks[i];
                std::map<uint16_t, float>::iterator it;
                it = all_neighbors[cur_id].find(cur_task->get_id());
                if (it != all_neighbors[cur_id].end()) {
                //if (all_neighbors[cur_id][cur_task->get_id()] != 0.0) {
                    stack.push(this->__tasks_map[cur_id]);
                    all_neighbors[cur_id].erase(it);
                    //all_neighbors[cur_task->get_id()][cur_id] = 0.0;
                    //all_neighbors[cur_id][cur_task->get_id()] = 0.0;
                    cur_id = cur_task->get_id();
                    break;
                }
            }
        }
    }
    path.push_back(this->__tasks_map[cur_id]);
    auto best_deliver = this->__ce_ref->closer_deliver(this->__tasks_map[cur_id], _robot);
    path.push_back(best_deliver.first);
    cost += best_deliver.second;
    
    //Calculando custo
    for (int i = 1; i < path.size() - 1; i++) {
        cost += this->__ce_ref->get_cost(path[i], path[i-1], _robot);
    }
    delete[] all_neighbors_ref;
            
#ifdef DEBUG
    std::cout << "\tReturned path: ";
    for (int i = 0; i < path.size(); i++) {
        Task* cur_task = path[i];
        if (cur_task->get_task_type() == Task::pick)
            std::cout << "t" << cur_task->get_id() << " ";
        else if (cur_task->get_task_type() == Task::deliver)
            std::cout << "d" << cur_task->get_id() << " ";
    }
    std::cout << std::endl << "\tCost = " << cost << std::endl;
#endif
    
#ifdef DEBUG
    fprintf(stdout, "LEAVING OF: std::tuple<std::vector<PickTask*>, float> Graph::euler_tour(Robot* _robot) [%s:%d]\n", __FILE__, __LINE__);
#endif
    return std::make_tuple(path, cost);
}

std::tuple<std::vector<Task*>, float> Graph::euler_tour_v2(Robot* _robot) {
#ifdef DEBUG
    fprintf(stdout, "\nGETTING IN: std::tuple<std::vector<PickTask*>, float> Graph::euler_tour(Robot* _robot) [%s:%d]\n", __FILE__, __LINE__);
#endif
    std::vector<PickTask*> pick_tasks = (this->__pick_tasks_ref);
    
    std::map<uint16_t, std::map<uint16_t, float> >* all_neighbors_ref = new std::map<uint16_t, std::map<uint16_t, float> >[pick_tasks.size()];
    std::map<uint16_t, std::map<uint16_t, float> >& all_neighbors = *(all_neighbors_ref);
    
#ifdef DEBUG
    PickTask::show_tasks(pick_tasks);
#endif
    
    //Computing neighbors and degrees
    std::vector<std::map<uint16_t, float>* > neighbors_ref;
    std::map<uint16_t, int> degrees;
    for (int i = 0; i < pick_tasks.size(); i++) {
        if (pick_tasks[i]->get_current_state() == PickTask::testing) {
            neighbors_ref.push_back(this->neighbors_of(pick_tasks[i], _robot, PickTask::testing));
            std::map<uint16_t, float>& neighbor = *(neighbors_ref[neighbors_ref.size()-1]);
            all_neighbors.insert({pick_tasks[i]->get_id(), neighbor});
            degrees.insert({pick_tasks[i]->get_id(), neighbor.size() * 2});
        }
    }
    
#ifdef DEBUG
    std::cout << "\t\tDegrees: ";
    for (int i = 0; i < pick_tasks.size(); i++) {
        if (pick_tasks[i]->get_current_state() == PickTask::testing) {
            std::cout << "t" << pick_tasks[i]->get_id() << ":" << degrees[pick_tasks[i]->get_id()] << " ";
        }
    }
    std::cout << std::endl;
#endif
    
#ifdef DEBUG
    std::cout << "\tNeighborhood (init):" << std::endl;
    for (int i = 0; i < pick_tasks.size(); i++) {
        PickTask* cur_task_i = pick_tasks[i];
        std::cout << "\t[ ";
        for (int j = 0; j < pick_tasks.size(); j++) {
            PickTask* cur_task_j = pick_tasks[j];
            std::cout << all_neighbors[cur_task_i->get_id()][cur_task_j->get_id()] << " ";
        }
        std::cout << "]" << std::endl;
    }
#endif
    
    // Encontrando a tarefa mais próxima do robô e definindo-a como a tarefa inicial
    float min_cost = FLT_MAX;
    PickTask* start_task = NULL;
    for (int i = 0; i < pick_tasks.size(); i++) {
        PickTask* cur_task = pick_tasks[i];
        float cur_cost = this->__ce_ref->get_cost(cur_task, _robot, _robot);
        if (cur_cost < min_cost) {
            min_cost = cur_cost;
            start_task = cur_task;
        }
    }
    float cost = min_cost;
    
#ifdef DEBUG
    std::cout << "\tStart task: t" << start_task->get_id() << std::endl;
#endif
    
     // Euler Tour
#ifdef DEBUG
    std::cout << "\tEuler tour" << std::endl;
#endif
    std::vector<PickTask*> path;
    std::stack<PickTask*> stack;
    uint16_t cur_id = start_task->get_id();
    
    while (!stack.empty() || degrees[cur_id] > 0) {
        //getchar();
#ifdef DEBUG
        std::cout << "\t\tDegrees: ";
        for (int i = 0; i < pick_tasks.size(); i++) {
            if (pick_tasks[i]->get_current_state() == PickTask::testing) {
                std::cout << "t" << pick_tasks[i]->get_id() << ":" << degrees[pick_tasks[i]->get_id()] << " ";
            }
        }
        std::cout << std::endl;
#endif
        // If current node has not any neighbour
        // add it to path and pop stack
        // set new current to the popped element
        if (degrees[cur_id] == 0) {
#ifdef DEBUG
            std::cout << "\t\tDegree of t" << cur_id << " is " << degrees[cur_id] << std::endl;
            std::cout << "\t\tPushing t" << cur_id << " (t" << this->__tasks_map[cur_id]->get_id() << ") to stack" << std::endl;
#endif
            path.push_back(this->__tasks_map[cur_id]);
            cur_id = stack.top()->get_id();
#ifdef DEBUG
            std::cout << "\t\tNow cur_id is " << cur_id << std::endl;
#endif
            stack.pop();
        }
        // If the current vertex has at least one
        // neighbour add the current vertex to stack,
        // remove the edge between them and set the
        // current to its neighbour.
        else {
#ifdef DEBUG
            std::cout << "\t\tDegree of t" << cur_id << " is " << degrees[cur_id] << std::endl;
            std::cout << "\t\tNeighborhood (before):" << std::endl;
            for (int i = 0; i < pick_tasks.size(); i++) {
                PickTask* cur_task_i = pick_tasks[i];
                std::cout << "\t\t[ ";
                for (int j = 0; j < pick_tasks.size(); j++) {
                    PickTask* cur_task_j = pick_tasks[j];
                    std::cout << all_neighbors[cur_task_i->get_id()][cur_task_j->get_id()] << " ";
                }
                std::cout << "]" << std::endl;
            }
            std::cout << "\t\tGetting the farther neighbor of t" << cur_id << std::endl;
#endif
            auto tpl = this->__get_farther_neighbor_cost_of(all_neighbors, cur_id);
            uint16_t temp_i = std::get<0>(tpl);
            uint16_t temp_j = std::get<1>(tpl);
            float temp_cost = std::get<2>(tpl);
#ifdef DEBUG
            std::cout << "\t\t\ttemp_i = " << temp_i << std::endl;
            std::cout << "\t\t\ttemp_j = " << temp_j << std::endl;
            std::cout << "\t\t\ttemp_cost = " << temp_cost << std::endl;
#endif
            if (temp_i || temp_j || temp_cost) {
#ifdef DEBUG
            std::cout << "\t\tPushing t" << cur_id << " (t" << this->__tasks_map[cur_id]->get_id() << ") to stack" << std::endl;
#endif
                stack.push(this->__tasks_map[cur_id]);
                all_neighbors[temp_i][temp_j] = 0.0;
                degrees[temp_i] -= 1;
                degrees[temp_j] -= 1;
                cur_id = temp_j;
#ifdef DEBUG
                std::cout << "\t\tall_neighbors[temp_i=" << temp_i << "][temp_j=" << temp_j << "] = " << all_neighbors[temp_i][temp_j] << std::endl;
                std::cout << "\t\tNeighborhood (after):" << std::endl;
                for (int i = 0; i < pick_tasks.size(); i++) {
                    PickTask* cur_task_i = pick_tasks[i];
                    std::cout << "\t\t[ ";
                    for (int j = 0; j < pick_tasks.size(); j++) {
                        PickTask* cur_task_j = pick_tasks[j];
                        std::cout << all_neighbors[cur_task_i->get_id()][cur_task_j->get_id()] << " ";
                    }
                    std::cout << "]" << std::endl;
                }
                std::cout << "\t\tDegrees: ";
                for (int i = 0; i < pick_tasks.size(); i++) {
                    if (pick_tasks[i]->get_current_state() == PickTask::testing) {
                        std::cout << "t" << pick_tasks[i]->get_id() << ":" << degrees[pick_tasks[i]->get_id()] << " ";
                    }
                }
                std::cout << std::endl;
                std::cout << "\t\tNow cur_id is " << cur_id << std::endl;
#endif
            }
        }
    }
#ifdef DEBUG
    std::cout << "\t\tExiting of Euler tour..." << std::endl;
#endif
    path.push_back(this->__tasks_map[cur_id]);
    
    // Removendo vértices repetidos:
#ifdef DEBUG
    std::cout << "\tDeleting repeating tasks..." << std::endl;
    std::cout << "\tInput path: ";
    for (int i = 0; i < path.size(); i++) {
        Task* cur_task = path[i];
        if (cur_task->get_task_type() == Task::pick)
            std::cout << "t" << cur_task->get_id() << " ";
        else if (cur_task->get_task_type() == Task::deliver)
            std::cout << "d" << cur_task->get_id() << " ";
    }
    std::cout << std::endl;
#endif
    std::vector<Task*> c_path;
    std::map<uint16_t, bool> marked;
    for (int i = 0; i < pick_tasks.size(); i++) {
        if (pick_tasks[i]->get_current_state() == PickTask::testing) {
            degrees.insert({pick_tasks[i]->get_id(), false});
        }
    }
    for (int i = 0; i < path.size(); i++) {
        PickTask* cur_task = path[i];
        if (!marked[cur_task->get_id()]) {
            c_path.push_back(path[i]);
            marked[cur_task->get_id()] = true;
        }
    }
    auto best_deliver = this->__ce_ref->closer_deliver(this->__tasks_map[cur_id], _robot);
    c_path.push_back(best_deliver.first);
    cost += best_deliver.second;
    
    //Calculando custo
    for (int i = 1; i < c_path.size() - 1; i++) {
        cost += this->__ce_ref->get_cost(c_path[i], c_path[i-1], _robot);
    }
    
    for (int i = 0; i < neighbors_ref.size(); i++) {
        delete[] neighbors_ref[i];
    }
    delete[] all_neighbors_ref;
            
#ifdef DEBUG
    std::cout << "\tReturned path: ";
    for (int i = 0; i < c_path.size(); i++) {
        Task* cur_task = c_path[i];
        if (cur_task->get_task_type() == Task::pick)
            std::cout << "t" << cur_task->get_id() << " ";
        else if (cur_task->get_task_type() == Task::deliver)
            std::cout << "d" << cur_task->get_id() << " ";
    }
    std::cout << std::endl << "\tCost = " << cost << std::endl;
#endif
    
#ifdef DEBUG
    fprintf(stdout, "LEAVING OF: uint32_t Graph::euler_cycle_v2(int _p) [%s:%d]\n", __FILE__, __LINE__);
#endif
    
    return std::make_tuple(c_path, cost);
}*/

std::tuple<std::vector<std::shared_ptr<Task> >, float> Graph::euler_tour_v3(std::shared_ptr<Robot> _robot) {
#ifdef DEBUG
    fprintf(stdout, "\nGETTING IN: std::tuple<std::vector<std::shared_ptr<Task> >, float> Graph::euler_tour_v3(std::shared_ptr<Robot>) [%s:%d]\n", __FILE__, __LINE__);
#endif
    std::vector<std::shared_ptr<PickTask> > pick_tasks = *(this->__pick_tasks_ref_v2);
    
    std::map<uint16_t, std::map<uint16_t, float> > all_neighbors;
    
#ifdef DEBUG
    RSE::show<std::vector<std::shared_ptr<PickTask> > >(pick_tasks);
#endif
    
    for (int i = 0; i < pick_tasks.size(); i++) {
        if (pick_tasks[i]->get_current_state() == PickTask::testing) {
            std::map<uint16_t, float> neighbors_of_i = this->neighbors_of_v2(pick_tasks[i], _robot, PickTask::testing);
            all_neighbors.insert({pick_tasks[i]->get_id(), neighbors_of_i});
        }
    }
    
#ifdef DEBUG
    std::cout << "\tNeighborhood:" << std::endl;
    for (int i = 0; i < pick_tasks.size(); i++) {
        std::shared_ptr<PickTask> cur_task_i = pick_tasks[i];
        std::cout << "\t[ ";
        for (int j = 0; j < pick_tasks.size(); j++) {
            std::shared_ptr<PickTask> cur_task_j = pick_tasks[j];
            std::cout << all_neighbors[cur_task_i->get_id()][cur_task_j->get_id()] << " ";
        }
        std::cout << "]" << std::endl;
    }
#endif
    
    // Encontrando a tarefa mais próxima do robô e definindo-a como a tarefa inicial
    float min_cost = FLT_MAX;
    std::shared_ptr<PickTask> start_task = NULL;
    for (int i = 0; i < pick_tasks.size(); i++) {
        std::shared_ptr<PickTask> cur_task = pick_tasks[i];
        float cur_cost = this->__ce_ref_v2->get_cost_v2(cur_task, _robot, _robot);
        if (cur_cost < min_cost) {
            min_cost = cur_cost;
            start_task = cur_task;
        }
    }
    float cost = min_cost;
    
#ifdef DEBUG
    std::cout << "\tStart task: t" << start_task->get_id() << std::endl;
#endif
    
    // Euler Tour
    std::vector<std::shared_ptr<Task> > path;
    std::stack<std::shared_ptr<PickTask> > stack;
    uint16_t cur_id = start_task->get_id();
    path.push_back(start_task);
    while(!stack.empty() || all_neighbors[cur_id].size() > 0){
        //Current node has no neighbors
        /*float sum = std::accumulate( all_neighbors[cur_id].begin(), all_neighbors[cur_id].end(), 0.0,
                      []( float acc, std::pair<uint16_t, float> p ) { return ( acc + p.second ); } );
        if (!sum) {*/
        if(all_neighbors[cur_id].empty()) {
            //add to circuit
            path.push_back(this->__tasks_map_v2[cur_id]);
            //remove last vertex from stack and set it to current
            cur_id = stack.top()->get_id();
            stack.pop();
        }
        //If current node has neighbors
        else{
            for (int i = 0; i < pick_tasks.size(); i++) {
                std::shared_ptr<PickTask> cur_task = pick_tasks[i];
                std::map<uint16_t, float>::iterator it;
                it = all_neighbors[cur_id].find(cur_task->get_id());
                if (it != all_neighbors[cur_id].end()) {
                //if (all_neighbors[cur_id][cur_task->get_id()] != 0.0) {
                    stack.push(this->__tasks_map_v2[cur_id]);
                    all_neighbors[cur_id].erase(it);
                    //all_neighbors[cur_task->get_id()][cur_id] = 0.0;
                    //all_neighbors[cur_id][cur_task->get_id()] = 0.0;
                    cur_id = cur_task->get_id();
                    break;
                }
            }
        }
    }
    path.push_back(this->__tasks_map_v2[cur_id]);
    auto best_deliver = this->__ce_ref_v2->closer_deliver_v2(this->__tasks_map_v2[cur_id], _robot);
    path.push_back(best_deliver.first);
    cost += best_deliver.second;
    
    //Calculando custo
    for (int i = 1; i < path.size() - 1; i++) {
        cost += this->__ce_ref_v2->get_cost_v2(path[i], path[i-1], _robot);
    }
            
#ifdef DEBUG
    std::cout << "\tReturned path: ";
    for (int i = 0; i < path.size(); i++) {
        std::shared_ptr<Task> cur_task = path[i];
        if (cur_task->get_task_type() == Task::pick)
            std::cout << "t" << cur_task->get_id() << " ";
        else if (cur_task->get_task_type() == Task::deliver)
            std::cout << "d" << cur_task->get_id() << " ";
    }
    std::cout << std::endl << "\tCost = " << cost << std::endl;
#endif
    
#ifdef DEBUG
    fprintf(stdout, "LEAVING OF: std::tuple<std::vector<std::shared_ptr<Task> >, float> Graph::euler_tour_v3(std::shared_ptr<Robot>) [%s:%d]\n", __FILE__, __LINE__);
#endif
    return std::make_tuple(path, cost);
}

/*std::map<uint16_t, float>* Graph::neighbors_of(PickTask* _task, Robot* _robot, PickTask::taskstat_t _stat) {
    std::vector<PickTask*> pick_tasks = (this->__pick_tasks_ref);
    std::map<uint16_t, float>* neighbors_ref = new std::map<uint16_t, float>[pick_tasks.size()];
    std::map<uint16_t, float>& neighbors = *(neighbors_ref);
    for (int i = 0; i < pick_tasks.size(); i++) {
        PickTask* cur_task = pick_tasks[i];
        if (cur_task->get_id() != _task->get_id() &&
                cur_task->get_current_state() == _stat) {
            float cost = this->__ce_ref->get_cost(cur_task, _task, _robot);
            neighbors.insert({cur_task->get_id(), cost});
        }
    }
    return neighbors_ref;
}*/

std::map<uint16_t, float> Graph::neighbors_of_v2(std::shared_ptr<PickTask> _task, std::shared_ptr<Robot> _robot, PickTask::taskstat_t _stat) {
    std::vector<std::shared_ptr<PickTask> > pick_tasks = *(this->__pick_tasks_ref_v2);
    std::map<uint16_t, float> neighbors;
    for (int i = 0; i < pick_tasks.size(); i++) {
        std::shared_ptr<PickTask> cur_task = pick_tasks[i];
        if (cur_task->get_id() != _task->get_id() &&
                cur_task->get_current_state() == _stat) {
            float cost = this->__ce_ref_v2->get_cost_v2(cur_task, _task, _robot);
            neighbors.insert({cur_task->get_id(), cost});
        }
    }
    return neighbors;
}

/*PickTask* Graph::restricted_nearest_neighbor(PickTask* _task, int32_t _demand, Robot* _robot) {
#ifdef DEBUG
    fprintf(stdout, "\nGETTING IN: PickTask* Graph::restricted_nearest_neighbor(PickTask* _task, int32_t _demand, Robot* _robot) [%s:%d]\n", __FILE__, __LINE__);
#endif
    std::vector<PickTask*> pick_tasks = (this->__pick_tasks_ref);
    
    std::map<uint16_t, float>* neighbors_map_ref = this->neighbors_of(_task, _robot, PickTask::waiting);
    std::map<uint16_t, float>& neighbors_map = *(neighbors_map_ref);
    
    // Declare neighbors vector
    std::vector<std::pair<uint16_t, float> > neighbors; 
  
    // Copy key-value pair from Map to vector of pairs 
    for (auto& it : neighbors_map) { 
        neighbors.push_back(it); 
    } 
    
#ifdef DEBUG
    std::cout << "Neighbors (before sorting)" << std::endl;
    for (auto it = std::begin(neighbors); it != std::end(neighbors); ++it) {
        PickTask* cur_task = this->__tasks_map[(*it).first];
        float cost = (*it).second;
        std::cout << "[id = " << cur_task->get_id() << ", demand = " << cur_task->get_demand() << ", cost = " << cost << "]\n";
    }
#endif
    
    std::sort(neighbors.begin(), neighbors.end(), PickTask::compare_tasks_by_cost);
    
#ifdef DEBUG
    std::cout << "Neighbors (after sorting)" << std::endl;
    for (auto it = std::begin(neighbors); it != std::end(neighbors); ++it) {
        PickTask* cur_task = this->__tasks_map[(*it).first];
        float cost = (*it).second;
        std::cout << "[id = " << cur_task->get_id() << ", demand = " << cur_task->get_demand() << ", cost = " << cost << "]\n";
    }
#endif
    
    for (int i = 0; i < neighbors.size(); i++) {
        PickTask* cur_task = this->__tasks_map[neighbors[i].first];
        if (cur_task->get_demand() + _demand <= _robot->get_current_dataset_capacity()) {
#ifdef DEBUG
            fprintf(stdout, "\t\tReturned task: %d\n", cur_task->get_id());
            fprintf(stdout, "LEAVING OF: PickTask* Graph::restricted_nearest_neighbor(PickTask* _task, int32_t _demand, Robot* _robot) [%s:%d]\n", __FILE__, __LINE__);
#endif
            delete neighbors_map_ref;
            return cur_task;
        }
    }
    
#ifdef DEBUG
    fprintf(stdout, "\t\tCannot find a suitable neighbor...\n");
    fprintf(stdout, "\t\tReturned value: NULL\n");
    fprintf(stdout, "LEAVING OF: PickTask* Graph::restricted_nearest_neighbor(PickTask* _task, int32_t _demand, Robot* _robot) [%s:%d]\n", __FILE__, __LINE__);
#endif
    delete neighbors_map_ref;
    return NULL;
}*/

std::shared_ptr<PickTask> Graph::restricted_nearest_neighbor_v2(std::shared_ptr<PickTask> _task, int32_t _demand, std::shared_ptr<Robot> _robot) {
#ifdef DEBUG
    fprintf(stdout, "\nGETTING IN: std::shared_ptr<PickTask> restricted_nearest_neighbor_v2(std::shared_ptr<PickTask> _task, int32_t _demand, std::shared_ptr<Robot> _robot) [%s:%d]\n", __FILE__, __LINE__);
#endif
    std::vector<std::shared_ptr<PickTask> > pick_tasks = *(this->__pick_tasks_ref_v2);
    
    std::map<uint16_t, float> neighbors_map = this->neighbors_of_v2(_task, _robot, PickTask::waiting);
    
    // Declare neighbors vector
    std::vector<std::pair<uint16_t, float> > neighbors; 
  
    // Copy key-value pair from Map to vector of pairs 
    for (auto& it : neighbors_map) { 
        neighbors.push_back(it); 
    } 
    
#ifdef DEBUG
    std::cout << "Neighbors (before sorting)" << std::endl;
    for (auto it = std::begin(neighbors); it != std::end(neighbors); ++it) {
        std::shared_ptr<PickTask> cur_task = this->__tasks_map_v2[(*it).first];
        float cost = (*it).second;
        std::cout << "[id = " << cur_task->get_id() << ", demand = " << cur_task->get_demand() << ", cost = " << cost << "]\n";
    }
#endif
    
    std::sort(neighbors.begin(), neighbors.end(), PickTask::compare_tasks_by_cost);
    
#ifdef DEBUG
    std::cout << "Neighbors (after sorting)" << std::endl;
    for (auto it = std::begin(neighbors); it != std::end(neighbors); ++it) {
        std::shared_ptr<PickTask> cur_task = this->__tasks_map_v2[(*it).first];
        float cost = (*it).second;
        std::cout << "[id = " << cur_task->get_id() << ", demand = " << cur_task->get_demand() << ", cost = " << cost << "]\n";
    }
#endif
    
    for (int i = 0; i < neighbors.size(); i++) {
        std::shared_ptr<PickTask> cur_task = this->__tasks_map_v2[neighbors[i].first];
        if (cur_task->get_demand() + _demand <= _robot->get_current_dataset_capacity()) {
#ifdef DEBUG
            fprintf(stdout, "\t\tReturned task: %d\n", cur_task->get_id());
            fprintf(stdout, "LEAVING OF: std::shared_ptr<PickTask> restricted_nearest_neighbor_v2(std::shared_ptr<PickTask> _task, int32_t _demand, std::shared_ptr<Robot> _robot) [%s:%d]\n", __FILE__, __LINE__);
#endif
            return cur_task;
        }
    }
    
#ifdef DEBUG
    fprintf(stdout, "\t\tCannot find a suitable neighbor...\n");
    fprintf(stdout, "\t\tReturned value: NULL\n");
    fprintf(stdout, "LEAVING OF: std::shared_ptr<PickTask> restricted_nearest_neighbor_v2(std::shared_ptr<PickTask> _task, int32_t _demand, std::shared_ptr<Robot> _robot) [%s:%d]\n", __FILE__, __LINE__);
#endif
    return NULL;
}

/*std::tuple<std::vector<Task*>, float> Graph::single_node_cycle(Robot* _robot) {
#ifdef DEBUG
    fprintf(stdout, "\nGETTING IN: float Graph::single_node_cycle(Robot* _robot) [%s:%d]\n", __FILE__, __LINE__);
#endif
    std::vector<PickTask*> pick_tasks = (this->__pick_tasks_ref);
    
    std::vector<bool> visited;
    for (int i = 0; i < pick_tasks.size(); i++) {
        PickTask* cur_task = pick_tasks[i];
        if (cur_task->get_current_state() == PickTask::waiting)
            visited.push_back(false);
        else
            visited.push_back(true);
    }
    
    float cost = FLT_MAX;
    int cur_addr;
    for (int i = 0; i < pick_tasks.size(); i++) {
        float cur_cost = this->__ce_ref->get_cost(pick_tasks[i], _robot, _robot);
        if (cur_cost < cost) {
            cost = cur_cost;
            cur_addr = i;
        }
    }
    
#ifdef DEBUG
    std::cout << "\tStart task: t" << pick_tasks[cur_addr]->get_id() << ", with cost: " << cost << std::endl;
#endif
    
    //Calcula o ciclo a partir da tarefa encontrada no laço anterior
    std::vector<Task*> path;
    int bkp_addr;
    do {
        bkp_addr = cur_addr;
        float min_cost = FLT_MAX;
        visited[cur_addr] = true;
        for (int i = 0; i < pick_tasks.size(); i++) {
            if (!visited[i]) {
                float cur_cost = this->__ce_ref->get_cost(pick_tasks[bkp_addr], pick_tasks[i], _robot);
                if (cur_cost < min_cost) {
                    min_cost = cur_cost;
                    cur_addr = i;
                }
            }
        }
        if (cur_addr == bkp_addr) {
            path.push_back(pick_tasks[cur_addr]);
            auto best_deliver = this->__ce_ref->closer_deliver(pick_tasks[cur_addr], _robot);
            path.push_back(best_deliver.first);
            cost += best_deliver.second;
        }
        else {
            path.push_back(pick_tasks[bkp_addr]);
            cost += min_cost;
        }
    } while (cur_addr != bkp_addr);
#ifdef DEBUG
    std::cout << "\tReturned path: ";
    for (int i = 0; i < path.size(); i++) {
        Task* cur_task = path[i];
        if (cur_task->get_task_type() == Task::pick)
            std::cout << "t" << cur_task->get_id() << " ";
        else if (cur_task->get_task_type() == Task::deliver)
            std::cout << "d" << cur_task->get_id() << " ";
    }
    std::cout << std::endl << "\tCost = " << cost << std::endl;
#endif
#ifdef DEBUG
    fprintf(stdout, "LEAVING OF: float Graph::single_node_cycle(Robot* _robot) [%s:%d]\n", __FILE__, __LINE__);
#endif
    
    return std::make_tuple(path, cost);
}*/

std::tuple<std::vector<std::shared_ptr<Task> >, float> Graph::single_node_cycle_v2(std::shared_ptr<Robot> _robot) {
#ifdef DEBUG
    fprintf(stdout, "\nGETTING IN: std::tuple<std::vector<std::shared_ptr<Task> >, float> Graph::single_node_cycle_v2(std::shared_ptr<Robot> _robot) [%s:%d]\n", __FILE__, __LINE__);
#endif
    std::vector<std::shared_ptr<PickTask> > pick_tasks = *(this->__pick_tasks_ref_v2);
    
    std::vector<bool> visited;
    for (int i = 0; i < pick_tasks.size(); i++) {
        std::shared_ptr<PickTask> cur_task = pick_tasks[i];
        if (cur_task->get_current_state() == PickTask::waiting)
            visited.push_back(false);
        else
            visited.push_back(true);
    }
    
    float cost = FLT_MAX;
    int cur_addr;
    for (int i = 0; i < pick_tasks.size(); i++) {
        float cur_cost = this->__ce_ref_v2->get_cost_v2(pick_tasks[i], _robot, _robot);
        if (cur_cost < cost) {
            cost = cur_cost;
            cur_addr = i;
        }
    }
    
#ifdef DEBUG
    std::cout << "\tStart task: t" << pick_tasks[cur_addr]->get_id() << ", with cost: " << cost << std::endl;
#endif
    
    //Calcula o ciclo a partir da tarefa encontrada no laço anterior
    std::vector<std::shared_ptr<Task> > path;
    int bkp_addr;
    do {
        bkp_addr = cur_addr;
        float min_cost = FLT_MAX;
        visited[cur_addr] = true;
        for (int i = 0; i < pick_tasks.size(); i++) {
            if (!visited[i]) {
                float cur_cost = this->__ce_ref_v2->get_cost_v2(pick_tasks[bkp_addr], pick_tasks[i], _robot);
                if (cur_cost < min_cost) {
                    min_cost = cur_cost;
                    cur_addr = i;
                }
            }
        }
        if (cur_addr == bkp_addr) {
            path.push_back(pick_tasks[cur_addr]);
            auto best_deliver = this->__ce_ref_v2->closer_deliver_v2(pick_tasks[cur_addr], _robot);
            path.push_back(best_deliver.first);
            cost += best_deliver.second;
        }
        else {
            path.push_back(pick_tasks[bkp_addr]);
            cost += min_cost;
        }
    } while (cur_addr != bkp_addr);
#ifdef DEBUG
    std::cout << "\tReturned path: ";
    for (int i = 0; i < path.size(); i++) {
        std::shared_ptr<Task> cur_task = path[i];
        if (cur_task->get_task_type() == Task::pick)
            std::cout << "t" << cur_task->get_id() << " ";
        else if (cur_task->get_task_type() == Task::deliver)
            std::cout << "d" << cur_task->get_id() << " ";
    }
    std::cout << std::endl << "\tCost = " << cost << std::endl;
#endif
#ifdef DEBUG
    fprintf(stdout, "LEAVING OF: std::tuple<std::vector<std::shared_ptr<Task> >, float> Graph::single_node_cycle_v2(std::shared_ptr<Robot> _robot) [%s:%d]\n", __FILE__, __LINE__);
#endif
    
    return std::make_tuple(path, cost);
}

/*std::tuple<std::vector<Task*>, float> Graph::tsp_christofides(Robot* _robot) {
#ifdef DEBUG
    fprintf(stdout, "\nGETTING IN: std::tuple<std::vector<Task*>, float> Graph::tsp_christofides(Robot* _robot) [%s:%d]\n", __FILE__, __LINE__);
#endif
    std::vector<PickTask*> pick_tasks = (this->__pick_tasks_ref);
#ifdef DEBUG
    std::cout << "Input tasks for TSP:" << std::endl;
    PickTask::show_tasks(pick_tasks);
    std::cout << "Creating object... ";
#endif
    TSP *tsp = new TSP(pick_tasks, _robot, this->__ce_ref);
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
    
    std::tuple<std::vector<Task*>, float> tpl =  tsp->final_circuit();
    delete tsp;
    
#ifdef DEBUG
    fprintf(stdout, "LEAVING OF: std::tuple<std::vector<Task*>, float> Graph::tsp_christofides(Robot* _robot) [%s:%d]\n", __FILE__, __LINE__);
#endif
    
    return tpl;
}*/

std::tuple<std::vector<std::shared_ptr<Task> >, float> Graph::tsp_christofides_v2(std::shared_ptr<Robot> _robot) {
#ifdef DEBUG
    fprintf(stdout, "\nGETTING IN: std::tuple<std::vector<std::shared_ptr<Task> >, float> Graph::tsp_christofides_v2(std::shared_ptr<Robot> _robot) [%s:%d]\n", __FILE__, __LINE__);
#endif
    std::vector<std::shared_ptr<PickTask> > pick_tasks = *(this->__pick_tasks_ref_v2);
#ifdef DEBUG
    std::cout << "Input tasks for TSP:" << std::endl;
    RSE::show<std::vector<std::shared_ptr<PickTask> > >(pick_tasks);
    std::cout << "Creating object... ";
#endif
    TSP *tsp = new TSP(pick_tasks, _robot, this->__ce_ref_v2);
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
    
    std::tuple<std::vector<std::shared_ptr<Task> >, float> tpl =  tsp->final_circuit_v2();
    delete tsp;
    
#ifdef DEBUG
    fprintf(stdout, "LEAVING OF: std::tuple<std::vector<std::shared_ptr<Task> >, float> Graph::tsp_christofides_v2(std::shared_ptr<Robot> _robot) [%s:%d]\n", __FILE__, __LINE__);
#endif
    
    return tpl;
}

Graph::~Graph() {
    //this->__ce_ref_v2 = NULL;
    //this->__pick_tasks_ref.clear();
    //this->__tasks_map.clear();
}