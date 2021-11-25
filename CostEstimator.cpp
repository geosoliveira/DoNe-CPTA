/* 
 * File:   CostEstimator.cpp
 * Author: geoso
 * 
 * Created on 17 de Fevereiro de 2021, 16:17
 */

#include "CostEstimator.hpp"

CostEstimator::CostEstimator(std::vector<PickTask*> _pick_tasks, std::vector<DeliverTask*> _deliver_tasks, std::vector<RechargeTask*> _recharge_tasks, std::vector<Robot*> _robots) {
#ifdef DEBUG
    fprintf(stdout, "\nGETTING IN: CostEstimator::CostEstimator(std::vector<PickTask> _pick_tasks, std::vector<DeliverTask> _deliver_tasks, std::vector<RechargeTask> _recharge_tasks, std::vector<Robot> _robots) [%s:%d]\n", __FILE__, __LINE__);
#endif
    this->__costing_type = none;
    for (auto it = std::begin(_pick_tasks); it != std::end(_pick_tasks); ++it) {
        this->__pick_tasks_ref.push_back(*it);
    }
    for (auto it = std::begin(_deliver_tasks); it != std::end(_deliver_tasks); ++it) {
        this->__deliver_tasks_ref.push_back(*it);
    }
    for (auto it = std::begin(_recharge_tasks); it != std::end(_recharge_tasks); ++it) {
        this->__recharge_tasks_ref.push_back(*it);
    }
    for (auto it = std::begin(_robots); it != std::end(_robots); ++it) {
        this->__robots_ref.push_back(*it);
    }
#ifdef DEBUG
    std::vector<PickTask*> pick_tasks = (this->__pick_tasks_ref);
    std::vector<DeliverTask*> deliver_tasks = (this->__deliver_tasks_ref);
    std::vector<Robot*> robots = (this->__robots_ref);
    Robot::show_robots(robots);
    PickTask::show_tasks(pick_tasks);
    DeliverTask::show_tasks(deliver_tasks);
#endif
#ifdef DEBUG
    fprintf(stdout, "LEAVING OF: CostEstimator::CostEstimator(std::vector<PickTask> _pick_tasks, std::vector<DeliverTask> _deliver_tasks, std::vector<RechargeTask> _recharge_tasks, std::vector<Robot> _robots) [%s:%d]\n", __FILE__, __LINE__);
#endif
}

CostEstimator::CostEstimator(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > _pick_tasks,
        std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > > _deliver_tasks,
        std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > > _recharge_tasks,
        std::shared_ptr<std::vector<std::shared_ptr<Robot> > > _robots) {
    this->__pick_tasks_ref_v2 = _pick_tasks;
    this->__deliver_tasks_ref_v2 = _deliver_tasks;
    this->__recharge_tasks_ref_v2 = _recharge_tasks;
    this->__robots_ref_v2 = _robots;
}

CostEstimator::costing_t CostEstimator::get_costing_type() {
    return this->__costing_type;
}

std::string CostEstimator::get_streamof_costing_type() {
    switch(this->__costing_type) {
        case(CostEstimator::none):
            return "none";
        case(CostEstimator::euc_2d):
            return "euc_2d";
        case(CostEstimator::manhattan):
            return "manhattan";
        case(CostEstimator::euc_time):
            return "euc_time";
        case(CostEstimator::manhattan_time):
            return "manhattan_time";
        case(CostEstimator::euc_energy):
            return "euc_energy";
        case(CostEstimator::manhattan_energy):
            return "manhattan_energy";
        case(CostEstimator::euc_time_energy):
            return "euc_time_energy";
        case(CostEstimator::manhattan_time_energy):
            return "manhattan_time_energy";
        default:
            std::cerr << "[" << __FILE__ <<":" << __LINE__ << "] Unknown costing type " << this->__costing_type << ". Exiting..." << std::endl;
            exit(0x7E9);
    }
}

/*std::vector<PickTask*> CostEstimator::get_pick_tasks_ref() {
    return this->__pick_tasks_ref;
}*/

std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > CostEstimator::get_pick_tasks_ref_v2() {
    return this->__pick_tasks_ref_v2;
}

/*std::vector<DeliverTask*> CostEstimator::get_deliver_tasks_ref() {
    return this->__deliver_tasks_ref;
}*/

std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > > CostEstimator::get_deliver_tasks_ref_v2() {
    return this->__deliver_tasks_ref_v2;
}

/*std::vector<RechargeTask*> CostEstimator::get_recharge_tasks_ref() {
    return this->__recharge_tasks_ref;
}*/

std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > > CostEstimator::get_recharge_tasks_ref_v2() {
    return this->__recharge_tasks_ref_v2;
}

/*std::vector<Robot*> CostEstimator::get_robots_ref() {
    return this->__robots_ref;
}*/

std::shared_ptr<std::vector<std::shared_ptr<Robot> > > CostEstimator::get_robots_ref_v2() {
    return this->__robots_ref_v2;
}

float CostEstimator::get_manhattan_energy_cost(Entity* _etty1, Entity* _etty2, Robot* _robot) {
    return ((float)(abs(_etty1->get_x() - _etty2->get_x()) + abs(_etty1->get_y() - _etty2->get_y()))) / _robot->get_max_linear_speed() * (_robot->get_battery_load() / _robot->get_max_battery_runtime());
}

float CostEstimator::get_manhattan_energy_cost_v2(std::shared_ptr<Entity> _etty1, std::shared_ptr<Entity> _etty2, std::shared_ptr<Robot> _robot) {
    return ((float)(abs(_etty1->get_x() - _etty2->get_x()) + abs(_etty1->get_y() - _etty2->get_y()))) / _robot->get_max_linear_speed() * (_robot->get_battery_load() / _robot->get_max_battery_runtime());
}

float CostEstimator::get_euclidean_energy_cost(Entity* _etty1, Entity* _etty2, Robot* _robot) {
    return (round(sqrt(pow((double)((double)_etty1->get_x() - (double)_etty2->get_x()), 2.0) + pow((double)((double)_etty1->get_y() - (double)_etty2->get_y()), 2.0)))) * (_robot->get_battery_load() / _robot->get_max_battery_runtime());
}

float CostEstimator::get_euclidean_energy_cost_v2(std::shared_ptr<Entity> _etty1, std::shared_ptr<Entity> _etty2, std::shared_ptr<Robot> _robot) {
    return (round(sqrt(pow((double)((double)_etty1->get_x() - (double)_etty2->get_x()), 2.0) + pow((double)((double)_etty1->get_y() - (double)_etty2->get_y()), 2.0)))) * (_robot->get_battery_load() / _robot->get_max_battery_runtime());
}

float CostEstimator::get_manhattan_time_cost(Entity* _etty1, Entity* _etty2, Robot* _robot) {
    return ((float)(abs(_etty1->get_x() - _etty2->get_x()) + abs(_etty1->get_y() - _etty2->get_y()))) / _robot->get_max_linear_speed();
}

float CostEstimator::get_manhattan_time_cost_v2(std::shared_ptr<Entity> _etty1, std::shared_ptr<Entity> _etty2, std::shared_ptr<Robot> _robot) {
    return ((float)(abs(_etty1->get_x() - _etty2->get_x()) + abs(_etty1->get_y() - _etty2->get_y()))) / _robot->get_max_linear_speed();
}

float CostEstimator::get_euclidean_time_cost(Entity* _etty1, Entity* _etty2, Robot* _robot) {
    return (round(sqrt(pow((double)((double)_etty1->get_x() - (double)_etty2->get_x()), 2.0) + pow((double)((double)_etty1->get_y() - (double)_etty2->get_y()), 2.0)))) / _robot->get_max_linear_speed();
}

float CostEstimator::get_euclidean_time_cost_v2(std::shared_ptr<Entity> _etty1, std::shared_ptr<Entity> _etty2, std::shared_ptr<Robot> _robot) {
    return (round(sqrt(pow((double)((double)_etty1->get_x() - (double)_etty2->get_x()), 2.0) + pow((double)((double)_etty1->get_y() - (double)_etty2->get_y()), 2.0)))) / _robot->get_max_linear_speed();
}

float CostEstimator::get_manhattan_cost(Entity* _etty1, Entity* _etty2) {
    return (float)(abs(_etty1->get_x() - _etty2->get_x()) + abs(_etty1->get_y() - _etty2->get_y()));
}

float CostEstimator::get_manhattan_cost_v2(std::shared_ptr<Entity> _etty1, std::shared_ptr<Entity> _etty2) {
    return (float)(abs(_etty1->get_x() - _etty2->get_x()) + abs(_etty1->get_y() - _etty2->get_y()));
}

float CostEstimator::get_euclidean_cost(Entity* _etty1, Entity* _etty2) {
    return round(sqrt(pow((double)((double)_etty1->get_x() - (double)_etty2->get_x()), 2.0) + pow((double)((double)_etty1->get_y() - (double)_etty2->get_y()), 2.0)));
}

float CostEstimator::get_euclidean_cost_v2(std::shared_ptr<Entity> _etty1, std::shared_ptr<Entity> _etty2) {
    return round(sqrt(pow((double)((double)_etty1->get_x() - (double)_etty2->get_x()), 2.0) + pow((double)((double)_etty1->get_y() - (double)_etty2->get_y()), 2.0)));
}

float CostEstimator::get_cost(Entity* _etty1, Entity* _etty2, Robot* _robot) {
#ifdef DEBUG
    /*std::cout << "Robot: " << _robot->get_id() << std::endl;
    std::cout << "Costing type: " << this->__costing_type << std::endl;
    std::cout << "Entity 1 location: (" << _etty1->get_x() << ", " << _etty1->get_y() << ")" << std::endl;
    std::cout << "Entity 2 location: (" << _etty2->get_x() << ", " << _etty2->get_y() << ")" << std::endl;*/
#endif
    switch(this->__costing_type) {
        case none:
            std::cerr << "[" << __FILE__ <<":" << __LINE__ << "] ERROR: Please use CostEstimator::set_costing_type() before using CostEstimator::get_cost(). Exiting..." << std::endl;
            exit(0x210B);
        case euc_2d:
            return this->get_euclidean_cost(_etty1, _etty2);
        case manhattan:
            return this->get_manhattan_cost(_etty1, _etty2);
        case euc_time:
            return this->get_euclidean_time_cost(_etty1, _etty2, _robot);
        case manhattan_time:
            return this->get_manhattan_time_cost(_etty1, _etty2, _robot);
        case euc_energy:
            return this->get_euclidean_energy_cost(_etty1, _etty2, _robot);
        case manhattan_energy:
            return this->get_manhattan_energy_cost(_etty1, _etty2, _robot);
        case euc_time_energy:
            std::cerr << "[" << __FILE__ <<":" << __LINE__ << "] Costing type " << this->get_streamof_costing_type() << " not implemented yet! Exiting... " << std::endl;
            exit(0xD29);
        case manhattan_time_energy:
            std::cerr << "[" << __FILE__ <<":" << __LINE__ << "] Costing type " << this->get_streamof_costing_type() << " not implemented yet! Exiting... " << std::endl;
            exit(0xD29);
        default:
            std::cerr << "[" << __FILE__ <<":" << __LINE__ << "] Unknown costing type " << this->__costing_type << ". Exiting..." << std::endl;
            exit(0x7E9);
    }
}

float CostEstimator::get_cost_v2(std::shared_ptr<Entity> _etty1, std::shared_ptr<Entity> _etty2, std::shared_ptr<Robot> _robot) {
    switch(this->__costing_type) {
        case none:
            std::cerr << "[" << __FILE__ <<":" << __LINE__ << "] ERROR: Please use CostEstimator::set_costing_type() before using CostEstimator::get_cost(). Exiting..." << std::endl;
            exit(0x210B);
        case euc_2d:
            return this->get_euclidean_cost_v2(_etty1, _etty2);
        case manhattan:
            return this->get_manhattan_cost_v2(_etty1, _etty2);
        case euc_time:
            return this->get_euclidean_time_cost_v2(_etty1, _etty2, _robot);
        case manhattan_time:
            return this->get_manhattan_time_cost_v2(_etty1, _etty2, _robot);
        case euc_energy:
            return this->get_euclidean_energy_cost_v2(_etty1, _etty2, _robot);
        case manhattan_energy:
            return this->get_manhattan_energy_cost_v2(_etty1, _etty2, _robot);
        case euc_time_energy:
            std::cerr << "[" << __FILE__ <<":" << __LINE__ << "] Costing type " << this->get_streamof_costing_type() << " not implemented yet! Exiting... " << std::endl;
            exit(0xD29);
        case manhattan_time_energy:
            std::cerr << "[" << __FILE__ <<":" << __LINE__ << "] Costing type " << this->get_streamof_costing_type() << " not implemented yet! Exiting... " << std::endl;
            exit(0xD29);
        default:
            std::cerr << "[" << __FILE__ <<":" << __LINE__ << "] Unknown costing type " << this->__costing_type << ". Exiting..." << std::endl;
            exit(0x7E9);
    }
}

float CostEstimator::get_cost_weighted_by_capacity(std::shared_ptr<Entity> _start, std::shared_ptr<PickTask> _ptask, std::shared_ptr<Robot> _robot) {
    uint32_t remaining_capacity = _robot->get_current_dataset_capacity() - _ptask->get_demand();
    uint32_t utility_count = 0, waiting_tasks_count = 0;
    float utility_percent, normal_cost, cost_weighted_by_capacity;
    std::vector<std::shared_ptr<PickTask> > pick_tasks = *(this->__pick_tasks_ref_v2);
    std::vector<std::shared_ptr<DeliverTask> > deliver_tasks = *(this->__deliver_tasks_ref_v2);
    float weighting_coefficient = 0.25;
#ifdef DEBUG
    /*std::cout << "Robot: " << _robot->get_id() << std::endl;
    std::cout << "Task: " << _ptask->get_id() << std::endl;
    std::cout << "Costing type: " << this->__costing_type << std::endl;
    std::cout << "Robot location: (" << _robot->get_x() << ", " << _robot->get_y() << ")" << std::endl;
    std::cout << "Robot current capacity: " << _robot->get_current_dataset_capacity() << std::endl;
    std::cout << "Task location: (" << _ptask->get_x() << ", " << _ptask->get_y() << ")" << std::endl;
    std::cout << "Task demand: " << _ptask->get_demand() << std::endl;
    std::cout << "Remaining capacity: " << remaining_capacity << std::endl;*/
#endif
    if (remaining_capacity >= 0) {
        for (int i = 0; i < pick_tasks.size(); i++) {
            if (pick_tasks[i]->get_current_state() == PickTask::waiting) {
                if (pick_tasks[i]->get_id() != _ptask->get_id() && pick_tasks[i]->get_demand() <= remaining_capacity)
                    utility_count++;
                waiting_tasks_count++;
            }
        }
        utility_percent = ((waiting_tasks_count - 1) != 0) ? (float)utility_count / (float)(waiting_tasks_count - 1) : 1.0;
        normal_cost = this->get_cost_v2(_start, _ptask, _robot);
        if (utility_percent == 0.0) {
            auto best_deliver_tuple = this->closer_deliver_v2(_ptask, _robot);
            normal_cost += best_deliver_tuple.second;
        }
        cost_weighted_by_capacity = ((weighting_coefficient - utility_percent) != 0.0) ? 
            fabs(normal_cost / (weighting_coefficient - utility_percent)) : FLT_MAX;
#ifdef DEBUG
        /*std::cout << "Waiting tasks: " << waiting_tasks_count << std::endl;
        std::cout << "Utility count: " << utility_count << std::endl;
        std::cout << "Utility percent: " << utility_percent << std::endl;
        std::cout << "Normal cost: " << normal_cost << std::endl;
        std::cout << "Weighted cost: " << cost_weighted_by_capacity << std::endl;*/
#endif
    }
    else {
        for (int i = 0; i < pick_tasks.size(); i++) {
            if (pick_tasks[i]->get_current_state() == PickTask::waiting) {
                if (pick_tasks[i]->get_id() != _ptask->get_id() && pick_tasks[i]->get_demand() < _robot->get_total_dataset_capacity() - _ptask->get_demand())
                    utility_count++;
                waiting_tasks_count++;
            }
        }
        utility_percent = ((waiting_tasks_count - 1) != 0) ? (float)utility_count / (float)(waiting_tasks_count - 1) : 1.0;
        auto best_deliver = this->closer_deliver_v2(_robot, _robot);
        normal_cost = this->get_cost_v2(_start, best_deliver.first, _robot) + 
                this->get_cost_v2(best_deliver.first, _ptask, _robot);
        if (utility_percent == 0.0) {
            auto best_deliver_tuple = this->closer_deliver_v2(_ptask, _robot);
            normal_cost += best_deliver_tuple.second;
        }
        cost_weighted_by_capacity = ((weighting_coefficient - utility_percent) != 0.0) ? 
            fabs(normal_cost / (weighting_coefficient - utility_percent)) : FLT_MAX;
    }
    return cost_weighted_by_capacity;
}

void CostEstimator::set_costing_type(CostEstimator::costing_t _c_type) {
    this->__costing_type = _c_type;
}

std::pair<DeliverTask*, float> CostEstimator::closer_deliver(Entity *_entty, Robot *_robot) {
    float minimum_cost = FLT_MAX;
    DeliverTask* best_deliver = NULL;
    std::vector<DeliverTask*> delivers = (this->__deliver_tasks_ref);
    for (int i = 0; i < delivers.size(); i++) {
        float curr_cost = this->get_cost(_entty, delivers[i], _robot);
        if (curr_cost < minimum_cost) {
            minimum_cost = curr_cost;
            best_deliver = delivers[i];
        }
    }
    return std::make_pair(best_deliver, minimum_cost);
}

std::pair<std::shared_ptr<DeliverTask>, float> CostEstimator::closer_deliver_v2(std::shared_ptr<Entity> _entty, std::shared_ptr<Robot> _robot) {
    float minimum_cost = FLT_MAX;
    std::shared_ptr<DeliverTask> best_deliver(NULL);
    std::vector<std::shared_ptr<DeliverTask> > delivers = *(this->__deliver_tasks_ref_v2);
    for (int i = 0; i < delivers.size(); i++) {
        float curr_cost = this->get_cost_v2(_entty, delivers[i], _robot);
        if (curr_cost < minimum_cost) {
            minimum_cost = curr_cost;
            best_deliver = delivers[i];
        }
    }
    return std::make_pair(best_deliver, minimum_cost);
}

void CostEstimator::show_me() {
    std::cout << "Use count of this->__pick_tasks_ref_v2: " << this->__pick_tasks_ref_v2.use_count() << std::endl;
    std::cout << "Use count of this->__deliver_tasks_ref_v2: " << this->__deliver_tasks_ref_v2.use_count() << std::endl;
    std::cout << "Use count of this->__recharge_tasks_ref_v2: " << this->__recharge_tasks_ref_v2.use_count() << std::endl;
    std::cout << "Use count of this->__robots_ref_v2: " << this->__robots_ref_v2.use_count() << std::endl;
    std::cout << "Costing type: " << this->get_streamof_costing_type() << std::endl;
    std::cout << "Pick tasks:" << std::endl;
    RSE::show<std::vector<std::shared_ptr<PickTask> > >(*(this->__pick_tasks_ref_v2));
    std::cout << "Deliver tasks:" << std::endl;
    RSE::show<std::vector<std::shared_ptr<DeliverTask> > >(*(this->__deliver_tasks_ref_v2));
    /*std::cout << "Recharge tasks:" << std::endl;
    RSE::show<std::vector<std::shared_ptr<RechargeTask> > >(*(this->__recharge_tasks_ref_v2));*/
    std::cout << "Robots:" << std::endl;
    RSE::show<std::vector<std::shared_ptr<Robot> > >(*(this->__robots_ref_v2));
}

CostEstimator::~CostEstimator() {
    this->__costing_type = none;
    this->__deliver_tasks_ref.clear();
    this->__pick_tasks_ref.clear();
    this->__recharge_tasks_ref.clear();
    this->__robots_ref.clear();
}

