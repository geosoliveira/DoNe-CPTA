/* 
 * File:   CostEstimator.hpp
 * Author: geoso
 *
 * Created on 17 de Fevereiro de 2021, 16:17
 */

#ifndef COSTESTIMATOR_HPP
#define	COSTESTIMATOR_HPP

#include <cmath>
#include "PickTask.hpp"
#include "DeliverTask.hpp"
#include "RechargeTask.hpp"
#include "Robot.hpp"

class CostEstimator {
public:
    typedef enum _tcosting { none = 0, euc_2d = 0x232, manhattan = 0x3BC, euc_time = 0x447, manhattan_time = 0x5D6, euc_energy = 0x527, manhattan_energy = 0x6B1, euc_time_energy = 0x741, manhattan_time_energy = 0x8CB} costing_t;

    CostEstimator() = delete;
    CostEstimator(std::vector<PickTask*>, std::vector<DeliverTask*>, std::vector<RechargeTask*>, std::vector<Robot*>);
    CostEstimator(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >, std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > >, std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > >, std::shared_ptr<std::vector<std::shared_ptr<Robot> > >);
    CostEstimator::costing_t get_costing_type();
    std::string get_streamof_costing_type();
    //std::vector<PickTask*> get_pick_tasks_ref();
    std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > get_pick_tasks_ref_v2();
    //std::vector<DeliverTask*> get_deliver_tasks_ref();
    std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > > get_deliver_tasks_ref_v2();
    //std::vector<RechargeTask*> get_recharge_tasks_ref();
    std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > > get_recharge_tasks_ref_v2();
    //std::vector<Robot*> get_robots_ref();
    std::shared_ptr<std::vector<std::shared_ptr<Robot> > > get_robots_ref_v2();
    float get_manhattan_energy_cost(Entity*, Entity*, Robot*);
    float get_manhattan_energy_cost_v2(std::shared_ptr<Entity>, std::shared_ptr<Entity>, std::shared_ptr<Robot>);
    float get_euclidean_energy_cost(Entity*, Entity*, Robot*);
    float get_euclidean_energy_cost_v2(std::shared_ptr<Entity>, std::shared_ptr<Entity>, std::shared_ptr<Robot>);
    float get_manhattan_time_cost(Entity*, Entity*, Robot*);
    float get_manhattan_time_cost_v2(std::shared_ptr<Entity>, std::shared_ptr<Entity>, std::shared_ptr<Robot>);
    float get_euclidean_time_cost(Entity*, Entity*, Robot*);
    float get_euclidean_time_cost_v2(std::shared_ptr<Entity>, std::shared_ptr<Entity>, std::shared_ptr<Robot>);
    float get_manhattan_cost(Entity*, Entity*);
    float get_manhattan_cost_v2(std::shared_ptr<Entity>, std::shared_ptr<Entity>);
    float get_euclidean_cost(Entity*, Entity*);
    float get_euclidean_cost_v2(std::shared_ptr<Entity>, std::shared_ptr<Entity>);
    float get_cost(Entity*, Entity*, Robot*);
    float get_cost_v2(std::shared_ptr<Entity>, std::shared_ptr<Entity>, std::shared_ptr<Robot>);
    float get_cost_weighted_by_capacity(std::shared_ptr<Entity>, std::shared_ptr<PickTask>, std::shared_ptr<Robot>);
    void set_costing_type(CostEstimator::costing_t);
    std::pair<DeliverTask*, float> closer_deliver(Entity*, Robot*);
    std::pair<std::shared_ptr<DeliverTask>, float> closer_deliver_v2(std::shared_ptr<Entity>, std::shared_ptr<Robot>);
    void show_me();
    ~CostEstimator();
private:
    CostEstimator::costing_t __costing_type;
    std::vector<PickTask*> __pick_tasks_ref;
    std::vector<DeliverTask*> __deliver_tasks_ref;
    std::vector<RechargeTask*> __recharge_tasks_ref;
    std::vector<Robot*> __robots_ref;
    
    std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > __pick_tasks_ref_v2;
    std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > > __deliver_tasks_ref_v2;
    std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > > __recharge_tasks_ref_v2;
    std::shared_ptr<std::vector<std::shared_ptr<Robot> > > __robots_ref_v2;
};

#endif	/* COSTESTIMATOR_HPP */

