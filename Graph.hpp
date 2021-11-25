/* 
 * File:   Graph.hpp
 * Author: geoso
 *
 * Created on 15 de Março de 2021, 21:46
 */

#ifndef GRAPH_HPP
#define	GRAPH_HPP

#include <memory>
#include <numeric>
#include <stack>
#include "CostEstimator.hpp"
#include "DeliverTask.hpp"
#include "PickTask.hpp"
#include "RechargeTask.hpp"
#include "Robot.hpp"
#include "TSP.hpp"

class Graph {
public:
    Graph() = delete;
    //Graph(std::vector<PickTask*>, CostEstimator*);
    Graph(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >, std::shared_ptr<CostEstimator>);
    ~Graph();
    //std::tuple<std::vector<Task*>, float> euler_tour(Robot*);
    //std::tuple<std::vector<Task*>, float> euler_tour_v2(Robot*);
    std::tuple<std::vector<std::shared_ptr<Task> >, float> euler_tour_v3(std::shared_ptr<Robot>);
    //std::map<uint16_t, float>* neighbors_of(PickTask*, Robot*, PickTask::taskstat_t);
    std::map<uint16_t, float> neighbors_of_v2(std::shared_ptr<PickTask>, std::shared_ptr<Robot>, PickTask::taskstat_t);
    //PickTask* restricted_nearest_neighbor(PickTask*, int32_t, Robot*);
    std::shared_ptr<PickTask> restricted_nearest_neighbor_v2(std::shared_ptr<PickTask>, int32_t, std::shared_ptr<Robot>);
    //std::tuple<std::vector<Task*>, float> single_node_cycle(Robot*);
    std::tuple<std::vector<std::shared_ptr<Task> >, float> single_node_cycle_v2(std::shared_ptr<Robot>);
    //std::tuple<std::vector<Task*>, float> tsp_christofides(Robot*);
    std::tuple<std::vector<std::shared_ptr<Task> >, float> tsp_christofides_v2(std::shared_ptr<Robot>);
private:
    //CostEstimator* __ce_ref;
    std::shared_ptr<CostEstimator> __ce_ref_v2;
    //std::vector<PickTask*> __pick_tasks_ref;
    std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > __pick_tasks_ref_v2;
    //std::map<uint16_t, PickTask*> __tasks_map;
    std::map<uint16_t, std::shared_ptr<PickTask> > __tasks_map_v2;
    //std::tuple<uint16_t, uint16_t, float> __get_farther_neighbor_cost_of(std::map<uint16_t, std::map<uint16_t, float> >, uint16_t);
};

#endif	/* GRAPH_HPP */

