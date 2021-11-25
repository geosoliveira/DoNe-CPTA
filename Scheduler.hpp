/* 
 * File:   Scheduler.hpp
 * Author: geoso
 *
 * Created on 10 de Março de 2021, 11:29
 */

#ifndef SCHEDULER_HPP
#define	SCHEDULER_HPP

#include <algorithm>
#include <array>
#include <numeric>
#include <queue> 
#include "Match.hpp"

class Scheduler {
public:
    Scheduler() = delete;
    Scheduler(std::shared_ptr<std::vector<std::shared_ptr<Robot> > >);
    void decrease_time_of_all(std::shared_ptr<Robot>);
    bool must_compute_domain();
    std::shared_ptr<Robot> next_robot_v2();
    void prepare_v2(std::shared_ptr<Domain>);
    static std::tuple<int, int, float, int, int> done(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >, std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > >, std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > >, std::shared_ptr<std::vector<std::shared_ptr<Robot> > >&, CostEstimator::costing_t);
    static std::tuple<int, int, float, int, int> done_cta(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >, std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > >, std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > >, std::shared_ptr<std::vector<std::shared_ptr<Robot> > >&, CostEstimator::costing_t);
    static std::tuple<int, int, float, int, int> done_pta(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >, std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > >, std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > >, std::shared_ptr<std::vector<std::shared_ptr<Robot> > >&, CostEstimator::costing_t);
    static std::tuple<int, int, float, int, int> done_cpta(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >, std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > >, std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > >, std::shared_ptr<std::vector<std::shared_ptr<Robot> > >&, CostEstimator::costing_t);
    static std::tuple<int, int, float, int, int> done_cpta_v2(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >, std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > >, std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > >, std::shared_ptr<std::vector<std::shared_ptr<Robot> > >&, CostEstimator::costing_t);
    static std::tuple<int, int, float, int, int> done_cpta_v3(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >, std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > >, std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > >, std::shared_ptr<std::vector<std::shared_ptr<Robot> > >&, CostEstimator::costing_t);
    static std::tuple<int, int, float, int, int> done_cpta_v4(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >, std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > >, std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > >, std::shared_ptr<std::vector<std::shared_ptr<Robot> > >&, CostEstimator::costing_t);
    static std::tuple<int, int, float, int, int> fifo(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >, std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > >, std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > >, std::shared_ptr<std::vector<std::shared_ptr<Robot> > >&, CostEstimator::costing_t);
    static std::tuple<int, int, float, int, int> greedy(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >, std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > >, std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > >, std::shared_ptr<std::vector<std::shared_ptr<Robot> > >&, CostEstimator::costing_t);
    static std::tuple<int, int, float, int, int> greedy_v2(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >, std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > >, std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > >, std::shared_ptr<std::vector<std::shared_ptr<Robot> > >&, CostEstimator::costing_t);
    static std::tuple<int, int, float, int, int> nCAR_v2(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >, std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > >, std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > >, std::shared_ptr<std::vector<std::shared_ptr<Robot> > >&, CostEstimator::costing_t);
    //void set_time(Robot*, float);
    float set_time_v2(std::shared_ptr<Robot>, std::shared_ptr<DeliverTask>, std::shared_ptr<Domain>, std::shared_ptr<CostEstimator>, CostEstimator::costing_t);
    float set_time_v2(std::shared_ptr<Robot>, std::shared_ptr<PickTask>, std::shared_ptr<Domain>, std::shared_ptr<CostEstimator>, CostEstimator::costing_t);
    void show_arrivals();
    ~Scheduler();
private:
    std::vector<std::tuple<std::shared_ptr<Robot>, float, int, bool> > __arrivals;
    static bool __compare_arrivals(std::tuple<std::shared_ptr<Robot>, float, int, bool>, std::tuple<std::shared_ptr<Robot>, float, int, bool>);
    static std::tuple<std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >, std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > > __feasible_route_v2(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >, std::shared_ptr<Robot>, std::shared_ptr<CostEstimator>);
};

#endif	/* SCHEDULER_HPP */

