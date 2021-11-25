/* 
 * File:   Match.hpp
 * Author: geoso
 *
 * Created on 9 de Março de 2021, 15:44
 */

#ifndef MATCH_HPP
#define	MATCH_HPP

#include "Domain.hpp"
#include "TSP.hpp"

class Match {
public:
    Match() = delete;
    Match(std::vector<PickTask*>, std::vector<Robot*>, Domain*, CostEstimator*);
    void game1_assignment_variety();
    void game2_utility_test();
    void game2_utility_test_v2();
    void game2_utility_test_v3();
    void game3_cost_comparison();
    //void game4_secondary_domain_extent();
    //void game5_christofides_test(CostEstimator*);
    //std::pair<PickTask*, float> get_alternative_task_for(Robot*);
    //PickTask* get_best_scoring_task();
    std::vector<PickTask*> get_best_scoring_task_v2();
    PickTask* get_unique_task();
    //uint32_t prepare(Robot*);
    uint32_t prepare_v2(Robot*);
    void show_scoreboard();
    ~Match();
private:
    Robot* __main_robot;
    std::vector<PickTask*> __main_tasks;
    std::map<uint16_t, int8_t> *__scoreboard;
    CostEstimator *__ce_ref;
    Domain *__dom_ref;
    std::vector<PickTask*> __all_pick_tasks_ref;
    std::vector<Robot*> __all_robots_ref;
    static bool __compare_tasks_by_cost(std::pair<PickTask*, float>, std::pair<PickTask*, float>);
};

#endif	/* MATCH_HPP */

