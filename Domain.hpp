/* 
 * File:   Domain.hpp
 * Author: geoso
 *
 * Created on 17 de Fevereiro de 2021, 18:49
 */

#ifndef DOMAIN_HPP
#define	DOMAIN_HPP

#include <map>
#include "CostEstimator.hpp"
#include "Graph.hpp"
#include "PickTask.hpp"
#include "Robot.hpp"
#include "TSP.hpp"

class Domain {
public:
    Domain() = delete;
    Domain(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >, std::shared_ptr<std::vector<std::shared_ptr<Robot> > >, std::shared_ptr<CostEstimator>);
    void clear_domain_lists();
    bool compute_domain_v1();
    bool compute_domain_v2();
    bool compute_domain_v3();
    bool compute_domain_v4();
    bool compute_domain_v5();
    void compute_dominants();
    std::vector<std::shared_ptr<PickTask> >  get_domain_of_robot(std::shared_ptr<Robot>);
    std::vector<std::shared_ptr<Robot> > get_dominants_of_task(std::shared_ptr<PickTask>);
    int get_n_primary_dominants_by_r_v2(std::shared_ptr<Robot>);
    std::map<uint16_t, std::tuple<std::shared_ptr<Robot>, std::vector<std::shared_ptr<PickTask> > > > get_dominants();
    std::tuple<std::shared_ptr<Robot>, std::vector<std::shared_ptr<PickTask> > > get_tuple_of_robot(uint16_t);
    std::tuple<std::shared_ptr<PickTask>, std::vector<std::shared_ptr<Robot> > > get_tuple_of_task(uint16_t);
    std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > assignment_variety(std::shared_ptr<Robot>, std::vector<std::shared_ptr<PickTask> >);
    std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > cost_comparison(std::shared_ptr<Robot>, std::vector<std::shared_ptr<PickTask> >);
    std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > euler_test(std::shared_ptr<Robot>, std::vector<std::shared_ptr<PickTask> >);
    std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > tsp_test(std::shared_ptr<Robot>, std::vector<std::shared_ptr<PickTask> >);
    std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > utility_test(std::shared_ptr<Robot>, std::vector<std::shared_ptr<PickTask> >);
    void show_domain_of_robot(std::shared_ptr<Robot>);
    void show_domains();
    void show_dominants();
    ~Domain();
private:
    std::map<uint16_t, std::tuple<std::shared_ptr<PickTask>, std::vector<std::shared_ptr<Robot> > > > __domain_list;
    std::map<uint16_t, std::tuple<std::shared_ptr<Robot>, std::vector<std::shared_ptr<PickTask> > > > __dominant_list;
    std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > __pick_tasks_ref;
    std::shared_ptr<std::vector<std::shared_ptr<Robot> > > __robots_ref;
    std::shared_ptr<CostEstimator> __ce_ref;
};

#endif	/* DOMAIN_HPP */

