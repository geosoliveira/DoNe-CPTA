/* 
 * File:   main.cpp
 * Author: geoso
 *
 * Created on 16 de Fevereiro de 2021, 21:13
 */

#include <cstdlib>
#include <chrono> 
#include "Parser.hpp"
#include "Scheduler.hpp"
#include "Graph.hpp"

using namespace std;

int RSE::current_execution;
int RSE::number_executions;
float RSE::cost_tolerance;

int main(int argc, char** argv) {
    /* Normal use...*/
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " [0:fifo|1:greedy|2:nCAR|3:done|4:done-cta|5:done_pta|6:done-cpta|7:done-cpta(v2)|8:done-cpta(v3)] <dataset_filename> <cost_tolerance>." << std::endl;
        exit(1);
    }
    
    auto entities = RSE::Parser::parse_file(argv[2]);
    RSE::cost_tolerance = atof(argv[3]);
    std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > shared_pick_tasks = std::get<0>(entities);
    std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > > shared_deliver_tasks = std::get<1>(entities);
    std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > > shared_recharge_tasks = std::get<2>(entities);
    std::shared_ptr<std::vector<std::shared_ptr<Robot> > > shared_robots = std::get<3>(entities);
    CostEstimator::costing_t costing_type = std::get<4>(entities);
    std::string dataset_name = std::get<5>(entities);
    
    /* Dispersão dos pontos no mapa */
    
    /*std::shared_ptr<CostEstimator> ce(new CostEstimator(shared_pick_tasks, shared_deliver_tasks, shared_recharge_tasks, shared_robots));
    ce->set_costing_type(costing_type);
    
    float average_x = 0.0, average_y = 0.0;
    for (int i = 1; i < shared_pick_tasks->size(); i++) {
        std::shared_ptr<PickTask> cur_pick_task = (*shared_pick_tasks)[i];
        average_x += (float)cur_pick_task->get_x();
        average_y += (float)cur_pick_task->get_y();
    }
    average_x /= (shared_pick_tasks->size() - 1);
    average_y /= (shared_pick_tasks->size() - 1);
    
    std::shared_ptr<PickTask> virtual_task(new PickTask(shared_pick_tasks->size() + 1,
            std::make_pair((uint16_t)round(average_x), (uint16_t)round(average_y)),
            Entity::task,
            Task::pick,
            PickTask::waiting));
    
    float average_distance_1 = 0.0;
    for (int i = 1; i < shared_pick_tasks->size(); i++) {
        std::shared_ptr<PickTask> cur_pick_task = (*shared_pick_tasks)[i];
        average_distance_1 += ce->get_cost_v2(cur_pick_task, virtual_task, (*shared_robots)[0]);
    }
    average_distance_1 /= (shared_pick_tasks->size() - 1);
    std::cout << "Average of dispersion of points on the map (1) " << average_distance_1 << std::endl;
    
    int ct = 0;
    float average_distance_2 = 0.0;
    for (int i = 1; i < shared_pick_tasks->size(); i++) {
        std::shared_ptr<PickTask> cur_pick_task_1 = (*shared_pick_tasks)[i];
        for (int j = 1; j < shared_pick_tasks->size(); j++) {
            std::shared_ptr<PickTask> cur_pick_task_2 = (*shared_pick_tasks)[j];
            if (cur_pick_task_1->get_id() != cur_pick_task_2->get_id()) {
                ct++;
                average_distance_2 += ce->get_cost_v2(cur_pick_task_1, cur_pick_task_2, (*shared_robots)[0]);
            }
        }
    }
    average_distance_2 /= ct;
    std::cout << "Average of dispersion of points on the map (2) " << average_distance_2 << std::endl;
    
    float average_distance_from_robots = 0.0;
    for (int i = 1; i < shared_pick_tasks->size(); i++) {
        std::shared_ptr<PickTask> cur_pick_task = (*shared_pick_tasks)[i];
        average_distance_from_robots += ce->get_cost_v2(cur_pick_task, (*shared_robots)[0], (*shared_robots)[0]);
    }
    average_distance_from_robots /= (shared_pick_tasks->size() - 1);
    std::cout << "Average distance from robots to tasks " << average_distance_from_robots << std::endl;
    
    /* Fim do cálculo da dispersão */
    
#ifdef DEBUG  
    RSE::show<std::vector<std::shared_ptr<Robot> > >(*shared_robots);
    RSE::show<std::vector<std::shared_ptr<PickTask> > >(*shared_pick_tasks);
    RSE::show<std::vector<std::shared_ptr<DeliverTask> > >(*shared_deliver_tasks);
    //RSE::show<std::vector<std::shared_ptr<RechargeTask> > >(*shared_recharge_tasks);
#endif
    
    static std::tuple<int, int, float, int, int> tpl_sch;
    
    auto start = std::chrono::high_resolution_clock::now();
    switch(atoi(argv[1])) {
        case 0:
            std::cerr << "Performing dataset '" << dataset_name.c_str() << "' by FIFO..." << std::endl;
            tpl_sch = Scheduler::fifo(shared_pick_tasks, shared_deliver_tasks, shared_recharge_tasks, shared_robots, costing_type);
            break;
        case 1:
            std::cerr << "Performing dataset '" << dataset_name.c_str() << "' by Greedy..." << std::endl;
            tpl_sch = Scheduler::greedy(shared_pick_tasks, shared_deliver_tasks, shared_recharge_tasks, shared_robots, costing_type);
            break;
        case 2:
            std::cerr << "Performing dataset '" << dataset_name.c_str() << "' by nCAR..." << std::endl;
            tpl_sch = Scheduler::nCAR_v2(shared_pick_tasks, shared_deliver_tasks, shared_recharge_tasks, shared_robots, costing_type);
            break;
        case 3:
            std::cerr << "Performing dataset '" << dataset_name.c_str() << "' by DoNe with cost tolerance " << RSE::cost_tolerance << "... " << std::endl;
            tpl_sch = Scheduler::done(shared_pick_tasks, shared_deliver_tasks, shared_recharge_tasks, shared_robots, costing_type);
            break;
        case 4:
            std::cerr << "Performing dataset '" << dataset_name.c_str() << "' by DoNe-CTA with cost tolerance " << RSE::cost_tolerance << "... " << std::endl;
            tpl_sch = Scheduler::done_cta(shared_pick_tasks, shared_deliver_tasks, shared_recharge_tasks, shared_robots, costing_type);
            break;
        case 5:
            std::cerr << "Performing dataset '" << dataset_name.c_str() << "' by DoNe-PTA with cost tolerance " << RSE::cost_tolerance << "... " << std::endl;
            tpl_sch = Scheduler::done_pta(shared_pick_tasks, shared_deliver_tasks, shared_recharge_tasks, shared_robots, costing_type);
            break;
        case 6:
            std::cerr << "Performing dataset '" << dataset_name.c_str() << "' by DoNe-CPTA with cost tolerance " << RSE::cost_tolerance << "... " << std::endl;
            tpl_sch = Scheduler::done_cpta(shared_pick_tasks, shared_deliver_tasks, shared_recharge_tasks, shared_robots, costing_type);
            break;
        case 7:
            std::cerr << "Performing dataset '" << dataset_name.c_str() << "' by DoNe-CPTA (v2) with cost tolerance " << RSE::cost_tolerance << "... " << std::endl;
            tpl_sch = Scheduler::done_cpta_v2(shared_pick_tasks, shared_deliver_tasks, shared_recharge_tasks, shared_robots, costing_type);
            break;
        case 8:
            std::cerr << "Performing dataset '" << dataset_name.c_str() << "' by DoNe-CPTA (v3) with cost tolerance " << RSE::cost_tolerance << "... " << std::endl;
            tpl_sch = Scheduler::done_cpta_v3(shared_pick_tasks, shared_deliver_tasks, shared_recharge_tasks, shared_robots, costing_type);
            break;
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    for (int i = 0; i < shared_robots->size(); i++) {
        (*shared_robots)[i]->show_task_list();
    }
    std::cout << "# Robots " << std::get<0>(tpl_sch) << std::endl;
    std::cout << "# Deliveries " << std::get<1>(tpl_sch) << std::endl;
    std::cout << "Cost " << std::get<2>(tpl_sch) << std::endl;
    std::cout << "Scheduling time " << duration.count() / 1000000.0 << std::endl;
    std::cout << "# Domain Comput " << std::get<3>(tpl_sch) << std::endl;
    std::cout << "# Domains with Unique Robots per Task " << std::get<4>(tpl_sch) << std::endl;
    
    //RSE::reset_all<std::vector<std::shared_ptr<RechargeTask> > >(*shared_recharge_tasks);
    RSE::reset_all<std::vector<std::shared_ptr<DeliverTask> > >(*shared_deliver_tasks);
    RSE::reset_all<std::vector<std::shared_ptr<PickTask> > >(*shared_pick_tasks);
    RSE::reset_all<std::vector<std::shared_ptr<Robot> > >(*shared_robots);
    //shared_recharge_tasks.reset();
    shared_deliver_tasks.reset();
    shared_pick_tasks.reset();
    shared_robots.reset();
    
    
    
    
    
    /* Benchmarking...*/
    /*if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <number_of_executions> <dataset_filename>" << std::endl;
        exit(1);
    }
    
    RSE::number_executions = atoi(argv[1]);
    RSE::cost_tolerance = 0.0;
    
    auto entities = RSE::Parser::parse_file(argv[2]);
    std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > shared_pick_tasks = std::get<0>(entities);
    std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > > shared_deliver_tasks = std::get<1>(entities);
    std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > > shared_recharge_tasks = std::get<2>(entities);
    std::shared_ptr<std::vector<std::shared_ptr<Robot> > > shared_robots = std::get<3>(entities);
    CostEstimator::costing_t costing_type = std::get<4>(entities);
    std::string dataset_name = std::get<5>(entities);
    
#ifdef DEBUG  
    RSE::show<std::vector<std::shared_ptr<Robot> > >(*shared_robots);
    RSE::show<std::vector<std::shared_ptr<PickTask> > >(*shared_pick_tasks);
    RSE::show<std::vector<std::shared_ptr<DeliverTask> > >(*shared_deliver_tasks);
    //RSE::show<std::vector<std::shared_ptr<RechargeTask> > >(*shared_recharge_tasks);
#endif
    
    static std::tuple<int, int, float, int, int> tpl_sch;
    double total_sch_time;
    
    ofstream out_stream;
    out_stream.open("results.txt", std::ofstream::out | std::ofstream::app);

    if(!out_stream){
        fprintf(stderr, "Can't open output file results.txt. Exiting...\n");
        exit(1);
    }
    
    out_stream << dataset_name.c_str() << "\t";
    
    std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > tasks_cp;
    std::shared_ptr<std::vector<std::shared_ptr<Robot> > > robots_cp;
    
    /*RSE::current_execution = 1;
    total_sch_time = 0.0;
    std::cerr << "Performing dataset '" << dataset_name.c_str() << "' by FIFO... " << std::endl;
    for (int i = 0; i < RSE::number_executions; i++) {
        tasks_cp = PickTask::clone_picktasks(*shared_pick_tasks);
        robots_cp = Robot::clone_robots(*shared_robots);
        auto start = std::chrono::high_resolution_clock::now();
        tpl_sch = Scheduler::fifo(tasks_cp, shared_deliver_tasks, shared_recharge_tasks, robots_cp, costing_type);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        total_sch_time += duration.count() / 1000000.0;
        RSE::current_execution++;
    }
    out_stream << std::get<0>(tpl_sch) << "\t" << std::get<1>(tpl_sch) << "\t" << std::get<2>(tpl_sch) << "\t" << (total_sch_time / ((double)RSE::number_executions)) << "\t\t";
    
    RSE::current_execution = 1;
    total_sch_time = 0.0;
    std::cerr << "Performing dataset '" << dataset_name.c_str() << "' by Greedy... " << std::endl;
    for (int i = 0; i < RSE::number_executions; i++) {
        tasks_cp = PickTask::clone_picktasks(*shared_pick_tasks);
        robots_cp = Robot::clone_robots(*shared_robots);
        auto start = std::chrono::high_resolution_clock::now();
        tpl_sch = Scheduler::greedy(tasks_cp, shared_deliver_tasks, shared_recharge_tasks, robots_cp, costing_type);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        total_sch_time += duration.count() / 1000000.0;
        RSE::current_execution++;
    }
    out_stream << std::get<0>(tpl_sch) << "\t" << std::get<1>(tpl_sch) << "\t" << std::get<2>(tpl_sch) << "\t" << (total_sch_time / ((double)RSE::number_executions)) << "\t\t";
    
    RSE::current_execution = 1;
    total_sch_time = 0.0;
    std::cerr << "Performing dataset '" << dataset_name.c_str() << "' by Greedy (v2)... " << std::endl;
    for (int i = 0; i < RSE::number_executions; i++) {
        tasks_cp = PickTask::clone_picktasks(*shared_pick_tasks);
        robots_cp = Robot::clone_robots(*shared_robots);
        auto start = std::chrono::high_resolution_clock::now();
        tpl_sch = Scheduler::greedy_v2(tasks_cp, shared_deliver_tasks, shared_recharge_tasks, robots_cp, costing_type);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        total_sch_time += duration.count() / 1000000.0;
        RSE::current_execution++;
    }
    out_stream << std::get<0>(tpl_sch) << "\t" << std::get<1>(tpl_sch) << "\t" << std::get<2>(tpl_sch) << "\t" << (total_sch_time / ((double)RSE::number_executions)) << "\t\t";
    
    RSE::current_execution = 1;
    total_sch_time = 0.0;
    std::cerr << "Performing dataset '" << dataset_name.c_str() << "' by nCAR... " << std::endl;
    for (int i = 0; i < RSE::number_executions; i++) {
        tasks_cp = PickTask::clone_picktasks(*shared_pick_tasks);
        robots_cp = Robot::clone_robots(*shared_robots);
        auto start = std::chrono::high_resolution_clock::now();
        tpl_sch = Scheduler::nCAR_v2(tasks_cp, shared_deliver_tasks, shared_recharge_tasks, robots_cp, costing_type);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        total_sch_time += duration.count() / 1000000.0;
        RSE::current_execution++;
    }
    out_stream << std::get<0>(tpl_sch) << "\t" << std::get<1>(tpl_sch) << "\t" << std::get<2>(tpl_sch) << "\t" << (total_sch_time / ((double)RSE::number_executions)) << "\t\t";
    
    RSE::current_execution = 1;
    total_sch_time = 0.0;
    std::cerr << "Performing dataset '" << dataset_name.c_str() << "' by DoNe with cost tolerance " << RSE::cost_tolerance << "... " << std::endl;
    for (int i = 0; i < RSE::number_executions; i++) {
        tasks_cp = PickTask::clone_picktasks(*shared_pick_tasks);
        robots_cp = Robot::clone_robots(*shared_robots);
        auto start = std::chrono::high_resolution_clock::now();
        tpl_sch = Scheduler::done(tasks_cp, shared_deliver_tasks, shared_recharge_tasks, robots_cp, costing_type);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        total_sch_time += duration.count() / 1000000.0;
        RSE::current_execution++;
    }
    out_stream << std::get<0>(tpl_sch) << "\t" << std::get<1>(tpl_sch) << "\t" << std::get<2>(tpl_sch) << "\t" << (total_sch_time / ((double)RSE::number_executions)) << "\t\t";
    
    RSE::current_execution = 1;
    total_sch_time = 0.0;
    std::cerr << "Performing dataset '" << dataset_name.c_str() << "' by DoNe-CTA with cost tolerance " << RSE::cost_tolerance << "... " << std::endl;
    for (int i = 0; i < RSE::number_executions; i++) {
        tasks_cp = PickTask::clone_picktasks(*shared_pick_tasks);
        robots_cp = Robot::clone_robots(*shared_robots);
        auto start = std::chrono::high_resolution_clock::now();
        tpl_sch = Scheduler::done_cta(tasks_cp, shared_deliver_tasks, shared_recharge_tasks, robots_cp, costing_type);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        total_sch_time += duration.count() / 1000000.0;
        RSE::current_execution++;
    }
    out_stream << std::get<0>(tpl_sch) << "\t" << std::get<1>(tpl_sch) << "\t" << std::get<2>(tpl_sch) << "\t" << (total_sch_time / ((double)RSE::number_executions)) << "\t\t";
    */
    /*RSE::current_execution = 1;
    total_sch_time = 0.0;
    std::cerr << "Performing dataset '" << dataset_name.c_str() << "' by DoNe-CPTA with cost tolerance " << RSE::cost_tolerance << "... " << std::endl;
    for (int i = 0; i < RSE::number_executions; i++) {
        tasks_cp = PickTask::clone_picktasks(*shared_pick_tasks);
        robots_cp = Robot::clone_robots(*shared_robots);
        auto start = std::chrono::high_resolution_clock::now();
        tpl_sch = Scheduler::done_cpta(tasks_cp, shared_deliver_tasks, shared_recharge_tasks, robots_cp, costing_type);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        total_sch_time += duration.count() / 1000000.0;
        RSE::current_execution++;
    }
    out_stream << std::get<0>(tpl_sch) << "\t" << std::get<1>(tpl_sch) << "\t" << std::get<2>(tpl_sch) << "\t" << (total_sch_time / ((double)RSE::number_executions)) << "\t\t";
    */
    /*RSE::current_execution = 1;
    total_sch_time = 0.0;
    std::cerr << "Performing dataset '" << dataset_name.c_str() << "' by DoNe-CPTA (v2) with cost tolerance " << RSE::cost_tolerance << "... " << std::endl;
    for (int i = 0; i < RSE::number_executions; i++) {
        tasks_cp = PickTask::clone_picktasks(*shared_pick_tasks);
        robots_cp = Robot::clone_robots(*shared_robots);
        auto start = std::chrono::high_resolution_clock::now();
        tpl_sch = Scheduler::done_cpta_v2(tasks_cp, shared_deliver_tasks, shared_recharge_tasks, robots_cp, costing_type);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        total_sch_time += duration.count() / 1000000.0;
        RSE::current_execution++;
    }
    out_stream << std::get<0>(tpl_sch) << "\t" << std::get<1>(tpl_sch) << "\t" << std::get<2>(tpl_sch) << "\t" << (total_sch_time / ((double)RSE::number_executions)) << "\t\t";
    
    RSE::current_execution = 1;
    total_sch_time = 0.0;
    std::cerr << "Performing dataset '" << dataset_name.c_str() << "' by DoNe-CPTA (v3) with cost tolerance " << RSE::cost_tolerance << "... " << std::endl;
    for (int i = 0; i < RSE::number_executions; i++) {
        tasks_cp = PickTask::clone_picktasks(*shared_pick_tasks);
        robots_cp = Robot::clone_robots(*shared_robots);
        auto start = std::chrono::high_resolution_clock::now();
        tpl_sch = Scheduler::done_cpta_v3(tasks_cp, shared_deliver_tasks, shared_recharge_tasks, robots_cp, costing_type);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        total_sch_time += duration.count() / 1000000.0;
        RSE::current_execution++;
    }
    out_stream << std::get<0>(tpl_sch) << "\t" << std::get<1>(tpl_sch) << "\t" << std::get<2>(tpl_sch) << "\t" << (total_sch_time / ((double)RSE::number_executions)) << "\t\t";
    */
    /*out_stream << std::endl;
    out_stream.close();
    
    //RSE::reset_all<std::vector<std::shared_ptr<RechargeTask> > >(*shared_recharge_tasks);
    RSE::reset_all<std::vector<std::shared_ptr<DeliverTask> > >(*shared_deliver_tasks);
    RSE::reset_all<std::vector<std::shared_ptr<PickTask> > >(*shared_pick_tasks);
    RSE::reset_all<std::vector<std::shared_ptr<Robot> > >(*shared_robots);
    //shared_recharge_tasks.reset();
    shared_deliver_tasks.reset();
    shared_pick_tasks.reset();
    shared_robots.reset();
    
    return EXIT_SUCCESS;*/
}

