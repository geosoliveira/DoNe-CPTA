/* 
 * File:   Pick.hpp
 * Author: geoso
 *
 * Created on 16 de Fevereiro de 2021, 23:52
 */

#ifndef PICK_HPP
#define	PICK_HPP

#include <algorithm>
#include "Task.hpp"
#include "Order.hpp"

class PickTask : public Task {
public:
    typedef enum tstat { unused = 0, waiting = 0x2F3, testing = 0x2FE, assigned = 0x34E } taskstat_t;
    
    PickTask() = delete;
    PickTask(uint16_t, Entity::entity_t, Task::task_t, PickTask::taskstat_t);
    PickTask(uint16_t, std::pair<uint16_t, uint16_t>, Entity::entity_t, Task::task_t, PickTask::taskstat_t);
    PickTask(std::shared_ptr<PickTask>);
    static bool compare_tasks_by_cost(std::pair<uint16_t, float>, std::pair<uint16_t, float>);
    PickTask::taskstat_t get_current_state();
    std::string get_streamof_current_state();
    int32_t get_demand();
    uint16_t get_height();
    std::vector<Order*> get_orders_ref();
    Order* get_order_ref(int);
    void set_current_state(PickTask::taskstat_t);
    void set_demand(int32_t);
    void set_height(uint16_t);
    void add_order_ref(Order*);
    static bool all_assigned(std::vector<std::shared_ptr<PickTask> >);
    static bool is_it_here(std::vector<PickTask*>, PickTask*);
    void show_me();
    std::string get_type_and_id();
    static std::vector<PickTask*>* copy_picktasks(std::vector<PickTask*>);
    static std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > copy_picktasks_v2(std::vector<std::shared_ptr<PickTask> >);
    static std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > clone_picktasks(std::vector<std::shared_ptr<PickTask> >);
    static bool delete_picktasks(std::vector<PickTask*>&, PickTask*);
    static bool delete_picktasks_v2(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >&, std::shared_ptr<PickTask>);
    static std::vector<std::shared_ptr<PickTask> > intersection(std::vector<std::shared_ptr<PickTask> >, std::vector<std::shared_ptr<PickTask> >);
    static void show_tasks(std::vector<PickTask*>);
    ~PickTask();
private:
    PickTask::taskstat_t __current_state;
    int32_t __demand;
    uint16_t __height;
    std::vector<Order*> __orders_ref;
};

#endif	/* PICK_HPP */

