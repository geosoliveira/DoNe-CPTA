/* 
 * File:   RechargeTask.hpp
 * Author: geoso
 *
 * Created on 17 de Fevereiro de 2021, 00:25
 */

#ifndef RECHARGETASK_HPP
#define	RECHARGETASK_HPP

#include "Task.hpp"

class RechargeTask : public Task {
public:
    RechargeTask();
    //RechargeTask(RechargeTask*);
    float get_charger_amperage();
    void set_charger_amperage(float);
    void show_me();
    std::string get_type_and_id();
    static void show_tasks(std::vector<RechargeTask*>);
    ~RechargeTask();
private:
    float __charger_amperage;
};

#endif	/* RECHARGETASK_HPP */

