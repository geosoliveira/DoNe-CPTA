/* 
 * File:   Parser.cpp
 * Author: geoso
 * 
 * Created on 17 de Fevereiro de 2021, 19:37
 */

#include "Parser.hpp"

using namespace std;

RSE::Parser::Parser() { }

void RSE::Parser::parse_robot_file(std::string _in_file, std::shared_ptr<Robot>& _robot) {
    ifstream in_stream;
    in_stream.open(_in_file.c_str(), ios::in);

    if(!in_stream){
        fprintf(stderr, "Can't open file %s! Make sure the file name is correct and try again...\n", _in_file.c_str());
        exit(1);
    }
    
    char in_line[1024];
    int state = 0;
    while(!in_stream.eof()){
        in_stream.getline(in_line, sizeof(in_line));
        char *left_side = strtok(in_line, ":");
        char *right_side = strtok(NULL, ":");
        char* pch;
        state++;
#ifdef DEBUG
        fprintf(stdout, "Processing %s:%s\n", left_side, right_side);
#endif
        switch(state) {
            case 1:
                _robot->set_name(right_side);
                break;
            case 2:
                _robot->set_type(right_side);
                break;
            case 3:
                pch = strtok(right_side, "|");
                while(pch) {
                    _robot->add_use_case(pch);
                    pch = strtok(NULL, "|");
                }
                break;
            case 4:
                _robot->add_description(right_side);
                break;
            case 5:
                (strcmp(right_side, "-")) ? _robot->set_length(atof(right_side)) : _robot->set_length(0.0);
                break;
            case 6:
                (strcmp(right_side, "-")) ? _robot->set_width(atof(right_side)) : _robot->set_width(0.0);
                break;
            case 7:
                (strcmp(right_side, "-")) ? _robot->set_height(atof(right_side)) : _robot->set_height(0.0);
                break;
            case 8:
                (strcmp(right_side, "-")) ? _robot->set_weight(atof(right_side)) : _robot->set_weight(0.0);
                break;
            case 9:
                if (strcmp(right_side, "-")) {
                    _robot->set_real_capacity(atoi(right_side));
                    _robot->set_dataset_capacity(atoi(right_side));
                }
                else {
                    fprintf(stderr, "Failed to get the robot's load capacity. Exiting...\n");
                    exit(1);
                }
                break;
            case 10:
                (strcmp(right_side, "-")) ? _robot->set_load_surface(atof(right_side)) : _robot->set_load_surface(0.0);
                break;
            case 11:
                if (strcmp(right_side, "-"))
                    _robot->set_min_linear_speed(atof(right_side));
                else {
                    fprintf(stderr, "Failed to get the robot's minimum linear speed. Exiting...\n");
                    exit(1);
                }
                break;
            case 12:
                if (strcmp(right_side, "-"))
                    _robot->set_max_linear_speed(atof(right_side));
                else {
                    fprintf(stderr, "Failed to get the robot's maximum linear speed. Exiting...\n");
                    exit(1);
                }
                break;
            case 13:
                (strcmp(right_side, "-")) ? _robot->set_angular_speed(atof(right_side)) : _robot->set_angular_speed(0.0);
                break;
            case 14:
                (strcmp(right_side, "-")) ? _robot->set_min_lifting_speed(atof(right_side)) : _robot->set_min_lifting_speed(0.0);
                break;
            case 15:
                (strcmp(right_side, "-")) ? _robot->set_max_lifting_speed(atof(right_side)) : _robot->set_max_lifting_speed(0.0);
                break;
            case 16:
                (strcmp(right_side, "-")) ? _robot->set_min_descending_speed(atof(right_side)) : _robot->set_min_descending_speed(0.0);
                break;
            case 17:
                (strcmp(right_side, "-")) ? _robot->set_max_descending_speed(atof(right_side)) : _robot->set_max_descending_speed(0.0);
                break;
            case 18:
                (strcmp(right_side, "-")) ? _robot->set_max_lifting_height(atoi(right_side)) : _robot->set_max_lifting_height(0);
                break;
            case 19:
                (strcmp(right_side, "-")) ? _robot->set_battery_load(atof(right_side)) : _robot->set_battery_load(0.0);
                break;
            case 20:
                (strcmp(right_side, "-")) ? _robot->set_min_battery_runtime(atof(right_side)) : _robot->set_min_battery_runtime(0.0);
                break;
            case 21:
                (strcmp(right_side, "-")) ? _robot->set_max_battery_runtime(atof(right_side)) : _robot->set_max_battery_runtime(0.0);
                break;
            case 22:
                (strcmp(right_side, "-")) ? _robot->set_battery_recharge_time(atof(right_side)) : _robot->set_battery_recharge_time(0.0);
                break;
            case 23:
                (strcmp(right_side, "-")) ? _robot->set_battery_life_cycles(atoi(right_side)) : _robot->set_battery_life_cycles(0);
                break;
            case 24:
                (strcmp(right_side, "-")) ? _robot->set_min_working_temperature(atoi(right_side)) : _robot->set_min_working_temperature(0);
                break;
            case 25:
                (strcmp(right_side, "-")) ? _robot->set_max_working_temperature(atoi(right_side)) : _robot->set_max_working_temperature(0);
                break;
            case 26:
                (strcmp(right_side, "-")) ? _robot->set_min_working_humidity(atof(right_side)) : _robot->set_min_working_humidity(0.0);
                break;
            case 27:
                (strcmp(right_side, "-")) ? _robot->set_max_working_humidity(atof(right_side)) : _robot->set_max_working_humidity(0.0);
                break;
            case 28:
                _robot->set_laser_scanners(false);
                _robot->set_proximity_sensors(false);
                _robot->set_3d_cameras(false);
                _robot->set_2d_cameras(false);
                _robot->set_rear_sonar(false);
                _robot->set_bumpers(false);
                _robot->set_speaker(false);
                if (strcmp(right_side, "-")) {
                    pch = strtok(right_side, "|");
                    while(pch) {
                        if (!strcmp(pch, "Laser_scanners"))
                             _robot->set_laser_scanners(true);
                        else if (!strcmp(pch, "3D_Cameras"))
                            _robot->set_3d_cameras(true);
                        else if (!strcmp(pch, "Proximity_sensors"))
                            _robot->set_proximity_sensors(true);
                        else if (!strcmp(pch, "Rear_sonar"))
                            _robot->set_rear_sonar(true);
                        else if (!strcmp(pch, "Bumpers"))
                            _robot->set_bumpers(true);
                        else if (!strcmp(pch, "Speaker"))
                            _robot->set_speaker(true);
                        else if (!strcmp(pch, "2D_Cameras"))
                            _robot->set_2d_cameras(true);
                        pch = strtok(NULL, "|");
                    }
                }
                break;
            default:
                break;
        }
    }
    in_stream.close();
}

std::tuple<std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >, std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > >,
        std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > >, std::shared_ptr<std::vector<std::shared_ptr<Robot> > >,
        CostEstimator::costing_t, std::string>
//std::tuple< std::vector<PickTask*>*, std::vector<DeliverTask*>*, std::vector<Robot*>*, CostEstimator::costing_t >
RSE::Parser::parse_file(std::string in_filename) {
#ifdef DEBUG
    fprintf(stdout, "\nGETTING IN: std::tuple<Task*, int, Robot*, int> Parser::parse_stdin(uint16_t _nRobots) [%s:%d]\n",  __FILE__, __LINE__);
#endif
    int automata_state = 0, curr_pick_task, curr_deliver_task, curr_recharge_task, curr_robot;
    int x, y;
    uint16_t n_pick_tasks, n_deliver_tasks, n_recharge_tasks, n_robots;
    std::string dataset_name;
    char stream[STREAM_SIZE];
    
    /*PickTask **pick_tasks;
    DeliverTask **deliver_tasks;
    RechargeTask **recharge_tasks;
    Robot **robots;*/
    std::shared_ptr<PickTask> *pick_tasks;
    std::shared_ptr<DeliverTask> *deliver_tasks;
    std::shared_ptr<RechargeTask> *recharge_tasks;
    std::shared_ptr<Robot> *robots;
    CostEstimator::costing_t costing_type;
    
    ifstream in_stream;
    in_stream.open(in_filename.c_str(), ios::in);

    if(!in_stream){
        fprintf(stderr, "Can't open input file %s. Exiting...\n", in_filename.c_str());
        exit(1);
    }

    while(!in_stream.eof()) {
        in_stream >> stream;
#ifdef DEBUG
        fprintf(stdout, "\t\t%d : %s\n", automata_state, stream);
#endif
        switch(automata_state) {
            case 0:
                if (!strcmp(stream, "NAME")) {
                    automata_state = 1;
                }
                break;
            case 1:
                automata_state = 2;
                break;
            case 2:
                dataset_name = stream;
                automata_state = 3;
                break;
            case 3:
                if (!strcmp(stream, "DIMENSION")) {
                    automata_state = 4;
                }
                break;
            case 4:
                automata_state = 5;
                break;
            case 5:
                n_pick_tasks = atoi(stream);
                pick_tasks = new std::shared_ptr<PickTask>[n_pick_tasks];
                automata_state = 6;
                break;
            case 6:
                if (!strcmp(stream, "N_ROBOTS")) {
                    automata_state = 7;
                }
                break;
            case 7:
                automata_state = 8;
                break;
            case 8:
                n_robots = atoi(stream);
                robots = new std::shared_ptr<Robot>[n_robots];
                automata_state = 9;
                break;
            case 9:
                if (!strcmp(stream, "N_DEPOTS")) {
                    automata_state = 10;
                }
                break;
            case 10:
                automata_state = 11;
                break;
            case 11:
                n_deliver_tasks = atoi(stream);
                deliver_tasks = new std::shared_ptr<DeliverTask>[n_deliver_tasks];
                automata_state = 12;
                break;
            case 12:
                if (!strcmp(stream, "EDGE_WEIGHT_TYPE")) {
                    automata_state = 13;
                }
                break;
            case 13:
                automata_state = 14;
                break;
            case 14:
                if (!strcmp(stream, "EUC_2D"))
                    costing_type = CostEstimator::euc_2d;
                else if (!strcmp(stream, "MANHATTAN"))
                    costing_type = CostEstimator::manhattan;
                else if (!strcmp(stream, "EUC_TIME"))
                    costing_type = CostEstimator::euc_time;
                else if (!strcmp(stream, "MANHATTAN_TIME"))
                    costing_type = CostEstimator::manhattan_time;
                else if (!strcmp(stream, "EUC_ENERGY"))
                    costing_type = CostEstimator::euc_energy;
                else if (!strcmp(stream, "MANHATTAN_ENERGY"))
                    costing_type = CostEstimator::manhattan_energy;
                else if (!strcmp(stream, "EUC_TIME_ENERGY"))
                    costing_type = CostEstimator::euc_time_energy;
                else if (!strcmp(stream, "MANHATTAN_TIME_ENERGY"))
                    costing_type = CostEstimator::manhattan_time_energy;
                else {
                    std::cerr << "[" << __FILE__ <<":" << __LINE__ << "] Unrecognized cost type: '" << stream << "'. Exiting..." << std::endl;
                    exit(1);
                }
                automata_state = 15;
                break;
            case 15:
                if (!strcmp(stream, "NODE_COORD_SECTION")) {
                    automata_state = 16;
                }
                break;
            case 16:
                curr_pick_task = atoi(stream);
                pick_tasks[curr_pick_task-1] = std::make_shared<PickTask>(curr_pick_task, Entity::task, Task::pick, PickTask::unused);
                automata_state = 17;
                break;
            case 17:
                x = atoi(stream);
                pick_tasks[curr_pick_task-1]->set_x(x);
                automata_state = 18;
                break;
            case 18:
                y = atoi(stream);
                pick_tasks[curr_pick_task-1]->set_y(y);
                automata_state = 19;
                break;
            case 19:
                if (!strcmp(stream, "DEMAND_SECTION")) {
                    automata_state = 22;
                }
                else {
                    curr_pick_task = atoi(stream);
                    pick_tasks[curr_pick_task-1] = std::make_shared<PickTask>(curr_pick_task, Entity::task, Task::pick, PickTask::waiting);
                    automata_state = 20;
                }
                break;
            case 20:
                pick_tasks[curr_pick_task-1]->set_x(atoi(stream));
                automata_state = 21;
                break;
            case 21:
                pick_tasks[curr_pick_task-1]->set_y(atoi(stream));
                automata_state = 19;
                break;
            case 22:
                curr_pick_task = atoi(stream);
                automata_state = 23;
                break;
            case 23:
                pick_tasks[curr_pick_task-1]->set_demand(atoi(stream));
                automata_state = 24;
                break;
            case 24:
                if (!strcmp(stream, "ROBOT_SECTION")) {
                    automata_state = 26;
                }
                else {
                    curr_pick_task = atoi(stream);
                    automata_state = 25;
                }
                break;
            case 25:
                pick_tasks[curr_pick_task-1]->set_demand(atoi(stream));
                automata_state = 24;
                break;    
            case 26:
                curr_robot = atoi(stream);
                robots[curr_robot-1] = std::make_shared<Robot>(curr_robot, Robot::idle);
                automata_state = 27;
                break;
            case 27:
                robots[curr_robot-1]->set_x(atoi(stream));
                automata_state = 28;
                break;
            case 28:
                robots[curr_robot-1]->set_y(atoi(stream));
                automata_state = 29;
                break;
            case 29:
                if (!strcmp(stream, "GEN")) {
                    automata_state = 30;
                }
                else {
                    Parser::parse_robot_file(stream, robots[curr_robot-1]);
                    automata_state = 31;
                }
                break;
            case 30:
                robots[curr_robot-1]->set_dataset_capacity(atoi(stream));
                robots[curr_robot-1]->set_min_linear_speed(1.0);
                robots[curr_robot-1]->set_max_linear_speed(1.0);
                automata_state = 31;
                break;
            case 31:
                if (!strcmp(stream, "DEPOT_SECTION")) {
                    automata_state = 36;
                }
                else {
                    curr_robot = atoi(stream);
                    robots[curr_robot-1] = std::make_shared<Robot>(curr_robot, Robot::idle);
                    automata_state = 32;
                }
                break;
            case 32:
                robots[curr_robot-1]->set_x(atoi(stream));
                automata_state = 33;
                break;  
            case 33:
                robots[curr_robot-1]->set_y(atoi(stream));
                automata_state = 34;
                break; 
            case 34:
                if (!strcmp(stream, "GEN")) {
                    automata_state = 35;
                }
                else {
                    Parser::parse_robot_file(stream, robots[curr_robot-1]);
                    automata_state = 31;
                }
                break;
            case 35:
                robots[curr_robot-1]->set_dataset_capacity(atoi(stream));
                robots[curr_robot-1]->set_min_linear_speed(1.0);
                robots[curr_robot-1]->set_max_linear_speed(1.0);
                automata_state = 31;
                break;
            case 36:
                curr_deliver_task = atoi(stream);
                deliver_tasks[curr_deliver_task-1] = std::make_shared<DeliverTask>(curr_deliver_task, Entity::task, Task::deliver);
                automata_state = 37;
                break;
            case 37:
                x = atoi(stream);
                deliver_tasks[curr_deliver_task-1]->set_x(x);
                automata_state = 38;
                break;
            case 38:
                y = atoi(stream);
                deliver_tasks[curr_deliver_task-1]->set_y(y);
                automata_state = 39;
                break;
            case 39:
                if (!strcmp(stream, "EOF")) {
                    automata_state = 42;
                }
                else {
                    curr_deliver_task = atoi(stream);
                    deliver_tasks[curr_deliver_task-1] = std::make_shared<DeliverTask>(curr_deliver_task, Entity::task, Task::deliver);
                    automata_state = 40;
                }
                break;
            case 40:
                deliver_tasks[curr_deliver_task-1]->set_x(atoi(stream));
                automata_state = 41;
                break;
            case 41:
                deliver_tasks[curr_deliver_task-1]->set_y(atoi(stream));
                automata_state = 39;
                break;    
            case 42:
                break; 
            default:
                break;
        }
    }
    in_stream.close();
    // Assigning dataset capacity for all robots:
    /*for (int i = 0; i < n_robots; i++) {
        uint32_t dataset_capacity = 
                dataset_capacity_weight * (uint32_t)round(((float)robots[i]->get_real_capacity() * (float)max_task_demand) / (float)min_robot_capacity);
        robots[i]->set_dataset_capacity(dataset_capacity);
    }*/
#ifdef DEBUG
    fprintf(stdout, "LEAVING OF: std::tuple<Task*, int, Robot*, int> Parser::parse_stdin(uint16_t _nRobots)\n");
#endif
    /*std::vector<PickTask*> *vec_pick_tasks = new vector<PickTask*>(pick_tasks, pick_tasks + n_pick_tasks);
    std::vector<DeliverTask*> *vec_deliver_tasks = new vector<DeliverTask*>(deliver_tasks, deliver_tasks + n_deliver_tasks);
    std::vector<RechargeTask*> *vec_recharge_tasks;
    std::vector<Robot*> *vec_robots = new vector<Robot*>(robots, robots + n_robots);*/
    
    std::vector<std::shared_ptr<PickTask> > vec_pick_tasks(pick_tasks, pick_tasks + n_pick_tasks);
    std::vector<std::shared_ptr<DeliverTask> > vec_deliver_tasks(deliver_tasks, deliver_tasks + n_deliver_tasks);
    std::vector<std::shared_ptr<RechargeTask> > vec_recharge_tasks;
    std::vector<std::shared_ptr<Robot> > vec_robots(robots, robots + n_robots);
    
    std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > shared_pick_tasks = std::make_shared<std::vector<std::shared_ptr<PickTask> > >(vec_pick_tasks);
    std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > > shared_deliver_tasks = std::make_shared<std::vector<std::shared_ptr<DeliverTask> > >(vec_deliver_tasks);
    std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > > shared_recharge_tasks;
    std::shared_ptr<std::vector<std::shared_ptr<Robot> > > shared_robots = std::make_shared<std::vector<std::shared_ptr<Robot> > >(vec_robots);
    //return std::make_tuple(vec_pick_tasks, vec_deliver_tasks, vec_robots, costing_type);
    return std::make_tuple(shared_pick_tasks, shared_deliver_tasks, shared_recharge_tasks, shared_robots, costing_type, dataset_name);
}

