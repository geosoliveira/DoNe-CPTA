/* 
 * File:   Robot.cpp
 * Author: geoso
 * 
 * Created on 17 de Fevereiro de 2021, 00:29
 */

#include "Robot.hpp"

Robot::Robot(uint16_t _id) {
    this->__id = _id;
    this->__scheduling.__cost = 0.0;
}

Robot::Robot(uint16_t _id, Robot::robotstate_t _state) {
    this->__id = _id;
    this->__state = _state;
    this->__scheduling.__cost = 0.0;
}

Robot::Robot(std::shared_ptr<Robot> _robot) {
    this->__id = _robot->get_id();
    this->__name = _robot->get_name();
    this->__state = _robot->get_state();
    this->set_entity_type(_robot->get_entity_type());
    this->set_position(_robot->get_position());
    this->__general_specs.__description = _robot->get_description();
    this->__general_specs.__dimensions = _robot->get_dimensions();
    this->__general_specs.__type = _robot->get_type();
    for (int i = 0; i < _robot->get_n_use_cases(); i++)
        this->__general_specs.__use_cases.push_back(_robot->get_use_case(i));
    this->__general_specs.__weight = _robot->get_weight();
    this->__load_specs.__dataset_capacity = _robot->get_total_dataset_capacity();
    this->__load_specs.__load_surface = _robot->get_load_surface();
    this->__load_specs.__real_capacity = _robot->get_real_capacity();
    this->__motion_specs.__angular_speed = _robot->get_angular_speed();
    this->__motion_specs.__linear_speed = _robot->get_linear_speed();
    this->__gripper_specs.__descending_speed = _robot->get_descending_speed();
    this->__gripper_specs.__lifting_speed = _robot->get_lifting_speed();
    this->__gripper_specs.__max_lifting_height = _robot->get_max_lifting_height();
    this->__battery_specs.__life_cycles = _robot->get_battery_life_cycles();
    this->__battery_specs.__load = _robot->get_battery_load();
    this->__battery_specs.__recharge_time = _robot->get_battery_recharge_time();
    this->__battery_specs.__runtime_interval = _robot->get_battery_runtime();
    this->__ambient_specs.__humidity_interval = _robot->get_working_humidity();
    this->__ambient_specs.__temperature_interval = _robot->get_working_temperature();
    this->__sensors.__2d_cameras = _robot->has_2d_cameras();
    this->__sensors.__3d_cameras = _robot->has_3d_cameras();
    this->__sensors.__bumpers = _robot->has_bumpers();
    this->__sensors.__laser_scanners = _robot->has_laser_scanners();
    this->__sensors.__proximity_sensors = _robot->has_proximity_sensors();
    this->__sensors.__rear_sonar = _robot->has_rear_sonar();
    this->__sensors.__speaker = _robot->has_speaker();
    this->__dyn_resources.__cur_battery_load = _robot->get_current_battery_load();
    this->__dyn_resources.__cur_capacity = _robot->get_current_dataset_capacity();
    this->__scheduling.__cost = _robot->get_scheduling_cost();
    for (int i = 0; i < _robot->get_n_tasks(); i++)
        this->__scheduling.__tasks_ref.push_back(_robot->get_task_ref(i));
}

void Robot::set_id(uint16_t _id) {
    this->__id = _id;
}

void Robot::set_name(std::string _name) {
    this->__name = _name;
}

void Robot::set_state(Robot::robotstate_t _state) {
    this->__state = _state;
}

void Robot::set_type(std::string _type) {
    this->__general_specs.__type = _type;
}

void Robot::set_length(float _length) {
    float w = std::get<1>(this->__general_specs.__dimensions);
    float h = std::get<2>(this->__general_specs.__dimensions);
    this->__general_specs.__dimensions = std::make_tuple(_length, w, h);
}

void Robot::set_width(float _width) {
    float l = std::get<0>(this->__general_specs.__dimensions);
    float h = std::get<2>(this->__general_specs.__dimensions);
    this->__general_specs.__dimensions = std::make_tuple(l, _width, h);
}

void Robot::set_height(float _height) {
    float l = std::get<0>(this->__general_specs.__dimensions);
    float w = std::get<1>(this->__general_specs.__dimensions);
    this->__general_specs.__dimensions = std::make_tuple(l, w, _height);
}

void Robot::set_weight(float _weight) {
    this->__general_specs.__weight = _weight;
}

void Robot::set_real_capacity(uint32_t _real_capacity) {
    this->__load_specs.__real_capacity = _real_capacity;
}

void Robot::set_dataset_capacity(int32_t _dataset_capacity) {
    this->__load_specs.__dataset_capacity = _dataset_capacity;
    this->__dyn_resources.__cur_capacity = _dataset_capacity;
}

void Robot::set_load_surface(float _load_surface) {
    this->__load_specs.__load_surface = _load_surface;
}

void Robot::set_min_linear_speed(float _min_linear_speed) {
    float max_linear_speed = this->__motion_specs.__linear_speed.second;
    this->__motion_specs.__linear_speed = std::make_pair(_min_linear_speed, max_linear_speed);
}

void Robot::set_max_linear_speed(float _max_linear_speed) {
    float min_linear_speed = this->__motion_specs.__linear_speed.first;
    this->__motion_specs.__linear_speed = std::make_pair(min_linear_speed, _max_linear_speed);
}

void Robot::set_angular_speed(float _angular_speed) {
    this->__motion_specs.__angular_speed = _angular_speed;
}

void Robot::set_min_lifting_speed(float _min_lifting_speed) {
    float max_lifting_speed = this->__gripper_specs.__lifting_speed.second;
    this->__gripper_specs.__lifting_speed = std::make_pair(_min_lifting_speed, max_lifting_speed);
}

void Robot::set_max_lifting_speed(float _max_lifting_speed) {
    float min_lifting_speed = this->__gripper_specs.__lifting_speed.first;
    this->__gripper_specs.__lifting_speed = std::make_pair(min_lifting_speed, _max_lifting_speed);
}

void Robot::set_min_descending_speed(float _min_descending_speed) {
    float max_descending_speed = this->__gripper_specs.__descending_speed.second;
    this->__gripper_specs.__descending_speed = std::make_pair(_min_descending_speed, max_descending_speed);
}

void Robot::set_max_descending_speed(float _max_descending_speed) {
    float min_descending_speed = this->__gripper_specs.__descending_speed.first;
    this->__gripper_specs.__descending_speed = std::make_pair(min_descending_speed, _max_descending_speed);
}

void Robot::set_max_lifting_height(uint16_t _max_lifting_height) {
    this->__gripper_specs.__max_lifting_height = _max_lifting_height;
}

void Robot::set_battery_load(float _battery_load) {
    this->__battery_specs.__load = _battery_load;
    this->__dyn_resources.__cur_battery_load = _battery_load;
}

void Robot::set_min_battery_runtime(float _min_battery_runtime) {
    float max_battery_runtime = this->__battery_specs.__runtime_interval.second;
    this->__battery_specs.__runtime_interval = std::make_pair(_min_battery_runtime, max_battery_runtime);
}

void Robot::set_max_battery_runtime(float _max_battery_runtime) {
    float min_battery_runtime = this->__battery_specs.__runtime_interval.first;
    this->__battery_specs.__runtime_interval = std::make_pair(min_battery_runtime, _max_battery_runtime);
}

void Robot::set_battery_recharge_time(float _battery_recharge_time) {
    this->__battery_specs.__recharge_time = _battery_recharge_time;
}

void Robot::set_battery_life_cycles(uint16_t _battery_life_cycles) {
    this->__battery_specs.__life_cycles = _battery_life_cycles;
}

void Robot::set_min_working_temperature(uint8_t _min_working_temperature) {
    uint8_t max_working_temperature = this->__ambient_specs.__temperature_interval.second;
    this->__ambient_specs.__temperature_interval = std::make_pair(_min_working_temperature, max_working_temperature);
}

void Robot::set_max_working_temperature(uint8_t _max_working_temperature) {
    uint8_t min_working_temperature = this->__ambient_specs.__temperature_interval.first;
    this->__ambient_specs.__temperature_interval = std::make_pair(min_working_temperature, _max_working_temperature);
}

void Robot::set_min_working_humidity(float _min_working_humidity) {
    float max_working_humidity = this->__ambient_specs.__humidity_interval.second;
    this->__ambient_specs.__humidity_interval = std::make_pair(_min_working_humidity, max_working_humidity);
}

void Robot::set_max_working_humidity(float _max_working_humidity) {
    float min_working_humidity = this->__ambient_specs.__humidity_interval.first;
    this->__ambient_specs.__humidity_interval = std::make_pair(min_working_humidity, _max_working_humidity);
}

void Robot::set_laser_scanners(bool _laser_scanners) {
    this->__sensors.__laser_scanners = _laser_scanners;
}

void Robot::set_proximity_sensors(bool _proximity_sensors) {
    this->__sensors.__proximity_sensors = _proximity_sensors;
}

void Robot::set_3d_cameras(bool _3d_cameras) {
    this->__sensors.__3d_cameras = _3d_cameras;
}

void Robot::set_2d_cameras(bool _2d_cameras) {
    this->__sensors.__2d_cameras = _2d_cameras;
}

void Robot::set_rear_sonar(bool _rear_sonar) {
    this->__sensors.__rear_sonar = _rear_sonar;
}

void Robot::set_bumpers(bool _bumpers) {
    this->__sensors.__bumpers = _bumpers;
}

void Robot::set_speaker(bool _speaker) {
    this->__sensors.__speaker = _speaker;
}

void Robot::set_scheduling_cost(float _scheduling_cost) {
    this->__scheduling.__cost = _scheduling_cost;
}

void Robot::reset_current_capacity() {
    this->__dyn_resources.__cur_capacity = this->__load_specs.__dataset_capacity;
}

void Robot::reset_current_battery_load() {
    this->__dyn_resources.__cur_battery_load = this->__battery_specs.__load;
}

void Robot::add_use_case(std::string _use_case) {
    this->__general_specs.__use_cases.push_back(_use_case);
}

void Robot::add_description(std::string _description) {
    this->__general_specs.__description = _description;
}

void Robot::add_task_ref(Task* _t_ref) {
    this->__scheduling.__tasks_ref.push_back(_t_ref);
}

void Robot::add_scheduling_cost(float _scheduling_cost) {
    this->__scheduling.__cost += _scheduling_cost;
}

std::shared_ptr<std::vector<std::shared_ptr<Robot> > > Robot::clone_robots(std::vector<std::shared_ptr<Robot> > _robots) {
    std::shared_ptr<std::vector<std::shared_ptr<Robot> > > cp_ref(new std::vector<std::shared_ptr<Robot> >[_robots.size()]);
    for (int i = 0; i < _robots.size(); i++) {
        std::shared_ptr<Robot> robot_cp(new Robot(_robots[i]));
        cp_ref->push_back(robot_cp);
    }
    return cp_ref;
}

void Robot::decrease_from_current_capacity(int32_t _capacity) {
    this->__dyn_resources.__cur_capacity -= _capacity;
}

void Robot::decrease_from_current_battery_load(float _battery_load) {
    this->__dyn_resources.__cur_battery_load -= _battery_load;
}

uint16_t Robot::get_id() {
    return this->__id;
}

std::string Robot::get_name() {
    return this->__name;
}

Robot::robotstate_t Robot::get_state() {
    return this->__state;
}

std::string Robot::get_type() {
    return this->__general_specs.__type;
}

int Robot::get_n_use_cases() {
    return this->__general_specs.__use_cases.size();
}

std::string Robot::get_use_case(int _addr) {
    return this->__general_specs.__use_cases[_addr];
}

std::string Robot::get_description() {
    return this->__general_specs.__description;
}

float Robot::get_length() {
    return std::get<0>(this->__general_specs.__dimensions);
}

float Robot::get_width() {
    return std::get<1>(this->__general_specs.__dimensions);
}

float Robot::get_height() {
    return std::get<2>(this->__general_specs.__dimensions);
}

std::tuple<float, float, float> Robot::get_dimensions() {
    return this->__general_specs.__dimensions;
}

float Robot::get_weight() {
    return this->__general_specs.__weight;
}

uint32_t Robot::get_real_capacity() {
    return this->__load_specs.__real_capacity;
}

int32_t Robot::get_total_dataset_capacity() {
    return this->__load_specs.__dataset_capacity;
}

float Robot::get_load_surface() {
    return this->__load_specs.__load_surface;
}

float Robot::get_min_linear_speed() {
    return this->__motion_specs.__linear_speed.first;
}

float Robot::get_max_linear_speed() {
    return this->__motion_specs.__linear_speed.second;
}

std::pair<float, float> Robot::get_linear_speed() {
    return this->__motion_specs.__linear_speed;
}

float Robot::get_angular_speed() {
    return this->__motion_specs.__angular_speed;
}

float Robot::get_min_lifting_speed() {
    return this->__gripper_specs.__lifting_speed.first;
}

float Robot::get_max_lifting_speed() {
    return this->__gripper_specs.__lifting_speed.second;
}

std::pair<float, float> Robot::get_lifting_speed() {
    return this->__gripper_specs.__lifting_speed;
}

float Robot::get_min_descending_speed() {
    return this->__gripper_specs.__descending_speed.first;
}

float Robot::get_max_descending_speed() {
    return this->__gripper_specs.__descending_speed.second;
}

std::pair<float, float> Robot::get_descending_speed() {
    return this->__gripper_specs.__descending_speed;
}

uint16_t Robot::get_max_lifting_height() {
    return this->__gripper_specs.__max_lifting_height;
}

float Robot::get_battery_load() {
    return this->__battery_specs.__load;
}

float Robot::get_min_battery_runtime() {
    return this->__battery_specs.__runtime_interval.first;
}

float Robot::get_max_battery_runtime() {
    return this->__battery_specs.__runtime_interval.second;
}

std::pair<float, float> Robot::get_battery_runtime() {
    return this->__battery_specs.__runtime_interval;
}

float Robot::get_battery_recharge_time() {
    return this->__battery_specs.__recharge_time;
}

uint16_t Robot::get_battery_life_cycles() {
    return this->__battery_specs.__life_cycles;
}

uint8_t Robot::get_min_working_temperature() {
    return this->__ambient_specs.__temperature_interval.first;
}

uint8_t Robot::get_max_working_temperature() {
    return this->__ambient_specs.__temperature_interval.second;
}

std::pair<uint8_t, uint8_t> Robot::get_working_temperature() {
    return this->__ambient_specs.__temperature_interval;
}

float Robot::get_min_working_humidity() {
    return this->__ambient_specs.__humidity_interval.first;
}

float Robot::get_max_working_humidity() {
    return this->__ambient_specs.__humidity_interval.second;
}

std::pair<float, float> Robot::get_working_humidity() {
    return this->__ambient_specs.__humidity_interval;
}

int Robot::get_n_sensors() {
    return (int)this->__sensors.__2d_cameras +
            (int)this->__sensors.__3d_cameras +
            (int)this->__sensors.__bumpers +
            (int)this->__sensors.__laser_scanners +
            (int)this->__sensors.__proximity_sensors +
            (int)this->__sensors.__rear_sonar +
            (int)this->__sensors.__speaker;
}

bool Robot::has_laser_scanners() {
    return this->__sensors.__laser_scanners;
}

bool Robot::has_proximity_sensors() {
    return this->__sensors.__proximity_sensors;
}

bool Robot::has_3d_cameras() {
    return this->__sensors.__3d_cameras;
}

bool Robot::has_2d_cameras() {
    return this->__sensors.__2d_cameras;
}

bool Robot::has_rear_sonar() {
    return this->__sensors.__rear_sonar;
}

bool Robot::has_bumpers() {
    return this->__sensors.__bumpers;
}

bool Robot::has_speaker() {
    return this->__sensors.__speaker;
}

int Robot::get_n_tasks() {
    return this->__scheduling.__tasks_ref.size();
}

std::vector<Task*> Robot::get_tasks_ref() {
    return this->__scheduling.__tasks_ref;
}

Task* Robot::get_task_ref(int _addr) {
    return this->__scheduling.__tasks_ref[_addr];
}

float Robot::get_scheduling_cost() {
    return this->__scheduling.__cost;
}

int32_t Robot::get_current_dataset_capacity() {
    return this->__dyn_resources.__cur_capacity;
}

float Robot::get_current_battery_load() {
    return this->__dyn_resources.__cur_battery_load;
}

void Robot::show_task_list() {
    fprintf(stdout, "Robot #%d: <my_initial_position> ", this->get_id());
    for (int i = 0; i < this->__scheduling.__tasks_ref.size(); i++) {
        Task task_deref = *(this->__scheduling.__tasks_ref[i]);
        if (task_deref.get_task_type() == Task::pick)
            fprintf(stdout, "t%d ", task_deref.get_id());
        else if (task_deref.get_task_type() == Task::deliver)
            fprintf(stdout, "d%d ", task_deref.get_id());
        else if (task_deref.get_task_type() == Task::recharge)
            fprintf(stdout, "x%d ", task_deref.get_id());
    }
    fprintf(stdout, "(%.2f)\n", this->__scheduling.__cost);
}

void Robot::show_me() {
    fprintf(stdout, "Robot %d:\n", this->__id);
    fprintf(stdout, "\tNAME: %s\n", this->__name.c_str());
    fprintf(stdout, "\tTYPE: %s\n", this->__general_specs.__type.c_str());
    fprintf(stdout, "\tPOSSIBLE USE-CASES: ");
    for (int i = 0; i < this->__general_specs.__use_cases.size(); i++) {
        fprintf(stdout, "%s ", this->__general_specs.__use_cases[i].c_str());
    }
    fprintf(stdout, "\n");
    fprintf(stdout, "\tDESCRIPTION: %s\n", this->__general_specs.__description.c_str());
    fprintf(stdout, "\tLOCATION: (x = %d, y = %d)\n", this->get_x(), this->get_y());
    (std::get<0>(this->__general_specs.__dimensions) != 0.0) ? fprintf(stdout, "\tLENGTH: %.2f centimeters\n", std::get<0>(this->__general_specs.__dimensions)) : fprintf(stdout, "\tLENGTH: N/A\n");
    (std::get<1>(this->__general_specs.__dimensions) != 0.0) ? fprintf(stdout, "\tWIDTH: %.2f centimeters\n", std::get<1>(this->__general_specs.__dimensions)) : fprintf(stdout, "\tWIDTH: N/A\n");
    (std::get<2>(this->__general_specs.__dimensions) != 0.0) ? fprintf(stdout, "\tHEIGHT: %.2f centimeters\n", std::get<2>(this->__general_specs.__dimensions)) : fprintf(stdout, "\tHEIGHT: N/A\n");
    (this->__general_specs.__weight != 0.0) ? fprintf(stdout, "\tWEIGHT: %.2f kilograms\n", this->__general_specs.__weight) : fprintf(stdout, "\tWEIGHT: N/A\n");
    fprintf(stdout, "\tLOAD CAPACITY (REAL): %d kilograms\n", this->__load_specs.__real_capacity);
    fprintf(stdout, "\tLOAD CAPACITY (ADAPTED FOR DATASET): %d u.m\n", this->__load_specs.__dataset_capacity);
    (this->__load_specs.__load_surface != 0.0) ? fprintf(stdout, "\tLOAD SURFACE: %.2f square centimeters\n", this->__load_specs.__load_surface) : fprintf(stdout, "\tLOAD SURFACE: N/A\n");
    fprintf(stdout, "\tLINEAR SPEED: from %.2f m/s (empty) to %.2f m/s (loaded)\n", this->__motion_specs.__linear_speed.first, this->__motion_specs.__linear_speed.second);
    (this->__motion_specs.__angular_speed != 0.0) ? fprintf(stdout, "\tANGULAR SPEED: %.2f rad/s\n", this->__motion_specs.__angular_speed) : fprintf(stdout, "\tANGULAR SPEED: N/A\n");
    (this->__gripper_specs.__lifting_speed.first != 0.0) ? fprintf(stdout, "\tLIFTING SPEED: from %.2f (empty) to %.2f (loaded) m/s\n", this->__gripper_specs.__lifting_speed.first, this->__gripper_specs.__lifting_speed.second) : fprintf(stdout, "\tLIFTING SPEED: N/A\n");
    (this->__gripper_specs.__descending_speed.first != 0.0) ? fprintf(stdout, "\tDESCENDING SPEED: from %.2f (empty) to %.2f (loaded) m/s\n", this->__gripper_specs.__descending_speed.first, this->__gripper_specs.__descending_speed.second) : fprintf(stdout, "\tDESCENDING SPEED: N/A\n");
    (this->__gripper_specs.__max_lifting_height != 0.0) ? fprintf(stdout, "\tMAX LIFTING HEIGHT: %d centimeters\n", this->__gripper_specs.__max_lifting_height) : fprintf(stdout, "\tMAX LIFTING HEIGHT: N/A\n");
    (this->__battery_specs.__load != 0.0) ? fprintf(stdout, "\tBATTERY LOAD: %.2f Ah\n", this->__battery_specs.__load) : fprintf(stdout, "\tBATTERY LOAD: N/A\n");
    (this->__battery_specs.__runtime_interval.first != 0.0) ? fprintf(stdout, "\tBATTERY RUNTIME: from %.2f h to %.2f h\n", this->__battery_specs.__runtime_interval.first, this->__battery_specs.__runtime_interval.second) : fprintf(stdout, "\tBATTERY RUNTIME: N/A\n");
    (this->__battery_specs.__recharge_time != 0.0) ? fprintf(stdout, "\tBATTERY RECHARGE TIME: %.2f Ah\n", this->__battery_specs.__recharge_time) : fprintf(stdout, "\tBATTERY RECHARGE TIME: N/A\n");
    (this->__battery_specs.__life_cycles != 0) ? fprintf(stdout, "\tBATTERY LIFE CYCLES: %d\n", this->__battery_specs.__life_cycles) : fprintf(stdout, "\tBATTERY LIFE CYCLES: N/A\n");
    (this->__ambient_specs.__temperature_interval.first != 0) ? fprintf(stdout, "\tAMBIENT TEMPERATURE: from %d ºC to %d ºC\n", this->__ambient_specs.__temperature_interval.first, this->__ambient_specs.__temperature_interval.second) : fprintf(stdout, "\tAMBIENT TEMPERATURE: N/A\n");
    (this->__ambient_specs.__humidity_interval.first != 0) ? fprintf(stdout, "\tAMBIENT HUMIDITY: from %.2f% to %.2f%\n", this->__ambient_specs.__humidity_interval.first, this->__ambient_specs.__humidity_interval.second) : fprintf(stdout, "\tAMBIENT HUMIDITY: N/A\n");
    fprintf(stdout, "\tSENSORS: ");
    if (this->get_n_sensors()) {
        if (this->__sensors.__laser_scanners)
            fprintf(stdout, "Laser_scanners ");
        if (this->__sensors.__proximity_sensors)
            fprintf(stdout, "Proximity_sensors ");
        if (this->__sensors.__3d_cameras)
            fprintf(stdout, "3D_Cameras ");
        if (this->__sensors.__2d_cameras)
            fprintf(stdout, "2D_Cameras ");
        if (this->__sensors.__rear_sonar)
            fprintf(stdout, "Rear_sonar ");
        if (this->__sensors.__bumpers)
            fprintf(stdout, "Bumpers ");
        if (this->__sensors.__speaker)
            fprintf(stdout, "Speaker ");
        fprintf(stdout, "\n");
    }
    else
        fprintf(stdout, "N/A\n");
    fprintf(stdout, "\tN TASKS: %d\n", this->get_n_tasks());
}

bool Robot::is_it_here(std::vector<Robot*> _robots, Robot* _rbt) {
    for (auto it = std::begin(_robots); it != std::end(_robots); ++it) {
        if ((*it)->get_id() == _rbt->get_id()) {
            return true;
        }
    }
    return false;
}

void Robot::show_robots(std::vector<Robot*> _robots) {
    for (auto it = std::begin(_robots); it != std::end(_robots); ++it) {
        (*it)->show_me();
    }
    fprintf(stdout, "\n");
}

Robot::~Robot() { }