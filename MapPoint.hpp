/* 
 * File:   MapPoint.hpp
 * Author: geoso
 *
 * Created on 17 de Fevereiro de 2021, 12:40
 */

#ifndef MAPPOINT_HPP
#define	MAPPOINT_HPP

#include "PickTask.hpp"
#include "DeliverTask.hpp"
#include "RechargeTask.hpp"
#include "Product.hpp"
#include "Robot.hpp"
#include "Obstacle.hpp"

class MapPoint {
public:
    MapPoint();
    Entity* get_entity_ref();
    Entity::entity_t get_entity_type();
    std::string get_streamof_entity_type();
    float get_connection_latency();
    float get_humidity();
    float get_luminosity();
    float get_pressure();
    float get_temperature();
    bool has_fire();
    bool has_smoke();
    void set_entity(Entity);
    void set_connection_latency(float);
    void set_fire(bool);
    void set_humidity(float);
    void set_luminosity(float);
    void set_pressure(float);
    void set_smoke(bool);
    void set_temperature(float);
    void remove_entity();
    ~MapPoint();
private:
    Entity *__entity_ref;
    float __connection_latency;
    float __humidity;
    float __luminosity;
    float __pressure;
    float __temperature;
    bool __fire;
    bool __smoke;
};

#endif	/* MAPPOINT_HPP */

