/* 
 * File:   Map.hpp
 * Author: geoso
 *
 * Created on 17 de Fevereiro de 2021, 13:16
 */

#ifndef MAP_HPP
#define	MAP_HPP

#include <exception>
#include "MapPoint.hpp"

class Map {
public:
    Map() = delete;
    Map(uint32_t _x_start, uint32_t _y_start, uint32_t _length, uint32_t _width, uint32_t _scale);
    MapPoint get_mappoint(uint16_t _x, uint16_t _y);
    MapPoint get_mappoint(std::pair<uint16_t, uint16_t>);
    MapPoint* get_mappoint_ref(uint16_t _x, uint16_t _y);
    MapPoint* get_mappoint_ref(std::pair<uint16_t, uint16_t>);
    uint32_t get_length();
    uint32_t get_width();
    uint32_t get_scale();
    uint32_t get_x_startpoint();
    uint32_t get_y_startpoint();
    Entity* get_entity_at(uint16_t _x, uint16_t _y);
    Entity* get_entity_at(std::pair<uint16_t, uint16_t>);
    Entity::entity_t get_entity_type_at(uint16_t _x, uint16_t _y);
    Entity::entity_t get_entity_type_at(std::pair<uint16_t, uint16_t>);
    std::string get_streamof_entity_type_at(uint16_t _x, uint16_t _y);
    std::string get_streamof_entity_type_at(std::pair<uint16_t, uint16_t>);
    float get_connection_latency_at(uint16_t _x, uint16_t _y);
    float get_connection_latency_at(std::pair<uint16_t, uint16_t>);
    float get_humidity_at(uint16_t _x, uint16_t _y);
    float get_humidity_at(std::pair<uint16_t, uint16_t>);
    float get_luminosity_at(uint16_t _x, uint16_t _y);
    float get_luminosity_at(std::pair<uint16_t, uint16_t>);
    float get_pressure_at(uint16_t _x, uint16_t _y);
    float get_pressure_at(std::pair<uint16_t, uint16_t>);
    float get_temperature_at(uint16_t _x, uint16_t _y);
    float get_temperature_at(std::pair<uint16_t, uint16_t>);
    bool has_fire_at(uint16_t _x, uint16_t _y);
    bool has_fire_at(std::pair<uint16_t, uint16_t>);
    bool has_smoke_at(uint16_t _x, uint16_t _y);
    bool has_smoke_at(std::pair<uint16_t, uint16_t>);
    void set_mappoint_at(MapPoint, uint16_t _x, uint16_t _y);
    void set_mappoint_at(MapPoint, std::pair<uint16_t, uint16_t>);
    void set_entity_at(Entity, uint16_t _x, uint16_t _y);
    void set_entity_at(Entity, std::pair<uint16_t, uint16_t>);
    void set_connection_latency_at(float, uint16_t _x, uint16_t _y);
    void set_connection_latency_at(float, std::pair<uint16_t, uint16_t>);
    void set_fire_at(bool, uint16_t _x, uint16_t _y);
    void set_fire_at(bool, std::pair<uint16_t, uint16_t>);
    void set_humidity_at(float, uint16_t _x, uint16_t _y);
    void set_humidity_at(float, std::pair<uint16_t, uint16_t>);
    void set_luminosity_at(float, uint16_t _x, uint16_t _y);
    void set_luminosity_at(float, std::pair<uint16_t, uint16_t>);
    void set_pressure_at(float, uint16_t _x, uint16_t _y);
    void set_pressure_at(float, std::pair<uint16_t, uint16_t>);
    void set_smoke_at(bool, uint16_t _x, uint16_t _y);
    void set_smoke_at(bool, std::pair<uint16_t, uint16_t>);
    void set_temperature_at(float, uint16_t _x, uint16_t _y);
    void set_temperature_at(float, std::pair<uint16_t, uint16_t>);
    void remove_entity_from(uint16_t _x, uint16_t _y);
    void remove_entity_from(std::pair<uint16_t, uint16_t>);
    ~Map();
private:
    MapPoint **__map;
    uint32_t __map_length;
    uint32_t __map_width;
    uint32_t __map_scale;
    uint32_t __x_startpoint;
    uint32_t __y_startpoint;
};

#endif	/* MAP_HPP */

