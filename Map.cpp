/* 
 * File:   Map.cpp
 * Author: geoso
 * 
 * Created on 17 de Fevereiro de 2021, 13:16
 */

#include "Map.hpp"

Map::Map(uint32_t _x_start, uint32_t _y_start, uint32_t _length, uint32_t _width, uint32_t _scale) {
    this->__x_startpoint = _x_start;
    this->__y_startpoint = _y_start;
    this->__map_length = _length;
    this->__map_width = _width;
    this->__map_scale = _scale;
    this->__map = new MapPoint*[this->__map_length];
    for (int i = 0; i < this->__map_length; i++)
        this->__map[i] = new MapPoint[this->__map_width];
}

MapPoint Map::get_mappoint(uint16_t _x, uint16_t _y) {
    return this->__map[_x - this->__x_startpoint][_y - this->__y_startpoint];
}

MapPoint Map::get_mappoint(std::pair<uint16_t, uint16_t> _location) {
    return this->__map[_location.first - this->__x_startpoint][_location.second - this->__y_startpoint];
}

MapPoint* Map::get_mappoint_ref(uint16_t _x, uint16_t _y) {
    return &(this->__map[_x - this->__x_startpoint][_y - this->__y_startpoint]);
}

MapPoint* Map::get_mappoint_ref(std::pair<uint16_t, uint16_t> _location) {
    return &(this->__map[_location.first - this->__x_startpoint][_location.second - this->__y_startpoint]);
}

uint32_t Map::get_length() {
    return this->__map_length;
}

uint32_t Map::get_width() {
    return this->__map_width;
}

uint32_t Map::get_scale() {
    return this->__map_scale;
}

uint32_t Map::get_x_startpoint() {
    return this->__x_startpoint;
}

uint32_t Map::get_y_startpoint() {
    return this->__y_startpoint;
}

Entity* Map::get_entity_at(uint16_t _x, uint16_t _y) {
    return this->__map[_x - this->__x_startpoint][_y - this->__y_startpoint].get_entity_ref();
}

Entity* Map::get_entity_at(std::pair<uint16_t, uint16_t> _location) {
    return this->__map[_location.first - this->__x_startpoint][_location.second - this->__y_startpoint].get_entity_ref();
}

Entity::entity_t Map::get_entity_type_at(uint16_t _x, uint16_t _y) {
    return this->__map[_x - this->__x_startpoint][_y - this->__y_startpoint].get_entity_type();
}

Entity::entity_t Map::get_entity_type_at(std::pair<uint16_t, uint16_t> _location) {
    return this->__map[_location.first - this->__x_startpoint][_location.second - this->__y_startpoint].get_entity_type();
}

std::string Map::get_streamof_entity_type_at(uint16_t _x, uint16_t _y) {
    return this->__map[_x - this->__x_startpoint][_y - this->__y_startpoint].get_streamof_entity_type();
}

std::string Map::get_streamof_entity_type_at(std::pair<uint16_t, uint16_t> _location) {
    return this->__map[_location.first - this->__x_startpoint][_location.second - this->__y_startpoint].get_streamof_entity_type();
}

float Map::get_connection_latency_at(uint16_t _x, uint16_t _y) {
    return this->__map[_x - this->__x_startpoint][_y - this->__y_startpoint].get_connection_latency();
}

float Map::get_connection_latency_at(std::pair<uint16_t, uint16_t> _location) {
    return this->__map[_location.first - this->__x_startpoint][_location.second - this->__y_startpoint].get_connection_latency();
}

float Map::get_humidity_at(uint16_t _x, uint16_t _y) {
    return this->__map[_x - this->__x_startpoint][_y - this->__y_startpoint].get_humidity();
}

float Map::get_humidity_at(std::pair<uint16_t, uint16_t> _location) {
    return this->__map[_location.first - this->__x_startpoint][_location.second - this->__y_startpoint].get_humidity();
}

float Map::get_luminosity_at(uint16_t _x, uint16_t _y) {
    return this->__map[_x - this->__x_startpoint][_y - this->__y_startpoint].get_luminosity();
}

float Map::get_luminosity_at(std::pair<uint16_t, uint16_t> _location) {
    return this->__map[_location.first - this->__x_startpoint][_location.second - this->__y_startpoint].get_luminosity();
}

float Map::get_pressure_at(uint16_t _x, uint16_t _y) {
    return this->__map[_x - this->__x_startpoint][_y - this->__y_startpoint].get_pressure();
}

float Map::get_pressure_at(std::pair<uint16_t, uint16_t> _location) {
    return this->__map[_location.first - this->__x_startpoint][_location.second - this->__y_startpoint].get_pressure();
}

float Map::get_temperature_at(uint16_t _x, uint16_t _y) {
    return this->__map[_x - this->__x_startpoint][_y - this->__y_startpoint].get_temperature();
}

float Map::get_temperature_at(std::pair<uint16_t, uint16_t> _location) {
    return this->__map[_location.first - this->__x_startpoint][_location.second - this->__y_startpoint].get_temperature();
}

bool Map::has_fire_at(uint16_t _x, uint16_t _y) {
    return this->__map[_x - this->__x_startpoint][_y - this->__y_startpoint].has_fire();
}

bool Map::has_fire_at(std::pair<uint16_t, uint16_t> _location) {
    return this->__map[_location.first - this->__x_startpoint][_location.second - this->__y_startpoint].has_fire();
}

bool Map::has_smoke_at(uint16_t _x, uint16_t _y) {
    return this->__map[_x - this->__x_startpoint][_y - this->__y_startpoint].has_smoke();
}

bool Map::has_smoke_at(std::pair<uint16_t, uint16_t> _location) {
    return this->__map[_location.first - this->__x_startpoint][_location.second - this->__y_startpoint].has_smoke();
}

void Map::set_mappoint_at(MapPoint _mappoint, uint16_t _x, uint16_t _y) {
    this->__map[_x - this->__x_startpoint][_y - this->__y_startpoint] = _mappoint;
}

void Map::set_mappoint_at(MapPoint _mappoint, std::pair<uint16_t, uint16_t> _location) {
    this->__map[_location.first - this->__x_startpoint][_location.second - this->__y_startpoint] = _mappoint;
}

void Map::set_entity_at(Entity _entity, uint16_t _x, uint16_t _y) {
    this->__map[_x - this->__x_startpoint][_y - this->__y_startpoint].set_entity(_entity);
}

void Map::set_entity_at(Entity _entity, std::pair<uint16_t, uint16_t> _location) {
    this->__map[_location.first - this->__x_startpoint][_location.second - this->__y_startpoint].set_entity(_entity);
}

void Map::set_connection_latency_at(float _conn_lcy, uint16_t _x, uint16_t _y) {
    this->__map[_x - this->__x_startpoint][_y - this->__y_startpoint].set_connection_latency(_conn_lcy);
}

void Map::set_connection_latency_at(float _conn_lcy, std::pair<uint16_t, uint16_t> _location) {
    this->__map[_location.first - this->__x_startpoint][_location.second - this->__y_startpoint].set_connection_latency(_conn_lcy);
}

void Map::set_fire_at(bool _fire, uint16_t _x, uint16_t _y) {
    this->__map[_x - this->__x_startpoint][_y - this->__y_startpoint].set_fire(_fire);
}

void Map::set_fire_at(bool _fire, std::pair<uint16_t, uint16_t> _location) {
    this->__map[_location.first - this->__x_startpoint][_location.second - this->__y_startpoint].set_fire(_fire);
}

void Map::set_humidity_at(float _humi, uint16_t _x, uint16_t _y) {
    this->__map[_x - this->__x_startpoint][_y - this->__y_startpoint].set_humidity(_humi);
}

void Map::set_humidity_at(float _humi, std::pair<uint16_t, uint16_t> _location) {
    this->__map[_location.first - this->__x_startpoint][_location.second - this->__y_startpoint].set_humidity(_humi);
}

void Map::set_luminosity_at(float _lumi, uint16_t _x, uint16_t _y) {
    this->__map[_x - this->__x_startpoint][_y - this->__y_startpoint].set_luminosity(_lumi);
}

void Map::set_luminosity_at(float _lumi, std::pair<uint16_t, uint16_t> _location) {
    this->__map[_location.first - this->__x_startpoint][_location.second - this->__y_startpoint].set_luminosity(_lumi);
}

void Map::set_pressure_at(float _press, uint16_t _x, uint16_t _y) {
    this->__map[_x - this->__x_startpoint][_y - this->__y_startpoint].set_pressure(_press);
}

void Map::set_pressure_at(float _press, std::pair<uint16_t, uint16_t> _location) {
    this->__map[_location.first - this->__x_startpoint][_location.second - this->__y_startpoint].set_pressure(_press);
}

void Map::set_smoke_at(bool _smoke, uint16_t _x, uint16_t _y) {
    this->__map[_x - this->__x_startpoint][_y - this->__y_startpoint].set_smoke(_smoke);
}

void Map::set_smoke_at(bool _smoke, std::pair<uint16_t, uint16_t> _location) {
    this->__map[_location.first - this->__x_startpoint][_location.second - this->__y_startpoint].set_smoke(_smoke);
}

void Map::set_temperature_at(float _temp, uint16_t _x, uint16_t _y) {
    this->__map[_x - this->__x_startpoint][_y - this->__y_startpoint].set_temperature(_temp);
}

void Map::set_temperature_at(float _temp, std::pair<uint16_t, uint16_t> _location) {
    this->__map[_location.first - this->__x_startpoint][_location.second - this->__y_startpoint].set_temperature(_temp);
}

void Map::remove_entity_from(uint16_t _x, uint16_t _y) {
    this->__map[_x - this->__x_startpoint][_y - this->__y_startpoint].remove_entity();
}

void Map::remove_entity_from(std::pair<uint16_t, uint16_t> _location) {
    return this->__map[_location.first - this->__x_startpoint][_location.second - this->__y_startpoint].remove_entity();
}

Map::~Map() {
    try {
        for (int i = 0; i < this->__map_length; i++)
            delete[] this->__map[i];;
        delete[] this->__map;
    }
    catch (std::exception& e) {
        std::cerr << "[" << __FILE__ <<":" << __LINE__ << "] An exception was caught while trying to destroy Map::__map, with message: '" << e.what() << "'. Exiting..." << std::endl;
        exit(0x1576);
    }
}

