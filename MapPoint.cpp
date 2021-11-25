/* 
 * File:   MapPoint.cpp
 * Author: geoso
 * 
 * Created on 17 de Fevereiro de 2021, 12:40
 */

#include "MapPoint.hpp"

MapPoint::MapPoint() {
    this->__entity_ref = NULL;
}

Entity* MapPoint::get_entity_ref() {
    return this->__entity_ref;
}

Entity::entity_t MapPoint::get_entity_type() {
    return this->__entity_ref->get_entity_type();
}

std::string MapPoint::get_streamof_entity_type() {
    return this->__entity_ref->get_streamof_entity_type();
}

float MapPoint::get_connection_latency() {
    return this->__connection_latency;
}

float MapPoint::get_humidity() {
    return this->__humidity;
}

float MapPoint::get_luminosity() {
    return this->__luminosity;
}

float MapPoint::get_pressure() {
    return this->__pressure;
}

float MapPoint::get_temperature() {
    return this->__temperature;
}

bool MapPoint::has_fire() {
    return this->__fire;
}

bool MapPoint::has_smoke() {
    return this->__smoke;
}

void MapPoint::set_entity(Entity _entity) {
    this->__entity_ref = &(_entity);
}

void MapPoint::set_connection_latency(float _conn_lcy) {
    this->__connection_latency = _conn_lcy;
}

void MapPoint::set_fire(bool _fire) {
    this->__fire = _fire;
}

void MapPoint::set_humidity(float _humi) {
    this->__humidity = _humi;
}

void MapPoint::set_luminosity(float _lumi) {
    this->__luminosity = _lumi;
}

void MapPoint::set_pressure(float _press) {
    this->__pressure = _press;
}

void MapPoint::set_smoke(bool _smoke) {
    this->__smoke = _smoke;
}

void MapPoint::set_temperature(float _temp) {
    this->__temperature = _temp;
}

void MapPoint::remove_entity() {
    this->__entity_ref = NULL;
}

MapPoint::~MapPoint() {
    this->__entity_ref = NULL;
}

