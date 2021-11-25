/* 
 * File:   System.cpp
 * Author: geoso
 * 
 * Created on 16 de Fevereiro de 2021, 21:17
 */

#include "System.hpp"

System::System() { }

float System::get_proximity_coefficient() {
    return this->__proximity_coefficient;
}

float System::get_weighting_coefficient() {
    return this->__weighting_coefficient;
}

void System::set_proximity_coefficient(float _pxt_cft) {
    this->__proximity_coefficient = _pxt_cft;
}

void System::set_weighting_coefficient(float _wgt_cft) {
    this->__weighting_coefficient = _wgt_cft;
}

System::~System() {}

