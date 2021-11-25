/* 
 * File:   Product.cpp
 * Author: geoso
 * 
 * Created on 16 de Fevereiro de 2021, 22:15
 */

#include "Product.hpp"

Product::Product() { }

Product::Product(Product*_prodct) {
    this->set_entity_type(_prodct->__entity_type);
    this->set_position(_prodct->__position);
    this->__name = _prodct->__name;
    this->__weight = _prodct->__weight;
}

std::string Product::get_name() {
    return this->__name;
}

uint32_t Product::get_weight() {
    return this->__weight;
}

void Product::set_name(std::string _name) {
    this->__name = _name;
}

void Product::set_weight(uint32_t _weight) {
    this->__weight = _weight;
}

Product::~Product() { }
