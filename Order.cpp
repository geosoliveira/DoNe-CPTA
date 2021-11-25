/* 
 * File:   Order.cpp
 * Author: geoso
 * 
 * Created on 16 de Fevereiro de 2021, 22:35
 */

#include "Order.hpp"

Order::Order(std::vector<Product> _products) {
    this->__creation_time = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < _products.size(); i++)
        this->__products_ref.push_back(&(_products[i]));
}

Order::Order(Order* _order) {
    this->__completion_time = _order->__completion_time;
    this->__creation_time = _order->__creation_time;
    for (auto it = std::begin(_order->__products_ref); it != std::end(_order->__products_ref); ++it) {
        this->__products_ref.push_back(new Product(*it));
    }
}

void Order::set_completion_time() {
    this->__completion_time = std::chrono::high_resolution_clock::now();
}

double Order::get_duration() {
    std::chrono::duration<double> diff = this->__completion_time - this->__creation_time;
    return diff.count();
}

Order::~Order() {
    RSE::destroy_all_elements_from(this->__products_ref);
}

