/* 
 * File:   Order.hpp
 * Author: geoso
 *
 * Created on 16 de Fevereiro de 2021, 22:35
 */

#ifndef ORDER_HPP
#define	ORDER_HPP

#include <vector>
#include "Product.hpp"
#include "RSE.hpp"
#include "System.hpp"

class Order {
public:
    Order() = delete;
    Order(std::vector<Product>);
    Order(Order*);
    void set_completion_time();
    double get_duration();
    ~Order();
private:
    System::system_time_t __creation_time;
    System::system_time_t __completion_time;
    std::vector<Product*> __products_ref;
};

#endif	/* ORDER_HPP */

