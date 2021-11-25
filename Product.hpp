/* 
 * File:   Product.hpp
 * Author: geoso
 *
 * Created on 16 de Fevereiro de 2021, 22:15
 */

#ifndef PRODUCT_HPP
#define	PRODUCT_HPP

#include "Entity.hpp"

class Product : public Entity {
public:
    Product();
    Product(Product*);
    std::string get_name();
    uint32_t get_weight();
    void set_name(std::string);
    void set_weight(uint32_t);
    ~Product();
private:
    std::string __name;
    uint32_t __weight;
};

#endif	/* PRODUCT_HPP */

