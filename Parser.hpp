/* 
 * File:   Parser.hpp
 * Author: geoso
 *
 * Created on 17 de Fevereiro de 2021, 19:37
 */

#ifndef PARSER_HPP
#define	PARSER_HPP

#include <fstream>
#include <cstdlib>
#include <memory>
#include <string>
#include <string.h>
#include <tuple>
#include <limits.h>
#include <string>
#include <fstream>
#include <math.h>
#include "RSE.hpp"
#include "PickTask.hpp"
#include "DeliverTask.hpp"
#include "RechargeTask.hpp"
#include "Robot.hpp"
#include "CostEstimator.hpp"
#include "System.hpp"

#define STREAM_SIZE 512

namespace RSE {
    class Parser {
    public:
        Parser();
        //static std::tuple< std::vector<PickTask*>*, std::vector<DeliverTask*>*, std::vector<Robot*>*, CostEstimator::costing_t >
        static std::tuple<std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >, std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > >,
        std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > >, std::shared_ptr<std::vector<std::shared_ptr<Robot> > >,
        CostEstimator::costing_t, std::string>
               parse_file(std::string);
        static void parse_robot_file(std::string, std::shared_ptr<Robot>&);
    private:

    };
}

#endif	/* PARSER_HPP */

