/* 
 * File:   TSP.hpp
 * Author: geoso
 *
 * Created on 11 de Março de 2021, 20:07
 */

#ifndef TSP_HPP
#define	TSP_HPP

#include <algorithm>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <queue>
#include <stack>
#include <stdio.h>
#include "CostEstimator.hpp"

class TSP {
private:
    struct City{ int x; int y; };
    std::vector<int>odds; // List of odd nodes
    int **cost; //Smaller cost matrix to find minimum matching on odd nodes
    std::vector<int> *adjList; //Adjacency list
    //std::vector<Task*> __tasks_ref;
    std::vector<std::shared_ptr<Task> > __tasks_v2;
    //Robot* __robot_ref;
    std::shared_ptr<Robot> __robot_ref_v2;
    std::shared_ptr<CostEstimator> __ce_ref;
    void findOdds();
public:
    int n; // Number of cities
    int **path_vals; // path
    int pathLength; //Shortest path length
    std::vector<int> circuit; //euler circuit
    std::vector<City> cities;
    std::vector<int> __indexes;
    int **graph; // n x n, pairwise distances between cities
    std::vector<int>* adjlist;
    TSP() = delete;
    //TSP(std::vector<PickTask*>, Robot*, CostEstimator*); // Constructor
    TSP(std::vector<std::shared_ptr<PickTask> >, std::shared_ptr<Robot>, std::shared_ptr<CostEstimator>); // Constructor
    ~TSP(); // Destructor
    void perfectMatching(); //Find perfect matching
    //std::tuple<std::vector<Task*>, float> final_circuit();
    std::tuple<std::vector<std::shared_ptr<Task> >, float> final_circuit_v2();
    void euler_tour(int start, std::vector<int> &path); //Find Euler tour
    void make_hamiltonian(std::vector<int> &path, int &pathCost); //Find Hamiltonian path
    void findMST(); // Prim's algorithm
    int getMinIndex(int key[], bool mst[]);
    void printPath();
    void printEuler();
    void printAdjList();
    void printCities();
    void printGraph();
    int get_size(){return n;};
    int get_distance(struct TSP::City c1, struct TSP::City c2);
    void fillMatrix();
    int findBestPath(int start);
};

#endif	/* TSP_HPP */

