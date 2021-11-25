/* 
 * File:   TSP.cpp
 * Author: geoso
 * 
 * Created on 11 de Março de 2021, 20:07
 */

#include "TSP.hpp"

//Constructor
/*TSP::TSP(std::vector<PickTask*> _pick_tasks, Robot* _robot, CostEstimator* _ce) {
#ifdef DEBUG
    fprintf(stdout, "\nGETTING IN: TSP::TSP(std::vector<PickTask*> _pick_tasks, CostEstimator* _ce) [%s:%d]\n", __FILE__, __LINE__);
#endif
    this->__ce_ref = _ce;
    this->__robot_ref = _robot;
    // O robô também entra -- criando uma tarefa a partir de informações válidas
    this->__tasks_ref.push_back(new PickTask(0, _robot->get_position(), Entity::task, Task::pick, PickTask::unused));
    for (auto it = std::begin(_pick_tasks); it != std::end(_pick_tasks); ++it) {
        this->__tasks_ref.push_back(*it);
    }
	
#ifdef DEBUG
    std::cout << "\tReading data... ";
#endif
    int count = 0;
    for (count = 0; count < this->__tasks_ref.size(); count++) {
            struct City newCity = {this->__tasks_ref[count]->get_x(), this->__tasks_ref[count]->get_y()};
            cities.push_back(newCity);
            this->__indexes.push_back(count);
    }
#ifdef DEBUG
    std::cout << "done!" << std::endl;
#endif

#ifdef DEBUG
    std::cout << "\tInitializing member variables... ";
#endif
    n = count;
    graph = new int*[n];
    for(int i = 0; i < n; i++){
            graph[i] = new int[n];
            for(int j = 0; j < n; j++){
                    graph[i][j] = 0.0;
            }
    }

    cost = new int*[n];
    for(int i = 0; i < n; i++){
            cost[i] = new int[n];
    }

    path_vals = new int*[n];
    for(int i = 0; i < n; i++){
            path_vals[i] = new int[n];
    }

    adjlist = new std::vector<int>[n];
    for(int i = 0; i < cities.size(); i++){
            struct City cur = cities[i];
    }
#ifdef DEBUG
    std::cout << "done!" << std::endl;
#endif
#ifdef DEBUG
    fprintf(stdout, "LEAVING OF: TSP::TSP(std::vector<PickTask*> _pick_tasks, CostEstimator* _ce) [%s:%d]\n", __FILE__, __LINE__);
#endif
}*/

//Constructor
TSP::TSP(std::vector<std::shared_ptr<PickTask> > _pick_tasks, std::shared_ptr<Robot> _robot_ref, std::shared_ptr<CostEstimator> _ce_ref) {
#ifdef DEBUG
    fprintf(stdout, "\nGETTING IN: TSP::TSP(std::vector<PickTask*> _pick_tasks, CostEstimator* _ce) [%s:%d]\n", __FILE__, __LINE__);
#endif
    this->__ce_ref = _ce_ref;
    this->__robot_ref_v2 = _robot_ref;
    // O robô também entra -- criando uma tarefa a partir de informações válidas
    std::shared_ptr<PickTask> cast_task(new PickTask(0, _robot_ref->get_position(), Entity::task, Task::pick, PickTask::unused));
    this->__tasks_v2.push_back(cast_task);
    for (auto it = std::begin(_pick_tasks); it != std::end(_pick_tasks); ++it) {
        this->__tasks_v2.push_back(*it);
    }
	
#ifdef DEBUG
    std::cout << "\tReading data... ";
#endif
    int count = 0;
    for (count = 0; count < this->__tasks_v2.size(); count++) {
            struct City newCity = {this->__tasks_v2[count]->get_x(), this->__tasks_v2[count]->get_y()};
            cities.push_back(newCity);
            this->__indexes.push_back(count);
    }
#ifdef DEBUG
    std::cout << "done!" << std::endl;
#endif

#ifdef DEBUG
    std::cout << "\tInitializing member variables... ";
#endif
    n = count;
    graph = new int*[n];
    for(int i = 0; i < n; i++){
            graph[i] = new int[n];
            for(int j = 0; j < n; j++){
                    graph[i][j] = 0.0;
            }
    }

    cost = new int*[n];
    for(int i = 0; i < n; i++){
            cost[i] = new int[n];
    }

    path_vals = new int*[n];
    for(int i = 0; i < n; i++){
            path_vals[i] = new int[n];
    }

    adjlist = new std::vector<int>[n];
    for(int i = 0; i < cities.size(); i++){
            struct City cur = cities[i];
    }
#ifdef DEBUG
    std::cout << "done!" << std::endl;
#endif
#ifdef DEBUG
    fprintf(stdout, "LEAVING OF: TSP::TSP(std::vector<PickTask*> _pick_tasks, CostEstimator* _ce) [%s:%d]\n", __FILE__, __LINE__);
#endif
}

int TSP::get_distance(struct TSP::City c1, struct TSP::City c2) {
    return abs(c1.x - c2.x) + abs(c1.y - c2.y);
	/*int dx = pow((float)(c1.x - c2.x),2);
	int dy = pow((float)(c1.y - c2.y),2);
	return floor((float)(sqrt(dx+dy) + .5));*/
}

void TSP::fillMatrix(){
    for(int i = 0; i < n; i++){
        std::shared_ptr<Entity> entty1(new Entity(cities[i].x, cities[i].y, Entity::task));
        for(int j = 0; j < n; j++){
            std::shared_ptr<Entity> entty2(new Entity(cities[j].x, cities[j].y, Entity::task));
            graph[i][j] = graph[j][i] = (int)round(this->__ce_ref->get_cost_v2(entty1, entty2, this->__robot_ref_v2));
            entty2.reset();
            //graph[i][j] = graph[j][i] = get_distance(cities[i],cities[j]);
        }
        entty1.reset();
    }
}


/******************************************************************************
  This function uses Prim's algorithm to determine a minimum spanning tree on
    the graph
******************************************************************************/

void TSP::findMST() {

  int *key = new int[n];
  bool *included = new bool[n];
  int *parent = new int[n];

  for (int i = 0; i < n; i++) {

    // set each key to infinity
    key[i] = std::numeric_limits<int>::max();

    // node node yet included in MST
    included[i] = false;

  }

  // root of MST has distance of 0 and no parent
  key[0] = 0;
  parent[0] = -1;

  for (int i = 0; i < n - 1; i++) {

    // find closes vertex not already in tree
    int k = getMinIndex(key, included);

    // set included to true for this vertex
    included[k] = true;

    // examine each unexamined vertex adjacent to most recently added
    for (int j = 0; j < n; j++) {

      // node exists, is unexamined, and graph[k][j] less than previous
      // key for u
      if (graph[k][j] && included[j] == false && graph[k][j] < key[j]) {

          // update parent
          parent[j] = k;

          // update key
          key[j] = graph[k][j];

      }
    }

  }

  // construct a tree by forming adjacency matrices
  for (int i = 0; i < n; i++) {

    int j = parent[i];

    if (j != -1) {

      adjlist[i].push_back(j);
      adjlist[j].push_back(i);

    }

  }

}


/******************************************************************************
  find the index of the closest unexamined node
******************************************************************************/

int TSP::getMinIndex(int key[], bool mst[]) {

  // initialize min and min_index
  float min = std::numeric_limits<float>::max();
  int min_index;

  // iterate through each vertex
  for (int i = 0; i < n; i++) {

    // if vertex hasn't been visited and has a smaller key than min
    if (mst[i] == false && key[i] < min) {

      // reassign min and min_index to the values from this node
      min = key[i];
      min_index = i;

    }

  }

  return min_index;

}


/******************************************************************************
  find all vertices of odd degree in the MST. Store them in an subgraph O
******************************************************************************/

void TSP::findOdds() {

  for (int i = 0; i < n; i++) {

    // if degree of vertex i is odd
    if ((adjlist[i].size() % 2) != 0) {

      // push vertex to odds, which is a representation of subgraph O
      odds.push_back(i);

    }

  }

}


void TSP::perfectMatching() {
  /************************************************************************************
   find a perfect matching M in the subgraph O using greedy algorithm but not minimum
  *************************************************************************************/
  int closest, length; //int d;
  std::vector<int>::iterator tmp, first;

  // Find nodes with odd degrees in T to get subgraph O
  findOdds();

  // for each odd node
  while (!odds.empty()) {
    first = odds.begin();
    std::vector<int>::iterator it = odds.begin() + 1;
    std::vector<int>::iterator end = odds.end();
    length = std::numeric_limits<int>::max();
    for (; it != end; ++it) {
      // if this node is closer than the current closest, update closest and length
      if (graph[*first][*it] < length) {
        length = graph[*first][*it];
        closest = *it;
        tmp = it;
      }
    } // two nodes are matched, end of list reached
    adjlist[*first].push_back(closest);
    adjlist[closest].push_back(*first);
    odds.erase(tmp);
    odds.erase(first);
  }
}

/*std::tuple<std::vector<Task*>, float> TSP::final_circuit() {
    std::vector<Task*> tasks = (this->__tasks_ref);
    std::vector<int> indexes = (this->__indexes);
    std::tuple<std::vector<Task*>, float> ret_circuit;
    std::vector<Task*> final_path;
    
#ifdef DEBUG
    std::cout << "Tasks:" << std::endl;
    Task::show_tasks(tasks);
    /*std::cout << "Indexes (" << indexes.size() << "): ";
    for (int i = 0; indexes.size(); i++) {
        std::cout << indexes[i] << " ";
    }
    std::cout << std::endl;*
#endif
    
    int i, start;
    for (start = 0; start < this->circuit.size(); start++) {
        if (indexes[this->circuit[start]] == 0) break; // This is the robot
    }
    i = (start + 1 == this->circuit.size()) ? 0 : start + 1;
    do {
        final_path.push_back(tasks[indexes[this->circuit[i++]]]);
        if (i == this->circuit.size()) i = 0;
    } while (i != start);
    
    // Cost from robot to first start:
    float cost = this->__ce_ref->get_cost(tasks[indexes[this->circuit[start]]], final_path[0], this->__robot_ref);
#ifdef DEBUG
    std::cout << "Cost from robot to t" << final_path[0]->get_id() << ": " << cost << std::endl;
#endif
    
    // Cost from task to task:
    for (int i = 1; i < final_path.size(); i++) {
        cost += this->__ce_ref->get_cost(final_path[i-1], final_path[i], this->__robot_ref);
#ifdef DEBUG
        std::cout << "Cost from t" << final_path[i-1]->get_id() << " to t" << final_path[i]->get_id() << ": " << this->__ce_ref->get_cost(final_path[i], final_path[i-1], this->__robot_ref) << std::endl;
#endif
    }

    // Cost from last task to best depot
    auto depot_tuple = this->__ce_ref->closer_deliver(final_path[final_path.size()-1], this->__robot_ref);
#ifdef DEBUG
    std::cout << "Cost from t" << final_path[final_path.size()-1]->get_id() << " to d" << depot_tuple.first->get_id() << ": " << depot_tuple.second << std::endl;
#endif
    final_path.push_back(depot_tuple.first);
    cost += depot_tuple.second;
    
#ifdef DEBUG
    std::cout << "Final path:" << std::endl;
    Task::show_tasks(final_path);
    std::cout << "Cost:" << cost << std::endl<< std::endl << std::endl;
#endif
    
    return std::make_tuple(final_path, cost);
}*/

std::tuple<std::vector<std::shared_ptr<Task> >, float> TSP::final_circuit_v2() {
    std::vector<std::shared_ptr<Task> > tasks = (this->__tasks_v2);
    std::vector<int> indexes = (this->__indexes);
    std::vector<std::shared_ptr<Task> > final_path;
    
#ifdef DEBUG
    std::cout << "Tasks:" << std::endl;
    RSE::show<std::vector<std::shared_ptr<Task> > >(tasks);
    /*std::cout << "Indexes (" << indexes.size() << "): ";
    for (int i = 0; indexes.size(); i++) {
        std::cout << indexes[i] << " ";
    }
    std::cout << std::endl;*/
#endif
    
    int i, start;
    for (start = 0; start < this->circuit.size(); start++) {
        if (indexes[this->circuit[start]] == 0) break; // This is the robot
    }
    i = (start + 1 == this->circuit.size()) ? 0 : start + 1;
    do {
        final_path.push_back(tasks[indexes[this->circuit[i++]]]);
        if (i == this->circuit.size()) i = 0;
    } while (i != start);
    
    // Cost from robot to first start:
    float cost = this->__ce_ref->get_cost_v2(tasks[indexes[this->circuit[start]]], final_path[0], this->__robot_ref_v2);
#ifdef DEBUG
    std::cout << "Cost from robot to t" << final_path[0]->get_id() << ": " << cost << std::endl;
#endif
    
    // Cost from task to task:
    for (int i = 1; i < final_path.size(); i++) {
        cost += this->__ce_ref->get_cost_v2(final_path[i-1], final_path[i], this->__robot_ref_v2);
#ifdef DEBUG
        std::cout << "Cost from t" << final_path[i-1]->get_id() << " to t" << final_path[i]->get_id() << ": " << this->__ce_ref->get_cost_v2(final_path[i], final_path[i-1], this->__robot_ref_v2) << std::endl;
#endif
    }

    // Cost from last task to best depot
    auto depot_tuple = this->__ce_ref->closer_deliver_v2(final_path[final_path.size()-1], this->__robot_ref_v2);
#ifdef DEBUG
    std::cout << "Cost from t" << final_path[final_path.size()-1]->get_id() << " to d" << depot_tuple.first->get_id() << ": " << depot_tuple.second << std::endl;
#endif
    final_path.push_back(depot_tuple.first);
    cost += depot_tuple.second;
    
#ifdef DEBUG
    std::cout << "Final path:" << std::endl;
    RSE::show<std::vector<std::shared_ptr<Task> > >(final_path);
    std::cout << "Cost:" << cost << std::endl<< std::endl << std::endl;
#endif
    
    return std::make_tuple(final_path, cost);
}

//find an euler circuit
void TSP::euler_tour(int start, std::vector<int> &path){
	//Create copy of adj. list
	std::vector<int> *tempList = new std::vector<int>[n];
	for(int i = 0; i < n; i++){
		tempList[i].resize(adjlist[i].size());
		tempList[i] = adjlist[i];
	}

	std::stack<int> stack;
	int pos = start;
	path.push_back(start);
	while(!stack.empty() || tempList[pos].size() > 0){
		//Current node has no neighbors
		if(tempList[pos].empty()){
			//add to circuit
			path.push_back(pos);
			//remove last vertex from stack and set it to current
			pos = stack.top();
			stack.pop();
		}
		//If current node has neighbors
		else{
			//Add vertex to stack
			stack.push(pos);
			//Take a neighbor
			int neighbor = tempList[pos].back();
			//Remove edge between neighbor and current vertex
			tempList[pos].pop_back();
			for(int i = 0; i < tempList[neighbor].size(); i++){
				if(tempList[neighbor][i] == pos){
					tempList[neighbor].erase(tempList[neighbor].begin()+i);
				}
			}
			//Set neighbor as current vertex
			pos = neighbor;
		}
	}
	path.push_back(pos);
}


//Make euler tour Hamiltonian
void TSP::make_hamiltonian(std::vector<int> &path, int &pathCost){

	//remove visited nodes from Euler tour
	bool *visited = new bool[n];
	for(int i = 0; i < n; i++){
		visited[i] = 0;
	}
	
	pathCost = 0.0;

	int root = path.front();
	std::vector<int>::iterator cur = path.begin();
	std::vector<int>::iterator iter = path.begin()+1;
	visited[root] = 1;

	//iterate through circuit
	bool addMore = true;
	while(iter != path.end()){
		if(!visited[*iter]){
			pathCost += graph[*cur][*iter];
			cur = iter;
			visited[*cur] = 1;
			iter = cur + 1;
		}	
		else{
			iter = path.erase(iter);
		}
	}
	//Add distance to root
	//if ( iter != path.end() ){
		pathCost += graph[*cur][*iter];
	//}
}

int TSP::findBestPath(int start){
	std::vector<int> path;
	euler_tour(start, path);
	int length;

	make_hamiltonian(path, length);
	return length;
}

void TSP::printPath() {
  for (std::vector<int>::iterator it = circuit.begin(); it != circuit.end()-1; ++it) {
    std::cout << *it << " to " << *(it+1) << " ";
    std::cout << graph[*it][*(it+1)] << std::endl;
  }
  std::cout << *(circuit.end()-1) << " to " << circuit.front();
  std::cout << "\nLength: " << pathLength << std::endl << std::endl;
};

void TSP::printEuler() {
  for (std::vector<int>::iterator it = circuit.begin(); it != circuit.end(); ++it)
    std::cout << *it << std::endl;
}

void TSP::printAdjList() {
  for (int i = 0; i < n; i++) {
    std::cout << i << ": "; //print which vertex's edge list follows
    for (int j = 0; j < (int)adjlist[i].size(); j++) {
      std::cout << adjlist[i][j] << " "; //print each item in edge list
    }
    std::cout << std::endl;
  }
};

void TSP::printCities(){
  std::cout << std::endl;
  int i = 0;
  for (std::vector<City>::iterator it = cities.begin(); it != cities.end(); ++it)
    std::cout << i++ << ":  " << it->x << " " << it->y << std::endl;
}

void TSP::printGraph() {
    for (int i = 0; i < n; i++) {
        std::cout << "[ ";
        for (int j = 0; j < n; j++) {
            std::cout << graph[i][j] << " ";
        }
        std::cout << "] " << std::endl;
    }
}

//Destructor
TSP::~TSP(){
    for(int i = 0; i < n; i++){
        delete [] graph[i];
        delete [] cost[i];
        delete [] path_vals[i];
    }
    delete [] path_vals;
    delete [] graph;
    delete [] cost;
    delete [] adjlist;
    
    this->__tasks_v2.clear();
    this->__indexes.clear();
    this->__ce_ref.reset();
    this->__robot_ref_v2.reset();
}