#include <iostream>
#include <cmath>
#include <math.h>
#include <stdio.h>
#include <vector>
#include </Users/henrycappel/Downloads/eigen-eigen-323c052e1731/Eigen/Dense>
#include <map>
#include <string>
#include <utility>
#include <algorithm>
#include <fstream>
#include <ostream>
#include <list>
//#include "ros/ros.h"

using namespace std;
//using namespace Eigen;
#define THIS_COM "/Users/henrycappel/Documents/"

void saveVector(const std::vector<double>& _vec, std::string _name,
                bool b_param = false);
void saveValue(double _value, std::string _name, bool b_param = false);
void cleaningFile(std::string file_name_, std::string& ret_file_, bool b_param);
static std::list<std::string> gs_fileName_string;  // global & static

void saveVector(const std::vector<double>& _vec, std::string _name,
                bool b_param) {
    std::string file_name;
    cleaningFile(_name, file_name, b_param);
    std::ofstream savefile(file_name.c_str(), std::ios::app);
    for (int i(0); i < _vec.size(); ++i) {
        savefile << _vec[i] << "\t";
    }
    savefile << "\n";
    savefile.flush();
}

void cleaningFile(std::string _file_name, std::string& _ret_file,
                  bool b_param) {
    if (b_param)
        _ret_file += THIS_COM;
    else
        _ret_file += THIS_COM;

    _ret_file += _file_name;
    _ret_file += ".txt";

    std::list<std::string>::iterator iter = std::find(
        gs_fileName_string.begin(), gs_fileName_string.end(), _file_name);
    if (gs_fileName_string.end() == iter) {
        gs_fileName_string.push_back(_file_name);
        remove(_ret_file.c_str());
    }
}



class node {
	public:
		node(){

			visited = false;
			obstacle = false;
			GlobalGoal = 1000;
			LocalGoal = 1000;
			f_score = GlobalGoal + LocalGoal;
			start = false;



		};
			bool obstacle;
			bool visited;
			double GlobalGoal;
			double LocalGoal;
			double f_score;
			double x;
			double y;
			bool start;
			string parent;
			string key;

};

double LocalHeuristic(vector<double> current, vector<double> neighbor){
	double distance = sqrt(pow(current[0]-neighbor[0],2) + pow(current[1]-neighbor[1],2));
	return distance;
}


double GlobalHeuristic(vector<double> neighbor, vector<double> goal){
	double distance = sqrt(pow(neighbor[0]-goal[0],2) + pow(neighbor[1]-goal[1],2));
	return distance;
}



void Create_Nodes(double x_i,double y_i,double x_f,double y_f, std::map<std::string, node>& nodeMap, double current_local_score,node current_node)
{
	//create vectors of all possible x,y values for both RF and LF
	//based on init config
	vector<double> x_i_vals;
	vector<double> y_i_vals;

	//values of newly created x,y values for both LF and RF
	double x_i_new;
	double y_i_new;


	//vector of coordinates of goal stance
	vector<double> goal;
	goal.push_back(x_f);
	goal.push_back(y_f);


	//vector of coordinates of current stance
	vector<double> current_pos;
	current_pos.push_back(x_i);
	current_pos.push_back(y_i);

	vector< vector<double> > coords;
	// theta_vals = [0,45,90,135,180];
	int k = -3;
	//append all new x,y values for both LF and RF going down and left from original coordinates
	while (k <= 3){
		double temp_val = 0.5*k;
		x_i_new = x_i + temp_val;
		y_i_new = y_i + temp_val;

		x_i_vals.push_back(x_i_new);
		y_i_vals.push_back(y_i_new);
		k++;
	}

	//int n = 1;
	//append all new x,y values for both LF and RF going up and right from original coordinates
	//while (n <= 1){  
	//	double temp_val = 0.1*n;
	//	x_i_new = x_i + temp_val;
	//	y_i_new = y_i + temp_val;

	//	x_i_vals.push_back(x_i_new);
	//	y_i_vals.push_back(y_i_new);
	//	n++;
	//}

	//create all possible coordinates for LF
	for (size_t i(0);i < x_i_vals.size();i++){
		for (size_t j(0);j < y_i_vals.size();j++){
			vector<double> x_y_coord;
			// x_y_coord_LF.resize(3);
			x_y_coord.push_back(x_i_vals[i]);
			x_y_coord.push_back(y_i_vals[j]);
			saveVector(x_y_coord,"coords_simple");
			coords.push_back(x_y_coord);			
			
		}	
	}

	//create all possible nodes for RF still, LF step, increment s
	for (size_t i(0);i < coords.size();i++)
	{
		//if ( sqrt(pow(LF_coords[i][0] - x_i_LF,2)+pow(LF_coords[i][1] - y_i_LF,2)) > 0.2 && sqrt(pow(LF_coords[i][0] - x_i_LF,2)+pow(LF_coords[i][1] - y_i_LF,2)) < 0.5){
		node insert1;
		insert1.obstacle = false;
		insert1.visited = false;
		insert1.GlobalGoal = GlobalHeuristic(coords[i],goal);
		insert1.LocalGoal = LocalHeuristic(current_pos,coords[i]) + current_local_score;
		insert1.f_score = insert1.GlobalGoal + insert1.LocalGoal + 0.5;
		insert1.x = coords[i][0];
		insert1.y = coords[i][1];
		insert1.key = (to_string(insert1.x) + to_string(insert1.y));
		insert1.parent = current_node.key;
		if (nodeMap.count(insert1.key) == 0){
			nodeMap.insert(pair<string,node>(insert1.key,insert1));
		}
		else if (nodeMap.count(insert1.key) > 0){
			map<string,node>::iterator it;
			it = nodeMap.find(insert1.key);
			if (it != nodeMap.end()){
				if (insert1.f_score < it->second.f_score && it->second.visited == false){
					it->second.f_score = insert1.f_score;
					it->second.parent = insert1.parent;
				}	
			}
		}
	}
}




node SortMap(map<string,node> nodeMap){
	int i = 0;
	double min_score = 1000;
	string min_key;
	node optimized_node;
	map<string, node>::iterator map_iter = nodeMap.begin();
	cout<<"size of map: "<< nodeMap.size()<<endl;
	for(map_iter; map_iter != nodeMap.end();map_iter++){
		if (map_iter -> second.visited == false && map_iter -> second.f_score < min_score) {
			min_score = map_iter -> second.f_score;
			min_key = map_iter -> first;
			optimized_node = nodeMap[min_key];
		}
	}
	cout << min_key <<endl;
	return optimized_node;
}

vector<node> OptimalPath(node current_node,std::map<std::string, node> nodeMap){
	vector<node> optimal_path;
	optimal_path.push_back(current_node);
	while (current_node.start == false){
		optimal_path.push_back(nodeMap[current_node.parent]);
		current_node = nodeMap[current_node.parent];
	}
	return optimal_path;

}




int main(){
	double x_i = 0;
	double y_i = 0;

	double x_f = 10;
	double y_f = 20;

	vector<double> start;
	vector<double> goal_pos;

	start.push_back(x_i);
	start.push_back(y_i);
	goal_pos.push_back(x_f);
	goal_pos.push_back(y_f);

	std::map<std::string, node> nodeMap;

	node current_node;
	current_node.visited = true;
	current_node.GlobalGoal = 1000;
	current_node.LocalGoal = 1000;
	current_node.f_score = current_node.LocalGoal + current_node.GlobalGoal;
	current_node.x = x_i;
	current_node.y = y_i;
	current_node.start = true;
	current_node.key = (to_string(current_node.x) + to_string(current_node.y));

	nodeMap.insert(pair<string,node>(current_node.key,current_node));

	vector<double> optimal_path;


	double current_local_score = 0;
	int i = 0;
	Create_Nodes(x_i,y_i,x_f,y_f, nodeMap, current_local_score, current_node);
	//map<string, node>::iterator map_iter = nodeMap.begin();
	//for(map_iter; map_iter != nodeMap.end();map_iter++){
	//	cout << "LF_x: "<< map_iter->second.x_L << "LF_y: " << map_iter->second.y_L << "RF_x: " << map_iter->second.x_R << "RF_y: " << map_iter->second.y_R << "fscore: " << map_iter->second.f_score << endl;
	

	while (!nodeMap.empty()){
		node current_node = SortMap(nodeMap);
		nodeMap[current_node.key].visited = true;
		//cout << "current node key: " << current_node.key << endl;
		//cout << "current node visited pre: " << current_node.visited << endl;
		//cout << "current node visited post: " << current_node.visited << endl;
		cout << "x: " << current_node.x << " " << "y: " << current_node.y << endl;
		cout << "optimal fscore: " << current_node.f_score << endl;
		//if (i == 0){
		//	map<string, node>::iterator map_iter = nodeMap.begin();
		//	for(map_iter; map_iter != nodeMap.end();map_iter++){
		//		cout << map_iter->second.GlobalGoal << endl;
		//break;

		if (abs(current_node.x - x_f) < 0.1 && abs(current_node.y- y_f) < 0.1){

			cout << "we made it" << endl;
			vector<node> optimal_path = OptimalPath(current_node,nodeMap);
			cout << "optimal path size: " << optimal_path.size() << endl;
			for (size_t i(0);i < optimal_path.size();i++){
				cout << "optimal path" << " " << "x: " << optimal_path[i].x << "y: " << optimal_path[i].y << endl;
				vector<double> optimal_coord;
				optimal_coord.push_back(optimal_path[i].x);
				optimal_coord.push_back(optimal_path[i].y);
				saveVector(optimal_coord,"optimal_path");
			}
			break;
		}
			//cout << optimal_path[];
		else{
			cout << "else" << endl;
			current_local_score = current_node.LocalGoal;
			cout << "current Local Goal: " << current_local_score << endl;
			Create_Nodes(current_node.x,current_node.y,x_f,y_f, nodeMap, current_local_score, current_node);
			

		}
	
	}

	

	
		
	//for (size_t i(0);i<LF_coords.size();i++){
	//	cout << LF_coords[i][0] << ',' << LF_coords[i][1] << ',' << LF_coords[i][2] << endl;
	//}

	
}
	