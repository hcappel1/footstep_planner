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
			LocalGoalLF = 1000;
			LocalGoalRF = 1000;
			f_score = GlobalGoal + LocalGoalLF + LocalGoalRF;
			start = false;



		};
			bool obstacle;
			bool visited;
			double GlobalGoal;
			double LocalGoalLF;
			double LocalGoalRF;
			double f_score;
			double x_R;
			double y_R;
			double x_L;
			double y_L;
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



//create neighbors based on initial config
void Create_Nodes(double x_i_LF,double y_i_LF,double x_i_RF, double y_i_RF,double x_f_LF,double y_f_LF,double x_f_RF, double y_f_RF, std::map<std::string, node>& nodeMap, double current_local_score_LF, double current_local_score_RF, node current_node)
{
	//create vectors of all possible x,y values for both RF and LF
	//based on init config
	vector<double> x_i_LF_vals;
	vector<double> y_i_LF_vals;
	vector<double> x_i_RF_vals;
	vector<double> y_i_RF_vals;

	//values of newly created x,y values for both LF and RF
	double x_i_LF_new;
	double y_i_LF_new;
	double x_i_RF_new;
	double y_i_RF_new;


	//vector of coordinates of goal stance
	vector<double> goal_LF;
	vector<double> goal_RF;
	goal_LF.push_back(x_f_LF);
	goal_LF.push_back(y_f_LF);
	goal_RF.push_back(x_f_RF);
	goal_RF.push_back(y_f_RF);


	//vector of coordinates of current stance
	vector<double> current_LF;
	vector<double> current_RF;
	current_LF.push_back(x_i_LF);
	current_LF.push_back(y_i_LF);
	current_RF.push_back(x_i_RF);
	current_RF.push_back(y_i_RF);

	vector< vector<double> > LF_coords;
	vector< vector<double> > RF_coords;
	// theta_vals = [0,45,90,135,180];
	int k = -1;
	//append all new x,y values for both LF and RF going down and left from original coordinates
	while (k <= 1){
		double temp_val = 0.2*k;
		x_i_LF_new = x_i_LF + temp_val;
		y_i_LF_new = y_i_LF + temp_val;
		x_i_RF_new = x_i_RF + temp_val;
		y_i_RF_new = y_i_RF + temp_val;

		x_i_LF_vals.push_back(x_i_LF_new);
		y_i_LF_vals.push_back(y_i_LF_new);
		x_i_RF_vals.push_back(x_i_RF_new);
		y_i_RF_vals.push_back(y_i_RF_new);
		k++;
	}

	//int n = 1;
	//append all new x,y values for both LF and RF going up and right from original coordinates
	//while (n < 2){  
	//	double temp_val = 0.2*n;
	//	x_i_LF_new = x_i_LF + temp_val;
	//	y_i_LF_new = y_i_LF + temp_val;
	//	x_i_RF_new = x_i_RF + temp_val;
	//	y_i_RF_new = y_i_RF + temp_val;

//		x_i_LF_vals.push_back(x_i_LF_new);
//		y_i_LF_vals.push_back(y_i_LF_new);
//		x_i_RF_vals.push_back(x_i_RF_new);
//		y_i_RF_vals.push_back(y_i_RF_new);
//		n++;
//	}

	//create all possible coordinates for LF
	for (size_t i(0);i < x_i_LF_vals.size();i++){
		for (size_t j(0);j < y_i_LF_vals.size();j++){
			if ( sqrt(pow(x_i_LF_vals[i] - x_i_LF,2)+pow(y_i_LF_vals[j] - y_i_LF,2)) > 0.1 && sqrt(pow(x_i_LF_vals[i] - x_i_LF,2)+pow(y_i_LF_vals[j] - y_i_LF,2)) < 0.6){
				vector<double> x_y_coord_LF;
				// x_y_coord_LF.resize(3);
				x_y_coord_LF.push_back(x_i_LF_vals[i]);
				x_y_coord_LF.push_back(y_i_LF_vals[j]);				
				saveVector(x_y_coord_LF, "LF_coords_new");
				LF_coords.push_back(x_y_coord_LF);
			}	
		}	
	}
	//create all possible coordinates for RF
	for (size_t i(0);i < x_i_RF_vals.size();i++){
		for (size_t j(0);j < y_i_RF_vals.size();j++){
			if ( sqrt(pow(x_i_RF_vals[i] - x_i_RF,2)+pow(y_i_RF_vals[j] - y_i_RF,2)) > 0.1 && sqrt(pow(x_i_RF_vals[i] - x_i_RF,2)+pow(y_i_RF_vals[j] - y_i_RF,2)) < 0.6){
				vector<double> x_y_coord_RF;
				//x_y_coord_RF.resize(3);
				x_y_coord_RF.push_back(x_i_RF_vals[i]);
				x_y_coord_RF.push_back(y_i_RF_vals[j]);
				saveVector(x_y_coord_RF, "RF_coords_new");
				RF_coords.push_back(x_y_coord_RF);
			}
		}
	}

	//create all possible nodes for RF still, LF step, increment s
	for (size_t i(0);i < LF_coords.size();i++){
		//if ( sqrt(pow(LF_coords[i][0] - x_i_LF,2)+pow(LF_coords[i][1] - y_i_LF,2)) > 0.2 && sqrt(pow(LF_coords[i][0] - x_i_LF,2)+pow(LF_coords[i][1] - y_i_LF,2)) < 0.5){
		node insert1;
		insert1.obstacle = false;
		insert1.visited = false;
		insert1.GlobalGoal = GlobalHeuristic(LF_coords[i],goal_LF);
		insert1.LocalGoalLF = LocalHeuristic(current_LF,LF_coords[i]) + current_local_score_LF;
		insert1.LocalGoalRF = current_local_score_RF;
		insert1.f_score = insert1.GlobalGoal + insert1.LocalGoalLF + insert1.LocalGoalRF;
		insert1.x_R = x_i_RF;
		insert1.y_R = y_i_RF;
		insert1.x_L = LF_coords[i][0];
		insert1.y_L = LF_coords[i][1];
		insert1.key = (to_string(insert1.x_R) + to_string(insert1.y_R) + to_string(insert1.x_L) + to_string(insert1.y_L));
		insert1.parent = current_node.key;
		if (nodeMap.count(insert1.key) == 0){
			if (sqrt(pow(insert1.x_R - insert1.x_L,2) + pow(insert1.y_R - insert1.y_L,2)) < 1.5){
				nodeMap.insert(pair<string,node>(insert1.key,insert1));
			}
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
	//create all possible nodes for LF still, RF step, increment s
	for (size_t i(0);i < RF_coords.size();i++){
		//if (sqrt(pow(RF_coords[i][0] - x_i_RF,2)+pow(RF_coords[i][1] - y_i_RF,2)) > 0.1 && sqrt(pow(RF_coords[i][0] - x_i_RF,2)+pow(RF_coords[i][1] - y_i_RF,2)) < 0.5){
		node insert2;
		insert2.obstacle = false;
		insert2.visited = false;
		insert2.GlobalGoal = GlobalHeuristic(RF_coords[i],goal_RF);
		insert2.LocalGoalRF = LocalHeuristic(current_RF,RF_coords[i]) + current_local_score_RF;
		insert2.LocalGoalLF = current_local_score_LF;
		insert2.f_score = insert2.GlobalGoal + insert2.LocalGoalRF + insert2.LocalGoalLF;
		insert2.x_R = RF_coords[i][0];
		insert2.y_R = RF_coords[i][1];
		insert2.x_L = x_i_LF;
		insert2.y_L = y_i_LF;
		insert2.key = (to_string(insert2.x_R) + to_string(insert2.y_R) + to_string(insert2.x_L) + to_string(insert2.y_L));
		insert2.parent = current_node.key;
		if (nodeMap.count(insert2.key) == 0){
			if (sqrt(pow(insert2.x_R - insert2.x_L,2) + pow(insert2.y_R - insert2.y_L,2)) < 1.5){
				nodeMap.insert(pair<string,node>(insert2.key,insert2));
			}
		}
		else if (nodeMap.count(insert2.key) > 0){
			map<string,node>::iterator it;
			it = nodeMap.find(insert2.key);
			if (it != nodeMap.end()){
				if (insert2.f_score < it->second.f_score && it->second.visited == false){
					it->second.f_score = insert2.f_score;
					it->second.parent = insert2.parent;
				}	
			}
		}
	}

}



node SortMap(map<string,node> nodeMap){
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





		//map<int, node>::iterator map_iter = nodeMap.begin();
	//for(map_iter;map_iter!=nodeMap.end();map_iter++)
	//	key: map_iter->first 	
	  //  value: map_iter->second







int main(){
	double x_i_LF = 2;
	double y_i_LF = 4;
	double x_i_RF = 4;
	double y_i_RF = 4;

	double x_f_LF = 5;
	double y_f_LF = 5;
	double x_f_RF = 7;
	double y_f_RF = 5;

	vector<double> start_LF;
	vector<double> start_RF;
	vector<double> goal_pos_LF;
	vector<double> goal_pos_RF;

	start_LF.push_back(x_i_LF);
	start_LF.push_back(y_i_LF);
	start_RF.push_back(x_i_RF);
	start_RF.push_back(y_i_RF);

	goal_pos_LF.push_back(x_f_LF);
	goal_pos_LF.push_back(y_f_LF);
	goal_pos_RF.push_back(x_f_RF);
	goal_pos_RF.push_back(y_f_RF);


	std::map<std::string, node> nodeMap;

	node current_node;
	current_node.visited = true;
	current_node.GlobalGoal = 500;
	current_node.LocalGoalLF = 200;
	current_node.LocalGoalRF = 200;
	current_node.f_score = current_node.LocalGoalLF + current_node.LocalGoalRF + current_node.GlobalGoal;
	current_node.x_L = x_i_LF;
	current_node.y_L = y_i_LF;
	current_node.x_R = x_i_RF;
	current_node.y_R = y_i_RF;
	current_node.start = true;
	current_node.key = (to_string(current_node.x_R) + to_string(current_node.y_R) + to_string(current_node.x_L) + to_string(current_node.y_L));

	nodeMap.insert(pair<string,node>(current_node.key,current_node));
	vector<double> optimal_path;
	

	double current_local_score_LF = 0;
	double current_local_score_RF = 0;
	Create_Nodes(x_i_LF,y_i_LF,x_i_RF,y_i_RF,x_f_LF,y_f_LF,x_f_RF,y_f_RF, nodeMap, current_local_score_LF,current_local_score_RF,current_node);
	//map<string, node>::iterator map_iter = nodeMap.begin();
	//for(map_iter; map_iter != nodeMap.end();map_iter++){
	//	cout << "LF_x: "<< map_iter->second.x_L << "LF_y: " << map_iter->second.y_L << "RF_x: " << map_iter->second.x_R << "RF_y: " << map_iter->second.y_R << "fscore: " << map_iter->second.f_score << endl;
	

	while (!nodeMap.empty()){
		node current_node = SortMap(nodeMap);
		nodeMap[current_node.key].visited = true;
		//cout << "current node key: " << current_node.key << endl;
		//cout << "current node visited pre: " << current_node.visited << endl;
		//cout << "current node visited post: " << current_node.visited << endl;
		cout << "LF_x: " << current_node.x_L<< " " << "LF_y: " << current_node.y_L<< " " << "RF_x: " << current_node.x_R<< " " << "RF_y: "<< current_node.y_R << endl;
		cout << "optimal fscore: " << current_node.f_score << endl;
		//if (i == 0){
		//	map<string, node>::iterator map_iter = nodeMap.begin();
		//	for(map_iter; map_iter != nodeMap.end();map_iter++){
		//		cout << map_iter->second.GlobalGoal << endl;
		//break;

		if (abs(current_node.x_R - x_f_RF) < 0.1 && abs(current_node.y_R - y_f_RF) < 0.1 && abs(current_node.x_L - x_f_LF) < 0.1 && abs(current_node.y_L - y_f_LF) < 0.1){

			cout << "we made it" << endl;
			vector<node> optimal_path = OptimalPath(current_node,nodeMap);
			cout << "optimal path size: " << optimal_path.size() << endl;
			for (size_t i(0);i < optimal_path.size();i++){
				cout << "optimal path" << " " << "x_LF: " << optimal_path[i].x_L << "y_LF: " << optimal_path[i].y_L << "x_RF: " << optimal_path[i].x_R << "y_RF" << optimal_path[i].y_R << endl;
				vector<double> optimal_coord_LF;
				vector<double> optimal_coord_RF;
				optimal_coord_LF.push_back(optimal_path[i].x_L);
				optimal_coord_LF.push_back(optimal_path[i].y_L);
				optimal_coord_RF.push_back(optimal_path[i].x_R);
				optimal_coord_RF.push_back(optimal_path[i].y_R);
				saveVector(optimal_coord_LF,"optimal_path_LF");
				saveVector(optimal_coord_RF,"optimal_path_RF");
			}
			break;
			//cout << optimal_path[];
		}
		else{
			current_local_score_LF = current_node.LocalGoalLF;
			current_local_score_RF = current_node.LocalGoalRF;
			cout << "current Local Goal: " << current_local_score_LF << "," << current_local_score_RF << endl;
			Create_Nodes(current_node.x_L,current_node.y_L,current_node.x_R,current_node.y_R,x_f_LF,y_f_LF,x_f_RF,y_f_RF, nodeMap, current_local_score_LF,current_local_score_RF,current_node);
			

		}
	
	}

	
		
	//for (size_t i(0);i<LF_coords.size();i++){
	//	cout << LF_coords[i][0] << ',' << LF_coords[i][1] << ',' << LF_coords[i][2] << endl;
	//}

	
}

