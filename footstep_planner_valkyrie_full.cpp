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
void Create_Nodes(double x_i_LF,double y_i_LF,double x_i_RF, double y_i_RF,double x_f_LF,double y_f_LF,double x_f_RF, double y_f_RF, std::map<std::string, node>& nodeMap, double current_local_score_LF, double current_local_score_RF)
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
	int k = -2;
	//append all new x,y values for both LF and RF going down and left from original coordinates
	while (k <= 0){
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

	int n = 1;
	//append all new x,y values for both LF and RF going up and right from original coordinates
	while (n < 2){  
		double temp_val = 0.2*n;
		x_i_LF_new = x_i_LF + temp_val;
		y_i_LF_new = y_i_LF + temp_val;
		x_i_RF_new = x_i_RF + temp_val;
		y_i_RF_new = y_i_RF + temp_val;

		x_i_LF_vals.push_back(x_i_LF_new);
		y_i_LF_vals.push_back(y_i_LF_new);
		x_i_RF_vals.push_back(x_i_RF_new);
		y_i_RF_vals.push_back(y_i_RF_new);
		n++;
	}

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
		insert1.f_score = insert1.GlobalGoal + insert1.LocalGoalLF + insert1.LocalGoalRF + 0.2;
		insert1.x_R = x_i_RF;
		insert1.y_R = y_i_RF;
		insert1.x_L = LF_coords[i][0];
		insert1.y_L = LF_coords[i][1];
		insert1.key = (to_string(insert1.x_R) + to_string(insert1.y_R) + to_string(insert1.x_L) + to_string(insert1.y_L));
		if (nodeMap.count(insert1.key) == 0){
			if (sqrt(pow(insert1.x_R - insert1.x_L,2) + pow(insert1.y_R - insert1.y_L,2)) < 3){
				nodeMap.insert(pair<string,node>(insert1.key,insert1));
			}
		}
		else if (nodeMap.count(insert1.key) > 0){
			map<string,node>::iterator it;
			it = nodeMap.find(insert1.key);
			if (it != nodeMap.end()){
				if (insert1.f_score < it->second.f_score && it->second.visited == false){
					it->second.f_score = insert1.f_score;
				}	
			}
		}
	}
	//create all possible nodes for LF still, RF step, increment s
	for (size_t i(0);i < RF_coords.size();i++){
		//if (sqrt(pow(RF_coords[i][0] - x_i_RF,2)+pow(RF_coords[i][1] - y_i_RF,2)) > 0.1 && sqrt(pow(RF_coords[i][0] - x_i_RF,2)+pow(RF_coords[i][1] - y_i_RF,2)) < 0.5){
		node insert3;
		insert3.obstacle = false;
		insert3.visited = false;
		insert3.GlobalGoal = GlobalHeuristic(RF_coords[i],goal_RF);
		insert3.LocalGoalRF = LocalHeuristic(current_RF,RF_coords[i]) + current_local_score_RF;
		insert3.LocalGoalLF = current_local_score_LF;
		insert3.f_score = insert3.GlobalGoal + insert3.LocalGoalRF + insert3.LocalGoalLF + 0.2;
		insert3.x_R = RF_coords[i][0];
		insert3.y_R = RF_coords[i][1];
		insert3.x_L = x_i_LF;
		insert3.y_L = x_i_RF;
		insert3.key = (to_string(insert3.x_R) + to_string(insert3.y_R) + to_string(insert3.x_L) + to_string(insert3.y_L));
		if (nodeMap.count(insert3.key) == 0){
			if (sqrt(pow(insert3.x_R - insert3.x_L,2) + pow(insert3.y_R - insert3.y_L,2)) < 3){
				nodeMap.insert(pair<string,node>(insert3.key,insert3));
			}
		}
		else if (nodeMap.count(insert3.key) > 0){
			map<string,node>::iterator it;
			it = nodeMap.find(insert3.key);
			if (it != nodeMap.end()){
				if (insert3.f_score < it->second.f_score && it->second.visited == false){
					it->second.f_score = insert3.f_score;
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





		//map<int, node>::iterator map_iter = nodeMap.begin();
	//for(map_iter;map_iter!=nodeMap.end();map_iter++)
	//	key: map_iter->first 	
	  //  value: map_iter->second







int main(){
	double x_i_LF = 2;
	double y_i_LF = 4;
	double x_i_RF = 4;
	double y_i_RF = 4;

	double x_f_LF = 10;
	double y_f_LF = 10;
	double x_f_RF = 12;
	double y_f_RF = 10;

	std::map<std::string, node> nodeMap;
	node lowest_node;

	double current_local_score_LF = 0;
	double current_local_score_RF = 0;
	int i = 0;
	Create_Nodes(x_i_LF,y_i_LF,x_i_RF,y_i_RF,x_f_LF,y_f_LF,x_f_RF,y_f_RF, nodeMap, current_local_score_LF,current_local_score_RF);
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
			break;
			//cout << optimal_path[];
		}
		else{
			cout << "else" << endl;
			current_local_score_LF = current_node.LocalGoalLF;
			current_local_score_RF = current_node.LocalGoalRF;
			cout << "current Local Goal: " << current_local_score_LF << "," << current_local_score_RF << endl;
			Create_Nodes(current_node.x_L,current_node.y_L,current_node.x_R,current_node.y_R,x_f_LF,y_f_LF,x_f_RF,y_f_RF, nodeMap, current_local_score_LF,current_local_score_RF);
			

		}
	
	}

	
		
	//for (size_t i(0);i<LF_coords.size();i++){
	//	cout << LF_coords[i][0] << ',' << LF_coords[i][1] << ',' << LF_coords[i][2] << endl;
	//}

	
}

