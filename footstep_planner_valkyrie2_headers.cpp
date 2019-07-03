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



		};
			bool obstacle;
			bool visited;
			double GlobalGoal;
			double LocalGoal;
			double f_score;
			double x_R;
			double y_R;
			double theta_R;
			double x_L;
			double y_L;
			double theta_L;
			float s;
			string key;

};

double LocalHeuristic(vector<double> current, vector<double> neighbor){
	double distance = sqrt(pow(current[0]-neighbor[0],2) + pow(current[1]-neighbor[1],2));
	double trans_cost = 0.7;
	double local_cost = distance + trans_cost;
	return local_cost;
}


double GlobalHeuristic(vector<double> neighborLF, vector<double> neighborRF,vector<double> goalLF,vector<double> goalRF){
	vector<double> midpoint_neighbor;
	midpoint_neighbor.push_back((neighborLF[0] + neighborRF[0])/2);
	midpoint_neighbor.push_back((neighborLF[1] + neighborRF[1])/2);
	vector<double> midpoint_goal;
	midpoint_goal.push_back((goalLF[0] + goalRF[0])/2);
	midpoint_goal.push_back((goalLF[1] + goalRF[1])/2); 
	double distance = sqrt(pow(midpoint_goal[0]-midpoint_neighbor[0],2) + pow(midpoint_goal[1]-midpoint_neighbor[1],2));
	return distance;
}



//create neighbors based on initial config
void Create_Nodes(double x_i_LF,double y_i_LF,double thetaLF,double x_i_RF, double y_i_RF,double thetaRF,double x_f_LF,double y_f_LF,double x_f_RF, double y_f_RF, std::map<std::string, node>& nodeMap, double current_local_score)
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

	//vector of all possible theta values for both LF and RF
	vector<double> theta_vals;
	theta_vals.resize(5);
	theta_vals[0]=0.0;
	theta_vals[1]=45;
	theta_vals[2]=90;
	theta_vals[3]=135;
	theta_vals[4]=180;

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
	current_LF.push_back(thetaLF);
	current_RF.push_back(x_i_RF);
	current_RF.push_back(y_i_RF);
	current_RF.push_back(thetaRF);

	vector< vector<double> > LF_coords;
	vector< vector<double> > RF_coords;
	// theta_vals = [0,45,90,135,180];
	int k = -10;
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
	while (n < 10){  
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
			for (size_t k(0);k < theta_vals.size();k++){
				if ( sqrt(pow(x_i_LF_vals[i] - x_i_LF,2)+pow(y_i_LF_vals[j] - y_i_LF,2)) > 0.2 && sqrt(pow(x_i_LF_vals[i] - x_i_LF,2)+pow(y_i_LF_vals[j] - y_i_LF,2)) < 0.5){
					vector<double> x_y_coord_LF;
					// x_y_coord_LF.resize(3);
					x_y_coord_LF.push_back(x_i_LF_vals[i]);
					x_y_coord_LF.push_back(y_i_LF_vals[j]);
					x_y_coord_LF.push_back(theta_vals[k]);
					saveVector(x_y_coord_LF, "LF_coords_new");
					LF_coords.push_back(x_y_coord_LF);
				}
			}	
		}	
	}
	//create all possible coordinates for RF
	for (size_t i(0);i < x_i_RF_vals.size();i++){
		for (size_t j(0);j < y_i_RF_vals.size();j++){
			for (size_t k(0);k < theta_vals.size();k++){
				if ( sqrt(pow(x_i_RF_vals[i] - x_i_RF,2)+pow(y_i_RF_vals[j] - y_i_RF,2)) > 0.2 && sqrt(pow(x_i_RF_vals[i] - x_i_RF,2)+pow(y_i_RF_vals[j] - y_i_RF,2)) < 0.5){
					vector<double> x_y_coord_RF;
					//x_y_coord_RF.resize(3);
					x_y_coord_RF.push_back(x_i_RF_vals[i]);
					x_y_coord_RF.push_back(y_i_RF_vals[j]);
					x_y_coord_RF.push_back(theta_vals[k]);
					saveVector(x_y_coord_RF, "RF_coords_new");
					RF_coords.push_back(x_y_coord_RF);
				}
			}
		}
	}

	//create all possible nodes for RF still, LF step, increment s
	for (size_t i(0);i < LF_coords.size();i++){
		//if ( sqrt(pow(LF_coords[i][0] - x_i_LF,2)+pow(LF_coords[i][1] - y_i_LF,2)) > 0.2 && sqrt(pow(LF_coords[i][0] - x_i_LF,2)+pow(LF_coords[i][1] - y_i_LF,2)) < 0.5){
		node insert1;
		insert1.obstacle = false;
		insert1.visited = false;
		insert1.s = 0.2;
		insert1.GlobalGoal = GlobalHeuristic(LF_coords[i],current_RF,goal_LF,goal_RF);
		insert1.LocalGoal = LocalHeuristic(current_LF,LF_coords[i]) + current_local_score;
		insert1.f_score = insert1.GlobalGoal + insert1.LocalGoal + 0.2;
		insert1.x_R = x_i_RF;
		insert1.y_R = y_i_RF;
		insert1.theta_R = thetaRF;
		insert1.x_L = LF_coords[i][0];
		insert1.y_L = LF_coords[i][1];
		insert1.theta_L = LF_coords[i][2];
		insert1.key = (to_string(insert1.x_R) + to_string(insert1.y_R) + to_string(insert1.theta_R) + to_string(insert1.x_L) + to_string(insert1.y_L) + to_string(insert1.theta_L) + to_string(insert1.s));
		if (nodeMap.count(insert1.key) == 0){
			nodeMap.insert(pair<string,node>(insert1.key,insert1));
		}
		else if (nodeMap.count(insert1.key) == 1){
			map<string,node>::iterator it;
			it = nodeMap.find(insert1.key);
			if (it != nodeMap.end()){
				if (insert1.f_score < it->second.f_score && it->second.visited == false){
					it->second.f_score = insert1.f_score;
				}	
			}

		}

			
			
			//don't increment s
		node insert2;
		insert2.obstacle = false;
		insert2.visited = false;
		insert2.s = 0.0;
		insert2.GlobalGoal = GlobalHeuristic(LF_coords[i],current_RF,goal_LF,goal_RF);
		insert2.LocalGoal = LocalHeuristic(current_LF,LF_coords[i]) + current_local_score;
		insert2.f_score = insert2.GlobalGoal + insert2.LocalGoal + 0.2;
		insert2.x_R = x_i_RF;
		insert2.y_R = y_i_RF;
		insert2.theta_R = thetaRF;
		insert2.x_L = LF_coords[i][0];
		insert2.y_L = LF_coords[i][1];
		insert2.theta_L = LF_coords[i][2];
		insert2.key = (to_string(insert2.x_R) + to_string(insert2.y_R) + to_string(insert2.theta_R) + to_string(insert2.x_L) + to_string(insert2.y_L) + to_string(insert2.theta_L) + to_string(insert2.s));
		if (nodeMap.count(insert2.key) == 0){
			nodeMap.insert(pair<string,node>(insert2.key,insert2));
		}
		else if (nodeMap.count(insert2.key) == 1){
			map<string,node>::iterator it;
			it = nodeMap.find(insert2.key);
			if (it != nodeMap.end()){
				if (insert2.f_score < it->second.f_score && it->second.visited == false){
					it->second.f_score = insert2.f_score;
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
		insert3.s = 0.2;
		insert3.GlobalGoal = GlobalHeuristic(current_LF,RF_coords[i],goal_LF,goal_RF);
		insert3.LocalGoal = LocalHeuristic(current_RF,RF_coords[i]) + current_local_score;
		insert3.f_score = insert3.GlobalGoal + insert3.LocalGoal + 0.2;
		insert3.x_R = RF_coords[i][0];
		insert3.y_R = RF_coords[i][1];
		insert3.theta_R = RF_coords[i][2];
		insert3.x_L = x_i_LF;
		insert3.y_L = x_i_RF;
		insert3.theta_L = thetaLF;
		insert3.key = (to_string(insert3.x_R) + to_string(insert3.y_R) + to_string(insert3.theta_R) + to_string(insert3.x_L) + to_string(insert3.y_L) + to_string(insert3.theta_L) + to_string(insert3.s));
		if (nodeMap.count(insert3.key) == 0){
			nodeMap.insert(pair<string,node>(insert3.key,insert3));
		}
		else if (nodeMap.count(insert3.key) == 1){
			map<string,node>::iterator it;
			it = nodeMap.find(insert3.key);
			if (it != nodeMap.end()){
				if (insert3.f_score < it->second.f_score && it->second.visited == false){
					it->second.f_score = insert3.f_score;
				}	

			}

		}
			
			//dont increment s
		node insert4;
		insert4.obstacle = false;
		insert4.visited = false;
		insert4.s = 0.0;
		insert4.GlobalGoal = GlobalHeuristic(current_LF,RF_coords[i],goal_LF,goal_RF);
		insert4.LocalGoal = LocalHeuristic(current_RF,RF_coords[i]) + current_local_score;
		insert4.f_score = insert4.GlobalGoal + insert4.LocalGoal;
		insert4.x_R = RF_coords[i][0];
		insert4.y_R = RF_coords[i][1];
		insert4.theta_R = RF_coords[i][2];
		insert4.x_L = x_i_LF;
		insert4.y_L = x_i_RF;
		insert4.theta_L = thetaLF;
		insert4.key = (to_string(insert4.x_R) + to_string(insert4.y_R) + to_string(insert4.theta_R) + to_string(insert4.x_L) + to_string(insert4.y_L) + to_string(insert4.theta_L) + to_string(insert4.s));
		if (nodeMap.count(insert4.key) == 0){
			nodeMap.insert(pair<string,node>(insert4.key,insert4));
		}
		else if (nodeMap.count(insert4.key) == 1){
			map<string,node>::iterator it;
			it = nodeMap.find(insert4.key);
			if (it != nodeMap.end()){
				if (insert4.f_score < it->second.f_score && it->second.visited == false){
					it->second.f_score = insert4.f_score;
				}						

			}
		}	

	}
		//nodeMap.insert(make_pair(i,temp));


		// LF_nodeset.push_back(temp);


	//create one more node with increment of s but same configuration

	node insert5;
	insert5.obstacle = false;
	insert5.visited = true;
	insert5.s = 100.0;
	insert5.GlobalGoal = GlobalHeuristic(current_LF,current_RF,goal_LF,goal_RF);
	insert5.LocalGoal = 0.0 + insert5.s;
	insert5.f_score = insert5.GlobalGoal + insert5.LocalGoal + current_local_score;
	insert5.x_R = current_RF[0];
	insert5.y_R = current_RF[1];
	insert5.theta_R = current_RF[2];
	insert5.x_L = current_LF[0];
	insert5.y_L = current_LF[1];
	insert5.theta_L = current_LF[2];
	insert5.key = (to_string(insert5.x_R) + to_string(insert5.y_R) + to_string(insert5.theta_R) + to_string(insert5.x_L) + to_string(insert5.y_L) + to_string(insert5.theta_L) + to_string(insert5.s));
	if (nodeMap.count(insert5.key) == 0){
		nodeMap.insert(pair<string,node>(insert5.key,insert5));
	}
	else if (nodeMap.count(insert5.key) == 1){
				map<string,node>::iterator it;
				it = nodeMap.find(insert5.key);
				if (it != nodeMap.end()){
					if (insert5.f_score < it->second.f_score && it->second.visited == false){
						it->second.f_score = insert5.f_score;
					}	

				}

			}

		//map<int, node>::iterator map_iter = nodeMap.begin();
	//for(map_iter;map_iter!=nodeMap.end();map_iter++)
	//	key: map_iter->first 	
	  //  value: map_iter->second
}



node SortMap(map<string,node> nodeMap,node current_node){
	int i = 0;
	double min_score = 1000;
	string min_key;
	node optimized_node;
	map<string, node>::iterator map_iter = nodeMap.begin();
	cout<<"size of map: "<< nodeMap.size()<<endl;
	for(map_iter; map_iter != nodeMap.end();map_iter++){
		if (map_iter -> second.visited == false && map_iter -> second.f_score < min_score && map_iter ->second.x_R != current_node.x_R && map_iter -> second.y_R != current_node.y_R && map_iter -> second.x_L != current_node.x_L && map_iter -> second.y_L != current_node.y_L){
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
	double y_i_RF = 2;
	double theta_LF = 0;
	double theta_RF = 0;

	double x_f_LF = 10;
	double y_f_LF = 10;
	double x_f_RF = 12;
	double y_f_RF = 10;
	double theta_f_LF = 0;
	double theta_f_RF = 0;

	std::map<std::string, node> nodeMap;
	node lowest_node;

	node current_node1;
	current_node1.x_L = x_i_LF;
	current_node1.y_L = y_i_LF;
	current_node1.x_R = x_i_RF;
	current_node1.y_R = y_i_RF;

	vector< vector<double> > RF_coords;
	double current_local_score = 0;
	int i = 0;
	Create_Nodes(x_i_LF,y_i_LF,theta_LF,x_i_RF,y_i_RF,theta_RF,x_f_LF,y_f_LF,x_f_RF,y_f_RF, nodeMap, current_local_score);
	//map<string, node>::iterator map_iter = nodeMap.begin();
	//for(map_iter; map_iter != nodeMap.end();map_iter++){
	//	cout << "LF_x: "<< map_iter->second.x_L << "LF_y: " << map_iter->second.y_L << "RF_x: " << map_iter->second.x_R << "RF_y: " << map_iter->second.y_R << "fscore: " << map_iter->second.f_score << endl;
	

	while (!nodeMap.empty()){
		node current_node = SortMap(nodeMap,current_node1);
		nodeMap[current_node.key].visited = true;
		current_node1 = current_node;
		//cout << "current node key: " << current_node.key << endl;
		//cout << "current node visited pre: " << current_node.visited << endl;
		//cout << "current node visited post: " << current_node.visited << endl;
		cout << "LF_x: " << current_node.x_L<< " " << "LF_y: " << current_node.y_L<< " " << "RF_x: " << current_node.x_R<< " " << "RF_y: " << current_node.y_R << "thetaR: " << current_node.theta_R << "thetaL: "<< current_node.theta_L << endl;
		cout << "optimal fscore: " << current_node.f_score << endl;
		//if (i == 0){
		//	map<string, node>::iterator map_iter = nodeMap.begin();
		//	for(map_iter; map_iter != nodeMap.end();map_iter++){
		//		cout << map_iter->second.GlobalGoal << endl;
		//break;

		if (abs(current_node.x_R - x_f_RF) < 0.1 && abs(current_node.y_R - y_f_RF) < 0.1 && abs(current_node.theta_R == theta_f_RF) && abs(current_node.x_L - x_f_LF) < 0.1 && abs(current_node.y_L - y_f_LF) < 0.1 && abs(current_node.theta_L == theta_f_LF)){

			cout << "we made it" << endl;
			break;
			//cout << optimal_path[];
		}
		else{
			cout << "else" << endl;
			current_local_score = current_node.LocalGoal;
			cout << "current Local Goal: " << current_local_score << endl;
			Create_Nodes(current_node.x_L,current_node.y_L,current_node.theta_L,current_node.x_R,current_node.y_R,current_node.theta_R,x_f_LF,y_f_LF,x_f_RF,y_f_RF, nodeMap, current_local_score);
			

		}
	
	}

	
		
	//for (size_t i(0);i<LF_coords.size();i++){
	//	cout << LF_coords[i][0] << ',' << LF_coords[i][1] << ',' << LF_coords[i][2] << endl;
	//}

	

	
}

