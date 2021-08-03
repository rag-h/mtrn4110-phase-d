// File:          z516972_MTRN4110_PhaseB.cpp
// Date:          12/07/2021
// Description:   Controller of E-puckk for Phase B - Path Planning
// Author:        Raghav Hariharan
// Platform:      Linux
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <algorithm> 
#include <iostream>
#include <fstream>
#include <vector>
using namespace std;
// All the webots classes are defined in the "webots" namespace
using namespace webots;
#include <cstring>
#include <string>
#include <iostream>
#include <fstream>
#define TIME_STEP 64
const std::string MAP = "../../Map.txt";
const std::string OUTPUT_FILE_NAME = "../../Solution.txt";

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node

vector< pair <int,int> > find_paths(int goal_x, int goal_y, int currentExploredValue, int horizontal[19][6], double vertical[19][6], int floodfill[9][5] );
void DFS(int start_x, int start_y,int goal_x,int goal_y, int currentExploredValue, int horizontal[19][6], double vertical[19][6], int floodfill[9][5], vector< pair <int,int> > path,char heading, int turns,string motion_plan);
string printPath(vector< pair <int,int> > path, int horizontal[10][6], double vertical[10][6],int start_x,int start_y,int end_x,int end_y, char heading);


vector<vector< pair <int,int> >> allPaths;
vector<int> allTurns;
vector<string> allMotionPlans;
int main(int argc, char **argv) {
  // create the Robot instance.
  ofstream output;
  output.open (OUTPUT_FILE_NAME);
  Robot *robot = new Robot();
  
     int horizontal_two[10][6];
     double vertical_two[10][6];
     int horizontal[19][6];
     double vertical[19][6];
     int start_x = 0;
     int start_y = 0;
     char heading = 'S';
     int goal_x = 0;
     int goal_y = 0;
    
     
     
    for(int k = 0; k < 6; k++){
      for (int j = 0; j < 10; j++){
          vertical_two[j][k] = 0;
          horizontal_two[j][k] = 0;
      }
      std::cout <<"" <<std::endl;
    }
     for(int k = 0; k < 6; k++){
      for (int j = 0; j < 19; j++){
          vertical[j][k] = 0;
          horizontal[j][k] = 0;
      }
      std::cout <<"" <<std::endl;
    }
    
    
     vector<string> array;
      string str;
          char ch[42];
          int line = 0;
     int i = 0;
    //  int j = 0;
      ifstream my_file (MAP);
      
      if (my_file.is_open()){
        std::cout<<"[z5162972_MTRN4110_PhaseB] "<< "Reading in map from ../../Map.txt..."<<std::endl;
        output <<"[z5162972_MTRN4110_PhaseB] "<< "Reading in map from ../../Map.txt...\n";
        
        while (std::getline(my_file, str)) {
           //std::cout<<"while"<<std::endl;
           strcpy(ch, str.c_str());
           
        if( line % 2) {
          for (int i = 0; i < str.length(); i= i+4){
          //std::cout<<"Vertical"<<std::endl;
            
              //cout << ch[i] <<" ";
              if(ch[i] == 0x20){
                  //cout << ch[i];
               // std::cout << "x: " << i/4 << " y :" << line/2 <<std::endl;
                  vertical_two[i/4][line/2] = 0;
                  //std::cout<< vertical_two[i][line/2] << "" <<endl;
                }else{
               // cout << ch[i] << " ";
               //  std::cout << "x: " << i/4 << " y :" << line/2 <<std::endl;
                  vertical_two[i/4][line/2] = 1;
                  //std::cout<< vertical_two[i][line/2] << "" <<endl;
                }
                          
          }for (int i = 0; i < str.length(); i++){
              // 2 = goal position (x)
              //cout << ch[i] <<" ";
              if(ch[i] == 'x'){
                  goal_x = i/4;
                  goal_y = line/2;
              }else if(ch[i] == 'v'){
                start_x = i/4;
                start_y = line/2;
                heading = 'S';
                //vertical_two[i/4][line/2] = 2;
              }else if(ch[i] == '^'){
                start_x = i/4;
                start_y = line/2;
                heading = 'N';
              }else if(ch[i] == '>'){
                start_x = i/4;
                start_y = line/2;
                heading = 'E';
              }else if(ch[i] == '<'){
                 start_x = i/4;
                start_y = line/2;
                heading = 'W';
              }
                
          }
          
          
        }else{
          for (int i = 1; i < str.length(); i= i+4){
             //std::cout<<"Horizontal"<<std::endl;
             // cout << ch[i] <<" ";
              if(ch[i] == 0x20){
                  //cout << ch[i];
                //std::cout << "x: " << i/4 << " y :" << line/2 <<std::endl;
                  horizontal_two[i/4][line/2] = 0;
                  //std::cout<< vertical_two[i][line/2] << "" <<endl;
                }else{
               // cout << ch[i] << " ";
                // std::cout << "x: " << i/4 << " y :" << line/2 <<std::endl;
                  horizontal_two[i/4][line/2] = 1;
                  //std::cout<< horizontal_two[i][line/2] << "" <<endl;
                }
                
          }     
          //std::cout<<"End For" <<std::endl;   
        }
          //std::cout<<"end else"<<std::endl;  
        std::cout<<"" <<std::endl;
        //std::cout << "line :"<< line <<std::endl;
        line++;
        //std::cout<<"line increment"<<std::endl;
        }
        
      }else{
        std::cout<<"[z5162972_MTRN4110_PhaseB] "<<"No such File"<<std::endl;
      }
      
      my_file.close();
      
      
    // std::cout<< "vertical_two AFTER"<< std::endl;
    // for(int k = 0; k < 6; k++){
    //   for (int j = 0; j < 10; j++){
    //      // std::cout<< horizontal_two[j][k] << " ";
    //      std::cout<< vertical_two[j][k] << " ";
    //   }
    //   std::cout <<"" <<std::endl;
    // }
    //  std::cout<< "horizontal_two AFTER"<< std::endl;
    // for(int k = 0; k < 6; k++){
    //   for (int j = 0; j < 10; j++){
    //       std::cout<< horizontal_two[j][k] << " ";
    //      //std::cout<< vertical_two[j][k] << " ";
    //   }
    //   std::cout <<"" <<std::endl;
    // }
    
    // std::cout << "start_x :" << start_x << " start_y: " << start_y << " heading: " <<heading <<std::endl; 
    // std::cout << "goal_x :" << goal_x << " goal_y: " << goal_y <<std::endl; 
    
     for(int k = 0; k < 6; k++){
      for (int j = 0; j < 10; j++){
          vertical[(j*2)][k] = vertical_two[j][k];
          horizontal[(j*2)][k] = horizontal_two[j][k];
      }
      //std::cout <<"" <<std::endl;
    }

    // std::cout<< "Vertical AFTER"<< std::endl;
    // for(int k = 0; k < 6; k++){
    //   for (int j = 0; j < 19; j++){
    //      // std::cout<< horizontal[j][k] << " ";
    //      std::cout<< vertical[j][k] << " ";
    //   }
    //   std::cout <<"" <<std::endl;
    // }
    //  std::cout<< "horizontal AFTER"<< std::endl;
    // for(int k = 0; k < 6; k++){
    //   for (int j = 0; j < 19; j++){
    //       std::cout<< horizontal[j][k] << " ";
    //      //std::cout<< vertical[j][k] << " ";
    //   }
    //   std::cout <<"" <<std::endl;
    // }
    vector <pair<int,int>> path;
    string ret;
    printPath(path,horizontal,vertical,start_x,start_y,goal_x,goal_y,heading);
    output <<ret;
    std::cout<<"[z5162972_MTRN4110_PhaseB] "<<"Map read in!"<<std::endl;
    output <<"[z5162972_MTRN4110_PhaseB] "<<"Map read in!\n";
    std::cout<<"[z5162972_MTRN4110_PhaseB] "<<"Finding shortest paths"<<std::endl;
    output <<"[z5162972_MTRN4110_PhaseB] "<<"Finding shortest paths\n";
    // FLOOD FILL
    int floodfill[9][5];
     for(int k = 0; k < 5; k++){
      for (int j = 0; j < 9; j++){
          floodfill[j][k] = 100;
      }
      std::cout <<"" <<std::endl;
    }
    
    floodfill[start_x][start_y] = 0;
    int currentExploredValue = 0;
    int mazeValueChanged = 1;
    
    while (mazeValueChanged != 0){
      mazeValueChanged = 0;
      for(int k = 0; k < 5; k++){ //For all Rows
        for (int j = 0; j < 9; j++){ // For all columns
         // std::cout << "j: " << j << " k: "<<k << " value: " << currentExploredValue << std::endl;
          if(floodfill[j][k] == currentExploredValue){
              // 0 = N;
              // 1 = S;
              // 2 = E;
              // 3 = W;
              if(k > 0){
                if(horizontal[j*2][k] != 1){
                  if(floodfill[j][k-1] > floodfill[j][k] + 1){
                    floodfill[j][k-1] = floodfill[j][k] + 1;
                     mazeValueChanged = 1;
                     //std::cout<< "NORTH" << std::endl;
                  }
                }
              }
              if(k!= 4){
                if(horizontal[j*2][k+1] != 1){
                  if(floodfill[j][k+1] > floodfill[j][k] + 1){
                    floodfill[j][k+1] = floodfill[j][k] + 1;
                     mazeValueChanged = 1;
                     //std::cout<< "SOUTH" << std::endl;
                  }
                }
              }
              if(j > 0){
                if(vertical[j*2][k] != 1){
                  if(floodfill[j-1][k] > floodfill[j][k] + 1){
                    floodfill[j-1][k] = floodfill[j][k] + 1;
                     mazeValueChanged = 1;
                     //std::cout<< "WEST" << std::endl;
                  }
                }
              }
              if(j!= 8){
                if(vertical[(j+1)*2][k] != 1){
                  if(floodfill[j+1][k] > floodfill[j][k] + 1){
                    floodfill[j+1][k] = floodfill[j][k] + 1;
                     mazeValueChanged = 1;
                     //std::cout<< "EAST" << std::endl;
                  }
                }
              }
            
          
          }
        }
    }
    currentExploredValue++;
    
    }
    
    //  for(int k = 0; k < 5; k++){
    //   for (int j = 0; j < 9; j++){
    //       std::cout<<floodfill[j][k]<< " ";
    //   }
    //   std::cout <<"" <<std::endl;
    // }
    
    
    //vector< pair <int,int> > path;
    //path = find_paths(goal_x, goal_y, currentExploredValue, horizontal, vertical, floodfill);

    // for (int i = 0; i < path.size(); i++)
    //     cout << "("<<path[i].first<<","<<path[i].second << "), ";
    // cout << endl;
    
    string motion_plan = to_string(start_y)+ to_string(start_x) + heading;
    DFS(start_x, start_y,goal_x,goal_y,1, horizontal, vertical, floodfill, path, heading,0,motion_plan);

    // for (int i = 0; i < path.size(); i++)
    //         cout << "("<<path[i].first<<","<<path[i].second << "), ";
    // std::cout<<"("<<goal_x<<","<<goal_y<<")"<<endl;
    //std::cout<<allPaths.size()<<std::endl;
    int path_number = 1;
    for(auto& it: allPaths){
      output << "[z5162972_MTRN4110_PhaseB] "<<"Path - "<<path_number<<":\n";
      std::cout <<"[z5162972_MTRN4110_PhaseB] "<<"Path - "<<path_number++<<":"<<std::endl;
      
      ret = printPath(it,horizontal,vertical,start_x,start_y,goal_x,goal_y,heading);
      output <<ret;
    }
    // for(auto& it: allTurns){
    //   std::cout<<it<<std::endl;
    // }

    std::cout<<"[z5162972_MTRN4110_PhaseB] "<< allPaths.size() <<" shortest paths found!"<<std::endl;
    output <<"[z5162972_MTRN4110_PhaseB] "<< allPaths.size() <<" shortest paths found!\n";
    std::cout<<"[z5162972_MTRN4110_PhaseB] "<< "Finding the shortest path with least turns..."<<std::endl;
    output << "[z5162972_MTRN4110_PhaseB] "<< "Finding the shortest path with least turns...\n";
    int minElementIndex = std::min_element(allTurns.begin(),allTurns.end()) - allTurns.begin();
      ret = printPath(allPaths[minElementIndex],horizontal,vertical,start_x,start_y,goal_x,goal_y,heading);
      output <<ret;
    std::cout<<"[z5162972_MTRN4110_PhaseB] "<< "Shortest path with least turns found!"<<std::endl;
    output << "[z5162972_MTRN4110_PhaseB] "<< "Shortest path with least turns found!\n";
    std::cout<<"[z5162972_MTRN4110_PhaseB] "<< "Path Plan ("<<allMotionPlans[minElementIndex].length()-3<<"): "<<allMotionPlans[minElementIndex]<<std::endl;
    output <<"[z5162972_MTRN4110_PhaseB] "<< "Path Plan ("<<allMotionPlans[minElementIndex].length()-3<<"): "<<allMotionPlans[minElementIndex] <<"\n";
    std::cout<<"[z5162972_MTRN4110_PhaseB] "<< "Writing path plan to ../../PathPlan.txt..."<<std::endl;
    output <<"[z5162972_MTRN4110_PhaseB] "<< "Writing path plan to ../../PathPlan.txt...\n";
    ofstream myfile;
    myfile.open ("../../PathPlan.txt");
    myfile << allMotionPlans[minElementIndex];
    myfile<<"\n";
    myfile.close();
    std::cout<<"[z5162972_MTRN4110_PhaseB] "<< "Path plan written to ../../PathPlan.txt!"<<std::endl;
    output << "[z5162972_MTRN4110_PhaseB] "<< "Path plan written to ../../PathPlan.txt!\n";
    output.close();
  delete robot;
  return 0;
}

string printPath(vector< pair <int,int> > path, int horizontal[10][6], double vertical[10][6],int start_x,int start_y,int end_x,int end_y, char heading){
  //ofstream output;
  //output.open ("../../Output.txt");
  vector< pair <int,int> > adjustedPath;
  string ret;
  int index = 0;
  for (auto& it : path) {
      //cout << it << ' ';
      adjustedPath.push_back( make_pair(it.first,(it.second*2)+1));
  }
  for(int k = 0; k < 6; k++){
       std::cout<<"[z5162972_MTRN4110_PhaseB] ";
       ret = ret + "[z5162972_MTRN4110_PhaseB] ";
      for (int j = 0; j < 17; j++){
          if(j == 0) {
            std::cout <<" ";
            ret = ret +" ";
          }
          if(horizontal[j][k] == 1){
            std::cout << "...";
            ret = ret +"...";
          }else{
            if(j%2){
              std::cout<< " ";
              ret = ret +" ";
            }else{ 
              std::cout << "   ";
              ret = ret +"   ";
            }
          }
      }
      std::cout <<"" <<std::endl;
      ret = ret +"\n";
      
      if(k < 5){
      std::cout<<"[z5162972_MTRN4110_PhaseB] ";
      ret = ret +"[z5162972_MTRN4110_PhaseB] ";
      for (int j = 0; j < 19; j++){
        auto it = std::find(adjustedPath.begin(), adjustedPath.end(), make_pair(k,j));
        if(it != adjustedPath.end()){
          index = it - adjustedPath.end() +1;
          if(abs(index) > 9){
            std::cout << " " << abs(index);
            ret = ret + " " + to_string(abs(index));
          }else{
            std::cout << " " << abs(index) << " ";
            ret = ret + " " +to_string(abs(index)) + " ";
          }
        }else if(j == (start_x*2 +1) && k == start_y){
          if(heading == 'S'){
            std::cout << " v ";
            ret = ret +" v ";
          }else if(heading == 'N'){
            std::cout << " ^ ";
            ret = ret +" ^ ";
          } else if(heading == 'W'){
            std::cout << " < ";
            ret = ret +" < ";
          }else if(heading == 'E'){
            std::cout << " > ";
            ret = ret +" > ";
          }
        }else if(j == (end_x*2 +1) && k == end_y) {
          std::cout << " x ";
          ret = ret +" x ";
        }else if(vertical[j][k] == 1){
            std::cout << "|";
            ret = ret +"|";
        }else{
            if(j%2){
              std::cout<< "   ";
              ret = ret +"   ";
            }else{ 
              std::cout << " ";
              ret = ret +" ";
            }
        }
          
      }
    }
      std::cout <<"" <<std::endl;
      
      //output.close();
      
  }
  return ret;

}

void DFS(int start_x, int start_y,int goal_x,int goal_y, int currentExploredValue, int horizontal[19][6], double vertical[19][6], int floodfill[9][5], vector< pair <int,int> > path, char heading,int turns,string motion_plan)
{
    int j = start_x;
    int k = start_y;
    if(j == goal_x && k == goal_y){
    //  std::cout<<"Append: "<<currentExploredValue<<std::endl;
      allPaths.push_back(path);
      allTurns.push_back(turns);
      allMotionPlans.push_back(motion_plan);
      return;
    }
      
      if(k > 0){
        //North
        if((floodfill[j][k-1] == currentExploredValue) && (horizontal[j*2][k] != 1)){
        //  std::cout<<"DFS - NORTH"<<"(" << j << "," << k-1 <<")"<<std::endl;
          path.push_back( make_pair(k-1,j));
          if(heading == 'N'){
            DFS(j,k-1,goal_x,goal_y,currentExploredValue+1, horizontal, vertical, floodfill,path,heading,turns,motion_plan+'F');
          }else if (heading == 'S'){
            DFS(j,k-1,goal_x,goal_y,currentExploredValue+1, horizontal, vertical, floodfill,path,'N',turns+2,motion_plan+'L'+'L'+'F');
          }else if (heading == 'W'){
            DFS(j,k-1,goal_x,goal_y,currentExploredValue+1, horizontal, vertical, floodfill,path,'N',turns +1,motion_plan+'R'+'F');
          }else if (heading == 'E'){
            DFS(j,k-1,goal_x,goal_y,currentExploredValue+1, horizontal, vertical, floodfill,path,'N',turns +1,motion_plan+'L'+'F');
          }
          path.pop_back();

          
        }
        //std::cout << "NORTH"<<std::endl;
      }
      if(k < 4){
        //SOUTH
        if((floodfill[j][k+1] == currentExploredValue) && (horizontal[j*2][k+1] != 1)){
       //   std::cout<<"DFS - SOUTH"<<"(" << j << "," << k+1 <<")"<<std::endl;
          path.push_back( make_pair(k+1,j));
          //DFS(j,k+1,goal_x,goal_y,currentExploredValue+1, horizontal, vertical, floodfill,path);
          if(heading == 'S'){
            DFS(j,k+1,goal_x,goal_y,currentExploredValue+1, horizontal, vertical, floodfill,path,heading,turns,motion_plan+'F');
          }else if (heading == 'N'){
            DFS(j,k+1,goal_x,goal_y,currentExploredValue+1, horizontal, vertical, floodfill,path,'S',turns+2,motion_plan+'L'+'L'+'F');
          }else if (heading == 'W'){
            DFS(j,k+1,goal_x,goal_y,currentExploredValue+1, horizontal, vertical, floodfill,path,'S',turns +1,motion_plan+'L'+'F');
          }else if (heading == 'E'){
            DFS(j,k+1,goal_x,goal_y,currentExploredValue+1, horizontal, vertical, floodfill,path,'S',turns +1,motion_plan+'R'+'F');
          }
          path.pop_back();
        }
        //std::cout << "SOUTH"<<std::endl;
      }
      if(j > 0){
        //WEST
        if((floodfill[j-1][k] == currentExploredValue) && (vertical[j*2][k] != 1)){
     //     std::cout<<"DFS - WEST"<<"(" << j-1 << "," << k<< ")"<<std::endl;
          path.push_back( make_pair(k,j-1));
          //DFS(j-1,k,goal_x,goal_y,currentExploredValue+1, horizontal, vertical, floodfill,path);    
          if(heading == 'W'){
            DFS(j-1,k,goal_x,goal_y,currentExploredValue+1, horizontal, vertical, floodfill,path,heading,turns,motion_plan+'F');
          }else if (heading == 'E'){
            DFS(j-1,k,goal_x,goal_y,currentExploredValue+1, horizontal, vertical, floodfill,path,'W',turns+2,motion_plan+'L'+'L'+'F');
          }else if(heading == 'N'){
            DFS(j-1,k,goal_x,goal_y,currentExploredValue+1, horizontal, vertical, floodfill,path,'W',turns +1,motion_plan+'L'+'F');
          }else if(heading == 'S'){
            DFS(j-1,k,goal_x,goal_y,currentExploredValue+1, horizontal, vertical, floodfill,path,'W',turns +1,motion_plan+'R'+'F');
          }
          path.pop_back();           
        }
        //std::cout << "WEST"<<std::endl;
      }
      if(j< 18){
        //EAST
        if((floodfill[j+1][k] == currentExploredValue) && (vertical[(j+1)*2][k] != 1)){
        //  std::cout<<"DFS - EAST"<<"(" << j+1 << "," << k<<")"<<std::endl;
          path.push_back( make_pair(k,j+1));
          //DFS(j+1,k,goal_x,goal_y,currentExploredValue+1, horizontal, vertical, floodfill,path);
          if(heading == 'E'){
            DFS(j+1,k,goal_x,goal_y,currentExploredValue+1, horizontal, vertical, floodfill,path,heading,turns,motion_plan+'F');
          }else if (heading == 'W'){
            DFS(j+1,k,goal_x,goal_y,currentExploredValue+1, horizontal, vertical, floodfill,path,'E',turns+2,motion_plan+'L'+'L'+'F');
          }else if (heading == 'N'){
            DFS(j+1,k,goal_x,goal_y,currentExploredValue+1, horizontal, vertical, floodfill,path,'E',turns +1,motion_plan+'R'+'F');
          }else if(heading == 'S'){
            DFS(j+1,k,goal_x,goal_y,currentExploredValue+1, horizontal, vertical, floodfill,path,'E',turns +1,motion_plan+'L'+'F');
          }
          path.pop_back();
        }
        //std::cout << "EAST"<<std::endl;
      }

    return;
}



vector< pair <int,int> > find_paths(int goal_x, int goal_y, int currentExploredValue, int horizontal[10][6], double vertical[10][6], int floodfill[9][5]){
  vector< pair <int,int> > path;
    path.push_back( make_pair(goal_y,goal_x));
    
    int curr_x = goal_x;
    int curr_y = goal_y;
    int pathUpdated = 1;
    currentExploredValue = currentExploredValue - 1;
    while(pathUpdated != 0){
      currentExploredValue --;
      if(currentExploredValue == 0){
        break;
      }
      pathUpdated = 0;
      for(int k = 0; k < 5; k++){ //For all Rows
          for (int j = 0; j < 9; j++){ // For all columns
            if(j == curr_x && k == curr_y){
            //std::cout << floodfill[curr_x][curr_y] << "," <<currentExploredValue <<std::endl;
              if(k > 0){
                if((floodfill[j][k-1] == currentExploredValue) && (horizontal[j*2][k] != 1)){
                  path.push_back( make_pair(k-1,j));
                  curr_x = j;
                  curr_y = k-1;
                  pathUpdated = 1;
                  continue;
                }
              }
              if(k < 4){
                if((floodfill[j][k+1] == currentExploredValue) && (horizontal[j*2][k+1] != 1)){
                  path.push_back( make_pair(k+1,j));
                  curr_x = j;
                  curr_y = k+1;
                  pathUpdated = 1;
                  continue;
                }
              }
              if(j > 0){
                if((floodfill[j-1][k] == currentExploredValue) && (vertical[j*2][k] != 1)){
                  path.push_back( make_pair(k,j-1));
                  curr_x = j-1;
                  curr_y = k;
                  pathUpdated = 1;
                  continue;
                }
              }
              if(j!= 8){
                if((floodfill[j+1][k] == currentExploredValue) && (vertical[(j+1)*2][k] != 1)){
                  path.push_back( make_pair(k,j+1));
                  curr_x = j+1;
                  curr_y = k;
                  pathUpdated = 1;
                  continue;
                }
              }
            
          
          }
          }
      }
    }
      reverse(path.begin(), path.end());
    
    return path;

}
