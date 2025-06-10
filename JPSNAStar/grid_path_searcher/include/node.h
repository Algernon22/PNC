#ifndef _NODE_H_
#define _NODE_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include "backward.hpp"

#define inf 1>>20 // 通过右移操作定义无穷大
struct GridNode;
typedef GridNode* GridNodePtr;

struct GridNode
{     
    int id;        // 1--> open set, -1 --> closed set  0 not expand
    Eigen::Vector3d coord;  // world 3D position
    Eigen::Vector3i dir;   // direction of expanding
    Eigen::Vector3i index; // grid 3D position
	
    double gScore, fScore;
    GridNodePtr cameFrom; // mark the father node
    std::multimap<double, GridNodePtr>::iterator nodeMapIt; // neighbor_node集合

    GridNode(Eigen::Vector3i _index, Eigen::Vector3d _coord){  
      id = 0;
      index = _index;
      coord = _coord;
      dir   = Eigen::Vector3i::Zero();

      gScore = inf;
      fScore = inf;
      cameFrom = NULL;
    }

    GridNode(){};
    ~GridNode(){};
};


#endif
