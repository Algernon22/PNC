#include "Astar_searcher.h"

using namespace std;
using namespace Eigen;

/*
    构建一个覆盖整个地图的三维网格节点数组 GridNodeMap，并为每个网格节点分配内存、初始化索引和物理坐标

    global_xyz_l  全局下限坐标
    global_xyz_u  全局上限坐标
    _resolution   物理世界中单个网格的边长
*/
void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{   
    // 设定 x y z 的边界
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);
    
    // 各维度上网格数量
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;

    // 计算整个三维空间的总网格数
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;
    // 分辨率的倒数
    inv_resolution = 1.0 / _resolution;    

    // 动态分配一维数组，大小为总网格数,将数组所有元素初始化为0
    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
    
    // 分配一个大小为 GLX_SIZE（x方向网格数）指针数组，每个元素指向一个 GridNodePtr **（即二维指针数组）
    // data 数组仅存储障碍物信息（轻量级），而 GridNodeMap 存储完整的节点信息（如代价、状态等）
    GridNodeMap = new GridNodePtr ** [GLX_SIZE]; 
    for(int i = 0; i < GLX_SIZE; i++){
        // 分配一个大小为 GLY_SIZE（x方向网格数）指针数组，每个元素指向一个 GridNodePtr *（即一维指针数组）
        GridNodeMap[i] = new GridNodePtr * [GLY_SIZE];
        for(int j = 0; j < GLY_SIZE; j++){
            // 分配一个大小为 GLZ_SIZE（x方向网格数）指针数组，每个元素指向一个 GridNodePtr 
            GridNodeMap[i][j] = new GridNodePtr [GLZ_SIZE];
            for( int k = 0; k < GLZ_SIZE;k++){
                // 储当前的索引(i, j, k)
                Vector3i tmpIdx(i,j,k);
                // 索引→物理坐标
                Vector3d pos = gridIndex2coord(tmpIdx);
                // 为每个索引创建节点对象
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
            }
        }
    }
}

void AstarPathFinder::resetGrid(GridNodePtr ptr)
{
    ptr->id = 0;
    ptr->cameFrom = NULL;
    ptr->gScore = inf;
    ptr->fScore = inf;
}

void AstarPathFinder::resetUsedGrids()
{   
    for(int i=0; i < GLX_SIZE ; i++)
        for(int j=0; j < GLY_SIZE ; j++)
            for(int k=0; k < GLZ_SIZE ; k++)
                resetGrid(GridNodeMap[i][j][k]);
}

/*
    将指定的物理坐标转换为网格索引后，标记对应的网格为障碍物状态
*/
void AstarPathFinder::setObs(const double coord_x, const double coord_y, const double coord_z)
{
    // 边界检查
    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl || 
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return;

    // 计算各维度上全局坐标的网格索引,static_cast<int> 直接截断小数部分（等价于向下取整）
    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);      

    // 标记障碍物
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

/*
    收集所有被访问过的网格节点,用于后续可视化。

*/
vector<Vector3d> AstarPathFinder::getVisitedNodes()
{   
    vector<Vector3d> visited_nodes;
    for(int i = 0; i < GLX_SIZE; i++)
        for(int j = 0; j < GLY_SIZE; j++)
            for(int k = 0; k < GLZ_SIZE; k++){   
                //  // visualize nodes in close list only
                if(GridNodeMap[i][j][k]->id == -1) 
                    // 将符合条件的节点的物理坐标加入结果列表
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }

    ROS_WARN("visited_nodes size : %d", visited_nodes.size());
    return visited_nodes;
}

/*
    将离散的网格索引 (i,j,k) 转换为连续的物理坐标 (x,y,z)
*/
Vector3d AstarPathFinder::gridIndex2coord(const Vector3i & index) 
{
    Vector3d pt;
    // +0.5是为了得到网格中心的坐标，而不是网格边缘的坐标
    // + gl_*l *轴的原点偏移量
    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

/*
    将连续的物理坐标 (x,y,z) 转换为离散的网格索引 (i,j,k)
*/ 
Vector3i AstarPathFinder::coord2gridIndex(const Vector3d & pt) 
{
    Vector3i idx;
    // 使用 max 函数确保索引不小于 0，使用 min 函数确保索引不大于 GLX_SIZE - 1
    // pt(0) - gl_*l == *轴的原点偏移量
    // pt(0) - gl_xl) * inv_resolution = 分辨率的数量，也就是索引
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);                  
    return idx;
}

/*
    先通过 coord2gridIndex 将物理坐标转换为离散的网格索引 (i,j,k)
    再通过 gridIndex2coord 将网格索引转换回网格中心点的物理坐标
*/
Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d & coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

// inline 是一种向编译器提出的建议，它告诉编译器尝试将函数体替换到每个调用该函数的地方（即内联展开）。这样做的目的是减少函数调用的开销，特别是对于那些体积小、频繁调用的函数。
// 在成员函数声明的末尾使用 const 表示该成员函数不会修改任何成员变量的值（即它是一个只读函数）
inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i & index) const
{
    // 调用了下面的isOccupied
    return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isFree(const Eigen::Vector3i & index) const
{
    return isFree(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && 
            idx_y >= 0 && idx_y < GLY_SIZE && 
            idx_z >= 0 && idx_z < GLZ_SIZE && 
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));

}

inline bool AstarPathFinder::isFree(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && 
            idx_y >= 0 && idx_y < GLY_SIZE && 
            idx_z >= 0 && idx_z < GLZ_SIZE && 
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));

}

/*
    获取当前节点的所有可行邻居节点（后继节点）。检查当前节点周围的 26 邻域，排除障碍物和自身节点，并计算当前节点到每个邻居节点的移动代价（欧氏距离）
    
    currentPtr：当前网格节点的指针（GridNodePtr）。
    neighborPtrSets：用于存储邻居节点指针的向量（输出参数）。
    edgeCostSets：用于存储当前节点到邻居节点的移动代价的向量（输出参数）。
*/
inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> & neighborPtrSets, vector<double> & edgeCostSets)
{   
    // 清空邻居节点和移动代价的容器，确保每次调用时结果独立
    neighborPtrSets.clear();
    edgeCostSets.clear();

    /*
        STEP 4: finish AstarPathFinder::AstarGetSucc yourself 
        please write your code below
    */

    Vector3i current_index = currentPtr->index;

    const int offsets[26][3] = {
        // 6 个面邻居（沿 x/y/z 轴移动 1 步）
        {-1,  0,  0}, {1,  0,  0},  // x 轴方向
        { 0, -1,  0}, {0,  1,  0},  // y 轴方向
        { 0,  0, -1}, {0,  0,  1},  // z 轴方向

        // 12 个边邻居（沿两个轴各移动 1 步）
        {-1, -1,  0}, {-1,  1,  0}, {1, -1,  0}, {1,  1,  0},  // xy 平面
        {-1,  0, -1}, {-1,  0,  1}, {1,  0, -1}, {1,  0,  1},  // xz 平面
        { 0, -1, -1}, { 0, -1,  1}, {0,  1, -1}, {0,  1,  1},  // yz 平面

        // 8 个角邻居（沿 x/y/z 轴各移动 1 步）
        {-1, -1, -1}, {-1, -1,  1}, {-1,  1, -1}, {-1,  1,  1},
        { 1, -1, -1}, { 1, -1,  1}, { 1,  1, -1}, { 1,  1,  1}
    };

    for (const auto &offset : offsets) {
        Vector3i neighbor_index(
            current_index(0) + offset[0],
            current_index(1) + offset[1],
            current_index(2) + offset[2]
        );
        // 检查是否非障碍物
        if (isFree(neighbor_index) && neighbor_index != current_index) {  
            neighborPtrSets.emplace_back(GridNodeMap[neighbor_index(0)][neighbor_index(1)][neighbor_index(2)]);
            edgeCostSets.emplace_back(
                sqrt(offset[0]*offset[0] + offset[1]*offset[1] + offset[2]*offset[2])
            );
        }
    }
}

// 辅助函数：计算三个数的中位数
double median(double a, double b, double c) {
    double arr[3] = {a, b, c};
    std::sort(arr, arr + 3);
    return arr[1];
}

/* 
    choose possible heuristic function you want
    Manhattan, Euclidean, Diagonal, or 0 (Dijkstra)
    Remember tie_breaker learned in lecture, add it here ?

    STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    please write your code below

*/
double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2) {
    if (!node1 || !node2) {
        ROS_ERROR("Invalid node pointer");
        return std::numeric_limits<double>::infinity();
    }
    double dx = abs(node2->index[0] - node1->index[0]);
    double dy = abs(node2->index[1] - node1->index[1]);
    double dz = abs(node2->index[2] - node1->index[2]);

    // 计算对角线距离
    double min_d = std::min({dx, dy, dz});
    double med_d = median(dx, dy, dz); 
    double diagonal = (dx + dy + dz) - (3.0 - std::sqrt(3.0)) * min_d - (2.0 - std::sqrt(2.0)) * (med_d - min_d);
    
    // Tie-breaker
    const double epsilon = 1e-3;
    return diagonal * (1.0 + epsilon);
}

void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt)
{   
    ros::Time time_1 = ros::Time::now();    

    // 将世界坐标系下的起点终点转换为网格索引
    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx   = coord2gridIndex(end_pt);
    goalIdx = end_idx;

    // 将网格索引转换回世界坐标，确保对齐网格中心
    start_pt = gridIndex2coord(start_idx);
    end_pt = gridIndex2coord(end_idx);

     // 创建起点和终点的节点对象
    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr = new GridNode(end_idx, end_pt);

    // 清空开放集(使用multimap实现，自动按fScore排序)
    openSet.clear();

    // 定义当前节点(Open List中f值最小的节点)和邻居节点指针
    GridNodePtr currentPtr = NULL;
    GridNodePtr neighborPtr = NULL;

    // 起点到起点的代价为0
    startPtr -> gScore = 0;
    // STEP 1: 启发式函数估计总代价
    startPtr -> fScore = getHeu(startPtr,endPtr);   
    startPtr -> id = 1; 
    startPtr -> coord = start_pt;
    // make_pair 接受两个参数，第一个参数是键，第二个参数是值,创建一个键值对
    openSet.insert( make_pair(startPtr -> fScore, startPtr) );

    /* STEP 2: 主循环前的准备工作 */
    // 定义邻居节点集合和边代价集合
    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;

    // this is the main loop
    while ( !openSet.empty() ){

        /* STEP 3: 从开放集中取出f值最小的节点 */
        // openSet.begin()->second指向键值对的第二个元素
        currentPtr = openSet.begin()->second;
        currentPtr->id = -1;
        openSet.erase(openSet.begin());

        // 终止条件检查 
        if( currentPtr->index == goalIdx ){
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            ROS_WARN("[A*]{sucess} Time in A* is %f ms, path cost is %f m", 
                   (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution);            
            return;
        }

        /* STEP 4: 获取当前节点的所有邻居节点 */
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);       

        /* STEP 5-7: 处理所有邻居节点 */      
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){

            // 当前邻居节点
            neighborPtr = neighborPtrSets[i];
            // 当前边的代价  
            double edge_cost = edgeCostSets[i];  

            /* 检查邻居节点状态 */
            if(neighborPtr -> id == 0){ 
                /* STEP 6: 处理新发现的节点 */
                // 更新g值
                neighborPtr->gScore = currentPtr->gScore + edge_cost;  
                // 计算f值
                neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr, endPtr); 
                // 记录父节点
                neighborPtr->cameFrom = currentPtr;  
                // 标记为在开放集中
                neighborPtr->id = 1;  
                openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));  
                continue;
            }
            else if (neighborPtr->id == 1) 
            {
                /* STEP 7: 处理已在开放集中的节点 */
                double tentative_gScore = currentPtr->gScore + edge_cost;
                // 找到更优路径，需要更新节点信息
                if (tentative_gScore < neighborPtr->gScore) 
                { 
                    // 从开放集中移除旧记录
                    auto range = openSet.equal_range(neighborPtr->fScore);
                    for (auto it = range.first; it != range.second; ++it) 
                    {
                        if (it->second->index == neighborPtr->index) 
                        {
                            openSet.erase(it);
                            break;
                        }
                    }
                    
                    // 更新节点信息
                    neighborPtr->gScore = tentative_gScore;
                    neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr, endPtr);
                    neighborPtr->cameFrom = currentPtr;
                    
                    // 重新加入开放集
                    openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));
                }
            }
            else{
                /* 节点已在闭合集中，标准A*不考虑重新开放 */
                continue;
            }
        }      
    }

    /* 搜索失败处理 */
    ros::Time time_2 = ros::Time::now();
    if((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec() );
}

vector<Vector3d> AstarPathFinder::getPath() 
{   
    // 存储最终路径坐标
    vector<Vector3d> path;
    // 临时存储路径节点指针
    vector<GridNodePtr> gridPath;

    /* STEP 8: 从终止节点回溯到起点，获取完整路径 */
    GridNodePtr currentPtr = terminatePtr;

    while (currentPtr != nullptr) 
    {
        // 将当前节点加入路径
        gridPath.push_back(currentPtr);
        
        // 移动到父节点
        currentPtr = currentPtr->cameFrom;
    }
    
    /* 将节点指针转换为坐标并反转顺序 */
    for (auto ptr : gridPath) 
    {
        // 直接使用节点存储的坐标，避免重复计算
        path.push_back(ptr->coord);
    }
        
    // 反转路径顺序(从起点到终点)
    reverse(path.begin(), path.end());

    return path;
}