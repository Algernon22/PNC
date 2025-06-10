#include "JPS_searcher.h"

using namespace std;
using namespace Eigen;

/*
    获取当前节点的后继节点（邻居节点）

    currentPtr 寻找所有有效的跳跃点后继节点（JPS算法的关键优化）。
    neighborPtrSets  后继节点集合
    edgeCostSets  移动代价集合
*/
inline void JPSPathFinder::JPSGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> & neighborPtrSets, vector<double> & edgeCostSets)
{
    // 清空输出容器，准备填充新数据
    neighborPtrSets.clear();
    edgeCostSets.clear();

    // norm1 = |dx| + |dy| + |dz|  表示当前移动方向的“强度”
    const int norm1 = abs(currentPtr->dir(0)) + abs(currentPtr->dir(1)) + abs(currentPtr->dir(2));

    // 自然邻居方向数量
    int num_neib  = jn3d->nsz[norm1][0];
    // 强制邻居方向数量
    int num_fneib = jn3d->nsz[norm1][1];

    // 将三维方向 (dx,dy,dz)（每个分量 ∈ {-1,0,1}）映射到 0~26 的唯一整数 id
    int id = (currentPtr->dir(0) + 1) + 3 * (currentPtr->dir(1) + 1) + 9 * (currentPtr->dir(2) + 1);

    // 遍历所有 自然邻居 + 强制邻居 方向
    for( int dev = 0; dev < num_neib + num_fneib; ++dev) {
        Vector3i neighborIdx;
        Vector3i expandDir;

        if( dev < num_neib ) {
            expandDir(0) = jn3d->ns[id][0][dev];
            expandDir(1) = jn3d->ns[id][1][dev];
            expandDir(2) = jn3d->ns[id][2][dev];
            
            if( !jump(currentPtr->index, expandDir, neighborIdx) )  
                continue;
        }
        else {
            int nx = currentPtr->index(0) + jn3d->f1[id][0][dev - num_neib];
            int ny = currentPtr->index(1) + jn3d->f1[id][1][dev - num_neib];
            int nz = currentPtr->index(2) + jn3d->f1[id][2][dev - num_neib];
            
            if( isOccupied(nx, ny, nz) ) {
                expandDir(0) = jn3d->f2[id][0][dev - num_neib];
                expandDir(1) = jn3d->f2[id][1][dev - num_neib];
                expandDir(2) = jn3d->f2[id][2][dev - num_neib];
                
                if( !jump(currentPtr->index, expandDir, neighborIdx) ) 
                    continue;
            }
            else
                continue;
        }

        GridNodePtr nodePtr = GridNodeMap[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
        nodePtr->dir = expandDir;
        
        neighborPtrSets.push_back(nodePtr);
        edgeCostSets.push_back(
            sqrt(
            (neighborIdx(0) - currentPtr->index(0)) * (neighborIdx(0) - currentPtr->index(0)) +
            (neighborIdx(1) - currentPtr->index(1)) * (neighborIdx(1) - currentPtr->index(1)) +
            (neighborIdx(2) - currentPtr->index(2)) * (neighborIdx(2) - currentPtr->index(2))   ) 
            );
    }
}

bool JPSPathFinder::jump(const Vector3i & curIdx, const Vector3i & expDir, Vector3i & neiIdx)
{
    neiIdx = curIdx + expDir;

    if( !isFree(neiIdx) )
        return false;

    if( neiIdx == goalIdx )
        return true;

    if( hasForced(neiIdx, expDir) )
        return true;

    const int id = (expDir(0) + 1) + 3 * (expDir(1) + 1) + 9 * (expDir(2) + 1);
    const int norm1 = abs(expDir(0)) + abs(expDir(1)) + abs(expDir(2));
    int num_neib = jn3d->nsz[norm1][0];

    for( int k = 0; k < num_neib - 1; ++k ){
        Vector3i newNeiIdx;
        Vector3i newDir(jn3d->ns[id][0][k], jn3d->ns[id][1][k], jn3d->ns[id][2][k]);
        if( jump(neiIdx, newDir, newNeiIdx) ) 
            return true;
    }

    return jump(neiIdx, expDir, neiIdx);
}

inline bool JPSPathFinder::hasForced(const Vector3i & idx, const Vector3i & dir)
{
    int norm1 = abs(dir(0)) + abs(dir(1)) + abs(dir(2));
    int id    = (dir(0) + 1) + 3 * (dir(1) + 1) + 9 * (dir(2) + 1);

    switch(norm1){
        case 1:
            // 1-d move, check 8 neighbors
            for( int fn = 0; fn < 8; ++fn ){
                int nx = idx(0) + jn3d->f1[id][0][fn];
                int ny = idx(1) + jn3d->f1[id][1][fn];
                int nz = idx(2) + jn3d->f1[id][2][fn];
                if( isOccupied(nx, ny, nz) )
                    return true;
            }
            return false;

        case 2:
            // 2-d move, check 8 neighbors
            for( int fn = 0; fn < 8; ++fn ){
                int nx = idx(0) + jn3d->f1[id][0][fn];
                int ny = idx(1) + jn3d->f1[id][1][fn];
                int nz = idx(2) + jn3d->f1[id][2][fn];
                if( isOccupied(nx, ny, nz) )
                    return true;
            }
            return false;

        case 3:
            // 3-d move, check 6 neighbors
            for( int fn = 0; fn < 6; ++fn ){
                int nx = idx(0) + jn3d->f1[id][0][fn];
                int ny = idx(1) + jn3d->f1[id][1][fn];
                int nz = idx(2) + jn3d->f1[id][2][fn];
                if( isOccupied(nx, ny, nz) )
                    return true;
            }
            return false;

        default:
            return false;
    }
}

inline bool JPSPathFinder::isOccupied(const Eigen::Vector3i & index) const
{
    return isOccupied(index(0), index(1), index(2));
}

inline bool JPSPathFinder::isFree(const Eigen::Vector3i & index) const
{
    return isFree(index(0), index(1), index(2));
}

inline bool JPSPathFinder::isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return  (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool JPSPathFinder::isFree(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

void JPSPathFinder::JPSGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt)
{
    ros::Time time_1 = ros::Time::now();    

    // 1. 将起点和终点转换为网格坐标
    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx   = coord2gridIndex(end_pt);
    goalIdx = end_idx;

    // 2. 确保起点和终点的坐标对齐网格中心
    start_pt = gridIndex2coord(start_idx);
    end_pt   = gridIndex2coord(end_idx);

    // 3. 初始化起点和终点的节点
    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr   = new GridNode(end_idx,   end_pt);

    // 4. 清空开放集（openSet），使用multimap实现（按fScore排序）
    openSet.clear();
    GridNodePtr currentPtr  = NULL;
    GridNodePtr neighborPtr = NULL;

    // 5. 将起点加入开放集
    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr,endPtr);   
    // STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    startPtr -> id = 1; 
    startPtr -> coord = start_pt;
    openSet.insert( make_pair(startPtr -> fScore, startPtr) );

    // 6. 初始化辅助变量
    double tentative_gScore;
    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;

    // 7. 主循环
    while ( !openSet.empty() ){
        // 7.1 从开放集中取出fScore最小的节点（multimap已按key排序）
        currentPtr = openSet.begin()->second;
        openSet.erase(openSet.begin());

        // 7.2 如果当前节点是目标点，则终止搜索 
        if( currentPtr->index == goalIdx ){
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            ROS_WARN("[JPS]{sucess} Time in JPS is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );    
            return;
        }

        // 7.3 获取当前节点的所有跳跃后继节点（JPS核心）
        JPSGetSucc(currentPtr, neighborPtrSets, edgeCostSets); //we have done it for you
        
        // 7.4 遍历所有后继节点         
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            neighborPtr = neighborPtrSets[i];
            tentative_gScore = currentPtr->gScore + edgeCostSets[i];

            // 7.4.1 如果邻居节点未被访问过
            if (neighborPtr->id != 1) {
                // 更新gScore和fScore
                neighborPtr->gScore = tentative_gScore;
                neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr, endPtr);
                // 标记为已访问
                neighborPtr->id = 1; 
                // 记录父节点
                neighborPtr->cameFrom = currentPtr; 

                // 将邻居节点加入开放集
                openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));
            }
            // 7.4.2 如果邻居节点在开放集中且新路径更优
            else if(tentative_gScore <= neighborPtr-> gScore){ 
                // 更新gScore和fScore
                neighborPtr->gScore = tentative_gScore;
                neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr, endPtr);
                neighborPtr->cameFrom = currentPtr;

                // 更新开放集中的节点（需先删除再重新插入）
                for (auto it = openSet.begin(); it != openSet.end(); ++it) {
                    if (it->second == neighborPtr) {
                        openSet.erase(it);
                        break;
                    }
                }
                openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));

                // JPS专用：更新扩展方向（用于后续跳跃）    
                for(int i = 0; i < 3; i++){
                    neighborPtr->dir(i) = neighborPtr->index(i) - currentPtr->index(i);
                    if( neighborPtr->dir(i) != 0)
                        neighborPtr->dir(i) /= abs( neighborPtr->dir(i) );
                }
            }      
        }
    }
    
    // 8. 搜索失败处理
    ros::Time time_2 = ros::Time::now();
    if((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in JPS path finding is %f", (time_2 - time_1).toSec() );
}