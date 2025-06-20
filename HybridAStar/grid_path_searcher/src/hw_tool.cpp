#include <hw_tool.h>

using namespace std;
using namespace Eigen;

void Homeworktool::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{   
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);
    
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;    

    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
}

void Homeworktool::setObs(const double coord_x, const double coord_y, const double coord_z)
{   
    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl || 
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return;

    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);      
    
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

bool Homeworktool::isObsFree(const double coord_x, const double coord_y, const double coord_z)
{
    Vector3d pt;
    Vector3i idx;
    
    pt(0) = coord_x;
    pt(1) = coord_y;
    pt(2) = coord_z;
    idx = coord2gridIndex(pt);

    int idx_x = idx(0);
    int idx_y = idx(1);
    int idx_z = idx(2);

    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

Vector3d Homeworktool::gridIndex2coord(const Vector3i & index) 
{
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

Vector3i Homeworktool::coord2gridIndex(const Vector3d & pt) 
{
    Vector3i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);                  
  
    return idx;
}

Eigen::Vector3d Homeworktool::coordRounding(const Eigen::Vector3d & coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

/*
    计算从起点状态（位置 + 速度）到目标位置的最优轨迹成本
*/
double Homeworktool::OptimalBVP(Eigen::Vector3d _start_position, Eigen::Vector3d _start_velocity, Eigen::Vector3d _target_position)
 {
    // 成本初始化为最大值
    double optimal_cost = std::numeric_limits<double>::max();  
    
    // 计算相对位移和速度
    const double dx = _target_position(0) - _start_position(0);
    const double dy = _target_position(1) - _start_position(1);
    const double dz = _target_position(2) - _start_position(2);
    const double vx = _start_velocity(0);
    const double vy = _start_velocity(1);
    const double vz = _start_velocity(2);

    // 预计算常用项
    const double dx_sq = dx * dx;
    const double dy_sq = dy * dy;
    const double dz_sq = dz * dz;
    const double dot_dv = dx * vx + dy * vy + dz * vz;  // 位移与速度的点积
    const double v_sq = vx * vx + vy * vy + vz * vz;    // 速度的平方和

    // 构造多项式方程：coeff[0] + coeff[1]*x + coeff[2]*x² + coeff[3]*x³ + coeff[4]*x⁴   -9(dx^2 + dy^2 + dz^2) + 12(dxvx + dyvy + dzvz)T - v^2 T^2 + T^4 = 0
    Eigen::VectorXd coeff(5);
    coeff << -9.0 * (dx_sq + dy_sq + dz_sq),
             12.0 * dot_dv,
             -v_sq,
             0.0,
             1.0;

    // 求解多项式方程的根
    Eigen::PolynomialSolver<double, Eigen::Dynamic> solver;
    solver.compute(coeff);
    const auto& roots = solver.roots();  // 获取所有根（复数形式）

    // 筛选实数根（忽略虚部大于1e-10的根）
    std::vector<double> real_roots;
    for (int i = 0; i < roots.size(); ++i) {
        if (std::abs(roots[i].imag()) > 1e-10) 
            continue;
        real_roots.push_back(roots[i].real());
    }

    // 计算每个实根对应的轨迹成本，并存储到 multimap（自动按 cost 排序）
    std::multimap<double, double> trajectory_costs;
    for (const double T : real_roots) {
        if (T <= 0.0) 
            continue;  // 忽略非正根

        // 计算成本函数 J(T)
        const double T_sq = T * T;
        const double T_cubed = T_sq * T;
        const double cost = T 
                          + 3.0 * (dx_sq + dy_sq + dz_sq) / T_cubed
                          - 6.0 * dot_dv / T_sq
                          + v_sq / T;

        if (cost > 0) {
            trajectory_costs.emplace(cost, T);  // 使用 emplace 提高效率
        }
    }

    // 返回最小成本（如果存在有效解）
    if (!trajectory_costs.empty()) {
        optimal_cost = trajectory_costs.begin()->first;
    }

    return optimal_cost;
}


