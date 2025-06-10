#ifndef _JPS_UTILS_H_
#define _JPS_UTILS_H_

#include <iostream>
// 预计算和存储三维JPS算法的 自然邻居 和 强制邻居方向
struct JPS3DNeib {
	// 自然邻居方向表
	int ns[27][3][26];
	// 强制邻居的障碍物检查方向
	int f1[27][3][12];
	// 强制邻居的实际跳跃方向
	int f2[27][3][12];
	// nsz contains the number of neighbors for the four different types of moves:
	// no move (norm 0):        26 neighbors always added
	//                          0 forced neighbors to check (never happens)
	//                          0 neighbors to add if forced (never happens)
	// straight (norm 1):       1 neighbor always added
	//                          8 forced neighbors to check
	//                          8 neighbors to add if forced
	// diagonal (norm sqrt(2)): 3 neighbors always added
	//                          8 forced neighbors to check
	//                          12 neighbors to add if forced
	// diagonal (norm sqrt(3)): 7 neighbors always added
	//                          6 forced neighbors to check
	//                          12 neighbors to add if forced
	static constexpr int nsz[4][2] = {
		{26, 0},  // norm=0（静止，无实际意义）
		{1, 8},   // norm=1（直线移动，如(1,0,0)）
		{3, 12},  // norm=2（对角线移动，如(1,1,0)）
		{7, 12}   // norm=3（体对角线移动，如(1,1,1)）
	};
	
	JPS3DNeib();
	private:
	void Neib(int dx, int dy, int dz, int norm1, int dev, int& tx, int& ty, int& tz);
	void FNeib( int dx, int dy, int dz, int norm1, int dev,
	    int& fx, int& fy, int& fz,
	    int& nx, int& ny, int& nz);
};

#endif