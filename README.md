JPSNAStar使用方法 

​	下载JPSNAtar下的文件到工作空间/src 

```
	cd catkin_ws 
	catkin_make 
	roslaunch grid_path_searcher demo.launch 
```

​	使用Goal3DTool选择终止点即可实现全局规划（红色线是JPS规划出的路线、黑色线是AStar规划出的路线）





HybridAStar -- 满足动力学约束的全局规划算法

​	下载HybridAStar下的文件到工作空间/src

		cd catkin_ws 
		catkin_make 
		roslaunch grid_path_searcher demo.launch 

​	使用Goal3DTool选择终止点即可实现全局规划中的第一次局部规划





Minimum Snap Trajectory Generation使用方法

​	键入以下命令来安装依赖项

```
sudo apt-get install gfortran
sudo apt-get install doxygen
```

​	手动解压缩中的包*OOQP.zip，分别进入两个文件下执行以下命令

```
./configure
make 
sudo make install
```

​	下载Minimum Snap Trajectory Generation下的文件到工作空间/src

	cd catkin_ws 
	catkin_make 
	roslaunch waypoint_trajectory_generator test.launch
