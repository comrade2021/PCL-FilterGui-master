#pragma once
#pragma warning(disable:4996)

#include "filter_gui.h"+
#include "ui_filter_gui.h"

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h> //PCL的PCD格式文件的输入输出头文件
#include <pcl/io/ply_io.h> //PCL的ply格式文件的输入输出头文件


class LoadPCDFile
{
public:
	explicit
		LoadPCDFile();

	static void loadPCD(PointCloudT::Ptr cloud, QString filename, bool* thread_ended);
};