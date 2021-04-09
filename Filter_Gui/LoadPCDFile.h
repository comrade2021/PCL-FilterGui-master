#pragma once
#pragma warning(disable:4996)

#include "filter_gui.h"+
#include "ui_filter_gui.h"

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h> //PCL��PCD��ʽ�ļ����������ͷ�ļ�
#include <pcl/io/ply_io.h> //PCL��ply��ʽ�ļ����������ͷ�ļ�


class LoadPCDFile
{
public:
	explicit
		LoadPCDFile();

	static void loadPCD(PointCloudT::Ptr cloud, QString filename, bool* thread_ended);
};