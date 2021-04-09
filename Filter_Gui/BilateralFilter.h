#pragma once
#pragma warning(disable:4996)

#include "filter_gui.h"

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/fast_bilateral.h>

//Ïß³Ì
#include <thread>


class BilateralFilter
{
public:
	explicit
		BilateralFilter();

	static void filting(PointCloudT::Ptr cloud_, PointCloudT::Ptr cloud_filtered_, bool* filte_ended, double sigma_s, double sigma_r);
};