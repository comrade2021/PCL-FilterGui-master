#pragma once
#pragma warning(disable:4996)

#include "filter_gui.h"

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/random.hpp>
#include <pcl/console/time.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/kdtree/kdtree_flann.h>

//Ïß³Ì
#include <thread>


class GaussianFilter
{
public:
	explicit
		GaussianFilter();

	static void filting(PointCloudT::Ptr cloud_, PointCloudT::Ptr cloud_filtered_, bool* filte_ended, float kernel_sigma, float kernel_threshold, float radius_search, int number_of_threads);
};