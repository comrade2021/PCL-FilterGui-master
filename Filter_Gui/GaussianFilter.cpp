#pragma once
#pragma warning(disable:4996)

#include "GaussianFilter.h"

GaussianFilter::GaussianFilter()
{
}

void GaussianFilter::filting(PointCloudT::Ptr cloud_, PointCloudT::Ptr cloud_filtered_, bool* filte_ended, float kernel_sigma, float kernel_threshold, float radius_search, int number_of_threads)
{
	//Set up the Gaussian Kernel
	pcl::filters::GaussianKernel<PointT, PointT>::Ptr kernel(new pcl::filters::GaussianKernel<PointT, PointT>);
	(*kernel).setSigma(kernel_sigma);//设置高斯函数sigma值
	(*kernel).setThresholdRelativeToSigma(kernel_threshold); //依据核函数方差获得关联作用域

	//Set up the KDTree
	pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
	(*kdtree).setInputCloud(cloud_);

	//Set up the Convolution Filter
	pcl::filters::Convolution3D <PointT, PointT, pcl::filters::GaussianKernel<PointT, PointT> > convolution;
	convolution.setKernel(*kernel);
	convolution.setInputCloud(cloud_);
	convolution.setSearchMethod(kdtree);
	convolution.setRadiusSearch(radius_search);//k近邻搜索的范围
	//在VS属性管理器-c++-语言-开启OpenMP支持-加速(pcl滤波器一般都有加速的设置)
	convolution.setNumberOfThreads(number_of_threads);//important! Set Thread number for openMP;设置为CPU核数
	std::cerr << "Gaussian Filter Start" << std::endl;
	convolution.convolve(*cloud_filtered_);
	std::cerr << "filte_ended:" << *filte_ended << std::endl;
	//执行结束后将全局变量置1，说明滤波过程已经结束
	*filte_ended = true;
	std::cerr << "filte_ended:" << *filte_ended << std::endl;
	std::cerr << "Gaussian Filter Ended" << std::endl;
}

