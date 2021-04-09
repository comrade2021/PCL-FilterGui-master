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
	(*kernel).setSigma(kernel_sigma);//���ø�˹����sigmaֵ
	(*kernel).setThresholdRelativeToSigma(kernel_threshold); //���ݺ˺��������ù���������

	//Set up the KDTree
	pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
	(*kdtree).setInputCloud(cloud_);

	//Set up the Convolution Filter
	pcl::filters::Convolution3D <PointT, PointT, pcl::filters::GaussianKernel<PointT, PointT> > convolution;
	convolution.setKernel(*kernel);
	convolution.setInputCloud(cloud_);
	convolution.setSearchMethod(kdtree);
	convolution.setRadiusSearch(radius_search);//k���������ķ�Χ
	//��VS���Թ�����-c++-����-����OpenMP֧��-����(pcl�˲���һ�㶼�м��ٵ�����)
	convolution.setNumberOfThreads(number_of_threads);//important! Set Thread number for openMP;����ΪCPU����
	std::cerr << "Gaussian Filter Start" << std::endl;
	convolution.convolve(*cloud_filtered_);
	std::cerr << "filte_ended:" << *filte_ended << std::endl;
	//ִ�н�����ȫ�ֱ�����1��˵���˲������Ѿ�����
	*filte_ended = true;
	std::cerr << "filte_ended:" << *filte_ended << std::endl;
	std::cerr << "Gaussian Filter Ended" << std::endl;
}

