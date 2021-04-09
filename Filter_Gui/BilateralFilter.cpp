#include "BilateralFilter.h"

BilateralFilter::BilateralFilter()
{
}

void BilateralFilter::filting(PointCloudT::Ptr cloud_, PointCloudT::Ptr cloud_filtered_, bool* filte_ended, double sigma_s, double sigma_r)
{
	// ´´½¨ÂË²¨Æ÷
	pcl::FastBilateralFilter<PointT> filter;
	filter.setInputCloud(cloud_);
	filter.setSigmaS(sigma_s);
	filter.setSigmaR(sigma_r);

	std::cerr << "Bilateral Filter Start" << std::endl;
	filter.applyFilter(*cloud_filtered_);
	*filte_ended = true;
	std::cerr << "Bilateral Filter Ended" << std::endl;
}
