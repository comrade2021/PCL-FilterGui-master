#include "LoadPCDFile.h"

LoadPCDFile::LoadPCDFile()
{
}

void LoadPCDFile::loadPCD(PointCloudT::Ptr cloud, QString filename, bool* thread_ended)
{
	PointCloudT::Ptr cloud_tmp(new PointCloudT);
	std::cout << "QString=" << filename.toStdString() <<endl;


	if (filename.isEmpty())
	{
		std::cerr << "filename is Empty" << std::endl;
		return;
	}

	int return_status;
	if (filename.endsWith(".pcd", Qt::CaseInsensitive))
		return_status = pcl::io::loadPCDFile(filename.toStdString(), *cloud_tmp);
	else
		return_status = pcl::io::loadPLYFile(filename.toStdString(), *cloud_tmp);

	if (return_status != 0)
	{
		PCL_ERROR("Error reading point cloud %s\n", filename.toStdString().c_str());
		return;
	}

	PCL_WARN("file has loaded\n");
	// If point cloud contains NaN values, remove them before updating the visualizer point cloud
	if (cloud_tmp->is_dense)
		pcl::copyPointCloud(*cloud_tmp, *cloud);
	else
	{
		PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
		std::vector<int> vec;
		pcl::removeNaNFromPointCloud(*cloud_tmp, *cloud, vec);
	}

	*thread_ended = true;//说明操作完成
	std::cerr << "(loadPCD)thread_ended="<< thread_ended << std::endl;
	std::cerr << "LoadPCDFile Ended" << std::endl;
}
