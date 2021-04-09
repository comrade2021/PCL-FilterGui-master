#pragma once

#include "filter_gui.h"
#include "ui_filter_gui.h"

// VTK PCL Boost
#include "vtkAutoInit.h"
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/io/file_io.h>
#include <boost/math/special_functions/round.hpp>
#include <vtkOutputWindow.h>
// Filter
#include <iostream>
#include <ctime>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <boost/random.hpp>
#include <pcl/console/time.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/fast_bilateral.h>
//线程
#include <thread>
#include <mutex>
#include "GaussianFilter.h"
#include "BilateralFilter.h"
#include "LoadPCDFile.h"

//vtkOutputWindow::SetGlobalWarningDisplay(0);//不弹出vtkOutputWindow窗口

//std::mutex mu;


Filter_Gui::Filter_Gui(QWidget* parent) :
	QMainWindow(parent),
	ui(new Ui::Filter_GuiClass)
{
	//初始化UI
	ui->setupUi(this);
	QString str;
	str = str.fromLocal8Bit("点云滤波GUI");
	this->setWindowTitle(str);

	Filter_Gui::filte_ended = true;
	Filter_Gui::coordinateSystemAdded = true;

	// Setup the cloud pointer
	cloud_.reset(new PointCloudT);
	cloud_filtered_.reset(new PointCloudT);
	// The number of points in the cloud
	cloud_->resize(500);
	cloud_filtered_->resize(500);

	// Fill the cloud with random points

	for (size_t i = 0; i < cloud_->points.size(); ++i)
	{
		cloud_->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
		
		cloud_filtered_->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_filtered_->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_filtered_->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}

	//qt组件的"信号-槽"绑定
	connect(ui->passThroughButton, SIGNAL(clicked()), this, SLOT(passThroughButtonPressed()));
	connect(ui->conditionalAndRemovelButton, SIGNAL(clicked()), this, SLOT(conditionalRemovelButtonPressed()));//tabWidget and
	connect(ui->conditionalOrRemovelButton, SIGNAL(clicked()), this, SLOT(conditionalRemovelButtonPressed()));//tabWidget or
	connect(ui->voxelGridButton, SIGNAL(clicked()), this, SLOT(voxelGridButtonPressed()));
	connect(ui->statisticalRemovelButton, SIGNAL(clicked()), this, SLOT(statisticalRemovelButtonPressed()));
	connect(ui->radiusRemovelButton, SIGNAL(clicked()), this, SLOT(radiusRemovelButtonPressed()));
	connect(ui->gaussianFilterButton, SIGNAL(clicked()), this, SLOT(gaussianFilterButtonPressed()));
	connect(ui->bilateralFilterButton, SIGNAL(clicked()), this, SLOT(bilateralFilterButtonPressed()));

	connect(ui->action_1_1_1, SIGNAL(triggered()), this, SLOT(loadFileButtonPressed_1()));//打开文件到视口一(cloud_)
	connect(ui->action_1_1_2, SIGNAL(triggered()), this, SLOT(loadFileButtonPressed_2()));//打开文件到视口二(会覆盖cloud_filtered_)
	connect(ui->action_1_2_2, SIGNAL(triggered()), this, SLOT(saveFileButtonPressed()));//保存视口二的数据(cloud_filtered_)
	connect(ui->action, SIGNAL(triggered()), this, SLOT(coordinateButtonPressed()));//菜单栏设置-开关坐标系

	display();
}

Filter_Gui::~Filter_Gui()
{
	delete ui;
}


// 1.直通滤波-"start"Button
void Filter_Gui::passThroughButtonPressed()
{
	std::string axis = ui->comboBox_1->currentText().toStdString();//方向
	int bool_negative = ui->checkBox_5->isChecked();
	double min = ui->doubleSpinBox_1->value();//min
	double max = ui->doubleSpinBox_2->value();//max

	//防止弹窗报指针数组等错误
	cloud_filtered_.reset(new PointCloudT);

	pcl::PassThrough<PointT> pass;
	pass.setInputCloud(cloud_);
	pass.setFilterFieldName(axis);
	pass.setFilterLimits(min, max);
	pass.setFilterLimitsNegative(bool_negative);//0:true 1:false

	std::cerr << "PassThrough Filter Start" << std::endl;
	pass.filter(*cloud_filtered_);
	std::cerr << "PassThrough Filter Ended" << std::endl;

	display();
}
// 2.条件滤波-"start"Button
void Filter_Gui::conditionalRemovelButtonPressed()
{
	cloud_filtered_.reset(new PointCloudT);

	//---------qt界面组件的值-------------
	//当前的tabWidget(0:and 1:or) 
	int currentTabIndex = ui->tabWidget->currentIndex();
	//两个可选择的维度
	std::string axis_1;
	std::string axis_2;
	//5个条件运算符 GT, GE, LT, LE, EQ
	int op_index_1, op_index_2;
	//两个范围值
	double value_1, value_2;
	//------------------------------------

	if (currentTabIndex == 0)//AND
	{
		axis_1 = ui->comboBox->currentText().toStdString();
		axis_2 = ui->comboBox_3->currentText().toStdString();
		op_index_1 = ui->comboBox_4->currentIndex();
		op_index_2 = ui->comboBox_5->currentIndex();
		value_1 = ui->doubleSpinBox->value();
		value_2 = ui->doubleSpinBox_3->value();
		bool keep_organized = ui->checkBox_3->isChecked();
		// 创建条件定义对象
		pcl::ConditionAnd<PointT>::Ptr range_cond(
			new pcl::ConditionAnd<PointT>());
		// 为条件定义对象添加比较算子,只有满足条件的点云可以通过
		// **因为x写成大写，报错abort() 。。。。
		range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new
			pcl::FieldComparison<PointT>(axis_1, pcl::ComparisonOps::CompareOp(op_index_1), value_1)));
		range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new
			pcl::FieldComparison<PointT>(axis_2, pcl::ComparisonOps::CompareOp(op_index_2), value_2)));

		// Build the filter
		pcl::ConditionalRemoval<PointT> condrem;
		condrem.setCondition(range_cond);
		condrem.setInputCloud(cloud_);
		condrem.setKeepOrganized(keep_organized);
		// 应用滤波器
		std::cerr << "Conditional Filter Start" << std::endl;
		condrem.filter(*cloud_filtered_);
	}
	else
	{
		axis_1 = ui->comboBox_6->currentText().toStdString();
		axis_2 = ui->comboBox_7->currentText().toStdString();
		op_index_1 = ui->comboBox_8->currentIndex();
		op_index_2 = ui->comboBox_9->currentIndex();
		value_1 = ui->doubleSpinBox_4->value();
		value_2 = ui->doubleSpinBox_5->value();
		bool keep_organized = ui->checkBox_4->isChecked();

		// 创建条件定义对象
		pcl::ConditionOr<PointT>::Ptr range_cond(
			new pcl::ConditionOr<PointT>());

		range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new
			pcl::FieldComparison<PointT>(axis_1, pcl::ComparisonOps::CompareOp(op_index_1), value_1)));
		range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new
			pcl::FieldComparison<PointT>(axis_2, pcl::ComparisonOps::CompareOp(op_index_2), value_2)));

		// Build the filter
		pcl::ConditionalRemoval<PointT> condrem;
		condrem.setCondition(range_cond);
		condrem.setInputCloud(cloud_);
		condrem.setKeepOrganized(keep_organized);
		// 应用滤波器
		std::cerr << "Conditional Filter Start" << std::endl;
		condrem.filter(*cloud_filtered_);
	}
	std::cerr << "Conditional Filter Ended" << std::endl;

	display();
}
// 3.体素下采样-"start"Button
void Filter_Gui::voxelGridButtonPressed()
{
	cloud_filtered_.reset(new PointCloudT);

	double lx, ly, lz;//体素大小(leafsize)
	bool bool_negtiva = ui->checkBox_2->isChecked();

	lx = ui->doubleSpinBox_6->value();
	ly = ui->doubleSpinBox_7->value();
	lz = ui->doubleSpinBox_8->value();

	// 滤波器
	pcl::VoxelGrid<PointT> sor;
	sor.setInputCloud(cloud_);
	sor.setLeafSize(lx, ly, lz);
	sor.setFilterLimitsNegative(bool_negtiva);

	std::cerr << "VoxelGrid Filter Start" << std::endl;
	sor.filter(*cloud_filtered_);
	std::cerr << "VoxelGrid Filter Ended" << std::endl;

	display();
}
// 4.统计滤波-"start"Button
void Filter_Gui::statisticalRemovelButtonPressed()
{
	cloud_filtered_.reset(new PointCloudT);

	int mean_k = ui->spinBox->value();//ui-统计滤波-近邻点数
	double stddev_mult = ui->doubleSpinBox_10->value(); //ui - 统计滤波 - 阈值
	bool bool_negtiva = ui->checkBox->isChecked(); // ui-统计滤波-setNegtive

	// 创建滤波器对象
	pcl::StatisticalOutlierRemoval<PointT> sor;
	sor.setInputCloud(cloud_);
	sor.setMeanK(mean_k);
	sor.setStddevMulThresh(stddev_mult);
	sor.setNegative(bool_negtiva);//是否将过滤器极性反转，获取被过滤点
	sor.filter(*cloud_filtered_);

	std::cerr << "Statistical Filter Ended" << std::endl;

	display();
}
// 5.半径滤波-"start"Button
void Filter_Gui::radiusRemovelButtonPressed()
{
	cloud_filtered_.reset(new PointCloudT);

	double radius = ui->doubleSpinBox_9->value();
	int min_neighbors = ui->spinBox_2->value();
	bool bool_negtiva = ui->checkBox_6->isChecked();
	// 创建滤波器
	// 修改半径滤波的两个参数，产生的效果不同
	// 半径大了会增加寻找k近邻的计算时间;半径与k值通常同步增减；类比画图橡皮来理解参数效果；
	// 0.01 15 是试出来的一组参数
	pcl::RadiusOutlierRemoval<PointT> outrem;
	outrem.setInputCloud(cloud_);
	outrem.setRadiusSearch(radius);
	outrem.setMinNeighborsInRadius(min_neighbors);
	outrem.setNegative(bool_negtiva);
	// 应用滤波器
	outrem.filter(*cloud_filtered_);

	std::cerr << "Radius Filter Ended" << std::endl;

	display();
}
// 6.高斯滤波-"start"Button
void Filter_Gui::gaussianFilterButtonPressed()
{
	//重置储存结果的指针，避免错误
	cloud_filtered_.reset(new PointCloudT);

	//qt 界面参数
	float kernel_sigma, kernel_threshold;
	double radius_search;
	int number_of_threads;

	kernel_sigma = ui->doubleSpinBox_11->value();
	kernel_threshold = ui->doubleSpinBox_12->value();
	radius_search = ui->doubleSpinBox_13->value();
	number_of_threads = ui->spinBox_3->value();

	//高斯滤波器
	//标记是否正在进行滤波
	filte_ended = false;
	GaussianFilter* filter = new GaussianFilter();;
	std::thread thread_1(&GaussianFilter::filting,cloud_, cloud_filtered_, &filte_ended,  kernel_sigma,  kernel_threshold,  radius_search,  number_of_threads);
	thread_1.detach();
	std::cerr << "thread GaussianFilter detach" << endl;

	//等待滤波完成，处理主线程的事件，防止未响应
	while (filte_ended == false)
	{
		_sleep(100);
		qApp->processEvents();
	}
	display();
}
// 7.双边滤波-"start"Button
void Filter_Gui::bilateralFilterButtonPressed()
{
	cloud_.reset(new PointCloudT);
	cloud_filtered_.reset(new PointCloudT);
	filte_ended = false;

	double sigma_s, sigma_r;
	sigma_s = ui->doubleSpinBox_14->value();
	sigma_r = ui->doubleSpinBox_15->value();

	//加载有序点云文件（不去除文件中的空值NaN）
	// You might want to change "/home/" if you're not on an *nix platform
	QString filename = QFileDialog::getOpenFileName(this, tr("Open point cloud"), "/home/", tr("Point cloud data (*.pcd *.ply)"));

	std::cout << filename.toStdString() << endl;

	PCL_INFO("File chosen: %s\n", filename.toStdString().c_str());
	PointCloudT::Ptr cloud_tmp(new PointCloudT);

	if (filename.isEmpty())
		return;

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
	if (cloud_tmp->is_dense)
		pcl::copyPointCloud(*cloud_tmp, *cloud_);
	else
	{
		PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
		std::vector<int> vec;
		pcl::removeNaNFromPointCloud(*cloud_tmp, *cloud_, vec);
	}
	display();//先把加载的有序点云除去空值并显示

	//双边滤波器
	BilateralFilter* filter = new BilateralFilter();
	std::thread thread_1(&BilateralFilter::filting, cloud_tmp, cloud_filtered_, &filte_ended, sigma_s, sigma_r);
	thread_1.detach();
	std::cerr << "thread BilateralFilter detach" << endl;
	//过滤器运行时，响应主线程的事件
	while (filte_ended == false)
	{
		_sleep(100);
		qApp->processEvents();
	}

	//去除过滤后点云中的空值
	if (cloud_filtered_->is_dense)
		display();
	else
	{
		PCL_WARN("Filted Cloud is not dense! Non finite points will be removed\n");
		std::vector<int> vec;
		//pcl::removeNaNFromPointCloud(*cloud_filtered_, *cloud_filtered_, vec);
		pcl::removeNaNFromPointCloud(*cloud_filtered_, vec);
		display();
	}
	
}

// 文件-打开-视口1
void
Filter_Gui::loadFileButtonPressed_1()
{
	cloud_.reset(new PointCloudT);
	filte_ended = false;//指示是否正在执行耗时的线程操作，暂时还不支持多个线程，使用GUI时要注意

	//使用文件加载类
	QString filename = QFileDialog::getOpenFileName(this, tr("Open point cloud"), "/home/", tr("Point cloud data (*.pcd *.ply)"));
	//错误__acrt_first_block == header异常，根本的原因是对象在析构时不正确的释放内存导致的,可以使用MD打包程序
	std::cout << filename.toStdString() << endl;
	PCL_INFO("File chosen: %s\n", filename.toStdString().c_str());

	std::cout << "filte_ended=" << filte_ended << endl;
	LoadPCDFile* loadPCDFile = new LoadPCDFile();
	std::thread thread_load_file_viewport_1(&LoadPCDFile::loadPCD, cloud_, filename, &filte_ended);
	thread_load_file_viewport_1.detach();
	std::cerr << "thread loadPCDFile_viewport_1 detach" << endl;
	std::cout << "filte_ended=" << filte_ended << endl;

	//等待滤波完成，处理主线程的事件，防止未响应
	while (filte_ended == false)
	{
		_sleep(100);
		//std::cout << "THREAD=" << thread_load_file_viewport_1.get_id() << endl;
		qApp->processEvents();
	}

	//// You might want to change "/home/" if you're not on an *nix platform
	//QString filename = QFileDialog::getOpenFileName(this, tr("Open point cloud"), "/home/", tr("Point cloud data (*.pcd *.ply)"));

	//std::cout << filename.toStdString() << endl;

	//PCL_INFO("File chosen: %s\n", filename.toStdString().c_str());
	//PointCloudT::Ptr cloud_tmp(new PointCloudT);

	//if (filename.isEmpty())
	//	return;

	//int return_status;
	//if (filename.endsWith(".pcd", Qt::CaseInsensitive))
	//	return_status = pcl::io::loadPCDFile(filename.toStdString(), *cloud_tmp);
	//else
	//	return_status = pcl::io::loadPLYFile(filename.toStdString(), *cloud_tmp);

	//if (return_status != 0)
	//{
	//	PCL_ERROR("Error reading point cloud %s\n", filename.toStdString().c_str());
	//	return;
	//}

	//PCL_WARN("file has loaded\n");
	//// If point cloud contains NaN values, remove them before updating the visualizer point cloud
	//if (cloud_tmp->is_dense)
	//	pcl::copyPointCloud(*cloud_tmp, *cloud_);
	//else
	//{
	//	PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
	//	std::vector<int> vec;
	//	pcl::removeNaNFromPointCloud(*cloud_tmp, *cloud_, vec);
	//}

	

	display();
}

// 文件-打开-视口2
void
Filter_Gui::loadFileButtonPressed_2()
{
	cloud_filtered_.reset(new PointCloudT);
	filte_ended = false;//指示是否正在执行耗时的线程操作，暂时还不支持多个线程，使用GUI时要注意


	//// You might want to change "/home/" if you're not on an *nix platform
	//QString filename = QFileDialog::getOpenFileName(this, tr("Open point cloud"), "/home/", tr("Point cloud data (*.pcd *.ply)"));

	//std::cout << filename.toStdString() << endl;

	//PCL_INFO("File chosen: %s\n", filename.toStdString().c_str());
	//PointCloudT::Ptr cloud_tmp(new PointCloudT);

	//if (filename.isEmpty())
	//	return;

	//int return_status;
	//if (filename.endsWith(".pcd", Qt::CaseInsensitive))
	//	return_status = pcl::io::loadPCDFile(filename.toStdString(), *cloud_tmp);
	//else
	//	return_status = pcl::io::loadPLYFile(filename.toStdString(), *cloud_tmp);

	//if (return_status != 0)
	//{
	//	PCL_ERROR("Error reading point cloud %s\n", filename.toStdString().c_str());
	//	return;
	//}
	//PCL_WARN("file has loaded\n");
	//// If point cloud contains NaN values, remove them before updating the visualizer point cloud
	//if (cloud_tmp->is_dense)
	//	pcl::copyPointCloud(*cloud_tmp, *cloud_filtered_);
	//else
	//{
	//	PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
	//	std::vector<int> vec;
	//	pcl::removeNaNFromPointCloud(*cloud_tmp, *cloud_filtered_, vec);
	//}

	//使用文件加载类
	QString filename = QFileDialog::getOpenFileName(this, tr("Open point cloud"), "/home/", tr("Point cloud data (*.pcd *.ply)"));
	std::cout << filename.toStdString() << endl;
	PCL_INFO("File chosen: %s\n", filename.toStdString().c_str());

	std::cout << "filte_ended=" << filte_ended << endl;
	LoadPCDFile* loadPCDFile = new LoadPCDFile();
	std::thread thread_1(&LoadPCDFile::loadPCD,cloud_filtered_, filename, &filte_ended);
	thread_1.detach();
	std::cerr << "thread loadPCDFile_viewport_2 detach" << endl;
	std::cout << "filte_ended=" << filte_ended << endl;

	//等待滤波完成，处理主线程的事件，防止未响应
	while (filte_ended == false)
	{
		_sleep(100);
		//std::cout << "THREAD=" << thread_1.get_id() << endl;
		qApp->processEvents();
	}

	display();
}

// 文件-保存-视口2
void
Filter_Gui::saveFileButtonPressed()
{
	// You might want to change "/home/" if you're not on an *nix platform
	QString filename = QFileDialog::getSaveFileName(this, tr("Open point cloud"), "/home/", tr("Point cloud data (*.pcd *.ply)"));

	PCL_INFO("File chosen: %s\n", filename.toStdString().c_str());

	if (filename.isEmpty())
		return;

	int return_status;
	if (filename.endsWith(".pcd", Qt::CaseInsensitive))
		return_status = pcl::io::savePCDFileBinary(filename.toStdString(), *cloud_filtered_);
	else if (filename.endsWith(".ply", Qt::CaseInsensitive))
		return_status = pcl::io::savePLYFileBinary(filename.toStdString(), *cloud_filtered_);
	else
	{
		filename.append(".ply");
		return_status = pcl::io::savePLYFileBinary(filename.toStdString(), *cloud_filtered_);
	}
	
	if (return_status != 0)
	{
		PCL_ERROR("Error writing point cloud %s\n", filename.toStdString().c_str());
		return;
	}
}

//显示滤波前后点云
void
Filter_Gui::display()
{
	viewer_.reset(new pcl::visualization::PCLVisualizer("viewer", false));

	// 多视口显示
	// --------------------------------------------------------
	// -----     Open 3D viewer and add point cloud       -----
	// --------------------------------------------------------
	viewer_->initCameraParameters();
	viewer_->resetCamera();

	int v1(0);
	//std::cout<<ui->qvtkWidget_1->geometry();
	viewer_->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer_->setBackgroundColor(0, 0, 0, v1);
	//viewer_->addText("v1", 10, 10, "v1 text", v1);//如果出现vtk刷屏报错opengl，注意中英文
	//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(128,128,128);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb1(cloud_, 128, 128, 128);
	viewer_->addPointCloud(cloud_, rgb1, "cloud", v1);

	int v2(0);
	viewer_->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer_->setBackgroundColor(0.1, 0.1, 0.1, v2);
	//viewer_->addText("v2", 10, 10, "v2 text", v2);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb2(cloud_filtered_, 128, 128, 128);
	viewer_->addPointCloud(cloud_filtered_, rgb2, "cloud_filtered", v2);

	//判断是否继续显示坐标系
	if (Filter_Gui::coordinateSystemAdded==true)
	{
		viewer_->addCoordinateSystem(1.0);
	}
	//viewer_->setShowFPS(true);

	viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
	viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_filtered");

	ui->qvtkWidget_1->SetRenderWindow(viewer_->getRenderWindow());
	viewer_->setupInteractor(ui->qvtkWidget_1->GetInteractor(), ui->qvtkWidget_1->GetRenderWindow());
	ui->qvtkWidget_1->update();
}

//开关坐标系
void
Filter_Gui::coordinateButtonPressed()
{
	if (Filter_Gui::coordinateSystemAdded == true)
	{
		viewer_->removeAllCoordinateSystems();
		Filter_Gui::coordinateSystemAdded = false;
		display();
	}
	else
	{
		viewer_->addCoordinateSystem(1.0);
		Filter_Gui::coordinateSystemAdded = true;
		display();
	}
}