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
//�߳�
#include <thread>
#include <mutex>
#include "GaussianFilter.h"
#include "BilateralFilter.h"
#include "LoadPCDFile.h"

//vtkOutputWindow::SetGlobalWarningDisplay(0);//������vtkOutputWindow����

//std::mutex mu;


Filter_Gui::Filter_Gui(QWidget* parent) :
	QMainWindow(parent),
	ui(new Ui::Filter_GuiClass)
{
	//��ʼ��UI
	ui->setupUi(this);
	QString str;
	str = str.fromLocal8Bit("�����˲�GUI");
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

	//qt�����"�ź�-��"��
	connect(ui->passThroughButton, SIGNAL(clicked()), this, SLOT(passThroughButtonPressed()));
	connect(ui->conditionalAndRemovelButton, SIGNAL(clicked()), this, SLOT(conditionalRemovelButtonPressed()));//tabWidget and
	connect(ui->conditionalOrRemovelButton, SIGNAL(clicked()), this, SLOT(conditionalRemovelButtonPressed()));//tabWidget or
	connect(ui->voxelGridButton, SIGNAL(clicked()), this, SLOT(voxelGridButtonPressed()));
	connect(ui->statisticalRemovelButton, SIGNAL(clicked()), this, SLOT(statisticalRemovelButtonPressed()));
	connect(ui->radiusRemovelButton, SIGNAL(clicked()), this, SLOT(radiusRemovelButtonPressed()));
	connect(ui->gaussianFilterButton, SIGNAL(clicked()), this, SLOT(gaussianFilterButtonPressed()));
	connect(ui->bilateralFilterButton, SIGNAL(clicked()), this, SLOT(bilateralFilterButtonPressed()));

	connect(ui->action_1_1_1, SIGNAL(triggered()), this, SLOT(loadFileButtonPressed_1()));//���ļ����ӿ�һ(cloud_)
	connect(ui->action_1_1_2, SIGNAL(triggered()), this, SLOT(loadFileButtonPressed_2()));//���ļ����ӿڶ�(�Ḳ��cloud_filtered_)
	connect(ui->action_1_2_2, SIGNAL(triggered()), this, SLOT(saveFileButtonPressed()));//�����ӿڶ�������(cloud_filtered_)
	connect(ui->action, SIGNAL(triggered()), this, SLOT(coordinateButtonPressed()));//�˵�������-��������ϵ

	display();
}

Filter_Gui::~Filter_Gui()
{
	delete ui;
}


// 1.ֱͨ�˲�-"start"Button
void Filter_Gui::passThroughButtonPressed()
{
	std::string axis = ui->comboBox_1->currentText().toStdString();//����
	int bool_negative = ui->checkBox_5->isChecked();
	double min = ui->doubleSpinBox_1->value();//min
	double max = ui->doubleSpinBox_2->value();//max

	//��ֹ������ָ������ȴ���
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
// 2.�����˲�-"start"Button
void Filter_Gui::conditionalRemovelButtonPressed()
{
	cloud_filtered_.reset(new PointCloudT);

	//---------qt���������ֵ-------------
	//��ǰ��tabWidget(0:and 1:or) 
	int currentTabIndex = ui->tabWidget->currentIndex();
	//������ѡ���ά��
	std::string axis_1;
	std::string axis_2;
	//5����������� GT, GE, LT, LE, EQ
	int op_index_1, op_index_2;
	//������Χֵ
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
		// ���������������
		pcl::ConditionAnd<PointT>::Ptr range_cond(
			new pcl::ConditionAnd<PointT>());
		// Ϊ�������������ӱȽ�����,ֻ�����������ĵ��ƿ���ͨ��
		// **��Ϊxд�ɴ�д������abort() ��������
		range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new
			pcl::FieldComparison<PointT>(axis_1, pcl::ComparisonOps::CompareOp(op_index_1), value_1)));
		range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new
			pcl::FieldComparison<PointT>(axis_2, pcl::ComparisonOps::CompareOp(op_index_2), value_2)));

		// Build the filter
		pcl::ConditionalRemoval<PointT> condrem;
		condrem.setCondition(range_cond);
		condrem.setInputCloud(cloud_);
		condrem.setKeepOrganized(keep_organized);
		// Ӧ���˲���
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

		// ���������������
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
		// Ӧ���˲���
		std::cerr << "Conditional Filter Start" << std::endl;
		condrem.filter(*cloud_filtered_);
	}
	std::cerr << "Conditional Filter Ended" << std::endl;

	display();
}
// 3.�����²���-"start"Button
void Filter_Gui::voxelGridButtonPressed()
{
	cloud_filtered_.reset(new PointCloudT);

	double lx, ly, lz;//���ش�С(leafsize)
	bool bool_negtiva = ui->checkBox_2->isChecked();

	lx = ui->doubleSpinBox_6->value();
	ly = ui->doubleSpinBox_7->value();
	lz = ui->doubleSpinBox_8->value();

	// �˲���
	pcl::VoxelGrid<PointT> sor;
	sor.setInputCloud(cloud_);
	sor.setLeafSize(lx, ly, lz);
	sor.setFilterLimitsNegative(bool_negtiva);

	std::cerr << "VoxelGrid Filter Start" << std::endl;
	sor.filter(*cloud_filtered_);
	std::cerr << "VoxelGrid Filter Ended" << std::endl;

	display();
}
// 4.ͳ���˲�-"start"Button
void Filter_Gui::statisticalRemovelButtonPressed()
{
	cloud_filtered_.reset(new PointCloudT);

	int mean_k = ui->spinBox->value();//ui-ͳ���˲�-���ڵ���
	double stddev_mult = ui->doubleSpinBox_10->value(); //ui - ͳ���˲� - ��ֵ
	bool bool_negtiva = ui->checkBox->isChecked(); // ui-ͳ���˲�-setNegtive

	// �����˲�������
	pcl::StatisticalOutlierRemoval<PointT> sor;
	sor.setInputCloud(cloud_);
	sor.setMeanK(mean_k);
	sor.setStddevMulThresh(stddev_mult);
	sor.setNegative(bool_negtiva);//�Ƿ񽫹��������Է�ת����ȡ�����˵�
	sor.filter(*cloud_filtered_);

	std::cerr << "Statistical Filter Ended" << std::endl;

	display();
}
// 5.�뾶�˲�-"start"Button
void Filter_Gui::radiusRemovelButtonPressed()
{
	cloud_filtered_.reset(new PointCloudT);

	double radius = ui->doubleSpinBox_9->value();
	int min_neighbors = ui->spinBox_2->value();
	bool bool_negtiva = ui->checkBox_6->isChecked();
	// �����˲���
	// �޸İ뾶�˲�������������������Ч����ͬ
	// �뾶���˻�����Ѱ��k���ڵļ���ʱ��;�뾶��kֵͨ��ͬ����������Ȼ�ͼ��Ƥ��������Ч����
	// 0.01 15 ���Գ�����һ�����
	pcl::RadiusOutlierRemoval<PointT> outrem;
	outrem.setInputCloud(cloud_);
	outrem.setRadiusSearch(radius);
	outrem.setMinNeighborsInRadius(min_neighbors);
	outrem.setNegative(bool_negtiva);
	// Ӧ���˲���
	outrem.filter(*cloud_filtered_);

	std::cerr << "Radius Filter Ended" << std::endl;

	display();
}
// 6.��˹�˲�-"start"Button
void Filter_Gui::gaussianFilterButtonPressed()
{
	//���ô�������ָ�룬�������
	cloud_filtered_.reset(new PointCloudT);

	//qt �������
	float kernel_sigma, kernel_threshold;
	double radius_search;
	int number_of_threads;

	kernel_sigma = ui->doubleSpinBox_11->value();
	kernel_threshold = ui->doubleSpinBox_12->value();
	radius_search = ui->doubleSpinBox_13->value();
	number_of_threads = ui->spinBox_3->value();

	//��˹�˲���
	//����Ƿ����ڽ����˲�
	filte_ended = false;
	GaussianFilter* filter = new GaussianFilter();;
	std::thread thread_1(&GaussianFilter::filting,cloud_, cloud_filtered_, &filte_ended,  kernel_sigma,  kernel_threshold,  radius_search,  number_of_threads);
	thread_1.detach();
	std::cerr << "thread GaussianFilter detach" << endl;

	//�ȴ��˲���ɣ��������̵߳��¼�����ֹδ��Ӧ
	while (filte_ended == false)
	{
		_sleep(100);
		qApp->processEvents();
	}
	display();
}
// 7.˫���˲�-"start"Button
void Filter_Gui::bilateralFilterButtonPressed()
{
	cloud_.reset(new PointCloudT);
	cloud_filtered_.reset(new PointCloudT);
	filte_ended = false;

	double sigma_s, sigma_r;
	sigma_s = ui->doubleSpinBox_14->value();
	sigma_r = ui->doubleSpinBox_15->value();

	//������������ļ�����ȥ���ļ��еĿ�ֵNaN��
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
	display();//�ȰѼ��ص�������Ƴ�ȥ��ֵ����ʾ

	//˫���˲���
	BilateralFilter* filter = new BilateralFilter();
	std::thread thread_1(&BilateralFilter::filting, cloud_tmp, cloud_filtered_, &filte_ended, sigma_s, sigma_r);
	thread_1.detach();
	std::cerr << "thread BilateralFilter detach" << endl;
	//����������ʱ����Ӧ���̵߳��¼�
	while (filte_ended == false)
	{
		_sleep(100);
		qApp->processEvents();
	}

	//ȥ�����˺�����еĿ�ֵ
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

// �ļ�-��-�ӿ�1
void
Filter_Gui::loadFileButtonPressed_1()
{
	cloud_.reset(new PointCloudT);
	filte_ended = false;//ָʾ�Ƿ�����ִ�к�ʱ���̲߳�������ʱ����֧�ֶ���̣߳�ʹ��GUIʱҪע��

	//ʹ���ļ�������
	QString filename = QFileDialog::getOpenFileName(this, tr("Open point cloud"), "/home/", tr("Point cloud data (*.pcd *.ply)"));
	//����__acrt_first_block == header�쳣��������ԭ���Ƕ���������ʱ����ȷ���ͷ��ڴ浼�µ�,����ʹ��MD�������
	std::cout << filename.toStdString() << endl;
	PCL_INFO("File chosen: %s\n", filename.toStdString().c_str());

	std::cout << "filte_ended=" << filte_ended << endl;
	LoadPCDFile* loadPCDFile = new LoadPCDFile();
	std::thread thread_load_file_viewport_1(&LoadPCDFile::loadPCD, cloud_, filename, &filte_ended);
	thread_load_file_viewport_1.detach();
	std::cerr << "thread loadPCDFile_viewport_1 detach" << endl;
	std::cout << "filte_ended=" << filte_ended << endl;

	//�ȴ��˲���ɣ��������̵߳��¼�����ֹδ��Ӧ
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

// �ļ�-��-�ӿ�2
void
Filter_Gui::loadFileButtonPressed_2()
{
	cloud_filtered_.reset(new PointCloudT);
	filte_ended = false;//ָʾ�Ƿ�����ִ�к�ʱ���̲߳�������ʱ����֧�ֶ���̣߳�ʹ��GUIʱҪע��


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

	//ʹ���ļ�������
	QString filename = QFileDialog::getOpenFileName(this, tr("Open point cloud"), "/home/", tr("Point cloud data (*.pcd *.ply)"));
	std::cout << filename.toStdString() << endl;
	PCL_INFO("File chosen: %s\n", filename.toStdString().c_str());

	std::cout << "filte_ended=" << filte_ended << endl;
	LoadPCDFile* loadPCDFile = new LoadPCDFile();
	std::thread thread_1(&LoadPCDFile::loadPCD,cloud_filtered_, filename, &filte_ended);
	thread_1.detach();
	std::cerr << "thread loadPCDFile_viewport_2 detach" << endl;
	std::cout << "filte_ended=" << filte_ended << endl;

	//�ȴ��˲���ɣ��������̵߳��¼�����ֹδ��Ӧ
	while (filte_ended == false)
	{
		_sleep(100);
		//std::cout << "THREAD=" << thread_1.get_id() << endl;
		qApp->processEvents();
	}

	display();
}

// �ļ�-����-�ӿ�2
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

//��ʾ�˲�ǰ�����
void
Filter_Gui::display()
{
	viewer_.reset(new pcl::visualization::PCLVisualizer("viewer", false));

	// ���ӿ���ʾ
	// --------------------------------------------------------
	// -----     Open 3D viewer and add point cloud       -----
	// --------------------------------------------------------
	viewer_->initCameraParameters();
	viewer_->resetCamera();

	int v1(0);
	//std::cout<<ui->qvtkWidget_1->geometry();
	viewer_->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer_->setBackgroundColor(0, 0, 0, v1);
	//viewer_->addText("v1", 10, 10, "v1 text", v1);//�������vtkˢ������opengl��ע����Ӣ��
	//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(128,128,128);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb1(cloud_, 128, 128, 128);
	viewer_->addPointCloud(cloud_, rgb1, "cloud", v1);

	int v2(0);
	viewer_->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer_->setBackgroundColor(0.1, 0.1, 0.1, v2);
	//viewer_->addText("v2", 10, 10, "v2 text", v2);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb2(cloud_filtered_, 128, 128, 128);
	viewer_->addPointCloud(cloud_filtered_, rgb2, "cloud_filtered", v2);

	//�ж��Ƿ������ʾ����ϵ
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

//��������ϵ
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