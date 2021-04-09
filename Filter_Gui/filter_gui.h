#pragma once
#include "ui_filter_gui.h"
// IO
#include <iostream>
// Point Cloud Library (PCL)
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
// Visualization Toolkit (VTK)
#include <vtkActor.h>
#include <vtkVersion.h>
#include <vtkRenderer.h>
#include <vtkPolyData.h>
#include <vtkPlaneSource.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
// Qt
#include <QMainWindow>
#include <QFileDialog>
// Boost
#include <boost/math/special_functions/round.hpp>

//注意：暂时只处理pcl::PointXYZRGBA，目前程序只使用此类型点云文件，有需要自行修改
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudT;

namespace Ui
{
    class Filter_GuiClass;
}

class Filter_Gui : public QMainWindow
{
    Q_OBJECT

public:
    explicit
        Filter_Gui(QWidget* parent = 0);//构造函数
    ~Filter_Gui();//析构函数

    void display();
    void display_when_filting();
    //两个视口的显示

public slots:
    //void printValueTest();//测试qt用

    void passThroughButtonPressed();//直通滤波-"start"Button
    void conditionalRemovelButtonPressed();//条件滤波-"start"Button
    void voxelGridButtonPressed();//体素下采样-"start"Button
    void statisticalRemovelButtonPressed();//统计滤波-"start"Button
    void radiusRemovelButtonPressed();//半径滤波-"start"Button
    void gaussianFilterButtonPressed();//高斯滤波-"start"Button
    void bilateralFilterButtonPressed();//双边滤波-"start"Button

    void
        loadFileButtonPressed_1();//文件-打开-视口1
    void
        loadFileButtonPressed_2();//文件-打开-视口2
    void
        saveFileButtonPressed();//保存-视口2
    void 
        coordinateButtonPressed();//设置开关坐标系

protected:
    /** @brief The PCL visualizer object */
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;

    // 程序中这两个点云结构变量，贯穿整个程序
    // cloud_:处理前点云
    // cloud_filtered_:处理后点云
    PointCloudT::Ptr cloud_;
    PointCloudT::Ptr cloud_filtered_;
    
    bool filte_ended;
    bool coordinateSystemAdded;
private:
    Ui::Filter_GuiClass* ui;
};