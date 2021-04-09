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

//ע�⣺��ʱֻ����pcl::PointXYZRGBA��Ŀǰ����ֻʹ�ô����͵����ļ�������Ҫ�����޸�
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
        Filter_Gui(QWidget* parent = 0);//���캯��
    ~Filter_Gui();//��������

    void display();
    void display_when_filting();
    //�����ӿڵ���ʾ

public slots:
    //void printValueTest();//����qt��

    void passThroughButtonPressed();//ֱͨ�˲�-"start"Button
    void conditionalRemovelButtonPressed();//�����˲�-"start"Button
    void voxelGridButtonPressed();//�����²���-"start"Button
    void statisticalRemovelButtonPressed();//ͳ���˲�-"start"Button
    void radiusRemovelButtonPressed();//�뾶�˲�-"start"Button
    void gaussianFilterButtonPressed();//��˹�˲�-"start"Button
    void bilateralFilterButtonPressed();//˫���˲�-"start"Button

    void
        loadFileButtonPressed_1();//�ļ�-��-�ӿ�1
    void
        loadFileButtonPressed_2();//�ļ�-��-�ӿ�2
    void
        saveFileButtonPressed();//����-�ӿ�2
    void 
        coordinateButtonPressed();//���ÿ�������ϵ

protected:
    /** @brief The PCL visualizer object */
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;

    // ���������������ƽṹ�������ᴩ��������
    // cloud_:����ǰ����
    // cloud_filtered_:��������
    PointCloudT::Ptr cloud_;
    PointCloudT::Ptr cloud_filtered_;
    
    bool filte_ended;
    bool coordinateSystemAdded;
private:
    Ui::Filter_GuiClass* ui;
};