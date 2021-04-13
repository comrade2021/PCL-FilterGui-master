#功能：
几种简单点云滤波方法的可视化QT界面

#所用程序版本：
系统:windows10
pcl:PCL-1.11.1-AllInOne-msvc2019-win64
qt:qt5.12.10-msvc2017_64
vtk:使用自己编译的64位vtk8.2，原因是pcl的AllInOne安装包中的第三方VTK代码库不适用（无qvtk）
vs2019+qt的vs扩展工具

#备注：
1.只是自己用来演示和练习的小程序，时间仓促水平有限，内容不够规范完善，仅供参考，希望能对大家提供些许帮助；
2.部分滤波方法未使用多线程实现，进行耗时操作可能出现qt界面主线程的未响应，可以参考GaussianFilter类自行修改添加；
3.只针对pointxyzrgba类型点云文件，其他格式未实践；
4.pcd文件夹中有一些测试用点云文件
5.pk文件夹中为打包的exe可直接执行文件
6.打包exe的运行库为多线程调试 DLL (/MDd)

#运行界面：


#参考：
1.《点云库PCL从入门到精通》（郭浩）
2.pcl官网:https://pointclouds.org/

#邮箱
wx171024@qq.com