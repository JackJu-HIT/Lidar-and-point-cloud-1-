/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);//高穗公路显示
        egoCar.render(viewer);//显示汽车1
        car1.render(viewer);//显示汽车2
        car2.render(viewer);//传送视觉显示显示汽车3
        car3.render(viewer);//显示汽车4
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar *lidar =new Lidar(cars,0);//我们在堆上实例化，所以我们有这个激光雷达的指针类型，我们使用新的关键词，输入是汽车，汽车实际上来自于高速公路的函数。
    //这就是我们模拟环境中的汽车列表。我们用零表示与地面是0斜率的，所以这是水平面，
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud=lidar->scan();//使用激光雷达，因为我们的指针在这，所以右箭头指向扫描scan函数（没有输入）这里将会是产生pcl点云 pclxyz指针，所以我们能做的是调用这个输入点云，我们称为光束，
    //renderRays(viewer,lidar->position,inputCloud);//但是我们如果想要显示他，我们称之为光束函数，我们会给他viewer和激光雷达。
    /**但是我们如果想要显示他，我们称之为光束函数，我们会给他viewer*
     * lidar->position:  其中position代表着激光雷达的位置（你可以在lidar里面结构提找到），所以他能告诉我们所产生的光束到底来自于那个位置。
     * 然后我们继续，并且给他传递一个我之前产生的点云
     * *
     * 所以现在我们有雷达模块在我们的车顶上，这里就是激光雷达射出不同的光束的位置
     * 我们将研究如何提高这个人分辨率，现在看起来非常稀疏，在我们讨论之前我们从没见过相同的定义
     * 找一个高分辨率的激光雷达，因为他现在一些。*/
    renderPointCloud(viewer,inputCloud,"inputcloud");
    // TODO:: Create point processor
              
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
//PCL可视化是一个引用传进来的，所以后续对他更改都想存在
    viewer->setBackgroundColor (0, 0, 0);//查看器的背景色可以设置为您喜欢的任何RGB颜色。在这种情况下，我们将其设置为黑色
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;//101
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);//查看复杂的点云通常会令人迷惑。为了使自己与世界保持一致，可以显示轴。
        //它们将沿X（红色），Y（绿色）和Z（蓝色）轴显示为三个圆柱
        //体。可以使用scale参数控制气缸的尺寸。在这种情况下，我们将其设置为1.0（如果未提供任何值，则它也是默认值）
        //。此方法的替代版本可用于将轴放置在世界上的任何位置。
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D 图像Viewer"));//这将创建查看器对象，
    //并为其提供一个漂亮的名称以显示在标题栏中。我们仅将其存储在智能指针中，以便可以在演示程序中传递它。通常，您不需要这样做。
    CameraAngle setAngle =  Side;//XY;
    initCamera(setAngle, viewer);
    simpleHighway(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}