/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	auto startTime = std::chrono::steady_clock::now();
	std::unordered_set<int> inliersResult;//inliear的结果，这是一个无序的数据集，这实际上可以容纳您的最佳inliers。数量越大越容易得到结果。初始大小为0
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers

	/***当该值大于零时，我将通过此处创建％缓冲线，然后计算inliers，并跟踪哪个是inlier中最好的模型。***/
	while (maxIterations--)   //最大迭代次数
	{
		// Randomly pick two points//随机选取两个点去创建你的点所以在这里，我将创建这组新的无序inliers，它将容纳ints
		
		std::unordered_set<int> inliers;
		
		while (inliers.size() < 2)//，而inliers.size小于2时，我将插入一个点，因此，在C加号中，我将此圆括号括起来，并在对cloud point.size进行调制。
			inliers.insert(rand() % (cloud->points.size()));//因此，这基本上是兰特可以做的非常大的数字，通过做这种模，我实际上只是从云返回零和点大小之间的某个值。
		//这就是云的总数量，介于云之间的某个位置，然后将其插入到内部。从某种意义上说，这是一个无序集合，从某种意义上说，这是一个无序集合，无序集合意味着它是一个HashSet，或者对于排序无关紧要，
		//我们只是散列到索引中，但是set只包含唯一元素。
		//因此，我们要避免的事情是连续两次选择相同的索引,假设我们选择了索引零，然后我们随机选择了索引零,对我们没好处，
		//所以如果我们选择了一个相同的，我我们讲继续浏览直到他有不同的元素，我们做了个索引0，又做了个索引50，
		float x1, y1, x2, y2;
		
		auto itr = inliers.begin();
		/**这代表了一些索引，因此我们正在索引云并获取该索引，我们获取其x值，获取其y值。**/
		
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;//这是我们插入inliers的是第一组点
		
		itr++;
		
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		
		float a = (y1 - y2);
		float b = (x2 - x1);
		float c = (x1 * y2 - x2 * y1);
		
		for (int index = 0; index < cloud->points.size(); index++)
		{
			if (inliers.count(index) > 0)
				continue;
				//这不是我们已经采样并从中创建一条线的两点之一，我们可以这样做，所以当我们依靠时，我们只是在看它是否包含元素，如果不是0，他确实包含这个元素，后面不用进行
			
			pcl::PointXYZ point = cloud->points[index];
			
			float x3 = point.x;
			float y3 = point.y;
			
			float d = fabs(a * x3 + b * y3 + c) / sqrt(a * a + b * b);
			
			if (d <= distanceTol)
				inliers.insert(index);//我们仅仅存索引
		}
		
		if (inliers.size() > inliersResult.size())//我们看inliers的尺寸大小，如果它大于inliers结果，这是我们在本节中要参考的最开始的结果，
		{
			inliersResult = inliers;//反复迭代，inliersResuklt就以后最符合的模型索引
		}
	}
	auto endTime = std::chrono::steady_clock::now();
	auto ellapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "Ransac took " << ellapsedTime.count() << " millicesonds" << std::endl;
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 10, 1.0);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
