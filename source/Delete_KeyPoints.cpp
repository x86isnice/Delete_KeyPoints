#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <cmath>
#include <ctime>
#include <fstream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <list>
#include <vector>

using namespace std;
using namespace pcl;

void List_pcd(const char *path,  std::list<std::string> & Pcd_List)
{
  //  printf("路径为[%s]\n", path);
    struct dirent* ent = NULL;
    DIR *pDir;
    pDir = opendir(path);
    //d_reclen：16表示子目录或以.开头的隐藏文件，24表示普通文本文件,28为二进制文件，还有其他……
    while (NULL != (ent=readdir(pDir)))
    {
        if (ent->d_reclen == 32)
        {
  //      std::cout << std::string(path)+"/"+ent->d_name << std::endl;
        Pcd_List.push_back(std::string(path)+"/"+ent->d_name);
        }
    }
}

void Get_Pure_Path(char *file1,char *file2,std::list<string> &pure_path)
{//file1 :: FPFH_Log   file2:: Classifier_Results
	  ifstream FPFH_Log(file1);
		  ifstream Classifier_Result(file2);
		  if (!FPFH_Log.is_open() || !Classifier_Result.is_open())
		  {
	         std::cout << "FPFH_log or Classifier file is not open "<< std::endl;
	         return ;
		  }
	       std::list<string> path1;
	       std::list<string> path2;
		  while (!FPFH_Log.eof())
		  {
			std::string str;
			getline(FPFH_Log,str);
			path1.push_back(str);
		  }

		  while (!Classifier_Result.eof())
		  {
			  std::string str;
			  getline(Classifier_Result,str);
			  path2.push_back(str);
		  }
          path2.pop_back();

		  for (std::list<string>::iterator i = path2.begin(); i != path2.end(); i++)
		  {
			  for (std::list<string>::iterator j = path1.begin(); j != path1.end(); j++)
			  {
				  if (j->find(i->substr(0,i->find_first_of(',')+2)) != string::npos && j->find("flat") == string::npos)
				  {
					  int position = j->find_first_of(':');
                      pure_path.push_back(j->substr(0,position));
                      std::cout << j->substr(0,position) << std::endl;
                      break;
				  }
			  }
		  }

		  std::cout << path1.size()<<" " << path2.size()<<" "<<pure_path.size() << std::endl;
}

char *itoa(int value,char *string,int radix)
{
   int rt=0;
   if(string==NULL)
      return NULL;
   if(radix<=0||radix>30)
      return NULL;
   rt=snprintf(string,radix,"%d",value);
   if(rt>radix)
      return NULL;
   string[rt]='\0';
   return string;
}

int main(int argc,char ** argv)
{
	//std::string filename = "/home/x86isnice/0627_keypoints_with_filter/rem_data/p2at_rem.pcd";
	//std::string filename = "../plane.pcd";
    //std::string filename = "/home/x86isnice/0627_keypoints_with_filter/outdata2_o.pcd";
	//  std::string filename = "/home/x86isnice/Documents/AA室外数据集合/vtk_global/global.pcd";
	  std::string filename = "/home/x86isnice/0627_keypoints_with_filter/rem_data/a100vo_rem.pcd";

	  std::list<string> pure_path;
	  std::list<string> all_cloud_path;
	  std::string dataset_file = "/home/x86isnice/0627_keypoints_with_filter/dataset1";

	  std::cout << "Reading " << filename << std::endl;
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);

	  if(pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloud_xyz) == -1) // load the file
	  {
	      PCL_ERROR ("Couldn't read file");
	      return -1;
	  }

      char *file1 = "/home/x86isnice/FPFH/FPFH_log.txt";
      char *file2 = "/home/x86isnice/FPFH/first_result.txt";
      Get_Pure_Path(file1 ,file2 ,pure_path);

      List_pcd(dataset_file.c_str(),  all_cloud_path);

	  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	  viewer.setBackgroundColor( 1.0,1.0, 1.0 );

	  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler (cloud_xyz,255,0, 0);
	  viewer.addPointCloud( cloud_xyz,cloud_color_handler, "cloud");

	  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_key(new pcl::PointCloud<pcl::PointNormal>);

	  pcl::io::loadPCDFile<pcl::PointNormal> ("/home/x86isnice/0627_keypoints_with_filter/source/key.pcd", *cloud_key);

	  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> keypoints_color_handler(cloud_key, 0 , 255, 0);
	  viewer.addPointCloud(cloud_key, keypoints_color_handler, "keypoints");
	  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "keypoints");

	  ////////////////////////////////////////////////////////////////////////////////

	  for (std::list<string>::iterator it = all_cloud_path.begin(); it != all_cloud_path.end(); it++)
	  {
		  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_part(new pcl::PointCloud<pcl::PointNormal>);

		  pcl::io::loadPCDFile<pcl::PointNormal> (*it, *cloud_part);
         // std::cout << *it <<std::endl;
		  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> keypoints_near_color(cloud_part, 0, 0, 255);
		  std::string label = it->c_str();
		  viewer.addPointCloud(cloud_part, keypoints_near_color, label+"_keyarea");
	  }
	  std::cout << " the best mistake I've ever made!" << std::endl;
	  for (std::list<string>::iterator it = pure_path.begin(); it != pure_path.end(); it++)
     {
         pcl::PointCloud<pcl::PointNormal>::Ptr cloud_part(new pcl::PointCloud<pcl::PointNormal>);

	     pcl::io::loadPCDFile<pcl::PointNormal> (*it, *cloud_part);

	     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> keypoints_near_color(cloud_part,255,255,0);
	     std::string label = it->c_str();
	     viewer.addPointCloud(cloud_part, keypoints_near_color,label+"_keypoints");
     }

	 ////////////////////////////////////////////////////////////////////////////
	  cout << __FILE__ << "              " << __LINE__ << endl;
	   while (!viewer.wasStopped ())
	  {
		viewer.spinOnce ();
		pcl_sleep(0.001);
	  }
	   return 0;
}

