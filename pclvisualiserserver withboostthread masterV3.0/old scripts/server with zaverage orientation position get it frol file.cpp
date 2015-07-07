/* \author Geoffrey Biggs */


#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>


#include <pcl/common/transforms.h>

#include <pcl/filters/voxel_grid.h>




#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/statistical_outlier_removal.h>

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <conv.h>


#include <iostream>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>

uint32_t ColorR []={ 255, 0, 0, 255, 50,0,0};
uint32_t ColorG []={ 0, 255, 0, 255, 0,0,50};
uint32_t ColorB []={ 0, 0, 255, 0  , 0,50,0};

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

static bool update=false;

void error(const char *msg)
{
    perror(msg);
    exit(1);
}








boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (std::string windowname)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer (windowname));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  viewer->setCameraPosition(0,0,-2,0,-1,0,0);
  return (viewer);
}






unsigned int text_id = 0;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getKeySym () == "r" && event.keyDown ())
  {
    std::cout << "r was pressed => removing all text" << std::endl;

    char str[512];
    for (unsigned int i = 0; i < text_id; ++i)
    {
      sprintf (str, "text#%03d", i);
      viewer->removeShape (str);
    }
    text_id = 0;
  }
}

void mouseEventOccurred (const pcl::visualization::MouseEvent &event,
                         void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
      event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
  {
    std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;

    char str[512];
    sprintf (str, "text#%03d", text_id ++);
    viewer->addText ("clicked here", event.getX (), event.getY (), str);
  }
}



pcl::PointCloud<pcl::PointXYZRGB>::Ptr
createPointCloud(char* buffer,int buffer_point_size){





pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

uint8_t r(255), g(15), b(15);

for(int k=0;k<buffer_point_size;k+=12) {
printf("%f, ", bytesToFloat(buffer[k], buffer[k+1], buffer[k+2], buffer[k+3]));
pcl::PointXYZRGB point;
point.x = bytesToFloat(buffer[k], buffer[k+1], buffer[k+2], buffer[k+3]);
point.y = bytesToFloat(buffer[k+4], buffer[k+5], buffer[k+6], buffer[k+7]);
point.z = bytesToFloat(buffer[k+8], buffer[k+9], buffer[k+10], buffer[k+11]);
uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                static_cast<uint32_t>(g) << 8  |
                static_cast<uint32_t>(b)        );
point.rgb = *reinterpret_cast<float*>(&rgb);
point_cloud_ptr->points.push_back (point);
r -= 5;
g += 5;

}

  point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
  point_cloud_ptr->height = 1;

return point_cloud_ptr;
}



pcl::PointCloud<pcl::PointXYZRGB>::Ptr
readFile(std::string fileName,float*averagez,float*xxx,float* yyy,float* zzz,float*xx,float* yy,float* zz){

pcl::PointCloud<pcl::PointXYZRGB>::Ptr res(new PointCloud());
    std::string line;
    ifstream myfile (fileName.c_str());

    uint8_t r(255), g(15), b(15);
    if (myfile.is_open()) {

        getline (myfile, line);
        sscanf (line.c_str(),"%f",averagez);
        getline (myfile, line);
        sscanf (line.c_str(),"%f;%f;%f",xx,yy,zz);
        getline (myfile, line);
        sscanf (line.c_str(),"%f;%f;%f",xxx,yyy,zzz);

     while (myfile.good() ) {

            getline (myfile, line);

            float x,y,z;
            sscanf (line.c_str(),"%f;%f;%f",&x,&y,&z);

            pcl::PointXYZRGB p;
            p.x=x;
            p.y=y;
            p.z=z;
            uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                            static_cast<uint32_t>(g) << 8  |
                            static_cast<uint32_t>(b)        );
            p.rgb = *reinterpret_cast<float*>(&rgb);
            res->points.push_back (p);
            r -= 5;
            g += 5;

        }
        myfile.close();
    } else {
        cout << "Unable to open file\n";
    }

return res;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
readFile(std::string fileName){
pcl::PointCloud<pcl::PointXYZRGB>::Ptr res(new PointCloud());
    std::string line;
    ifstream myfile (fileName.c_str());
    uint8_t r(255), g(15), b(15);
    if (myfile.is_open()) {
    while (myfile.good() ) {

            getline (myfile, line);

            float x,y,z;
            sscanf (line.c_str(),"%f;%f;%f",&x,&y,&z);

            pcl::PointXYZRGB p;
            p.x=x;
            p.y=y;
            p.z=z;
            uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                            static_cast<uint32_t>(g) << 8  |
                            static_cast<uint32_t>(b)        );
            p.rgb = *reinterpret_cast<float*>(&rgb);
            res->points.push_back (p);
            r -= 5;
            g += 5;

        }
        myfile.close();
    } else {
        cout << "Unable to open file\n";
    }

return res;
}







void unittreatment(int *newsockfd,bool *update, boost::shared_ptr<pcl::visualization::PCLVisualizer> * viewer){

    //printf("---------------------%f\n", cloud->points[0].x );

       int n;

       char* buffer=new char [4];

       if (read(*newsockfd,buffer,4) < 0) error("ERROR reading from socket");

       int points_no =(int)bytesToFloat(buffer[0], buffer[1], buffer[2], buffer[3]) - 1;
       delete [] buffer;

       int buffer_point_size=points_no*(int)sizeof(float);
       buffer=new char[buffer_point_size];
       if (read(*newsockfd,buffer,buffer_point_size) < 0) error("ERROR reading from socket");

    printf("%d\n", buffer_point_size );
    printf("Message from client:\n");



    //convertBytesToPointCloud(buffer,buffer_point_size);


    printf("\n");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr = readFile("output1.csv");
   // pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr =createPointCloud(buffer,buffer_point_size);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud_ptr);
   (*viewer)->addPointCloud<pcl::PointXYZRGB> (point_cloud_ptr, rgb, "sample cloud");

       n = write(*newsockfd,"I got your message",18);
       if (n < 0) error("ERROR writing to socket");

    *update=true;
    close(*newsockfd);
    //exit(0);
}



void server(int *sockfd,bool *update, boost::shared_ptr<pcl::visualization::PCLVisualizer> * viewer){


    //preparation du socket
    int newsockfd, portno, pid, visualizerpid;
    socklen_t clilen;
    struct sockaddr_in serv_addr, cli_addr;

   *sockfd = socket(AF_INET, SOCK_STREAM, 0);

   if (*sockfd < 0)
       error("ERROR opening socket");
    bzero((char *) &serv_addr, sizeof(serv_addr));
    portno = 60588;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);
    if (bind(*sockfd, (struct sockaddr *) &serv_addr,
             sizeof(serv_addr)) < 0)
             error("ERROR on binding");
    listen(*sockfd,5);
    clilen = sizeof(cli_addr);






    while (1)
    {
    newsockfd = accept(*sockfd, (struct sockaddr *) &cli_addr, &clilen);



     boost::thread unittreatmentthread(unittreatment,&newsockfd,update,viewer);


   }

}

Eigen::Vector3f quaternionMutltiply(Eigen::Vector4f quat, Eigen::Vector3f vec){
     float num = quat.x() * 2.0f;
     float num2 = quat.y() * 2.0f;
     float num3 = quat.z() * 2.0f;
     float num4 = quat.x()* num;
     float num5 = quat.y() * num2;
     float num6 = quat.z() * num3;
     float num7 = quat.x() * num2;
     float num8 = quat.x() * num3;
     float num9 = quat.y ()* num3;
     float num10 = quat.w() * num;
     float num11 = quat.w ()* num2;
     float num12 = quat.w ()* num3;
     Eigen::Vector3f  result;
     result.x() = (1.0f - (num5 + num6)) * vec.x ()+ (num7 - num12) * vec.y() + (num8 + num11) * vec.z();
     result.y() = (num7 + num12) * vec.x() + (1.0f - (num4 + num6)) * vec.y() + (num9 - num10) * vec.z();
     result.z() = (num8 - num11) * vec.x() + (num9 + num10) * vec.y() + (1.0f - (num4 + num5)) * vec.z();
     return result;
 }

// --------------
// -----Main-----
// --------------
int
main (int argc, char** argv)
{



 boost::shared_ptr<pcl::visualization::PCLVisualizer>  viewerinitial = rgbVis("Default PointCloud");
  boost::shared_ptr<pcl::visualization::PCLVisualizer>  viewer = rgbVis("Treated PointCloud");

bool update=false;

int sockfd;
boost::thread serverthread(server,&sockfd,&update, &viewer);





pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB> ());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_inliers (new pcl::PointCloud<pcl::PointXYZRGB> ());
float xx, yy,zz,posx,posy,posz,averagez;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr = readFile("output1.csv",&averagez,&posx,&posy,&posz,&xx,&yy,&zz),cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
std::cout<<"orientation "<<xx<<" "<<yy<<" "<<zz<<std::endl;
std::cout<<"orientation "<<xx/3*180<<" "<<yy/3*180<<" "<<zz/3*180<<std::endl;




float theta = xx/3*M_PI - M_PI/2;
 Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();
transform_1.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitX()));
theta = -zz/3*M_PI;
transform_1.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitY()));
//theta = -yy/3*M_PI;
//transform_1.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));


Eigen::Vector3f point_on_line(0,0,0), line_direction(0,0,1);

line_direction=transform_1*line_direction;

pcl::ModelCoefficients line_coeff;
line_coeff.values.resize (6);    // We need 6 values
line_coeff.values[0] = point_on_line.x ();
line_coeff.values[1] = point_on_line.y ();
line_coeff.values[2] = point_on_line.z ();

line_coeff.values[3] = line_direction.x ();
line_coeff.values[4] = line_direction.y ();
line_coeff.values[5] = line_direction.z ();
viewerinitial->addLine (line_coeff);



//std::cout<<point_cloud_ptr->points[0].x<<" "<<point_cloud_ptr->points[0].y<<" "<<point_cloud_ptr->points[0].z<<std::endl;
pcl::VoxelGrid<pcl::PointXYZRGB> sorvg;
 sorvg.setInputCloud (point_cloud_ptr);
 sorvg.setLeafSize (0.01f, 0.01f, 0.01f);
 sorvg.filter(*cloud_filtered);

/*
float theta = -M_PI/5.2;
 Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();
transform_1.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitX()));
pcl::transformPointCloud (*cloud_filtered, *cloud_filtered, transform_1);
*/

 pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_filtered);
 (viewerinitial)->addPointCloud<pcl::PointXYZRGB> (cloud_filtered, rgb, "sample cloud");
std::cout <<"cloud filtred size :"<< cloud_filtered->size()<< std::endl;

 // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.1);




  int i=0, nr_points = (int) cloud_filtered->points.size ();
    while (cloud_filtered->points.size () > 0.3 * nr_points)
    {
      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud (cloud_filtered);
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0)
      {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        break;
      }

      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<pcl::PointXYZRGB> extract;
      extract.setInputCloud (cloud_filtered);
      extract.setIndices (inliers);
      extract.setNegative (false);

      // Get the points associated with the planar surface
      extract.filter (*cloud_plane);
      //std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

      // Remove the planar inliers, extract the rest
      extract.setNegative (true);
      extract.filter (*cloud_f);
      *cloud_filtered = *cloud_f;
    }


    // Create the filtering object
     pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
     sor.setInputCloud (cloud_filtered);
     sor.setMeanK (50);
     sor.setStddevMulThresh (1.0);
     sor.filter (*cloud_filtered_inliers);


std::cout << "cloud filtred after plane extraction : "<<cloud_filtered_inliers->size()<< std::endl;


    // Creating the KdTree object for the search method of the extraction
     pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
     tree->setInputCloud (cloud_filtered_inliers);

     std::vector<pcl::PointIndices> cluster_indices;
     pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
     ec.setClusterTolerance (0.2); // 2cm
     ec.setMinClusterSize (100);
     ec.setMaxClusterSize (25000);
     ec.setSearchMethod (tree);
     ec.setInputCloud (cloud_filtered_inliers);
     ec.extract (cluster_indices);

 std::cout << "cluster number : "<<cluster_indices.size()<< std::endl;
     int j = 0;

       for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
       {
         pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);

         for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){

             j=j%(sizeof(ColorB)/sizeof(*ColorB));
             cloud_filtered_inliers->points[*pit].rgb = *reinterpret_cast<float*>(new uint32_t(static_cast<uint32_t>(ColorR[j]) << 16 |
                                                                                       static_cast<uint32_t>(ColorG[j]) << 8  |
                                                                                       static_cast<uint32_t>(ColorB[j])        ));

             cloud_cluster->points.push_back (cloud_filtered_inliers->points[*pit]);
         }

             cloud_cluster->width = cloud_cluster->points.size ();
             cloud_cluster->height = 1;
             cloud_cluster->is_dense = true;

         std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;


         pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_cluster);
         (viewer)->addPointCloud<pcl::PointXYZRGB> (cloud_cluster, rgb, "sample cloud"+j);






         j++;
       }

      // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_filtered);
       //(viewer)->addPointCloud<pcl::PointXYZRGB> (cloud_filtered, rgb, "sample cloud");


//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_filtered);
//(viewer)->addPointCloud<pcl::PointXYZRGB> (cloud_filtered, rgb, "sample cloud");


      while (!viewer->wasStopped () && !viewerinitial->wasStopped ())
      {
        if(update){
        std::cout << "test0: " << update << std::endl;
        update=false;
        }


        viewer->spinOnce (100);
        viewerinitial->spinOnce(100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));

      }

      pthread_cancel(serverthread.native_handle());
      close(sockfd);




return 0;

}
