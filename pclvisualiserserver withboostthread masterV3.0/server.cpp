/* \author Geoffrey Biggs */


#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>


#include <pcl/common/transforms.h>

#include <pcl/filters/voxel_grid.h>


#include <pcl/filters/passthrough.h>

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
#include <sys/ioctl.h>

#include <functional>
#include <string>


#include <conv.h>


#include <iostream>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>

uint32_t ColorR []={ 255, 0, 0, 255, 50,0,0};
uint32_t ColorG []={ 0, 255, 0, 255, 0,0,50};
uint32_t ColorB []={ 0, 0, 255, 0  , 0,50,0};

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (std::string);
boost::shared_ptr<pcl::visualization::PCLVisualizer>  viewerinitial = rgbVis("Default PointCloud");
boost::shared_ptr<pcl::visualization::PCLVisualizer>  viewer = rgbVis("Treated PointCloud");


typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

static bool update=false;

void error(const char *msg)
{
    perror(msg);
    exit(1);
}



std::string obstacle_detection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered){

    //pcl::transformPointCloud (*cloud_filtered, *cloud_filtered, transform_2);

    pcl::PassThrough<pcl::PointXYZRGB> pass(true);
      pass.setInputCloud (cloud_filtered);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (0.0, 0.6);
      pass.filter (*cloud_filtered);

      pass.setFilterFieldName ("x");
      pass.setFilterLimits (-0.2, 0.2);
      pass.filter (*cloud_filtered);

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_filtered);
    viewer->removeAllPointClouds();
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud_filtered, rgb, "sample cloud");
   // std::cout <<"cloud filtred size :"<< cloud_filtered->size()<< std::endl;

    if(cloud_filtered->size()>0)
    return "co";
    else
    return "ok";

}


std::string environement_identification(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered){
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_inliers (new pcl::PointCloud<pcl::PointXYZRGB> ());

    // Create the segmentation object for the planar model and set all the parameters
     pcl::SACSegmentation<pcl::PointXYZRGB> seg;
     pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
     pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
     seg.setOptimizeCoefficients (true);
     seg.setModelType (pcl::SACMODEL_PLANE);
     seg.setMethodType (pcl::SAC_RANSAC);
     seg.setMaxIterations (100);
     seg.setDistanceThreshold (0.02);




     int i=0, nr_points = (int) cloud_filtered->points.size ();
       //while (cloud_filtered->points.size () > 0.3 * nr_points)
       //{
         // Segment the largest planar component from the remaining cloud
         seg.setInputCloud (cloud_filtered);
         seg.segment (*inliers, *coefficients);
         if (inliers->indices.size () == 0)
         {
           std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
           //break;
         }else{

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
      // }


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
        ec.setClusterTolerance (0.09); // 2cm
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
            (viewer)->addPointCloud<pcl::PointXYZRGB> (cloud_cluster, rgb, "cloud identification"+j);

            j++;
          }


          return "ok";

}


std::string chose_command(std::string control, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
std::string res;
     if (control.compare("collision") == 0)
       res=obstacle_detection(cloud);

     else if (control.compare("identification") == 0)
       res=environement_identification(cloud);
     else
         res="no";
       //std::cout << "no treatment for this type of command"<<std::endl;

return res;
}


bool threads = true;

void stop_all_threads(){
    threads = !threads;
    std::cout<<" all son threads stopped : " << threads <<std::endl;
}
bool breakk=false;
void testkeyboard(){
breakk=true;
}


unsigned int text_id = 0;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getKeySym () == "t" && event.keyDown ())
  {

       testkeyboard();
  }
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






boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (std::string windowname)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer (windowname));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  viewer->setCameraPosition(0,0,-2,0,-1,0,0);

  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
  viewer->registerMouseCallback (mouseEventOccurred, (void*)&viewer);

  return (viewer);
}







pcl::PointCloud<pcl::PointXYZRGB>::Ptr
convertBytesToPointCloud(char* buffer,int buffer_point_size){

pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

uint8_t r(255), g(15), b(15);

for(int k=0;k<buffer_point_size;k+=12) {

pcl::PointXYZRGB point;
point.x = bytesToFloat(buffer[k], buffer[k+1], buffer[k+2], buffer[k+3]);
point.y = bytesToFloat(buffer[k+4], buffer[k+5], buffer[k+6], buffer[k+7]);
point.z = bytesToFloat(buffer[k+8], buffer[k+9], buffer[k+10], buffer[k+11]);
//printf("%f,%f,%f\n ", point.x,point.y,point.z);

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

        //getline (myfile, line);
        //sscanf (line.c_str(),"%f",averagez);
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


template<class Type> Type readbuffer4(int *newsockfd,const char* chars){

    char* buffer=new char [4];
    int n=read(*newsockfd,buffer,4);
    if ( n< 0) error("ERROR reading from socket");

    //point no
    Type res =(Type)bytesToFloat(buffer[0], buffer[1], buffer[2], buffer[3]);

    //std::cout << chars<<res << std::endl;


    delete [] buffer;
    return res;

}


void unittreatment(int *newsockfd,bool *update, boost::shared_ptr<pcl::visualization::PCLVisualizer> * viewer){



    int points_no=0;
       float averagez, Yaw, Pitch, Roll, posx, posy, posz;

char* delimiter=new char[6];
char* buffer = NULL/*, *min=NULL*/;
 while(threads){

         //std::cout<<"delimiter : "<<delimiter<<std::endl;
         if(read(*newsockfd,delimiter,6)<0)continue;
        // std::cout<< "loop" << std::endl;
         if(strcmp(delimiter,"#)#)#)")==0){
             int len = 0;
             ioctl(*newsockfd, FIONREAD, &len);
             char pad[len];
             if (len > 0) {
               len = read(*newsockfd, pad, len);
             }
             std::cout<<"son stopped"<<std::endl;
             *update=true;
             //TODO close son soket
             close(*newsockfd);
             delete newsockfd;
             pthread_cancel(pthread_self());

             break;
         }

         if(strcmp(delimiter,"(.(.(.")!=0)continue;

        points_no=(int)readbuffer4<int>(newsockfd,"points_no ");
        if(points_no<=0)continue;
        averagez=readbuffer4<float>(newsockfd,    "averagez  ");
        if(averagez<=0.300)continue;

        Yaw=readbuffer4<float>(newsockfd,         "Yaw       ");
        if(Yaw<-4 || Yaw>4)continue;
        Pitch=readbuffer4<float>(newsockfd,       "Pitch     ");
        if(Pitch<-4 || Pitch>4)continue;

        Roll=readbuffer4<float>(newsockfd,        "Roll      ");
        if(Roll<-4 || Roll>4)continue;


        posx=readbuffer4<float>(newsockfd,        "posx      ");
          if(posx<-7 || posx>7)continue;
        posy=readbuffer4<float>(newsockfd,        "posy      ");
          if(posy<-7 || posy>7)continue;
        posz=readbuffer4<float>(newsockfd,        "posz      ");


        //TOODO wait until all data are sent


        std::cout << "------------------------------------------------------" << std::endl;


       int buffer_point_size=points_no*sizeof(float);
       delete[]buffer;
       buffer=new char[buffer_point_size];

       boost::this_thread::sleep (boost::posix_time::milliseconds (500));
       buffer[buffer_point_size-1]='0';

       if (read(*newsockfd,buffer,buffer_point_size) < 0) error("ERROR reading from socket");

       if(buffer[buffer_point_size-1]=='0')continue;

      /* delete[]buffer;
       buffer=new char[buffer_point_size];
       bzero(buffer,buffer_point_size);
       delete[]min;
       min=new char[3];
       int l=0;
       int count=buffer_point_size;
       while(buffer[buffer_point_size-1]==0 && !breakk){
       //if (read(*newsockfd,buffer,buffer_point_size) < 0) error("ERROR reading from socket");
           if (read(*newsockfd,min,3) < 0) error("ERROR reading from socket");
           else{
               bzero(min,3);
               buffer[l]=min[0];
               buffer[l+1]=min[1];
               buffer[l+2]=min[2];
               std::cout <<buffer[l]<<" "<<buffer[l+1]<<" "<<buffer[l+2]<<std::endl;
               if(min[2]!=0)l+=3;
               else if((min[0]!=0 && min[1]==0))l+=2;
               else if((min[0]==0 && min[1]!=0))l+=1;
            }
       }
*/


       //std::cout << "buffer_point_size"<<buffer_point_size << std::endl;



   pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr =convertBytesToPointCloud(buffer,buffer_point_size);

  /* pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud_ptr);
   (*viewer)->removePointCloud("sample cloud");
   (*viewer)->addPointCloud<pcl::PointXYZRGB> (point_cloud_ptr, rgb, "sample cloud");
    */


   std::string decision = chose_command("collision",point_cloud_ptr);

/*
       for(int k=0;k<buffer_point_size;k+=12) {

       pcl::PointXYZRGB point;
       point.x = bytesToFloat(buffer[k], buffer[k+1], buffer[k+2], buffer[k+3]);
       point.y = bytesToFloat(buffer[k+4], buffer[k+5], buffer[k+6], buffer[k+7]);
       point.z = bytesToFloat(buffer[k+8], buffer[k+9], buffer[k+10], buffer[k+11]);
       printf("%f,%f,%f\n ", point.x,point.y,point.z);

       }
       printf("point_no %d\n",(int)(points_no+8));
*/

       if (write(*newsockfd,decision.c_str(),2) < 0) error("ERROR writing to socket");
       *update=true;


    }
}



void server(int *sockfd,bool *update, boost::shared_ptr<pcl::visualization::PCLVisualizer> * viewer){


    //preparation du socket
    int portno, pid, visualizerpid;
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
    int* newsockfd = new int;
    *newsockfd = accept(*sockfd, (struct sockaddr *) &cli_addr, &clilen);


     std::cout<<"son connected"<<std::endl;
     boost::thread unittreatmentthread(unittreatment,newsockfd,update,viewer);
   /*  if(!breakk){
     boost::this_thread::sleep (boost::posix_time::seconds (20));
     std::cout<<"son stopped"<<std::endl;
     pthread_cancel(unittreatmentthread.native_handle());
     close(newsockfd);
     }*/
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






pcl::PointCloud<pcl::PointXYZRGB>::Ptr VoxelGrid_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr){

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::VoxelGrid<pcl::PointXYZRGB> sorvg;
     sorvg.setInputCloud (point_cloud_ptr);
     sorvg.setLeafSize (0.01f, 0.01f, 0.01f);
     sorvg.filter(*cloud_filtered);


return cloud_filtered;
}



void show_device_direction(float yaw,float pitch,float roll){

    float theta = yaw/3*M_PI - M_PI/2;
     Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();
    transform_1.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitX()));

     Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
     transform_2 =transform_1;

     theta = -roll/3*M_PI;
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
    viewer->addLine (line_coeff);

}

// --------------
// -----Main-----
// --------------
int
main (int argc, char** argv)
{

bool update=false;

int sockfd;
boost::thread serverthread(server,&sockfd,&update, &viewer);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB> ());



float yaw, pitch,roll,posx,posy,posz,averagez;
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr = readFile("output12.csv",&averagez,&posx,&posy,&posz,&yaw,&pitch,&roll);

//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud_ptr);
//viewerinitial->addPointCloud<pcl::PointXYZRGB> (point_cloud_ptr, rgb, "sample cloud");

//show_device_direction(yaw, pitch, roll);
//cloud_filtered=VoxelGrid_filter(point_cloud_ptr);
//chose_command("collision",cloud_filtered);
//chose_command("identification",cloud_filtered);


      while (1)
      {
        if(viewer->wasStopped () || viewerinitial->wasStopped ())break;
        if(update){
        std::cout << " cout refresh loop "<< std::endl;
        update=false;
        }



        viewer->spinOnce (100);
        viewerinitial->spinOnce(100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));

      }
      stop_all_threads();
      close(sockfd);
      pthread_cancel(serverthread.native_handle());
      pthread_cancel(pthread_self());
      std::cout<<" all threads stopped " << !threads <<std::endl;




return 0;

}
