/* \author Geoffrey Biggs */


#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>



#include <stdio.h>
#include <csignal>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <conv.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

static int *update;
volatile float *sharedbuffer;
volatile int *shared_buffer_size;

void error(const char *msg)
{
    perror(msg);
    exit(1);
}


void dostuff (int sock)
{

//printf("---------------------%f\n", cloud->points[0].x );

   int n;

   char* buffer=new char [4];

   if (read(sock,buffer,4) < 0) error("ERROR reading from socket");

   int points_no =(int)bytesToFloat(buffer[0], buffer[1], buffer[2], buffer[3]) - 1;
   delete [] buffer;

   int buffer_point_size=points_no*(int)sizeof(float);
   buffer=new char[buffer_point_size];
   if (read(sock,buffer,buffer_point_size) < 0) error("ERROR reading from socket");

printf("%d\n", buffer_point_size );
printf("Message from client:\n");



//convertBytesToPointCloud(buffer,buffer_point_size);
 *shared_buffer_size=points_no;

int j;
for(int k=0;k<buffer_point_size;k+=4) {
 sharedbuffer[j]=bytesToFloat(buffer[k], buffer[k+1], buffer[k+2], buffer[k+3]);
std::cout<<" "<<sharedbuffer[j]<<std::endl;
j++;
}
printf("\n");




   n = write(sock,"I got your message",18);
   if (n < 0) error("ERROR writing to socket");
}




boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis ()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
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
createPointCloud(){

pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);


 uint8_t r(255), g(15), b(15);
  for (int z=0; z < *shared_buffer_size; z +=3)
  {
      pcl::PointXYZRGB point;
      point.x = sharedbuffer[z];
      point.y = sharedbuffer[z+1];
      point.z = sharedbuffer[z+2];
      std::cout<<point.x<<" "<<point.y<<" "<<point.z<<std::endl;
      uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                      static_cast<uint32_t>(g) << 8  |
                      static_cast<uint32_t>(b)        );
      point.rgb = *reinterpret_cast<float*>(&rgb);
      point_cloud_ptr->points.push_back (point);

      r -= 12;
      g += 12;

  }

  point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
  point_cloud_ptr->height = 1;

return point_cloud_ptr;
}


void sigHandler(int signo)
{
    std::cout << "Fenetre férmé de la part du process id: " << signo << std::endl;
    exit(signo);
}


// --------------
// -----Main-----
// --------------
int
main (int argc, char** argv)
{

    //preparation du socket
    int sockfd, newsockfd, portno, pid, visualizerpid;
    socklen_t clilen;
    struct sockaddr_in serv_addr, cli_addr;

   sockfd = socket(AF_INET, SOCK_STREAM, 0);

   if (sockfd < 0)
       error("ERROR opening socket");
    bzero((char *) &serv_addr, sizeof(serv_addr));
    portno = 60588;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);
    if (bind(sockfd, (struct sockaddr *) &serv_addr,
             sizeof(serv_addr)) < 0)
             error("ERROR on binding");
    listen(sockfd,5);
    clilen = sizeof(cli_addr);



    update = (int *)mmap(NULL, sizeof *update, PROT_READ | PROT_WRITE,
                    MAP_SHARED | MAP_ANONYMOUS, -1, 0);

    *update = 0;

    shared_buffer_size = (int*)mmap(0, sizeof(int), PROT_READ|PROT_WRITE,
               MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    *shared_buffer_size=50000;

     sharedbuffer = (float*)mmap(0, (*shared_buffer_size)*sizeof(float), PROT_READ|PROT_WRITE,
                   MAP_SHARED | MAP_ANONYMOUS, -1, 0);

     memset((void *)sharedbuffer, 0, (*shared_buffer_size)*sizeof(float));








 boost::shared_ptr<pcl::visualization::PCLVisualizer>  viewer = rgbVis();










  visualizerpid = fork();
  if (visualizerpid < 0)
  error("ERROR on fork");
  if (visualizerpid == 0)  {





    //  std::cout << "entier par le fils: " << point_cloud_ptr->points[0].x << std::endl;

   //son prepare for close window event
      signal(SIGHUP, sigHandler);
      while (1)
      {
      newsockfd = accept(sockfd,
             (struct sockaddr *) &cli_addr, &clilen);

        if (newsockfd < 0)
           error("ERROR on accept");
       pid = fork();
       if (pid < 0)
           error("ERROR on fork");
       if (pid == 0)  {
           close(sockfd);

           dostuff(newsockfd);
           *update = 1;

           exit(0);
       }
       else{ close(newsockfd);}
     }

      exit(0);

  }else{


      while (!viewer->wasStopped ())
      {

        if(*update){

            std::cout<<"first value "<<sharedbuffer[2]<<std::endl;
        //std::cout << "test0: " << update << std::endl;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr=createPointCloud();
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud_ptr);

            viewer->addPointCloud<pcl::PointXYZRGB> (point_cloud_ptr, rgb, "sample cloud");
        *update=0;
        }
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));

      }

      kill(visualizerpid, SIGHUP);
      close(sockfd);
  }


return 0;

}
