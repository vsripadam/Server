/**
*       @file opencv_reproject_image_pcl.cpp
*       @brief Reproject an image to Point Cloud using OpenCV and PCL.
*       @author Martin Peris (http://www.martinperis.com)
*       @date 06/01/2012
*/

/*
        opencv_reproject_image_pcl.cpp - Reproject an image to Point Cloud
        using OpenCV and PCL. The program receives from command line an 
        rgb-image (left image of stereo rig) a disparity-image (obtained with 
        some stereo matching algorithm) and the matrix Q (Generated at calibration 
        stage). It displays the 3D reconstruction of the scene using PCL.
        Copyright (c) 2012 Martin Peris (http://www.martinperis.com).
        All right reserved.
        
        This application is free software; you can redistribute it and/or
        modify it under the terms of the GNU Lesser General Public
        License as published by the Free Software Foundation; either
        version 2.1 of the License, or (at your option) any later version.
        
        This application is distributed in the hope that it will be useful,
        but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
        Lesser General Public License for more details.
        
        You should have received a copy of the GNU Lesser General Public
        License along with this application; if not, write to the Free Software
        Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <string>
#include <pcl/common/common_headers.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <vtkTriangleFilter.h>
#include <vtkTriangle.h>
#include <vtkCleanPolyData.h>
#include <vtkGenericDataObjectReader.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <arpa/inet.h>

//Kids, using this kind of #define is quite dirty, don't do it at home!!
#define CUSTOM_REPROJECT
/*** To understand the CUSTOM_REPROJECT code, please read Chapter 12 of the book
  Learning OpenCV: Computer Vision with the OpenCV Library. (Page 435) 
  I am using it because cv::reprojectImageTo3D is not giving me the expected
  results for some reason.
  
  If you want to use this program with cv::reprojectImageTo3D please comment
  the previous #define CUSTOM_REPROJECT and recompile.
    
***/

using namespace cv;
using namespace std;

void error(const char *msg)
{
    perror(msg);
    exit(1);
}

void recv(int sockfd, void* buffer, int size) {
  if(read(sockfd,buffer,size) < 0)
    error("receive failed");
}
int main( int argc, char** argv )
{
  int sockfd, newsockfd, portno;
  socklen_t clilen;
  char buffer[1024];
  char str[INET_ADDRSTRLEN];
  int optval = 1;
  struct sockaddr_in serv_addr, cli_addr;
  int n;

  struct addrinfo hints, *info, *p;
  int gai_result;

  char hostname[1024];
  hostname[1023] = '\0';
  gethostname(hostname, 1023);

  memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_UNSPEC; /*either IPV4 or IPV6*/
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_flags = AI_CANONNAME;

  if ((gai_result = getaddrinfo(hostname, "http", &hints, &info)) != 0) {
    fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(gai_result));
    exit(1);
  }

  for(p = info; p != NULL; p = p->ai_next) {
    printf("hostname: %s\n", p->ai_canonname);
  }

  freeaddrinfo(info);

  if (argc < 2) {
    fprintf(stderr,"ERROR, no port provided\n");
    exit(1);
  }
  sockfd = socket(AF_INET, SOCK_STREAM, 0);

  if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR,(const void *)&optval,sizeof(int)) < 0)
      return -1; 

  if (sockfd < 0) 
     error("ERROR opening socket");
  bzero((char *) &serv_addr, sizeof(serv_addr));
  portno = atoi(argv[1]);
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  serv_addr.sin_port = htons(portno);
  if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) 
    error("ERROR on binding");

  inet_ntop(AF_INET, &(serv_addr.sin_addr), str, INET_ADDRSTRLEN);
  std::cout << "Binding successful on server addr " << str << ":" << portno << std::endl;
  
  listen(sockfd,5);
  std::cout << "got here" << std::endl;
  clilen = sizeof(cli_addr);
  newsockfd = accept(sockfd, 
              (struct sockaddr *) &cli_addr, 
              &clilen);
  std::cout << "passed accept" << std::endl;
  if (newsockfd < 0) 
       error("ERROR on accept");
  bzero(buffer,1024);

  std::cout << "CONNECTED" << std::endl;

  

  /*n = read(newsockfd,buffer,1023);
    if (n < 0) error("ERROR reading from socket");

  std::cout << "message was: " << buffer << std::endl;*/
  
  double Q03, Q13, Q23, Q32, Q33;
  int height;
  int width;

  int numImages;

  recv(newsockfd, (void*) &Q03, sizeof(Q03));
  recv(newsockfd, (void*) &Q13, sizeof(Q13));
  recv(newsockfd, (void*) &Q23, sizeof(Q23));
  recv(newsockfd, (void*) &Q32, sizeof(Q33));
  recv(newsockfd, (void*) &Q33, sizeof(Q33));
  recv(newsockfd, (void*) &height, sizeof(height));
  recv(newsockfd, (void*) &width, sizeof(width));
  recv(newsockfd, (void*) &numImages, sizeof(numImages));

  cv::Mat img = cv::Mat::zeros( height,width, CV_8UC3);
  int imgSize = img.total()*img.elemSize();
  uchar sockData[imgSize];

  //Receive data here

  if ((read(newsockfd, buffer, imgSize))<0) {
      error("recv failed");
  }

  // Assign pixel value to img

  int ptr=0;
  for (int i = 0;  i < img.rows; i++) {
      for (int j = 0; j < img.cols; j++) {                                     
          img.at<cv::Vec3b>(i,j) = cv::Vec3b(sockData[ptr+ 0],sockData[ptr+1],sockData[ptr+2]);
          ptr=ptr+3;
      }
  }
  //Mat image(690,690,CV_8UC3,*buffer);
  //imwrite("/home/securitas/images/prova.jpg",image);
  close(newsockfd);
  close(sockfd);

  //namedWindow( "Server", CV_WINDOW_AUTOSIZE );// Create a window for display.
  //imshow( "Server", image );  
  //waitKey(0);
  
  return 0;
}


int reprojectDisparity(double Q03, double Q13, double Q23, double Q32, double Q33, cv::Mat &img_rgb, cv::Mat &img_disparity, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &point_cloud_ptr) {

  //If size of Q is not 4x4 exit
  /*if (Q.cols != 4 || Q.rows != 4)
  {
    std::cerr << "ERROR: Q size is not 4x4" << std::endl;
    return 1;
  }*/

#ifdef CUSTOM_REPROJECT
  //Get the interesting parameters from Q
  /*double Q03, Q13, Q23, Q32, Q33;
  Q03 = Q.at<double>(0,3);
  Q13 = Q.at<double>(1,3);
  Q23 = Q.at<double>(2,3);
  Q32 = Q.at<double>(3,2);
  Q33 = Q.at<double>(3,3);*/
  
  std::cout << "Q(0,3) = "<< Q03 <<"; Q(1,3) = "<< Q13 <<"; Q(2,3) = "<< Q23 <<"; Q(3,2) = "<< Q32 <<"; Q(3,3) = "<< Q33 <<";" << std::endl;
  
#endif  
  
  //Load rgb-image
  if (img_rgb.data == NULL)
  {
    std::cerr << "ERROR: rgb-image empty" << std::endl;
    return 1;
  }
  
  //Load disparity image
  if (img_disparity.data == NULL)
  {
    std::cerr << "ERROR: disparity-image empty" << std::endl;
    return 1;
  }
  
  //Both images must be same size
  if (img_rgb.size() != img_disparity.size())
  {
    std::cerr << "ERROR: rgb-image and disparity-image have different sizes " << std::endl;
    return 1;
  }
  
#ifndef CUSTOM_REPROJECT
  //Create matrix that will contain 3D corrdinates of each pixel
  cv::Mat recons3D(img_disparity.size(), CV_32FC3);
  
  //Reproject image to 3D
  std::cout << "Reprojecting image to 3D..." << std::endl;
  cv::reprojectImageTo3D( img_disparity, recons3D, Q, false, CV_32F );
#endif  
  //Create point cloud and fill it
  std::cout << "Creating Point Cloud..." <<std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  
  double px, py, pz;
  uchar pr, pg, pb;
  
  for (int i = 0; i < img_rgb.rows; i++)
  {
    uchar* rgb_ptr = img_rgb.ptr<uchar>(i);
#ifdef CUSTOM_REPROJECT
    uchar* disp_ptr = img_disparity.ptr<uchar>(i);
#else
    double* recons_ptr = recons3D.ptr<double>(i);
#endif
    for (int j = 0; j < img_rgb.cols; j++)
    {
      //Get 3D coordinates
#ifdef CUSTOM_REPROJECT
      uchar d = disp_ptr[j];
      if ( d == 0 ) continue; //Discard bad pixels
      double pw = -1.0 * static_cast<double>(d) * Q32 + Q33; 
      px = static_cast<double>(j) + Q03;
      py = static_cast<double>(i) + Q13;
      pz = Q23;
      
      px = px/pw;
      py = py/pw;
      pz = pz/pw;

#else
      px = recons_ptr[3*j];
      py = recons_ptr[3*j+1];
      pz = recons_ptr[3*j+2];
#endif
      
      //Get RGB info
      pb = rgb_ptr[3*j];
      pg = rgb_ptr[3*j+1];
      pr = rgb_ptr[3*j+2];
      
      //Insert info into point cloud structure
      pcl::PointXYZRGB point;
      pcl::PointXYZ point2;
      point2.x = point.x = px;
      point2.y = point.y = py;
      point2.z = point.z = pz;

      uint32_t rgb = (static_cast<uint32_t>(pr) << 16 |
              static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
      point.rgb = *reinterpret_cast<float*>(&rgb);
      point_cloud_ptr->points.push_back (point);
      point_cloud_ptr_xyz->points.push_back (point2);

    }
  }
  point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
  point_cloud_ptr->height = 1;
  point_cloud_ptr_xyz->width = (int) point_cloud_ptr_xyz->points.size();
  point_cloud_ptr_xyz->height = 1;

  std::cout << "Created Point Cloud..." <<std::endl;

  pcl::io::savePCDFileASCII ("test_pcd.pcd", *point_cloud_ptr);
  std::cout << "Created Point Cloud..." <<std::endl;
  pcl::io::savePCDFileASCII ("test_pcd_xyz.pcd", *point_cloud_ptr_xyz);
  pcl::io::savePLYFileASCII ("output.ply", *point_cloud_ptr_xyz); 
 
  std::cout << "reconstruction complete" << std::endl;

  return 0;

}

