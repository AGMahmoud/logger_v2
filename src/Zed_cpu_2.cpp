#include <stdio.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <chrono>
#include "Zed_cpu_2.hpp"
#include <unistd.h>
#include <sys/stat.h>
#include <iostream>
#include <fstream>

#define WIDTH_ID 3
#define HEIGHT_ID 4
#define FPS_ID 5

using namespace std;
bool zed_run   = true;
int f_counter=1;
char img_dir[100];
void correctFramerate(int resolution, double& frame_rate)
  {
    double max_frame_rate;
    std::string reso_str = "";
    switch (resolution)
    {
      case 0:
        max_frame_rate = 15;
        reso_str = "2K";
        break;
      case 1:
        max_frame_rate = 30;
        reso_str = "FHD";
        break;
      case 2:
        max_frame_rate = 60;
        reso_str = "HD";
        break;
      case 3:
        max_frame_rate = 100;
        reso_str = "VGA";
        break;
      default:
        printf("Unknow resolution passed");
        return;
    }
    if (frame_rate > max_frame_rate)
      printf("frame_rate(%fHz) too high for resolution(%s), downgraded to %fHz", frame_rate, reso_str.c_str(),
               max_frame_rate);
    frame_rate = max_frame_rate;
  }

void zed_stop()
{
zed_run=false;
}

void open_new_img_dir(){

struct stat st;

//char x;
    char command[100];
    sprintf(img_dir, "../data%d", f_counter);
    while ((stat(img_dir,&st) == 0)&&(st.st_mode & S_IFDIR != 0)){
        f_counter++;
        sprintf(img_dir, "../data%d", f_counter);
    }
std::cout<<f_counter<<std::endl;
//std::cin>>x;
	//fclose(t_file);	
    sprintf(img_dir,"../data%d/sequence",f_counter);
    sprintf(command, "mkdir ../data%d", f_counter);
    system(command);
    usleep(1000);
    sprintf(command, "mkdir ../data%d/sequence", f_counter);
    system(command);
}

void *zed_main(void *thread_id)
{
	open_new_img_dir();
	usleep(1000);
	ofstream timestamp;
        char cur_path[100];
	char cur_file[100];

        strcpy(cur_path,img_dir);
	strcat(cur_path,"/time_stamp.txt");
	timestamp.open(cur_path);
        //std::cout<<cur_path<<std::endl;	
	double frame_rate_=10.0;
	int resolution_= 3;
	std::string reso_str= "HD";
	//correctFramerate(resolution_, frame_rate_);
	printf("Try to initialize the camera with HD and ");
    	StereoCamera zed(std::string("/dev/video0"), resolution_, frame_rate_);
    	printf("Initialized the camera");
	cv::Mat left_image, right_image;
	int frame_id=0;
	double time_now;
	std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();

   	 while(zed_run)
		{
			if (!zed.getImages(left_image, right_image))
      		{
        		printf("Can't find camera");
      		}
      		else
      		{
      	    	 std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
      	    	 time_now= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - time_start).count();
      		 frame_id++;
                 //std::cout<<"counter no "<<f_counter<<std::endl;
		 strcpy(cur_path,img_dir);
		 sprintf(cur_file,"/left%06d.jpg",frame_id);
		 strcat(cur_path,cur_file);
		 cv::imwrite(cur_path,left_image);

		 strcpy(cur_path,img_dir);
		 sprintf(cur_file,"/right%06d.jpg",frame_id);
		 strcat(cur_path,cur_file);
		 cv::imwrite(cur_path,right_image);

		 //std::cout<<cur_path<<std::endl;
		 if(frame_id%15==0)
      		 std::cout<<std::endl<<"zed frame "<<frame_id<<"  logged\n";
      		 timestamp<<frame_id<<"\t"<<time_now<<std::endl;
       		 //printf("%06d \t %lf \t Success, found camera \n",frame_id,time_now);
       		 cv::imshow("left", left_image);

       		 cv::imshow("right", right_image);
     		 }



		}
	std::cout << "\nGPS Thread Exited";
    pthread_exit(NULL);
}


