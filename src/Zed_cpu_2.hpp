#include <stdio.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <chrono>
#include "StereoCamera.cpp"



void correctFramerate(int resolution, double& frame_rate);

void zed_stop();
void *zed_main(void *thread_id);



