#include <stdio.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <chrono>

#define WIDTH_ID 3
#define HEIGHT_ID 4
#define FPS_ID 5

class StereoCamera
{
public:
  /**
   * @brief      { stereo camera driver }
   *
   * @param[in]  resolution  The resolution
   * @param[in]  frame_rate  The frame rate
   */
  StereoCamera(std::string device_name, int resolution, double frame_rate) //: frame_rate_(30.0)
  {
    camera_ = new cv::VideoCapture(device_name,cv::CAP_ANY);
    cv::Mat raw;
    cv::Mat left_image;
    cv::Mat right_image;
    setResolution(resolution);
    // // this function doesn't work very well in current Opencv 2.4, so, just use ROS to control frame rate.
    setFrameRate(frame_rate);

    printf("Stereo Camera Set Resolution %d, width %f, height %f", resolution, camera_->get(WIDTH_ID),
             camera_->get(HEIGHT_ID));
  }

  ~StereoCamera()
  {
    // std::cout << "Destroy the pointer" << std::endl;
    delete camera_;
  }

  /**
   * @brief      Sets the resolution.
   *
   * @param[in]  type  The type
   */
  void setResolution(int type)
  {
    switch (type)
    {
      case 0:
        width_ = 4416;
        height_ = 1242;
        break;
      case 1:
        width_ = 3840;
        height_ = 1080;
        break;
      case 2:
        width_ = 2560;
        height_ = 720;
        break;
      case 3:
        width_ = 1344;
        height_ = 376;
        break;
      default:
        printf("Unknow resolution passed to camera: %d", type);
    }

    camera_->set(WIDTH_ID, width_);
    camera_->set(HEIGHT_ID, height_);
    // make sure that the number set are right from the hardware
    width_ = camera_->get(WIDTH_ID);
    height_ = camera_->get(HEIGHT_ID);
  }

  /**
   * @brief      Sets the frame rate.
   *
   * @param[in]  frame_rate  The frame rate
   */
  void setFrameRate(double frame_rate)
  {
    camera_->set(cv::CAP_PROP_FPS , frame_rate);
    frame_rate_ = camera_->get(FPS_ID);
    printf("................................................%lf",frame_rate_);
  }

  /**
   * @brief      Gets the images.
   *
   * @param      left_image   The left image
   * @param      right_image  The right image
   *
   * @return     The images.
   */
  bool getImages(cv::Mat& left_image, cv::Mat& right_image)
  {
    cv::Mat raw;
    if (camera_->grab())
    {
      camera_->retrieve(raw);
      cv::Rect left_rect(0, 0, width_ / 2, height_);
      cv::Rect right_rect(width_ / 2, 0, width_ / 2, height_);
      left_image = raw(left_rect);
      right_image = raw(right_rect);
      cv::waitKey(10);
      return true;
    }
    else
    {
      return false;
    }
  }
private:
  cv::VideoCapture* camera_;
  int width_;
  int height_;
  double frame_rate_;
  bool cv_three_;

};

