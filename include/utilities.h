#ifndef PANDORA_UTILITY_H
#define PANDORA_UTILITY_H

#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>

void conv_yuv400_to_mat(cv::Mat &dst, void *pYUV400, int nWidth, int nHeight, int bit_depth);
void YUVToRGB(const int &iY, const int &iU, const int &iV, int &iR, int &iG, int &iB);
void yuv422_to_rgb24(unsigned char *uyvy422, unsigned char *rgb24, int width, int height);
void conv_yuv422_to_mat(cv::Mat &dst, void *pYUV422, int nWidth, int nHeight, int bit_depth);
bool load_camera_intrinsics(const std::string &filename, std::vector<cv::Mat> &camera_k_list, std::vector<cv::Mat> &camera_d_list);
int decompressJpeg(unsigned char *jpgBuffer, unsigned long jpgSize, unsigned char * &bmp, unsigned long& bmpSize);

#endif