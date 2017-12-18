
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <boost/lexical_cast.hpp>
#include "utilities.h"
#include "yaml-cpp/yaml.h"
#include <jpeglib.h>


void conv_yuv400_to_mat(cv::Mat &dst, void *pYUV400, int nWidth, int nHeight, int bit_depth)
{
	IplImage *yimg;

	if (!pYUV400)
	{
		return;
	}

	if (bit_depth == 8)
	{
		yimg = cvCreateImageHeader(cvSize(nWidth, nHeight), IPL_DEPTH_8U, 1);
	}
	else
	{
		yimg = cvCreateImageHeader(cvSize(nWidth, nHeight), IPL_DEPTH_16U, 1);
	}

	cvSetData(yimg, (unsigned char *)pYUV400, nWidth);
	dst = cv::cvarrToMat(yimg);
	cvReleaseImageHeader(&yimg);
}

void YUVToRGB(const int &iY, const int &iU, const int &iV, int &iR, int &iG, int &iB)
{
	assert(&iR != NULL && &iG != NULL && &iB != NULL);

	iR = iY + 1.13983 * (iV - 128);
	iG = iY - 0.39465 * (iU - 128) - 0.58060 * (iV - 128);
	iB = iY + 2.03211 * (iU - 128);

	iR = iR > 255 ? 255 : iR;
	iR = iR < 0 ? 0 : iR;

	iG = iG > 255 ? 255 : iG;
	iG = iG < 0 ? 0 : iG;

	iB = iB > 255 ? 255 : iB;
	iB = iB < 0 ? 0 : iB;
}

void yuv422_to_rgb24(unsigned char *uyvy422, unsigned char *rgb24, int width, int height)
{
	int iR, iG, iB;
	int iY0, iY1, iU, iV;
	int i = 0;
	int j = 0;
	for (i = 0; i < width * height * 2; i += 4)
	{
		iU = uyvy422[i + 0];
		iY0 = uyvy422[i + 1];
		iV = uyvy422[i + 2];
		iY1 = uyvy422[i + 3];

		YUVToRGB(iY0, iU, iV, iR, iG, iB);
		rgb24[j++] = iR;
		rgb24[j++] = iG;
		rgb24[j++] = iB;
		YUVToRGB(iY1, iU, iV, iR, iG, iB);
		rgb24[j++] = iR;
		rgb24[j++] = iG;
		rgb24[j++] = iB;
	}
}

void conv_yuv422_to_mat(cv::Mat &dst, void *pYUV422, int nWidth, int nHeight, int bit_depth)
{
	if (!pYUV422)
	{
		return;
	}
	// static unsigned char rgb24_buffer[PIC_MAX_WIDTH*PIC_MAX_HEIGHT*3]; // todo, temporarily
	unsigned char *rgb24_buffer = new unsigned char[nWidth * nHeight * 3];
	yuv422_to_rgb24((unsigned char *)pYUV422, rgb24_buffer, nWidth, nHeight);
	dst = cv::Mat(nHeight, nWidth, CV_8UC3, rgb24_buffer).clone();
	delete[] rgb24_buffer;
}

bool load_camera_intrinsics(const std::string &filename, std::vector<cv::Mat> &camera_k_list, std::vector<cv::Mat> &camera_d_list)
{
	if ((access(filename.c_str(), 0)) == -1)
	{
		printf("invalid intrinsicFile\n");
		return false;
	}
	YAML::Node yn = YAML::LoadFile(filename);
	std::string camera_id;
	cv::Mat camera_k, camera_d;
	for (int i = 0; i < 5; i++)
	{
		// camera_id = std::to_string(i);
		camera_id = boost::lexical_cast<std::string>(i);

		if (yn[camera_id]["K"].IsDefined())
		{
			camera_k = cv::Mat::zeros(3, 3, CV_64FC1);
			for (int i = 0; i < yn[camera_id]["K"].size(); ++i)
			{
				camera_k.at<double>(i) = yn[camera_id]["K"][i].as<double>();
			}
			camera_k_list.push_back(camera_k);
		}
		else
		{
			printf("invalid intrinsicFile content\n");
			return false;
		}
		if (yn[camera_id]["D"].IsDefined())
		{
			camera_d = cv::Mat::zeros(yn[camera_id]["D"].size(), 1, CV_64FC1);
			for (int i = 0; i < yn[camera_id]["D"].size(); ++i)
			{
				camera_d.at<double>(i) = yn[camera_id]["D"][i].as<double>();
			}
			camera_d_list.push_back(camera_d);
		}
		else
		{
			printf("invalid intrinsicFile content\n");
			return false;
		}
	}
	return true;
}


void my_output_message (j_common_ptr ptr)
{
  return;
}

void print_mem(unsigned char* mem , unsigned int size)
{
	int i =0;
	for(i = 0 ; i < size ; i++)
	{
		printf("%02x " , mem[i]);
	}
	printf("\n");
}

int decompressJpeg(unsigned char *jpg_buffer, unsigned long jpg_size, unsigned char * &bmp, unsigned long& bmpSize)
{
  // Variables for the decompressor itself
  struct jpeg_decompress_struct cinfo;
  struct jpeg_error_mgr jerr;

  unsigned char *bmp_buffer;
  int row_stride, width, height, pixel_size;

  cinfo.err = jpeg_std_error(&jerr);
  cinfo.err->output_message = my_output_message;
	cinfo.err->error_exit = my_output_message;

	// print_mem(jpg_buffer + (jpg_size - 16) , 16);

  jpeg_create_decompress(&cinfo);

  jpeg_mem_src(&cinfo, jpg_buffer, jpg_size);

  int rc = jpeg_read_header(&cinfo, TRUE);

  if (rc != 1)
  {
    return -1;
  }

  jpeg_start_decompress(&cinfo);

  width = cinfo.output_width;
  height = cinfo.output_height;
  pixel_size = cinfo.output_components;

  bmpSize = width * height * pixel_size;
  bmp_buffer = (unsigned char *)malloc(bmpSize);
  row_stride = width * pixel_size;

  while (cinfo.output_scanline < cinfo.output_height)
  {
    unsigned char *buffer_array[1];
    buffer_array[0] = bmp_buffer +
                      (cinfo.output_scanline) * row_stride;

    jpeg_read_scanlines(&cinfo, buffer_array, 1);
  }
  bmp = bmp_buffer;
  jpeg_finish_decompress(&cinfo);
  jpeg_destroy_decompress(&cinfo);
  return 0;
}