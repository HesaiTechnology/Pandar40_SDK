#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <syslog.h>
#include <sys/stat.h>
#include <jpeglib.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sys/time.h>

void my_output_message (j_common_ptr ptr)
{
  return;
}

int decompressjpeg(unsigned char *jpg_buffer, unsigned long jpg_size, unsigned char * &bmp, unsigned long& bmpSize)
{
  // Variables for the decompressor itself
  struct jpeg_decompress_struct cinfo;
  struct jpeg_error_mgr jerr;

  unsigned char *bmp_buffer;
  int row_stride, width, height, pixel_size;

  cinfo.err = jpeg_std_error(&jerr);
  cinfo.err->trace_level = 1;
  cinfo.err->output_message = my_output_message;
  printf("%d\n" , cinfo.err->trace_level);
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
  free(jpg_buffer);
  return 0;
}

int main(int argc, char *argv[])
{
  int rc, i, j;
  if (argc != 2)
  {
    fprintf(stderr, "USAGE: %s filename.jpg\n", argv[0]);
    exit(EXIT_FAILURE);
  }
  struct stat file_info;
  unsigned long jpg_size;
  unsigned char *jpg_buffer;

  rc = stat(argv[1], &file_info);
  if (rc)
  {
    exit(EXIT_FAILURE);
  }
  jpg_size = file_info.st_size;
  jpg_buffer = (unsigned char *)malloc(jpg_size + 100);

  int fd = open(argv[1], O_RDONLY);
  i = 0;
  while (i < jpg_size)
  {
    rc = read(fd, jpg_buffer + i, jpg_size - i);
    i += rc;
  }
  close(fd);

  unsigned char *bmp;
  unsigned long bmpSize;

  struct timeval tv_s;
struct timeval tv_e;

gettimeofday(&tv_s , NULL);


  decompressjpeg(jpg_buffer, jpg_size, bmp, bmpSize);
gettimeofday(&tv_e , NULL);
	printf("elasped : %d \n " , tv_e.tv_usec - tv_s.tv_usec);

  cv::Mat mat = cv::Mat(720, 1280, CV_8UC3, bmp);
  cv::imwrite("result.jpg", mat);
  return 0;

  int ofd = open("result.bmp", O_RDWR | O_CREAT, 0666);
  // printf("tail: %02x\n", (char)((char*)pic->yuv)[pic->header.len - 1]);
  printf("bmpsize: %d\n", bmpSize);
  write(ofd, bmp, bmpSize);
  // free(bmp);
  close(fd);
}