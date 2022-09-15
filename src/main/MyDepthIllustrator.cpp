#include "MyDepthIllustrator.h"
#include <opencv/highgui.h>
#include <stdint.h>

void myDrawRgbDepthMap(CvMat* img_depth, int min_dst, int rg, int gb, int max_dst, CvMat* dst)
{
	int h = img_depth->rows;
	int w = img_depth->cols;
	CvMat* colored_depth = dst;
	const int depth_pix_size = 2;
	for(int i = 0; i < h; i++)
	{
		for(int j = 0; j < w; j++)
		{
			uint16_t depth = *((uint16_t*)(img_depth->data.ptr + (i * w + j) * depth_pix_size));
			uint8_t r = 0;
			uint8_t g = 0; 
			uint8_t b = 0;

			if(depth < min_dst)
			{
			}
			else if(depth <= rg)
			{
				g = (255 * depth) / rg;
				r = 255 - g;
			}
			else if(depth <= gb)
			{
				b = (255 * (depth - rg)) / (gb - rg);
				g = 255 -b;
			}
			else if(depth < max_dst)
			{
				b = 255;
			}
			else
			{
			}
			((uint8_t*)colored_depth->data.ptr)[(i * w + j)*3 + 0] = b;
			((uint8_t*)colored_depth->data.ptr)[(i * w + j)*3 + 1] = g;
			((uint8_t*)colored_depth->data.ptr)[(i * w + j)*3 + 2] = r;
		}
	}
}

void myShowRgbDepthMap(CvMat* img_depth, int min_dst, int rg, int gb, int max_dst, const char* window_name)
{
	int h = img_depth->rows;
	int w = img_depth->cols;
	CvMat* colored_depth = cvCreateMat(h, w, CV_8UC3);
	
	myDrawRgbDepthMap(img_depth, min_dst, rg, gb, max_dst, colored_depth);
	cvShowImage(window_name, colored_depth);
	cvReleaseMat(&colored_depth);
}



//MyDepthIllustrator::MyDepthIllustrator()
//{
//	CvMat* dst_ = 0;
//	min_dst_ = 0;
//	rg_ = 2000;
//	gb_ = 5000;
//	max_dst_ = 10000;
//}
//
//MyDepthIllustrator::~MyDepthIllustrator()
//{
//	if(dst_ != 0)
//		cvReleaseMat(&dst_);
//}
//
//int MyDepthIllustrator::update(CvMat* src, CvMat* dst)
//{
//	int h = src->rows;
//	int w = src->cols;
//	CvMat* img_depth = src;
//	CvMat* colored_depth;
//	if(dst != 0)
//	{
//		colored_depth = dst;
//		dst_ = dst;
//	}
//	else
//	{
//		if(dst_ == 0)
//		{
//			dst_ = cvCreateMat(h, w, CV_8UC3);
//		}
//		colored_depth = dst_;
//	}
//
//	for(int i = 0; i < h_; i++)
//	{
//		for(int j = 0; j < w_; j++)
//		{
//			float depth = img_depth->data.fl[i * w_ + j];
//			uint8_t r = 0;
//			uint8_t g = 0; 
//			uint8_t b = 0;
//			float rg = 1000; //mm
//			float gb = 2000; //mm
//			float binf = 5000; //mm
//
//			if(depth < min_dst_ || depth > max_dst_)
//			{
//			}
//			else if(depth <= rg)
//			{
//				g = (255 * depth) / rg;
//				r = 255 - g;
//			}
//			else if(depth <= gb)
//			{
//				b = (255 * (depth - rg)) / (gb - rg);
//				g = 255 -b;
//			}
//			else
//			{
//				b = 255;
//			}
//			((uint8_t*)colored_depth->data.ptr)[(i * w_ + j)*3 + 0] = b;
//			((uint8_t*)colored_depth->data.ptr)[(i * w_ + j)*3 + 1] = g;
//			((uint8_t*)colored_depth->data.ptr)[(i * w_ + j)*3 + 2] = r;
//		}
//	}
//
//	return 0;
//}
