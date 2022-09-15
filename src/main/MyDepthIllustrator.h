#ifndef MY_DEPTH_ILLUSTRATOR
#define MY_DEPTH_ILLUSTRATOR

#include <opencv/cv.h>

void myDrawRgbDepthMap(CvMat* depth_map, int min_dst, int rg, int gb, int max_dst, CvMat* dst);
void myShowRgbDepthMap(CvMat* depth_map, int min_dst, int rg, int gb, int max_dst, const char* window_name);
void myShowRgbDepthMap(cv::Mat &depth_map, int min_dst, int rg, int gb, int max_dst, const char* window_name);

//class MyDepthIllustrator
//{
//	int h_;
//	int w_;
//	CvMat* dst_;
//	int min_dst_;
//	int rg_;
//	int gb_;
//	int max_dst_;
//public:
//	MyDepthIllustrator();
//	~MyDepthIllustrator();
//	void init(int min_dst, int rg, int gb, int max_dst);
//	update(CvMat* src, CvMat* dst);
//	show();
//};



#endif
