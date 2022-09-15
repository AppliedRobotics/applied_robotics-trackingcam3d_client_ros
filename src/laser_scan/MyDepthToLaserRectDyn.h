#ifndef MY_DEPTH_TO_LASER_RECT_DYN_H
#define MY_DEPTH_TO_LASER_RECT_DYN_H

#include "MyPerformanceMeter.h"
#include <opencv2/opencv.hpp>
#include <stdint.h>

class MyDepthToLaserRectDyn
{
	int w_;
	int h_;
	CvMat* Q_;
	CvMat* depth_to_laser_rot_mat_;
	CvMat* mxy_; //Where to move image pixels to rotate camera to virtual rangefinder position 
	CvMat* depth_rectification_factor_map_; //How depth changes at each point wherein

	CvMat* depth_lut_;
	
	const CvMat* depth_src_;
	CvMat* depth_rect_;

	int laser_beam_n_;

	float min_dst_; //[m]
	float max_dst_; //[m]
	
	float f_;
	float t_;
	int cx_;
	int cy_;

	float depth_multiplier_;
	float y_multiplier_;
	float min_distance_max_depth_;
	float max_distance_min_depth_;

	float* distance_lut_;

	int y_min_;
	int y_max_;

	float min_height_; //[m]
	float max_height_; //[m]

	//YMinMaxDepthLutPoint* y_lut = new YMinMaxDepthLutPoint[h];

	float laser_right_angle_;
    float laser_left_angle_;
    float laser_angle_of_view_;
	float laser_c_angle_;
	int* laser_beam_pixel_lut_; //1D array that maps laser beams to rectified image pixels x (xr)
 
	//Represent in ROS laser_scan coordinates where 0 is along x axis
    float laser_min_angle_;
    float laser_max_angle_;
    float laser_scan_increment_;

	struct LineFillerSrcInfo
	{
		int32_t x;
		float dst;
	};

	struct DepthSrcByColInfo		//Where we got depth from
	{
		int32_t y;
		float depth;
	};

    struct FillerSrcInfo	/* Data about good pixel where we got data for current bad pixel and distance between them */
    {
        int32_t x;
        int32_t y;
        int32_t dst;
    };

	LineFillerSrcInfo* line_src_;
	float* line_;	//1D array of laser data [m] in image CS for now
	float* laser_data_; //[m]
	DepthSrcByColInfo* depth_sorce_point_info_;	// 1D array

    FillerSrcInfo* fs_; // for 2d nearest method

	int optimization_;
	int input_type_;

	int init_rect_map(CvMat* R_laser_to_cam);
	int init_rect_map_int(CvMat* R_laser_to_cam);
	int init_rect_map_simd128(CvMat* R_laser_to_cam);
	int init_rect_map_simd128_2(CvMat* R_laser_to_cam);
	int pre_init_rect_map_statics();
	int init_depth_restriction_lut();
	int init_distance_calculation_lut();
	int init_laser_parameters_and_lut();
    int rect_restrict_by_cols();
	int fill_line();
	int select_laser_beams();

	int simd_endian_test();

	int rect();

    int rect_restrict();
    int fill2d();
    int get_line_from_filled_depth_map();

public:
	MyDepthToLaserRectDyn();
	~MyDepthToLaserRectDyn();

	int init(int img_w, int img_h, CvMat* Q, int laser_beam_n, float min_dst, float max_dst, float min_height, float max_height, int roi_h);
	int update(CvMat* src, CvMat* R_laser_to_cam);

    enum{METHOD_NEAREST_BY_COLS = 0, METHOD_NEAREST_2D = 1};
    int method_;
    MyPerformanceMeter perf_;

	int draw(CvMat* img_show);
	void show();

	void operator()(CvMat* img_depth, CvMat* R_laser_to_cam);

	enum{OPTIMIZATION_NO = 0, OPTIMIZATION_SIMD128 = 1};

	static int depth_u16_to_f32(CvMat* src, CvMat* dst);
	//int depth_u16_to_f32_simd128(CvMat* src, CvMat* dst);

	int set_optimization(int optimization);

	const float* get_laser_data();
	int get_laser_beam_n();

	float get_laser_min_angle();
    float get_laser_max_angle();
    float get_laser_scan_increment();

	int get_optimization();
};

#endif