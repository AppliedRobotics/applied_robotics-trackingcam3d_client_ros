#include "MyDepthToLaserRectDyn.h"
#include <opencv2/core/hal/intrin.hpp>

#ifdef _MSC_VER
#	define BEGIN_PACK __pragma(pack(push, 1))
#	define END_PACK   __pragma(pack(pop))
#	define PACK
#elif defined(__GNUC__)
#	define BEGIN_PACK
#	define END_PACK
#	define PACK		  __attribute__ ((packed))
#else
#define PACK
#endif

#define BAD_DEPTH -1
#define ASSIGN_BAD_DEPTH(_x_) _x_ = BAD_DEPTH
#define IS_DEPTH_GOOD(_x_) _x_ > 0
#define IS_DEPTH_BAD(_x_) _x_ <= 0

#define BAD_DISTANCE -1
#define ASSIGN_BAD_DISTANCE(_x_) _x_ = BAD_DISTANCE
#define IS_DISTANCE_GOOD(_x_) _x_ > 0
#define IS_DISTANCE_BAD(_x_) _x_ <= 0


BEGIN_PACK

typedef struct PACK 
{
	int16_t x;
	int16_t y;
}Point16S;

typedef struct PACK 
{
	uint16_t x;
	uint16_t y;
}Point16U;

typedef struct PACK 
{
	float min;
	float max;
}DepthRestriction32F;

END_PACK

MyDepthToLaserRectDyn::MyDepthToLaserRectDyn()
{
}

MyDepthToLaserRectDyn::~MyDepthToLaserRectDyn()
{
}

int MyDepthToLaserRectDyn::init(int img_w, int img_h, CvMat* Q, int laser_beam_n, float min_dst, float max_dst, float min_height, float max_height, int roi_h)
{
	w_ = img_w;
	h_ = img_h;
	Q_ = cvCloneMat(Q);

	laser_beam_n_ = laser_beam_n;

	min_dst_ = min_dst;
	max_dst_ = max_dst;
	float min_dst_mm = min_dst * 1000;
	float max_dst_mm  = max_dst * 1000;
	
	f_ = (float)(CV_MAT_ELEM(*Q_, double, 2, 3));			//focal length
	t_ = (float)( -1 / CV_MAT_ELEM(*Q_, double, 3, 2));		//stereo baseQ_f_ = 609.719;
	cx_ = (int)(-CV_MAT_ELEM(*Q_, double, 0, 3));
	cy_ = (int)(-CV_MAT_ELEM(*Q_, double, 1, 3));

	depth_multiplier_ = (float)((double)f_ * t_);
	y_multiplier_ = (float)(1/(double)f_);
	min_distance_max_depth_ = depth_multiplier_ / min_dst_mm;
	max_distance_min_depth_ = depth_multiplier_ / max_dst_mm;

	y_min_ = cy_ - roi_h / 2;
	y_max_ = cy_ + roi_h / 2;

	min_height_ = min_height;
	max_height_ = max_height;

	depth_rect_ = cvCreateMat(h_, w_, CV_32F);
	depth_to_laser_rot_mat_ = cvCreateMat(2, 3, CV_32F);
	cv2DRotationMatrix(cvPoint2D32f(300, 155), -5, 1, depth_to_laser_rot_mat_);

	mxy_ = cvCreateMat(h_, w_, CV_16SC2);
	depth_rectification_factor_map_ = cvCreateMat(h_, w_, CV_32F);

	init_depth_restriction_lut();
	init_distance_calculation_lut();
	init_laser_parameters_and_lut();

	line_ = new float[w_];
	line_src_ = new LineFillerSrcInfo[w_];
	laser_data_ = new float[laser_beam_n_];
	depth_sorce_point_info_ = new DepthSrcByColInfo[w_];
    fs_ = new FillerSrcInfo[w_*h_];

    method_ = METHOD_NEAREST_BY_COLS;
	optimization_ = OPTIMIZATION_NO;
	
     perf_ = MyPerformanceMeter("MyDepthToLaserRectDyn", false, false);

	return 0;
}

/*  Assume: fx = fy = f invariant to rotation (like ROS and OpenCV calibration do)
	Camera CS		Laser CS
	X = xZ/f		X' = x'Z'/fx
	Y = yZ/f		Y' = y'Z'/fy
	Z				Z'

	R'P' = P
	R' (X' Y' Z')^T = (X Y Z)^T 
	R' (x'Z'/f y'Z'/f Z')^T = (xZ/f yZ/f Z)^T
	Z'*R'*(x' y' f)^T = Z*(x y f)^T
	... let R'(x' y' f)^T = R'S' = (a' b' c')^T = A' ...
	x = a'(Z'/Z) = a'f/c'
	y = b'(Z'/Z) = b'f/c'
	Z'/Z = f/c'

	Denote x, y, z, X, Y, Z, d as is,
	x', y', z', X', Y', Z', d' as xr, yr, zr, Xr, Yr, Zr, dr
*/


int MyDepthToLaserRectDyn::pre_init_rect_map_statics()
{
	return 0;
}

int MyDepthToLaserRectDyn::init_rect_map(CvMat* R_laser_to_cam)
{
	if(optimization_ == OPTIMIZATION_SIMD128)
		return init_rect_map_simd128(R_laser_to_cam);

	Point16S* mxy_pt_ptr = ((Point16S*)mxy_->data.ptr);
	CvMat* R = R_laser_to_cam;

	float R02_f_ = R->data.fl[2]*f_;
	float R12_f_ = R->data.fl[5]*f_;
	float R22_f_ = R->data.fl[8]*f_;

	for(int i = y_min_; i < y_max_; i++)
	{
		for(int j = 0; j < w_; j++)
		{
			int xr = j - cx_;
			int yr = i - cy_;

			float a = R->data.fl[0]*xr + R->data.fl[1]*yr + R02_f_;//R->data.fl[2]*f_;
			float b = R->data.fl[3]*xr + R->data.fl[4]*yr + R12_f_;//R->data.fl[5]*f_;
			float c = R->data.fl[6]*xr + R->data.fl[7]*yr + R22_f_;//R->data.fl[8]*f_;

			float f_to_c = f_ / c;

			float x = a * f_to_c;
			float y = b * f_to_c;
			float dr_factor = f_to_c;

			int dst_x = x + cx_;
			int dst_y = y + cy_;
			
			dst_x = dst_x < 0? 0: dst_x >= w_? w_ - 1: dst_x;
			dst_y = dst_y < 0? 0: dst_y >= h_? h_ - 1: dst_y;

			mxy_pt_ptr[i*w_ + j].x = dst_x;
			mxy_pt_ptr[i*w_ + j].y = dst_y;
			depth_rectification_factor_map_->data.fl[i * w_ + j] = dr_factor;
		}
	}

	return 0;
}

int MyDepthToLaserRectDyn::simd_endian_test()
{
#if CV_SIMD128
	cv::v_int32x4 v_x = cv::v_int32x4(1, 2, 3, 4);
	cv::v_int16x8 v_x16 = cv::v_reinterpret_as_s16(v_x);
	int16_t x[8];
	cv::v_store(x, v_x16);
	return (x[0] == 1) && (x[1] == 0) ? 0: 1;
#endif
	return 0;
}

int MyDepthToLaserRectDyn::init_rect_map_simd128(CvMat* R_laser_to_cam)
{
#if CV_SIMD128
	Point16S* mxy_pt_ptr = ((Point16S*)mxy_->data.ptr);
	float* R = R_laser_to_cam->data.fl;

	float R02_f_ = R[2]*f_;
	float R12_f_ = R[5]*f_;
	float R22_f_ = R[8]*f_;

	cv::v_float32x4 v_R02_f_ = cv::v_setall_f32(R02_f_);
	cv::v_float32x4 v_R12_f_ = cv::v_setall_f32(R12_f_);
	cv::v_float32x4 v_R22_f_ = cv::v_setall_f32(R22_f_);

	cv::v_float32x4 v_R[3][3];
	for(int i = 0; i < 3; i++)
		for(int j = 0; j < 3; j++)
			v_R[i][j] = cv::v_setall_f32(R[i*3 + j]);
	
	cv::v_float32x4 v_f = cv::v_setall_f32(f_);
	cv::v_int32x4 v_cx = cv::v_setall_s32(cx_);
	cv::v_int32x4 v_cy = cv::v_setall_s32(cy_);

	for(int i = y_min_; i < y_max_; i++)
	{
		for(int j = 0; j < w_; j += 4)
		{
			cv::v_float32x4 v_xr = cv::v_setall_f32((float)(j - cx_));
			v_xr += cv::v_float32x4(0, 1, 2, 3);

			cv::v_float32x4 v_yr = cv::v_setall_f32((float)(i - cy_));

			cv::v_float32x4 v_a = v_R[0][0] * v_xr + v_R[0][1]*v_yr + v_R02_f_;
			cv::v_float32x4 v_b = v_R[1][0] * v_xr + v_R[1][1]*v_yr + v_R12_f_;
			cv::v_float32x4 v_c = v_R[2][0] * v_xr + v_R[2][1]*v_yr + v_R22_f_;

			//cv::v_float32x4 v_a = cv::v_setall_f32(R[0]) * v_xr + cv::v_setall_f32(R[1])*v_yr + v_R02_f_;
			//cv::v_float32x4 v_b = cv::v_setall_f32(R[3]) * v_xr + cv::v_setall_f32(R[4])*v_yr + v_R12_f_;
			//cv::v_float32x4 v_c = cv::v_setall_f32(R[6]) * v_xr + cv::v_setall_f32(R[7])*v_yr + v_R22_f_;

			cv::v_float32x4 f_to_c = v_f / v_c;

			cv::v_float32x4 v_x = v_a * f_to_c;
			cv::v_float32x4 v_y = v_b * f_to_c;
			cv::v_float32x4 v_dr_factor = f_to_c;

			cv::v_int32x4 v_dst_x = cv::v_round(v_x) + v_cx;
			cv::v_int32x4 v_dst_y = cv::v_round(v_y) + v_cy;

			v_dst_x = cv::v_max(cv::v_setzero_s32(), v_dst_x);
			v_dst_y = cv::v_max(cv::v_setzero_s32(), v_dst_y);
			v_dst_x = cv::v_min(cv::v_setall_s32(w_ - 1), v_dst_x);
			v_dst_y = cv::v_min(cv::v_setall_s32(h_ - 1), v_dst_y);

			//int16_t ttt[12];
			//cv::v_store_interleave(ttt, cv::v_reinterpret_as_s16(v_dst_x), cv::v_reinterpret_as_s16(v_dst_y), cv::v_setzero_s16());
			//cv::v_in

			cv::v_store_low((int16_t*)(mxy_pt_ptr + i*w_ + j), cv::v_reinterpret_as_s16(v_dst_x));
			cv::v_store_low((int16_t*)(mxy_pt_ptr + i*w_ + j), cv::v_reinterpret_as_s16(v_dst_x));

			cv::v_int16x8 v_x16 = cv::v_reinterpret_as_s16(v_dst_x);
			cv::v_int16x8 v_y16 = cv::v_reinterpret_as_s16(v_dst_y << 16);
			cv::v_int16x8 v_xy16 = v_x16 | v_y16;
			cv::v_store((int16_t*)(mxy_pt_ptr + i*w_ + j), v_xy16);
			
			//cv::v_int16x8 v_xy = cv::v_pack(v_dst_x, v_dst_y);


			//cv::v_store_interleave((int16_t*)(mxy_pt_ptr + i*w_ + j), cv::v_reinterpret_as_s16(v_dst_x), cv::v_reinterpret_as_s16(v_dst_y), cv::v_setzero_s16());
			cv::v_store(depth_rectification_factor_map_->data.fl + i * w_ + j, v_dr_factor);

			/*int xx[4];
			int yy[4];

			cv::v_store(xx, v_dst_x);
			cv::v_store(yy, v_dst_y);
			for(int k = 0; k < 4; k++)
			{
				mxy_pt_ptr[i * w_ + j + k].x = xx[k];
				mxy_pt_ptr[i * w_ + j + k].y = yy[k];
			}*/
		}
	}
#endif
	return 0;
}

int MyDepthToLaserRectDyn::rect()
{
	return 0;
}

int MyDepthToLaserRectDyn::init_depth_restriction_lut()
{
	float min_height_mm = min_height_ * 1000;
	float max_height_mm = max_height_ * 1000;
	float min_dst_mm = min_dst_ * 1000;
	float max_dst_mm = max_dst_ * 1000;
	depth_lut_ = cvCreateMat(h_, w_, CV_32FC4);
	DepthRestriction32F* depth_lut_ptr = (DepthRestriction32F*)depth_lut_->data.ptr  + y_min_*w_;
	for(int i = y_min_; i < h_; i++)
	{
		for(int j = 0; j < w_; j++, depth_lut_ptr++)
		{
			//Y <= max_height_mm;
			//Y = Z*(y - cy)/f;
			//Z*(y - cy)/f <= max_height
			//Z <= max_height_mm * f / (y - cy);
			//...
			//Z >= min_height_mm * f / (y - cy);

			float max_depth_by_height;

			if(i < cy_) //upside, chech max_height
				max_depth_by_height = -max_height_ * f_ / (i - cy_);
			else if(i > cy_)
				max_depth_by_height = -min_height_ * f_ / (i - cy_);
			else
				max_depth_by_height = 1e37;


			//dist = sqrt(Z^2 + X^2) <= max_dst_mm;
			//Z^2 + x^2*Z^2/f^2 <= max_dst_mm^2;
			//Z^2 <= max_dst_mm^2 / (1 + x^2 / f^2);
			//Z^2 <= max_dst_mm^2 * f^2 / (f^2 + x^2);
			//Z <= max_dst_mm * f / sqrt(f^2 + x^2);
			//Z >= min_dst_mm * f / sqrt(f^2 + x^2);

			if(abs(f_*f_ + (j - cx_)*(j - cx_)) > 1e-10) //check divider
			{
				depth_lut_ptr->min = min_dst_ * f_ / sqrt(f_*f_ + (j - cx_)*(j - cx_));
				depth_lut_ptr->max = max_dst_ * f_ / sqrt(f_*f_ + (j - cx_)*(j - cx_));
			}
			else
			{
				depth_lut_ptr->min = -1e37;
				depth_lut_ptr->max =  1e37;
			}

			if(max_depth_by_height < depth_lut_ptr->max)
				depth_lut_ptr->max = max_depth_by_height;

			//depth_lut_ptr->min = min_depth_by_Y > min_depth_by_distance? min_depth_by_Y: min_depth_by_distance;
			//depth_lut_ptr->max = max_depth_by_Y < max_depth_by_distance? max_depth_by_Y: max_depth_by_distance;
			//depth_lut_ptr->min = depth_lut_ptr->min > 0? depth_lut_ptr->min: 1;
		}
	}

	return 0;
}

int MyDepthToLaserRectDyn::init_distance_calculation_lut()
{
	int h = h_;
	int w = w_;

	distance_lut_ = new float[h*w];
	for(int i = 0; i < h; i++)
	{
		for(int j = 0; j < w; j++)
		{
			distance_lut_[i*w + j] = sqrt(f_*f_ + (j - cx_)*(j - cx_)) / f_; //m
		}
	}
	return 0;
}

int MyDepthToLaserRectDyn::init_laser_parameters_and_lut()
{
	laser_right_angle_ = - (float)atan((double)(w_ - cx_)/f_); //rad
    laser_left_angle_ = (float)atan((double)(cx_/f_)); //rad
    laser_angle_of_view_ = laser_left_angle_ - laser_right_angle_;
 
	//Represent in ROS laser_scan coordinates where 0 is along x axis
    laser_min_angle_ = laser_right_angle_; //rad
    laser_max_angle_ = laser_left_angle_;
    laser_scan_increment_ = laser_angle_of_view_/laser_beam_n_;
	laser_c_angle_ = atan((float)cx_ / f_);

	laser_beam_pixel_lut_ = new int[laser_beam_n_];
	for(int i = 0; i < laser_beam_n_; i++)
	{
		int xr = f_ * tan(i * laser_scan_increment_ - laser_c_angle_) + cx_;
		laser_beam_pixel_lut_[i] = xr < 0? 0: xr > w_ - 1? w_ - 1: xr;
	}

	return 0;
}

int MyDepthToLaserRectDyn::rect_restrict_by_cols()
{
	const CvMat* depth = depth_src_;
	CvMat* rect = depth_rect_;
	int w = w_;
	int h = h_;
	int y_min = y_min_;
	int y_max = y_max_;
	float f = f_;
	float t = t_;
	DepthSrcByColInfo* dsi = depth_sorce_point_info_;

	cvZero(rect);
	for(int j = 0; j < w; j++)
	{
		//dsi[j].y = y_min;
		dsi[j].depth = BAD_DEPTH;
	}

	float* rect_ptr = (float*)rect->data.ptr + y_min * w;
	float* depth_ptr = (float*)depth->data.ptr;
	Point16S* mxy_pt_ptr = ((Point16S*)mxy_->data.ptr) + y_min * w_;
	DepthRestriction32F* depth_lut_ptr = (DepthRestriction32F*)depth_lut_->data.ptr + y_min * w;
	for(int i = y_min; i < y_max; i++)
	{
		for(int j = 0; j < w; j++, mxy_pt_ptr++, depth_lut_ptr++, rect_ptr++)
		{
			float val = depth_ptr[mxy_pt_ptr->y * w + mxy_pt_ptr->x] * depth_rectification_factor_map_->data.fl[i*w_ + j];
			//printf("%3.1f ", val);
			if(depth_lut_ptr->min <= val && val <= depth_lut_ptr->max)
			{
                //*rect_ptr = val;
				if(val > dsi[j].depth)
				{
					dsi[j].depth = val;
					dsi[j].y = i;
				}
			}
		}
	}

	for(int j = 0; j < w; j++)
	{
		if(IS_DEPTH_BAD(dsi[j].depth))
			line_[j] = BAD_DISTANCE;
		else
		{
			line_[j] = distance_lut_[dsi[j].y*w_ + j] * dsi[j].depth;
		}
	}

	return 0;
}

int MyDepthToLaserRectDyn::fill_line()
{
	int w = w_;
	int h = h_;

	for(int j = 0; j < w; j++)
	{
		line_src_[j].x = j;
		if(IS_DISTANCE_GOOD(line_[j]))
		{
			line_src_[j].dst = 0;
		}
		else
		{
			line_src_[j].dst = 0x7FFFF;
		}
	}

	//1st run (left to right)
	for(int j = 0; j < w - 1; j++)
	{
		if(IS_DISTANCE_BAD(line_[j]))
			continue;

		int new_dst = (j + 1)- line_src_[j].x;
		new_dst = new_dst < 0? -new_dst: new_dst;
		if(new_dst < line_src_[j + 1].dst)
		{
			line_[j + 1] = line_[j];
			line_src_[j + 1].dst = new_dst;
			line_src_[j + 1].x = line_src_[j].x;
		}
	}

	//2nd run (rigth to left)
	for(int j = w - 2; j > 0; j--)
	{
		if(IS_DISTANCE_BAD(line_[j]))
			continue;
		int new_dst = (j - 1) - line_src_[j].x;
		new_dst = new_dst < 0? -new_dst: new_dst;
		if(new_dst < line_src_[j - 1].dst)
		{
			line_[j - 1] = line_[j];
			line_src_[j - 1].dst = new_dst;
			line_src_[j - 1].x = line_src_[j].x;
		}
	}
	return 0;
}

int MyDepthToLaserRectDyn::select_laser_beams()
{
	for(int i = 0; i < laser_beam_n_; i++)
	{
        laser_data_[i] = line_[laser_beam_pixel_lut_[i]];
	}

	return 0;
}

int MyDepthToLaserRectDyn::rect_restrict()
{
    const CvMat* depth = depth_src_;
    CvMat* rect = depth_rect_;
    int w = w_;
    int h = h_;
    int y_min = y_min_;
    int y_max = y_max_;
    float f = f_;
    float t = t_;
    DepthSrcByColInfo* dsi = depth_sorce_point_info_;

    cvZero(rect);
    for(int j = 0; j < w; j++)
    {
        //dsi[j].y = y_min;
        dsi[j].depth = BAD_DEPTH;
    }

    float* rect_ptr = (float*)rect->data.ptr + y_min * w;
    float* depth_ptr = (float*)depth->data.ptr;
    Point16S* mxy_pt_ptr = ((Point16S*)mxy_->data.ptr) + y_min * w_;
    DepthRestriction32F* depth_lut_ptr = (DepthRestriction32F*)depth_lut_->data.ptr + y_min * w;
    for(int i = y_min; i < y_max; i++)
    {
        for(int j = 0; j < w; j++, mxy_pt_ptr++, depth_lut_ptr++, rect_ptr++)
        {
            float val = depth_ptr[mxy_pt_ptr->y * w + mxy_pt_ptr->x] * depth_rectification_factor_map_->data.fl[i*w + j];
			//printf("%3.1f ", val);
            if(depth_lut_ptr->min <= val && val <= depth_lut_ptr->max)
            {
                *rect_ptr = val;
            }
            else
                *rect_ptr = BAD_DEPTH;
        }
    }

    return 0;
}

int MyDepthToLaserRectDyn::fill2d()
{
    MyPerformanceMeter perf("fill_2b", false, false);
    FillerSrcInfo* fs = fs_;

    CvMat* src = depth_rect_;
    CvMat* dst = depth_rect_;
    int w  = w_;
    int h = h_;
    //float min_val = min_val_;
    //float max_val = max_val_;

    perf.go("prepare");
    for(int i = y_min_; i < y_max_; i++)
    {
        float* src_row_ptr = (float*)(src->data.ptr + i * src->step);
        float* c_row_ptr = (float*)(dst->data.ptr + i * dst->step);
        for(int j = 0; j < w; j++)
        {
            float src_val = src_row_ptr[j];
            if(IS_DEPTH_BAD(src_val))
            {
                fs[i * w + j].dst = 0x7FFFFFFF;
            }
            else
            {
                fs[i * w + j].x = j;
                fs[i * w + j].y = i;
                fs[i * w + j].dst = 0;
            }
        }
    }
    perf.stop();

    perf.go("1st run");
    for(int i = y_min_ + 1; i < y_max_ - 1; i++)
    {
        float* c_row_ptr = (float*)(dst->data.ptr + i * dst->step);

        for(int j = 0; j < w - 1; j++)
        {
            float cp_val = c_row_ptr[j];
            if(IS_DEPTH_BAD(cp_val))
                continue;

            FillerSrcInfo cfi = fs[i * w + j];

            int tpy0 = i;
            int tpx0 = j + 1;
            int new_dst0 = (cfi.x - tpx0)*(cfi.x - tpx0) + (cfi.y - tpy0)*(cfi.y - tpy0);
            if(new_dst0 < fs[tpy0*w + tpx0].dst)
            {
                c_row_ptr[tpx0] = cp_val;
                FillerSrcInfo* fi = fs + tpy0*w + tpx0;
                fi->x = cfi.x;
                fi->y = cfi.y;
                fi->dst = new_dst0;
            }

            int tpy1 = i + 1;
            int tpx1 = j;
            //int new_dst1 = (cfi.x - tpx1)*(cfi.x - tpx1) + (cfi.y - tpy1)*(cfi.y - tpy1);
            //(cx - (tx - 1))*(cx - (tx - 1)) = (cx - tx + 1)*(cx - tx + 1) =
            //	= cx^2 - cx*tx + cx - cx*tx + tx^2 - tx + cx - tx + 1 = (cx^2 - 2*cx*tx + tx^2) + 2*cx - 2*tx + 1 = (cx - tx)*(cx - tx) + 2*(cx - tx) + 1;
            //(cy - (ty + 1))*(cy - (ty + 1)) = (cy - ty - 1)*(cy - ty - 1) =
            //	= cy^2 - cy*ty - cy - cy*ty + ty^2 + ty - cy + ty + 1 = (cy^2 - 2*cy*ty + ty^2) - 2*cy + 2*ty + 1 = (cy - ty)*(cy - ty) + 2*(ty - cy) + 1;
            //(cx - (tx - 1))*(cx - (tx - 1)) + (cy - (ty + 1))*(cy - (ty + 1)) = (cx - tx)*(cx - tx) + (cy - ty)*(cy - ty) + 2*(cx - tx + ty - cy + 1)
            int new_dst1 = new_dst0 + ((cfi.x - cfi.y + tpy1 - tpx0) << 1);
            if(new_dst1 < fs[tpy1*w + tpx1].dst)
            {
                float* t_row_ptr = c_row_ptr + w;
                t_row_ptr[tpx1] = cp_val;
                FillerSrcInfo* fi = fs + tpy1*w + tpx1;
                fi->x = cfi.x;
                fi->y = cfi.y;
                fi->dst = new_dst1;
            }
        }
    }
    perf.stop();

    perf.go("2nd run");
    for(int i = y_max_ - 2; i >= y_min_ + 2; i--)
    {
        float* c_row_ptr = (float*)(dst->data.ptr + i * dst->step);

        for(int j = w - 2; j >= 2; j--)
        {
            float cp_val = c_row_ptr[j];
            if(IS_DEPTH_BAD(cp_val))
                continue;

            FillerSrcInfo cfi = fs[i * w + j];

            int tpy0 = i - 1;
            int tpx0 = j;

            int dx0 = cfi.x - tpx0;
            int dy0 = cfi.y - tpy0;
            int new_dst0 = (cfi.x - tpx0)*(cfi.x - tpx0) + (cfi.y - tpy0)*(cfi.y - tpy0);
            if(new_dst0 < fs[tpy0*w + tpx0].dst)
            {
                float* t_row_ptr = c_row_ptr - w;
                t_row_ptr[tpx0] = cp_val;
                FillerSrcInfo* fi = fs + tpy0*w + tpx0;
                fi->x = cfi.x;
                fi->y = cfi.y;
                fi->dst = new_dst0;
            }

            int tpy1 = i;
            int tpx1 = j - 1;
            //int new_dst1 = (cfi.x - tpx1)*(cfi.x - tpx1) + (cfi.y - tpy1)*(cfi.y - tpy1);
            //(cx - (tx - 1))*(cx - (tx - 1)) = (cx - tx + 1)*(cx - tx + 1) =
            //	= cx^2 - cx*tx + cx - cx*tx + tx^2 - tx + cx - tx + 1 = (cx^2 - 2*cx*tx + tx^2) + 2*cx - 2*tx + 1 = (cx - tx)*(cx - tx) + 2*(cx - tx) + 1;
            //(cy - (ty + 1))*(cy - (ty + 1)) = (cy - ty - 1)*(cy - ty - 1) =
            //	= cy^2 - cy*ty - cy - cy*ty + ty^2 + ty - cy + ty + 1 = (cy^2 - 2*cy*ty + ty^2) - 2*cy + 2*ty + 1 = (cy - ty)*(cy - ty) + 2*(ty - cy) + 1;
            //(cx - (tx - 1))*(cx - (tx - 1)) + (cy - (ty + 1))*(cy - (ty + 1)) = (cx - tx)*(cx - tx) + (cy - ty)*(cy - ty) + 2*(cx - tx + ty - cy + 1)
            int new_dst1 = new_dst0 + ((cfi.x - cfi.y + tpy1 - tpx0) << 1);
            if(new_dst1 < fs[tpy1*w + tpx1].dst)
            {
                c_row_ptr[tpx1] = cp_val;
                FillerSrcInfo* fi = fs + tpy1*w + tpx1;
                fi->x = cfi.x;
                fi->y = cfi.y;
                fi->dst = new_dst1;
            }
        }
    }
    perf.stop();

    perf.print_all();
    return 0;
}

int MyDepthToLaserRectDyn::get_line_from_filled_depth_map()
{
    int w = w_;
    for(int j = 0; j < w; j++)
    {
        float depth = depth_rect_->data.fl[cy_*w_ + j];
        if(IS_DEPTH_BAD(depth))
            line_[j] = BAD_DISTANCE;
        line_[j] = distance_lut_[cy_*w_ + j] * depth;
    }
    return 0;
}

int MyDepthToLaserRectDyn::draw(CvMat* img_show)
{
	float show_scale_ = 1;
	CvSize show_size_ = cvSize(w_, h_);
	float show_dst_ = 5;
	show_scale_ = show_size_.height/show_dst_;

	//Draw data
	cvZero(img_show);
	CvScalar pt_color = cvScalar(255, 0, 0);
	for(int i = 0; i < laser_beam_n_; i++)
	{
		float* ptr_line_3d = laser_data_;//(float*)(line_3d_->data.ptr) + i*3;
		float dst = laser_data_[i];
		float angle = i * laser_scan_increment_ - laser_c_angle_;
		float x = dst * sin(angle);
		float z = dst * cos(angle);

		//if(0 < z_3d && z_3d < max_dst_mm_)
		{
			//printf("x = %5.1f z = %5.1f \n", x_3d, z_3d);

			CvPoint p;
			p.x = cvRound(x * show_scale_) + show_size_.width/2;  //camera X (left/right) axis
			p.y = show_size_.height - cvRound(z * show_scale_);  //camera Z (depth)

			if(	0 < p.x && p.x < show_size_.width   &&  	0 < p.y && p.y < show_size_.height)
				cvCircle(img_show, p, 2, pt_color);
		}
	}
	cvLine(img_show, cvPoint(img_show->width/2, img_show->height), cvPoint(0, 0), cvScalar(255, 255, 0));
	cvLine(img_show, cvPoint(img_show->width/2, img_show->height), cvPoint(img_show->width, 0), cvScalar(255, 255, 0));
	return 0;
}

int MyDepthToLaserRectDyn::depth_u16_to_f32(CvMat* src, CvMat* dst)
{
//	if(optimization_ == OPTIMIZATION_SIMD128)
//		return depth_u16_to_f32_simd128(src, dst);

	uint16_t* s = (uint16_t*)src->data.ptr;
	float* d = (float*)dst->data.ptr;
	int n = src->cols * src->rows;
	for(int i = 0; i < n; i++, s++, d++)
	{
		*d = ((float)(*s)) / 1000;
	}

	return 0;
}

//int MyDepthToLaserRectDyn::depth_u16_to_f32_simd128(CvMat* src, CvMat* dst)
//{
//
//}

void MyDepthToLaserRectDyn::show()
{
	CvMat* img_show = cvCreateMat(h_, w_, CV_8U);
	draw(img_show);
	//("laser scan", img_show);
	cvReleaseMat(&img_show);
}

void MyDepthToLaserRectDyn::operator()(CvMat* img_depth, CvMat* R_laser_to_cam)
{
	update(img_depth, R_laser_to_cam);
}

int MyDepthToLaserRectDyn::update(CvMat* src, CvMat* R_laser_to_cam)
{
	depth_src_ = src;

    perf_.go("init map");
	init_rect_map(R_laser_to_cam);
    perf_.stop();

    if(method_ == METHOD_NEAREST_BY_COLS)
    {
        perf_.go("rect restrict");
        rect_restrict_by_cols();
        perf_.stop();

        perf_.go("fill");
        fill_line();
        perf_.stop();
    }
    else
    {
        perf_.go("rect restrict");
        rect_restrict();
        perf_.stop();

        perf_.go("fill2d");
        fill2d();
        perf_.stop();

        perf_.go("get line");
        get_line_from_filled_depth_map();
        perf_.stop();
    }

    perf_.go("select beams");
	select_laser_beams();
    perf_.stop();

	//cvShowImage("rect", depth_rect_);

    perf_.print_all();

	return 0;
}

int MyDepthToLaserRectDyn::set_optimization(int optimization)
{
	if(optimization == OPTIMIZATION_SIMD128)
	{
#if CV_SIMD128
		if(simd_endian_test() == 0) //success
		{
			optimization_ = optimization;
			return 0;
		}
#endif
		return -1;
	}

	optimization_ = optimization;

	return 0;
}

const float* MyDepthToLaserRectDyn::get_laser_data()		{return laser_data_;}
int MyDepthToLaserRectDyn::get_laser_beam_n()				{return laser_beam_n_;}
float MyDepthToLaserRectDyn::get_laser_min_angle()			{return laser_min_angle_;}
float MyDepthToLaserRectDyn::get_laser_max_angle()			{return laser_max_angle_;}
float MyDepthToLaserRectDyn::get_laser_scan_increment()		{return laser_scan_increment_;}

int MyDepthToLaserRectDyn::get_optimization()				{return optimization_;}
