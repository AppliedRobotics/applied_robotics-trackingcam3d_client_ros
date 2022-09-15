#include "tlive_stream.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include "MyPerformanceMeter.h"
#include <lz4.h>
#include <lz4hc.h>
#include <zstd.h>
#include "MyFastRemapper.h"
#include <tconfig/tconfig.h>

namespace tlive {
using namespace tconfig;

class TimeStamp
{
public:
	TimeStamp(unsigned long long int timestamp_ms) {unix_s = timestamp_ms/1000; unix_ms = timestamp_ms - unix_s;}
	int unix_s;
	int unix_ms;
};

template <typename T> struct TbbStlContainerInvoker {
  void operator()(T& it) const {it();}
};
//#include <opencv2/highgui/highgui.hpp>

int StreamJob::encode()
{
	size = 0;
	width = img->cols;
	height = img->rows;
	MyPerformanceMeter perf("codec " + img_name, false, false);
	perf.go("enc");
	if(CV_MAT_TYPE(img->type) == CV_8UC2) //src YUV422 detected
	{
		if(encoding == "yuv422")
		{
			size = img->step * img->rows;
			pt = (char*)img->data.ptr;
		}
		else if(encoding == "yuv422_lz4" || encoding == "yuv422_zstd")
		{
			perf.go("cvt");
			int img_size = img->step * img->rows;
			int max_compressed_size = img_size * 1.5;
			
			pt = new char[max_compressed_size];
			is_allocated = true;

			cv::Mat s1ch(img->rows, img->cols*2, CV_8UC1, img->data.ptr);
			cv::Mat tr;
			cv::transpose(s1ch, tr);
			perf.stop();

			perf.go("enc core");
			if(encoding == "yuv422_lz4")
				size = LZ4_compress_default((char*)tr.data, pt, img_size, max_compressed_size);
			else
				size = ZSTD_compress(pt, max_compressed_size, tr.data, img_size, 1);
			perf.stop();
		}
		else if(encoding == "jpg" || encoding == "jp2" || encoding == "png" || encoding == "bmp" )
		{
			cv::Mat src_yuv = cv::cvarrToMat(img);
			cv::Mat bgr8(img->rows, img->cols, CV_8UC3);
			//perf.go("cvt");
			cv::cvtColor(src_yuv, bgr8, cv::COLOR_YUV2BGR_YUYV);
			//perf.stop();
			std::string cv_enc_name = encoding == "jpg"? ".jpg": encoding == "jp2"? ".jp2": encoding == "png"? ".png": ".bmp";
			//std::vector<int> flags;
			//flags.push_back(CV_IMWRITE_JPEG_QUALITY);
			//flags.push_back(10);
			vec.reserve(100*1024);
			cv::imencode(cv_enc_name, bgr8, vec);//, flags);
			pt = (char*)vec.data();
			size = vec.size();
		}
	}
	else if(CV_MAT_TYPE(img->type) == CV_8UC1) //src gray image detected
	{
		if(encoding == "gray8")
		{
			size = img->step * img->rows;
			pt = (char*)img->data.ptr;
		}
		else if(encoding == "gray8_lz4" || encoding == "gray8_zstd")
		{
			int img_size = img->step * img->rows;
			int max_compressed_size = img_size * 2;
			pt = new char[max_compressed_size];
			is_allocated = true;
			//cvSmooth(img, img, CV_GAUSSIAN, 5, 5);
			
			if(encoding == "gray8_lz4")
				size = LZ4_compress_default((char*)img->data.ptr, pt, img_size, max_compressed_size);
				//size = LZ4_compress_HC((char*)img->data.ptr, (char*)(&vec[0]), img_size, max_compressed_size, 4);
			else
				size = ZSTD_compress(pt, max_compressed_size, (char*)img->data.ptr, img_size, 9);
		}
		else if(encoding == "jpg" || encoding == "jp2" || encoding == "png" || encoding == "bmp" )
		{
			cv::Mat src = cv::cvarrToMat(img);
			std::string cv_enc_name = encoding == "jpg"? ".jpg": encoding == "jp2"? ".jp2": encoding == "png"? ".png": ".bmp";
			vec.reserve(img->step * img->rows);
			cv::imencode(cv_enc_name, src, vec);
			pt = (char*)vec.data();
			size = vec.size();
		}
	}
	else if(CV_MAT_TYPE(img->type) == CV_8UC3) //src YUV422 detected
	{
		if(encoding == "yuv422")
		{
			cv::Mat src_yuv = cv::cvarrToMat(img);
			size = img->cols * img->rows * 2;
			pt = new char[size];
			is_allocated = true;
			cv::Mat dst(img->rows, img->cols, CV_8UC2, pt);
			MyFastYuv2Yuv422Compresser cmpr;
			CvMat* cvDst = img;
			cmpr(img, cvDst);
		}
		else if(encoding == "yuv422_lz4" || encoding == "yuv422_zstd")
		{
			perf.go("cvt");
			cv::Mat yuv = cv::cvarrToMat(img);
			cv::Mat yuv422(img->height, img->width, CV_8UC2);
			CvMat* cvDst = img;
			MyFastYuv2Yuv422Compresser cmpr;
			cmpr(img, cvDst);

			int img_size = yuv422.step * yuv422.rows;
			int max_compressed_size = img_size * 1.5;
			
			pt = new char[max_compressed_size];
			is_allocated = true;

			cv::Mat s1ch(yuv422.rows, yuv422.cols*2, CV_8UC1, yuv422.data);
			cv::Mat tr;
			cv::transpose(s1ch, tr);
			perf.stop();

			perf.go("enc core");
			if(encoding == "yuv422_lz4")
				size = LZ4_compress_default((char*)tr.data, pt, img_size, max_compressed_size);
			else
				size = ZSTD_compress(pt, max_compressed_size, tr.data, img_size, 1);
			perf.stop();
		}
		else if(encoding == "jpg" || encoding == "jp2" || encoding == "png" || encoding == "bmp" )
		{
			cv::Mat src_yuv = cv::cvarrToMat(img);
			cv::Mat yuv422(img->height, img->width, CV_8UC2);
			cv::Mat bgr8;
			CvMat* cvDst = img;
			MyFastYuv2Yuv422Compresser cmpr;
			cmpr(img, cvDst);

			cv::cvtColor(yuv422, bgr8, cv::COLOR_YUV2BGR_YUYV);
			std::string cv_enc_name = encoding == "jpg"? ".jpg": encoding == "jp2"? ".jp2": encoding == "png"? ".png": ".bmp";
			//std::vector<int> flags;
			//flags.push_back(CV_IMWRITE_JPEG_QUALITY);
			//flags.push_back(50);
			vec.reserve(bgr8.cols * bgr8.rows * 3);
			cv::imencode(cv_enc_name, bgr8, vec);//, flags);
			pt = (char*)vec.data();
			size = vec.size();
		}
	}
	else if(CV_MAT_TYPE(img->type) == CV_16UC1 || CV_MAT_TYPE(img->type) == CV_16SC1) //src gray image detected
	{
		if(encoding == "gray16" || encoding == "gray16s")
		{
			size = img->step * img->rows;
			pt = (char*)img->data.ptr;
		}
		else if(encoding == "gray16_lz4" || encoding == "gray16_zstd" || encoding == "gray16s_lz4" || encoding == "gray16s_zstd")
		{
			int img_size = img->step * img->rows;
			int max_compressed_size = img_size * 2;
			pt = new char[max_compressed_size];
			is_allocated = true;
			//cvSmooth(img, img, CV_GAUSSIAN, 5, 5);
			
			if(encoding == "gray16_lz4" || encoding == "gray16s_lz4")
				size = LZ4_compress_default((char*)img->data.ptr, pt, img_size, max_compressed_size);
				//size = LZ4_compress_HC((char*)img->data.ptr, (char*)(&vec[0]), img_size, max_compressed_size, 4);
			else
				size = ZSTD_compress(pt, max_compressed_size, (char*)img->data.ptr, img_size, 9);
		}
		else if(encoding == "jp2" || encoding == "png")
		{
			cv::Mat src = cv::cvarrToMat(img);
			std::string cv_enc_name = encoding == "jp2"? ".jp2": ".png";
			vec.reserve(src.cols * src.rows * 2);
			cv::imencode(cv_enc_name, src, vec);
			pt = (char*)vec.data();
			size = vec.size();
		}
		else if(encoding == "jpg" || encoding == "bmp")
		{
			cv::Mat src = cv::cvarrToMat(img);
			cv::Mat gray32(img->rows, img->cols, CV_32SC1);
			src.convertTo(gray32, CV_32SC1);
			std::string cv_enc_name = encoding == "jpg"? ".jpg": ".bmp";
			//std::vector<int> flags;
			//flags.push_back(CV_IMWRITE_JPEG_QUALITY);
			//flags.push_back(50);
			vec.reserve(src.cols * src.rows * 2);
			cv::imencode(cv_enc_name, gray32, vec);
			pt = (char*)vec.data();
			size = vec.size();
		}
	}
	//perf.stop("enc");
	//perf.print_all();
	//printf("Size %d Kb\n", size/1024);
	return size;
}

static void my_free (void *data, void *hint)
{
    delete[] data;
}

void Stream::stream()
{
	TimeStamp timestamp(timestamp_ms_);
	TimeStamp settings_timestamp(settings_timestamp_ms_);
	std::string header_str;
	put_param(header_str, "time", timestamp.unix_s);
	put_param(header_str, "time_ms", timestamp.unix_ms);
	put_param(header_str, "settings_time", settings_timestamp.unix_s);
	put_param(header_str, "settings_time_ms", settings_timestamp.unix_ms);

	const int send_buf_size = 512 + 640*480*3*5;
	const int send_header_size = 512;
	char* send_buf = new char[send_buf_size];
	int addr = send_header_size;
	
//	perf_.go("encoding");
	tbb::parallel_for_each(stream_jobs.begin(), stream_jobs.end(), TbbStlContainerInvoker<StreamJob>());

	std::vector<CopyJob> copy_jobs;
	for(std::vector<StreamJob>::iterator it = stream_jobs.begin(); it != stream_jobs.end(); it++)
	{
		if(addr + it->size > send_buf_size)
			break;
		copy_jobs.push_back(CopyJob(it->pt, send_buf + addr, it->size));
		put_param_as_hex(header_str, it->img_name, addr);
		put_param(header_str, it->img_name + "_encoding", it->encoding);
		put_param(header_str, it->img_name + "_width", it->width);
		put_param(header_str, it->img_name + "_height", it->height);
		//if(it->encoding == "lz4" || it->encoding == "zstd")
		//	put_param(header_str, it->img_name + "_size", it->size);
		addr += it->size;
	}
	
	tbb::parallel_for_each(copy_jobs.begin(), copy_jobs.end(), TbbStlContainerInvoker<CopyJob>());
	memcpy(send_buf, header_str.data(), header_str.length() + 1);
//	perf_.stop();
	last_size = addr;
	send_data(send_buf, addr, my_free, NULL);
}

class DummyRootTask : public tbb::task {tbb::task* execute() {return NULL;}};

void Stream::stream_async()
{
	/* Copy all input data */
	std::vector<CvMatCloneJob> clone_jobs;
	for(std::vector<StreamJob>::iterator it = stream_jobs.begin(); it != stream_jobs.end(); it++)
		clone_jobs.push_back(CvMatCloneJob(it->img, it->img));
	tbb::parallel_for_each(clone_jobs.begin(), clone_jobs.end(), TbbStlContainerInvoker<CvMatCloneJob>());
	for(int i = 0; i < stream_jobs.size(); i++)
		stream_jobs[i].img = clone_jobs[i].dst;

	/* Run async task */
	root_task = new(tbb::task::allocate_root()) DummyRootTask;
	root_task->set_ref_count(2);
	StreamTask* stream_task = new (root_task->allocate_child()) StreamTask;
	stream_task->stream = this;
	is_task = true;
	tbb::task::spawn(*stream_task);
}

void Stream::join()
{
	if(is_task)
	{
		root_task->wait_for_all();
		is_task = false;

		//We need to realease input bufs as we grabbed data copy for async case */
		for(std::vector<StreamJob>::iterator it = stream_jobs.begin(); it != stream_jobs.end(); it++)
		{
			if(it->img != NULL)
				cvReleaseMat(&it->img);
		}
	}
}

void Stream::add_job(CvMat* _img, std::string _img_name, std::string _encoding)
{
	stream_jobs.push_back(StreamJob(_img, _img_name, _encoding));
}

void Stream::set_timestamp_ms(unsigned long long int timestamp_ms) {timestamp_ms_ = timestamp_ms;}
void Stream::set_settings_timestamp_ms(unsigned long long int timestamp_ms) {settings_timestamp_ms_ = timestamp_ms;}

};
