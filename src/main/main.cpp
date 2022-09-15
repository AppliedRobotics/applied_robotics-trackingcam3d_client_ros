// RtspClient.cpp : Defines the entry point for the console application.
//

#include <stdio.h>
#include "TrackingCam3d_Client.h"
#include "MyFpsMeter.h"

#include <ros/ros.h>
#include <image_geometry/stereo_camera_model.h>
#include <image_transport/image_transport.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/distortion_models.h>
#include "RosSCalibConfigurator.h"
#include "MyDepthIllustrator.h"
//#include <cv_bridge/cv_bridge.h>

#define CLIENT_CMD 0xff

MyFpsMeter fps_("fps", 0.1);

static std::vector<int> str_to_ints(std::string str, int n);
static std::string parse_destination(char* buf);
static int parse_cmd(char* buf, char** payload);
static sensor_msgs::CameraInfo gen_unrectified_info(int width, int height, float fx, float fy, std::string frame_id);
int publish_image(image_transport::Publisher* pub, CvMat* img, const char* format, std::string frame_id, ros::Time stamp);
int publish_cam_info(ros::Publisher* pub, sensor_msgs::CameraInfo* info, ros::Time stamp);

std::vector<TrackingCam3d_Client> cams;
std::vector<std::string> cams_names;

bool is_stop_ctrl_shell = false;
bool ctrl_shell(void *user_data)
{
	char buf[2000];
	printf(">>");
	while(!is_stop_ctrl_shell)
	{
		fgets(buf, 2000, stdin);
		char* cmd;
		int target = parse_cmd(buf, &cmd);
		if(target == CLIENT_CMD)
		{
			printf(">>");
			continue;
		}
		cmd[strlen(cmd) - 1] = '\0';
		cams[target].ctrl_send(cmd, strlen(cmd) + 1);
		cams[target].ctrl_recv(buf, 2000);
		std::string cam_name = cams_names[target];
		std::string cam_addr = cams[target].get_addr();
		printf("%s (%s): %s\n>>", cam_name.c_str(), cam_addr.c_str(), buf);
	}

	delete[] buf;

	return true;
}

class TrackingCam3d_ClientRos
{
public:
	//ros::NodeHandle nh;
	TrackingCam3d_Client *cam;
	std::string name;
	std::string frame_id;
	std::map<std::string, image_transport::Publisher> pub_img;
	std::map<std::string, std::string> ros_format;

	ros::Publisher pub_info_left;
	ros::Publisher pub_info_right;
//	ros::Publisher pub_left_camera_info;

	RosSCalibConfigurator info;
	//sensor_msgs::CameraInfo info_left;
	//sensor_msgs::CameraInfo info_right;

	boost::shared_ptr<camera_info_manager::CameraInfoManager> info_manager_left;
	boost::shared_ptr<camera_info_manager::CameraInfoManager> info_manager_right;

	void init(TrackingCam3d_Client *cam, std::string name, image_transport::ImageTransport &it, std::vector<int> img_size, std::string frame_id, std::string info_url, int push_calib);
	void publish(std::string img_name, CvMat *img, ros::Time stamp);
	void publish_info(ros::Time stamp);
	std::string alive(int request_n);
};

void TrackingCam3d_ClientRos::init(TrackingCam3d_Client *cam, std::string name, image_transport::ImageTransport &it, std::vector<int> img_size, std::string frame_id, std::string info_url, int push_calib)
{
	ros::NodeHandle nh("~");
	this->frame_id = frame_id;
	this->name = name;
	this->cam = cam;

	pub_img["left"] = it.advertise(name + "/left/image_raw", 1);
	pub_img["right"] = it.advertise(name + "/right/image_raw", 1);
	pub_img["left_rect"] = it.advertise(name + "/left/image_rect", 1);
	pub_img["right_rect"] = it.advertise(name + "/right/image_rect", 1);
	pub_img["left_gray_rect"]= it.advertise(name + "/left/image_rect_mono", 1);
	pub_img["right_gray_rect"] = it.advertise(name + "/right/image_rect_mono", 1);
	pub_img["depth"] = it.advertise(name + "/depth/image_rect", 5);

	pub_info_left = nh.advertise<sensor_msgs::CameraInfo>(name + "/left/camera_info", 1);
	pub_info_right = nh.advertise<sensor_msgs::CameraInfo>(name + "/right/camera_info", 1);
	//pub_info_depth pub_depth_camera_info = n.advertise<sensor_msgs::CameraInfo>("/" + cam_names[i] + "depth/camera_info", 1);

	std::stringstream calib_name_left_ss;
	ros::NodeHandle nh_left(nh, name + "/left");
	calib_name_left_ss << name + "_left_" << img_size[0];
	std::string info_file_url_left = info_url.length()? info_url + calib_name_left_ss.str() + ".yaml": "";
	//info_manager_left.reset(new camera_info_manager::CameraInfoManager(nh_left, calib_name_left_ss.str(), info_file_url_left));
	info_manager_left.reset(new camera_info_manager::CameraInfoManager(nh_left, calib_name_left_ss.str(), info_file_url_left));

	std::stringstream calib_name_right_ss;
	ros::NodeHandle nh_right(nh, name + "/right");
	calib_name_right_ss << name + "_right_" << img_size[0];
	std::string info_file_url_right = info_url.length()? info_url + calib_name_right_ss.str() + ".yaml": "";
	info_manager_right.reset(new camera_info_manager::CameraInfoManager(nh_right, calib_name_right_ss.str(), info_file_url_right));

	if(info_manager_left->isCalibrated() && info_manager_right->isCalibrated())
	{
		ROS_INFO("Using saved calibration...");
		info.info[0] = info_manager_left->getCameraInfo();
		info.info[1] = info_manager_right->getCameraInfo();
		info.info[0].header.frame_id = info.info[1].header.frame_id = frame_id;

		info.info_to_params();
		for(std::map<std::string, tconfig::Param>::iterator it = info.params.begin(); it != info.params.end(); it++)
		{
			std::string cmd = it->first + "=" + it->second.str();
			cam->ctrl_send(cmd.c_str(), cmd.size() + 1);
			char buf[2000];
			cam->ctrl_recv(buf, 2000);
		}

		if(push_calib)
		{
			char* cmd = "calib.save=1";
			cam->ctrl_send(cmd, strlen(cmd) + 1);
			char buf[2000];
			cam->ctrl_recv(buf, 2000);
		}
	}
	else
	{
		//info_left = info_right = gen_unrectified_info(img_size[0], img_size[1], 100, 100, frame_id);
		info.info[0].header.frame_id = info.info[1].header.frame_id = frame_id;
		std::string cmd;
		for(std::map<std::string, tconfig::Param>::iterator it = info.params.begin(); it != info.params.end(); it++)
		{
			std::string cmd = it->first + "?";
			cam->ctrl_send(cmd.c_str(), cmd.size() + 1);
			char req_buf[2000];
			cam->ctrl_recv(req_buf, 2000);
			std::map<std::string, tconfig::Param> req;
			tconfig::parse_params(req, req_buf);
			if(req.find(it->first) == req.end())
			{
				printf("Failed to get camera %s calibration params %s!\n", name.c_str(), it->first.c_str());
				return;
			}
			it->second = req[it->first];
		}
		info.params_to_info();
	}
}

void TrackingCam3d_ClientRos::publish(std::string img_name, CvMat *img, ros::Time stamp)
{
	publish_image(&((*pub_img.find(img_name)).second), img, ros_format[img_name].c_str(), frame_id, stamp);
}

void TrackingCam3d_ClientRos::publish_info(ros::Time stamp)
{
	publish_cam_info(&pub_info_left, &info.info[0], stamp);
	publish_cam_info(&pub_info_right, &info.info[1], stamp);
}



std::string TrackingCam3d_ClientRos::alive(int request_n)
{
	std::string cmd;
	for(std::map<std::string, image_transport::Publisher>::iterator it = pub_img.begin(); it != pub_img.end(); it++)
	{
		if(it->second.getNumSubscribers())
			tconfig::put_param(cmd, it->first, request_n);
	}
	return cmd;
}

static sensor_msgs::CameraInfo gen_unrectified_info(int width, int height, float fx, float fy, std::string frame_id)
{
	sensor_msgs::CameraInfo info;
	/* We are reporting information about the *depth* sensor here. */

	info.width = width;
	info.height = height;

	// No distortion
	info.D.resize(5, 0.0);
	info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

	// Simple camera matrix: square pixels (fx = fy), principal point at center
	info.K.assign(0.0);
	info.K[0] = fx;
	info.K[4] = fy;
	info.K[2] = (width /2.0) - 0.5;
	info.K[5] = (height/2.0) - 0.5;
	info.K[8] = 1.0;

	// No separate rectified image plane, so R = I
	info.R.assign(0.0);
	info.R[0] = info.R[4] = info.R[8] = 1.0;

	// Then P=K(I|0) = (K|0)
	info.P.assign(0.0);
	info.P[0] = fx;
	info.P[5] = fy;
	info.P[2] = info.K[2]; // cx
	info.P[6] = info.K[5]; // cy
	info.P[10] = 1.0;

	info.header.frame_id = frame_id;

	return info;
}

/* returns target */
static int parse_cmd(char* buf, char** payload)
{
	if(strstr(buf, "local_fps") == buf)
	{
		fps_.print_fps(false);
		return CLIENT_CMD;
	}

	for(int i = 0; i < cams_names.size(); i++)
	{
		if(!strcmp(cams_names[i].c_str(), buf))
		{
			*payload = buf + cams_names[i].size();
			return i;
		}
	}

	*payload = buf;

	return 0;
}

void connectCb()
{
//	if (pub_depth_image_.getNumSubscribers() == 0)
//	{
//		sub_disp_.shutdown();
//	}
//	else
//	{
//		ros::NodeHandle nh;
//		sub_disp_ = nh.subscribe("disparity", 1, disparity_image_callback);
//		sensor_msgs::CameraInfo cam_info_left = *ros::topic::waitForMessage<sensor_msgs::CameraInfo>("left/camera_info", nh);
//		sensor_msgs::CameraInfo cam_info_right = *ros::topic::waitForMessage<sensor_msgs::CameraInfo>("right/camera_info", nh);

//		float fx = cam_info_left.P[0*4 + 0];
//		float cx = cam_info_left.P[0*4 + 2];
//		float fy = cam_info_left.P[1*4 + 1];
//		float cy = cam_info_left.P[1*4 + 2];

//		float fx_right = cam_info_right.P[0*4 +0];
//		float cx_right = cam_info_right.P[0*4 + 2];
//		float T = -cam_info_right.P[0*4 + 3] / fx_right * 1000;

//		ft_ = fx * T;
//	}
}

int publish_image(image_transport::Publisher* pub, CvMat* img, const char* format, std::string frame_id, ros::Time stamp)
{
	if(!pub->getNumSubscribers())
		return 0;

	sensor_msgs::ImagePtr msg(new sensor_msgs::Image());
	cv::Mat m = cv::cvarrToMat(img);
	int cv_type = m.type();
	msg->header.frame_id = frame_id;
	msg->header.stamp = stamp;
	msg->width = img->width;
	msg->height = img->height;
	switch(cv_type)
	{
		case CV_8UC1:	msg->encoding = sensor_msgs::image_encodings::MONO8; break;
		case CV_8UC2:	cv::cvtColor(cv::cvarrToMat(img), m, cv::COLOR_YUV2BGR_YUYV);
		case CV_8UC3:	msg->encoding = sensor_msgs::image_encodings::BGR8; break;
		case CV_16UC1:	msg->encoding = sensor_msgs::image_encodings::MONO16; break;
		default: return -1;
	}
	msg->step = img->step;
	int sz = m.rows * m.step;
	msg->data.resize(sz);
	memcpy(&msg->data[0], m.data, sz);
	pub->publish(msg);
	return 0;
}

int publish_cam_info(ros::Publisher* pub, sensor_msgs::CameraInfo* info, ros::Time stamp)
{
	//if(!pub->getNumSubscribers())
	//	return 0;
	info->header.stamp = stamp;
	pub->publish(*info);
	return 0;
}

static std::vector<int> str_to_ints(std::string str, int n)
{
	std::vector<int> vals;
	int reads = 0;
	const char *ptr = str.c_str();
	char *endptr;
	while(reads < n)
	{
		vals.push_back(strtol(ptr, &endptr, 0));
		if(ptr == endptr)
			break;
		ptr = endptr;
		reads++;
	}
	return vals;
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "trackingcam3d_client_ros");
	ros::NodeHandle nh("~");
	ros::SubscriberStatusCallback connect_cb = boost::bind(connectCb);

	std::string frame_id;
	nh.param<std::string>("frame_id", frame_id, "frame_");

	std::string info_url;
	nh.param<std::string>("info_url", info_url, "");

	int alive_period = 10;
	nh.param<int>("alive_period", alive_period, alive_period);

	int alive_request = 100;
	nh.param<int>("alive_request", alive_request, alive_request);

	int push_calib = 0;
	nh.param<int>("push_calib", push_calib, push_calib);
	ROS_INFO("Calib");

	std::string calib_img_size_str;
	std::vector<int> calib_img_size;
	calib_img_size.push_back(640);
	calib_img_size.push_back(480);
	if(nh.getParam("calib_img_size", calib_img_size_str))
	{
		std::vector<int> sz = str_to_ints(calib_img_size_str, 2);
		if(sz.size() >= 2)
			calib_img_size = sz;
	}


	if(argc > 1)
	{
		std::string arg = argv[1];
		if(arg == "help" || arg == "-h" || arg == "--h")
			fprintf(stderr, "Usage: trackingcam3d_grabber [server1 address (default tcp://localhost)] [server2 address] ... \n");
	}

	int cam_n = argc - 1;
	cams.resize(cam_n? cam_n: 1);
	for(int i = 0; i < cam_n; i++)
	{

		cams[i].open(argv[i + 1]);
	}

	if(cam_n == 0)
	{
		cam_n = 1;
		cams[0].open("tcp://localhost");
	}

	for(int i = 0; i < cam_n; i++)
	{
		char buf[100];
		sprintf(buf, "trackingcam3d%d", i);
		cams_names.push_back(buf);
	}

	tbb::tbb_thread* ctrl_shell_thread = new tbb::tbb_thread(&ctrl_shell, &cams);


	image_transport::ImageTransport it = image_transport::ImageTransport(nh);
	std::vector<TrackingCam3d_ClientRos> ros_cam;

	ros_cam.resize(cam_n);
	for(int i = 0; i < cam_n; i++)
	{
		ros_cam[i].init(&cams[i], cams_names[i], it, calib_img_size, frame_id + cams_names[i], info_url, push_calib);
	}

	int alive_cnt = 0;
	int key = -1;
	while (ros::ok() && key != 27 )
	{
		ros::Time stamp = ros::Time::now();

		if(alive_cnt++ >= alive_period)
			alive_cnt = 0;
		bool is_time_to_alive = alive_cnt;
		if(is_time_to_alive)
		{
			for(int i = 0; i < cams.size(); i++)
			{
				std::string cmd = ros_cam[i].alive(alive_request);
				if(cmd.size())
				{
					char buf[2000];
					cams[i].ctrl_send(cmd.c_str(), strlen(cmd.c_str()) + 1);
					cams[i].ctrl_recv(buf, 2000);
				}
			}
		}

		bool is_somthing_new = false;
		for(int i = 0; i < cams.size(); i++)
		{
			std::map<std::string, CvMat*> images = cams[i].receive();
			for(std::map<std::string, CvMat*>::iterator it = images.begin(); it != images.end(); it++)
			{
				is_somthing_new = true;
				std::string name = cams[i].get_addr() + it->first;
				cv::Mat img = cv::cvarrToMat(it->second);
				//if(img.type() == CV_8UC1)
				//	cv::imshow(name, img);
				//else if(img.type() == CV_8UC3)
				//	cv::imshow(name, img);
				//else if(img.type() == CV_16UC1)
				//	myShowRgbDepthMap(img, 100, 1000, 2000, 10000, name.c_str());
				//else if(img.type() == CV_8UC2)
				//{
				//	cv::Mat m;
				//	cv::cvtColor(img, m, cv::COLOR_YUV2BGR_YUYV);
				//	cv::imshow(name, m);
				//}
				ros_cam[i].publish(it->first, it->second, stamp);
			}
			ros_cam[i].publish_info(stamp);
		}
				
		if(is_somthing_new)
			fps_.update();

		//key = cvWaitKey(1);

		ros::spinOnce();
    }
    
	is_stop_ctrl_shell = true;
	if(ctrl_shell_thread->joinable())
		ctrl_shell_thread->join();
	
	//  We never get here, but clean up anyhow

	
	return 0;
}



