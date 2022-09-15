#ifndef ROS_SCALIB_CONFIGURATOR_H
#define ROS_SCALIB_CONFIGURATOR_H

#include <tconfig/tconfig.h>
//#include <t3dproc/MySCalib.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <iostream>

#ifdef WIN32
#	define PATH_SEPARATOR "\\"
#else
#	define PATH_SEPARATOR "/"
#endif

class RosSCalibConfigurator : public tconfig::TextClass
{
    int hash_;
    bool is_changed_;
    bool is_hash_changed() {int h = hash_params(); if(h != hash_) {hash_ = h; return true;} return false;}


    template<typename T, int N> boost::array<T, N> vec_to_arr(std::vector<T> v) {
        boost::array<T, N> a;
        for(int i = 0; i < N; i++)
            a[i] = v[i];
        return a;
    }

    template<typename T, int N> std::vector<T> arr_to_vec(boost::array<T, N> a) {
        std::vector<T> v;
        for(int i = 0; i < N; i++)
            v.push_back(a[i]);
        return v;
    }

    //return cv::Mat(rows, cols, CV_64F, v.data()).clone();}

    //std::map<std::string, int> hashes;
public:
    sensor_msgs::CameraInfo info[2];

    RosSCalibConfigurator(): hash_(0), is_changed_(false) {
        params["calib.cam1.image_width"]				= tconfig::Param(640);
        params["calib.cam1.image_height"]				= tconfig::Param(480);
        params["calib.cam1.camera_name"]				= tconfig::Param("left_640");
        params["calib.cam1.camera_matrix"]				= tconfig::Param("500 0   320 "
                                                                                         "0   500 240 "
                                                                                         "0   0   1   ");
        params["calib.cam1.distortion_model"]			= tconfig::Param("plumb_bob", "plumb_bob");
        params["calib.cam1.distortion_coefficients"]	= tconfig::Param("0 0 0 0 0 ");
        params["calib.cam1.rectification_matrix"]		= tconfig::Param("1 0 0 "
                                                                                 "0 1 0 "
                                                                                 "0 0 1 ");
        params["calib.cam1.projection_matrix"]			= tconfig::Param("500 0   320 0 "
                                                                                 "0   500 240 0 "
                                                                                 "0   0   1   0 ");

        params["calib.cam2.image_width"]				= tconfig::Param(640);
        params["calib.cam2.image_height"]				= tconfig::Param(480);
        params["calib.cam2.camera_name"]				= tconfig::Param("right_640");
        params["calib.cam2.camera_matrix"]				= tconfig::Param("500 0   320 "
                                                                                         "0   500 240 "
                                                                                         "0   0   1   ");
        params["calib.cam2.distortion_model"]			= tconfig::Param("plumb_bob", "plumb_bob");
        params["calib.cam2.distortion_coefficients"]	= tconfig::Param("0 0 0 0 0 ");
        params["calib.cam2.rectification_matrix"]		= tconfig::Param("1 0 0 "
                                                                                 "0 1 0 "
                                                                                 "0 0 1");
        params["calib.cam2.projection_matrix"]			= tconfig::Param("500 0   320 -50000 "
                                                                                 "0   500 240  0     "
                                                                                 "0   0   1    0     ");

        //Currently unsupported parameters
//        info[0].roi.x_offset = info[0].roi.y_offset = info[0].binning_x = info[0].binning_y = 0;
//        info[0].roi.width = info[0].width;
//        info[0].roi.height = info[0].height;

//        info[1].roi.x_offset = info[1].roi.y_offset = info[1].binning_x = info[1].binning_y = 0;
//        info[1].roi.width = info[1].width;
//        info[1].roi.height = info[1].height;
    }

    void params_to_info() {
        info[0].width           	= params["calib.cam1.image_width"].i();
        info[0].height          	= params["calib.cam1.image_height"].i();
        //info[0].camera_name		= params["calib.cam1.camera_name"].str();
        info[0].K			= vec_to_arr<double, 3*3>(params["calib.cam1.camera_matrix"].dvec());
        info[0].distortion_model	= params["calib.cam1.distortion_model"].str();
        info[0].D			= params["calib.cam1.distortion_coefficients"].dvec();
        info[0].R			= vec_to_arr<double, 3*3>(params["calib.cam1.rectification_matrix"].dvec());
        info[0].P			= vec_to_arr<double, 3*4>(params["calib.cam1.projection_matrix"].dvec());

        info[1].width                   = params["calib.cam2.image_width"].i();
        info[1].height                  = params["calib.cam2.image_height"].i();
        //info[1].camera_name		= params["calib.cam2.camera_name"].str();
        info[1].K			= vec_to_arr<double, 3*3>(params["calib.cam2.camera_matrix"].dvec());
        info[1].distortion_model	= params["calib.cam2.distortion_model"].str();
        info[1].D			= params["calib.cam2.distortion_coefficients"].dvec();
        info[1].R                       = vec_to_arr<double, 3*3>(params["calib.cam2.rectification_matrix"].dvec());
        info[1].P			= vec_to_arr<double, 3*4>(params["calib.cam2.projection_matrix"].dvec());
    }

    void info_to_params() {
        params["calib.cam1.image_width"] = (int)info[0].width;
        params["calib.cam1.image_height"] = (int)info[0].height;
        //params["calib.cam1.camera_name"] = info[0].camera_name;
        params["calib.cam1.camera_matrix"].set((double*)info[0].K.data(), 3*3);
        params["calib.cam1.distortion_model"] = info[0].distortion_model;
        params["calib.cam1.distortion_coefficients"].set((double*)info[0].D.data(), 1*5);
        params["calib.cam1.rectification_matrix"].set((double*)info[0].R.data(), 3*3);
        params["calib.cam1.projection_matrix"].set((double*)info[0].P.data(), 3*4);

        params["calib.cam2.image_width"] = (int)info[1].width;
        params["calib.cam2.image_height"] = (int)info[1].height;
        //params["calib.cam2.camera_name"] = info[1].camera_name;
        params["calib.cam2.camera_matrix"].set((double*)info[1].K.data(), 3*3);
        params["calib.cam2.distortion_model"] = info[1].distortion_model;
        params["calib.cam2.distortion_coefficients"].set((double*)info[1].D.data(), 1*5);
        params["calib.cam2.rectification_matrix"].set((double*)info[1].R.data(), 3*3);
        params["calib.cam2.projection_matrix"].set((double*)info[1].P.data(), 3*4);
    }

//    void update() {
//        int h = hash_params();
//        is_changed_ = (h != hash_);
//        if(is_changed_)
//            params_to_calib();
//        hash_ = h;
//    };
};




#endif
