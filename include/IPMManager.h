/* ----------------------------------------------------------------------------
*
* Created by pjy on 23. 6. 1.
*
* -------------------------------------------------------------------------- */


#ifndef IPM_MANAGER_H
#define IPM_MANAGER_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <opencv2/opencv.hpp>


enum Mode {
    Normal,
    Adaptive,
    Extended
};


class IPMManager
{
    //! sample 1 ----------------------------------------------------
    // const float FX_ = 479.619; 
    // const float FY_ = 479.96602;
    // const float CX_ = 840.60503;
    // const float CY_ = 610.61653;
    // const float CAM_HEIGHT_ = 0.735;
    // const float VIR_CAM_X_ = 3.5;
    // const float VIR_CAM_Z_ = 4.3;         // + : left, - : right
    // const float VIR_CAM_Y_ = 0.0; 
    // const float theta0_ = 0.0*M_PI/180;
    // const float init_roll_ = 0.0*M_PI/180;


    //! sample 2 & 3 ----------------------------------------------------
    const float FX_ = 1318.93;
    const float FY_ = 1320.25;
    const float CX_ = 1337.66;
    const float CY_ = 1004.15;
    const float CAM_HEIGHT_ = 0.561;
    const float VIR_CAM_X_ = 3.0;
    const float VIR_CAM_Y_ = 0.0;            // + : left, - : right
    const float VIR_CAM_Z_ = 3.0;
    const float theta0_ = 0.0*M_PI/180;
    const float init_roll_ = 0.0*M_PI/180;

    double roll_;
    double pitch_;

    cv::Mat front_image_;
    cv::Mat BEV_image_;

public:
    IPMManager() : roll_(0.0), pitch_(0.0) {};
    ~IPMManager() {};

    void IPMProcess(cv::Mat &image, std::vector<double> &pose, Mode mode);

    void quat2euler(Eigen::Quaterniond &quaternion, Eigen::Vector3d &euler);

    void rollCompensation(float &c, float &r, float theta_r);

    void IPMBackward(Mode mode);

    cv::Mat getBEVImage() {  return BEV_image_;  };
};


#endif