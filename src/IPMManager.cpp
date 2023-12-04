#include <iostream>
#include <vector>

#include "IPMManager.h"



void IPMManager::IPM_process(cv::Mat &image, std::vector<double> &pose, Mode mode)
{
    front_image_ = image;

    // if rotation data is quaternion
    if (pose.size() == 4)
    {
        Eigen::Quaterniond quat;
        Eigen::Vector3d euler;
        quat.w() = pose[0];
        quat.x() = pose[1];
        quat.y() = pose[2];
        quat.z() = pose[3];

        quat2euler(quat, euler);

        roll_ = euler.x();
        pitch_ = euler.y();
    }
    // if rotation data is roll, pitch, yaw
    else
    {
        roll_ = pose[0];
        pitch_ = pose[1];
    }

    if (mode == Normal)
        std::cout << "Normal mode" << std::endl;
    if (mode == Adaptive)
        std::cout << "Adaptive mode" << std::endl;
    if (mode == Extended)
        std::cout << "Extended mode" << std::endl;


    // get BEV image
    IPM_backward(mode);
}



void IPMManager::quat2euler(Eigen::Quaterniond &quat, Eigen::Vector3d &euler)
{
    // Roll
    double sinr_cosp = 2.0*(quat.w()*quat.x() + quat.y()*quat.z());
    double cosr_cosp = 1.0 - 2.0*(quat.x()*quat.x() + quat.y()*quat.y());
    euler(0) = atan2(sinr_cosp, cosr_cosp);

    // Pitch
    double sinp = sqrt(1.0 + 2.0*(quat.w()*quat.y() - quat.x()*quat.z()));
    double cosp = sqrt(1.0 - 2.0*(quat.w()*quat.y() - quat.x()*quat.z()));
    euler(1) = 2.0*atan2(sinp, cosp) - M_PI/2.0;
    
    // Yaw
    double siny_cosp = 2.0*(quat.w()*quat.z() + quat.x()*quat.z());
    double cosy_cosp = 1.0 - 2.0*(quat.y()*quat.y() + quat.z()*quat.z());
    euler(2) = atan2(siny_cosp, cosy_cosp);
}



void IPMManager::roll_compensation(float &c, float &r, float theta_r)
{
    Eigen::Matrix2f rot;
    Eigen::MatrixXf cr(1, 2), rot_cr(1, 2);

    rot << cos(theta_r), -sin(theta_r), sin(theta_r), cos(theta_r);
    cr << c, r;

    rot_cr = cr*rot;

    c = rot_cr(0,0);
    r = rot_cr(0,1);
}



void IPMManager::IPM_backward(Mode mode)
{   
    const int img_w = 480;
    const int img_h = 640;
    const float vir_fx = 480.0;
    const float vir_fy = 481.2;
    const float vir_cx = 239.5;
    const float vir_cy = 319.5;

    cv::Size IPM_size(img_w, img_h);
    BEV_image_ = cv::Mat::zeros(IPM_size, CV_8UC3);

    float theta_p = pitch_;
    float theta_r = roll_ - init_roll_;
    //! General IPM mode (no compensation)
    if (mode == Normal)
    {
        theta_p = 0.0f;
        theta_r = 0.0f;
    }

    Eigen::Matrix3f RT, K;
    RT << 0.0, -1.0, VIR_CAM_Y_,
        -1.0, 0.0, VIR_CAM_X_,
        0.0, 0.0, VIR_CAM_Z_;

    K << vir_fx, 0, vir_cx,
        0, vir_fy, vir_cy,
        0, 0, 1;

    Eigen::Matrix3f Kinv = K.inverse();
    Eigen::Matrix3f RTinv = RT.inverse();

    for(int v=0; v<IPM_size.height; v++){
        uint8_t* ipm_ptr_r = BEV_image_.ptr<uint8_t>(v);
        for (int u=0; u<IPM_size.width; u++){

            Eigen::Vector3f pixels;
            pixels(0) = u;
            pixels(1) = v;
            pixels(2) = 1;

            Eigen::Vector3f norms, grounds;
            norms = Kinv*pixels;

            grounds = RTinv*norms;
            grounds(0) = grounds.x() / grounds.z();
            grounds(1) = grounds.y() / grounds.z();
            grounds(2) = 0.0;


            // backward warping
            float r = FX_*(tan(theta0_+theta_p)-(CAM_HEIGHT_ / grounds(0))) / ((CAM_HEIGHT_ / grounds(0))*tan(theta0_+theta_p)+1);
            float theta = atan(r/FX_);
            float c = -(FX_*grounds(1)/grounds(0))*(cos(theta0_+theta+theta_p)/cos(theta+theta_p));
            
            //! Extended and Adaptive mode (roll compensation)
            if (mode == Extended)
                roll_compensation(c, r, -theta_r);
            
            auto c2u = [this](float c) {return CX_ + 0.5 + c;};
            auto r2v = [this](float r) {return CY_ + 0.5 - r;};

            int u0 = floor(c2u(c));
            int v0 = floor(r2v(r));

            float u_weight = c2u(c) - u0;
            float v_weight = r2v(r) - v0; 
            
            
            // bilinear interpolation
            if (0 <= v0 && v0 <= front_image_.rows-1 && 0 <= u0 && u0 <= front_image_.cols-1){
                uchar* data_ptr1 = front_image_.ptr<uchar>(v0);
                uchar* data_ptr2 = front_image_.ptr<uchar>(v0+1);

                
                uchar R_val = round((1-u_weight)*((1-v_weight)*data_ptr1[3*u0+0]+v_weight*data_ptr2[3*u0+0])
                                + u_weight*((1-v_weight)*data_ptr1[3*(u0+1)+0]+v_weight*data_ptr2[3*(u0+1)+0]));

                uchar G_val = round((1-u_weight)*((1-v_weight)*data_ptr1[3*u0+1]+v_weight*data_ptr2[3*u0+1])
                                + u_weight*((1-v_weight)*data_ptr1[3*(u0+1)+1]+v_weight*data_ptr2[3*(u0+1)+1]));
                                                
                uchar B_val = round((1-u_weight)*((1-v_weight)*data_ptr1[3*u0+2]+v_weight*data_ptr2[3*u0+2])
                                + u_weight*((1-v_weight)*data_ptr1[3*(u0+1)+2]+v_weight*data_ptr2[3*(u0+1)+2]));

                ipm_ptr_r[3*u+0] = R_val;
                ipm_ptr_r[3*u+1] = G_val;
                ipm_ptr_r[3*u+2] = B_val;
            }
        }
    }
}