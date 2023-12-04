#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include "IPMManager.h"


void set_path(std::vector<std::string> &vImage_path,
              std::vector<std::vector<double>> &vImuRPY);
void test_image();


//! Path setting
const std::string input = "sample1.png";
const std::string input_imu = "sample1_imu.txt";

//* image path setting
// image
const std::string data_root = "../data/input/" + input;
// imu
const std::string imu_path = "../data/" + input_imu;

//* Image is saved this path
const std::string write_path = "../data/output/";


// select mode
bool is_write = false;
bool is_show = true;
bool test_mode = false;


std::vector<std::string> vImage_path;
std::vector<std::vector<double>> vOdomRPY, vImuRPY; 


int main()
{   
    set_path(vImage_path, vImuRPY);

    //* BEV image test
    if (test_mode == true)
        test_image();


    IPMManager IPM;

    for(int i=0; i<vImage_path.size(); ++i)
    {
        cv::Mat image = cv::imread(vImage_path[i], cv::IMREAD_UNCHANGED);;
        std::istringstream iss(vImage_path[i]);
        std::string image_name;
        char separator = '/';
        while (getline(iss, image_name, separator));
        {
        }
        std::cout << "-------------------------------------------" << std::endl;
        std::vector<double> pose;
        pose = vImuRPY[i];
        pose[0] = pose[0];
        pose[1] = pose[1];
        std::cout << "Magnitude of roll & pitch (radian)" << std::endl;
        std::cout << "roll : " << pose[0] << "  pitch : " << pose[1] << std::endl;
        std::cout << std::endl;


        IPM.IPM_process(image, pose, Normal);
        cv::Mat BEV_image_normal = IPM.get_BEV_image().clone();

        IPM.IPM_process(image, pose, Adaptive);
        cv::Mat BEV_image_adaptive = IPM.get_BEV_image().clone();

        IPM.IPM_process(image, pose, Extended);
        cv::Mat BEV_image_extended = IPM.get_BEV_image().clone();

        if (is_show == true)
        {
            cv::namedWindow("input_image", 0);
            cv::resizeWindow("input_image", image.rows/1.5, image.cols/1.5);
            cv::imshow("input_image", image);

            cv::imshow("BEV image normal", BEV_image_normal);
            cv::imshow("BEV image adaptive", BEV_image_adaptive);
            cv::imshow("BEV image extended", BEV_image_extended);
            cv::waitKey(0);
        }

        if (is_write == true)
        {
            cv::imwrite(write_path + image_name + "normal", BEV_image_normal);
            cv::imwrite(write_path + image_name + "adaptive", BEV_image_adaptive);
            cv::imwrite(write_path + image_name + "extended", BEV_image_extended);
            std::cout << image_name << " images is saved!!" << std::endl;
        }
    }

    return 0;
}



void set_path(std::vector<std::string> &vImage_path,
              std::vector<std::vector<double>> &vImuRPY)
{
    // read imu
    std::ifstream fImuRPY;
    fImuRPY.open(imu_path.c_str());
    
    while(fImuRPY){
        std::string s;
        getline(fImuRPY, s);
        if(!s.empty()){
            char separator = ' ';
            std::istringstream iss(s);
            std::string buf;
            std::vector<double> temp;

            while(getline(iss, buf, separator)){
                temp.push_back(std::stod(buf));
            }
            vImuRPY.push_back(temp);
        }
    }

    cv::glob(data_root, vImage_path, false);
    std::sort(vImage_path.begin(), vImage_path.end());
}



//! If you want to check the results of single image.
void test_image()
{
    cv::Mat image = cv::imread(data_root, cv::IMREAD_UNCHANGED);
    bool mode = Normal;

    IPMManager test_IPM;

    std::vector<double> test_pose;
    test_pose = vImuRPY[0];
    test_pose[0] = -test_pose[0];
    test_pose[1] = -test_pose[1];


    test_IPM.IPM_process(image, test_pose, Normal);

    cv::Mat test_BEV_image = test_IPM.get_BEV_image();

    cv::namedWindow("image", 0);
    cv::resizeWindow("image", image.rows/1.5, image.cols/1.5);
    cv::imshow("image", image);

    cv::imshow("test_BEV_image", test_BEV_image);
    cv::waitKey(0);

    exit(0);
}



