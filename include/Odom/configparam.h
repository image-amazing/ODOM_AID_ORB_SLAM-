#ifndef CONFIGPARAM_H
#define CONFIGPARAM_H

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace ORB_SLAM2
{

class ConfigParam
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ConfigParam(std::string configfile);

    double _testDiscardTime;

    static Eigen::Matrix4d GetEigToc();
    static cv::Mat GetMatToc();
    static Eigen::Matrix4d GetEigT_co();
    static cv::Mat GetMatT_co();
    
    static int GetLocalWindowSize();
    static double GetImageDelayToOdom();

    std::string _bagfile;
    std::string _limageTopic;
    std::string _rimageTopic;
    std::string _odomTopic;

    static std::string getTmpFilePath();
    static std::string _tmpFilePath;

private:
    static Eigen::Matrix4d _EigToc;
    static cv::Mat _MatToc;
    static Eigen::Matrix4d _EigTco;
    static cv::Mat _MatTco;

    static int _LocalWindowSize;
    static double _ImageDelayToOdom;
};

}

#endif // CONFIGPARAM_H
