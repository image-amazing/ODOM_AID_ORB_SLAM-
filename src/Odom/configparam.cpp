#include "Odom/configparam.h"

namespace ORB_SLAM2
{

Eigen::Matrix4d ConfigParam::_EigToc = Eigen::Matrix4d::Identity();
cv::Mat ConfigParam::_MatToc = cv::Mat::eye(4,4,CV_32F);
Eigen::Matrix4d ConfigParam::_EigTco = Eigen::Matrix4d::Identity();
cv::Mat ConfigParam::_MatTco = cv::Mat::eye(4,4,CV_32F);

int ConfigParam::_LocalWindowSize = 10;
double ConfigParam::_ImageDelayToOdom = 0;
std::string ConfigParam::_tmpFilePath = "";

ConfigParam::ConfigParam(std::string configfile)
{
    cv::FileStorage fSettings(configfile, cv::FileStorage::READ);

    std::cout<<std::endl<<std::endl<<"Parameters: "<<std::endl;

    _testDiscardTime = fSettings["test.DiscardTime"];

    fSettings["test.InitVIOTmpPath"] >> _tmpFilePath;
    std::cout<<"save tmp file in "<<_tmpFilePath<<std::endl;

    fSettings["bagfile"] >> _bagfile;
    std::cout<<"open rosbag: "<<_bagfile<<std::endl;
    fSettings["limagetopic"] >> _limageTopic;
    fSettings["rimagetopic"] >> _rimageTopic;
    fSettings["odomtopic"] >> _odomTopic;
    std::cout<<"left image topic: "<<_limageTopic<<std::endl;
    std::cout<<"right image topic: "<<_rimageTopic<<std::endl;
    std::cout<<"odom topic: "<<_odomTopic<<std::endl;
    

    _LocalWindowSize = fSettings["LocalMapping.LocalWindowSize"];
    std::cout<<"local window size: "<<_LocalWindowSize<<std::endl;

    _ImageDelayToOdom = fSettings["Camera.delaytoodom"];
    std::cout<<"timestamp image delay to odom: "<<_ImageDelayToOdom<<std::endl;

    {
        cv::FileNode Toc_ = fSettings["Camera.Toc"];
        Eigen::Matrix<double,3,3> R;
        R << 	Toc_[0], Toc_[1], Toc_[2],
                Toc_[4], Toc_[5], Toc_[6],
                Toc_[8], Toc_[9], Toc_[10];
        Eigen::Quaterniond qr(R);
        R = qr.normalized().toRotationMatrix();
        Eigen::Matrix<double,3,1> t( Toc_[3], Toc_[7], Toc_[11]);
        _EigToc = Eigen::Matrix4d::Identity();
        _EigToc.block<3,3>(0,0) = R;
        _EigToc.block<3,1>(0,3) = t;
        _MatToc = cv::Mat::eye(4,4,CV_32F);
        for(int i=0;i<4;i++)
            for(int j=0;j<4;j++)
                _MatToc.at<float>(i,j) = _EigToc(i,j);

        _EigTco = Eigen::Matrix4d::Identity();
        _EigTco.block<3,3>(0,0) = R.transpose();
        _EigTco.block<3,1>(0,3) = -R.transpose()*t;
	_MatTco = cv::Mat::eye(4,4,CV_32F);
        for(int i=0;i<4;i++)
            for(int j=0;j<4;j++)
                _MatTco.at<float>(i,j) = _EigTco(i,j);

        // Tbc_[0], Tbc_[1], Tbc_[2], Tbc_[3], Tbc_[4], Tbc_[5], Tbc_[6], Tbc_[7], Tbc_[8], Tbc_[9], Tbc_[10], Tbc_[11], Tbc_[12], Tbc_[13], Tbc_[14], Tbc_[15];
        std::cout<<"Toc inited:"<<std::endl<<_EigToc<<std::endl<<_MatToc<<std::endl;
        std::cout<<"Tco inited:"<<std::endl<<_EigTco<<std::endl<<_MatTco<<std::endl;
        std::cout<<"Toc*Tco:"<<std::endl<<_EigToc*_EigTco<<std::endl<<_MatToc*_MatTco<<std::endl;
    }
    
}

std::string ConfigParam::getTmpFilePath()
{
    return _tmpFilePath;
}

Eigen::Matrix4d ConfigParam::GetEigToc()
{
    return _EigToc;
}

cv::Mat ConfigParam::GetMatToc()
{
    return _MatToc.clone();
}

Eigen::Matrix4d ConfigParam::GetEigT_co()
{
    return _EigTco;
}

cv::Mat ConfigParam::GetMatT_co()
{
    return _MatTco.clone();
}

int ConfigParam::GetLocalWindowSize()
{
    return _LocalWindowSize;
}

double ConfigParam::GetImageDelayToOdom()
{
    return _ImageDelayToOdom;
}

}
