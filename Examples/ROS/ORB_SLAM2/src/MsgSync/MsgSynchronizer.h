#ifndef MSGSYNCHRONIZER_H
#define MSGSYNCHRONIZER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <queue>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

using namespace std;

typedef boost::shared_ptr<nav_msgs::Odometry const> OdomConstPtr;

namespace ORBVIO
{
class MsgSynchronizer
{
public:
    enum Status{
        NOTINIT = 0,
        INIT,
        NORMAL
    };

    MsgSynchronizer(const double& imagedelay = 0.);
    ~MsgSynchronizer();

    // add messages in callbacks
    void addLeftImageMsg(const sensor_msgs::ImageConstPtr &limgmsg);
    void addRightImageMsg(const sensor_msgs::ImageConstPtr &rimgmsg);
    //void addImuMsg(const sensor_msgs::ImuConstPtr &imumsg);
    void addOdomMsg(const OdomConstPtr &odommsg);

    // loop in main function to handle all messages
    //bool getRecentMsgs(sensor_msgs::ImageConstPtr &imgmsg, std::vector<sensor_msgs::ImuConstPtr> &vimumsgs);
    bool getRecentMsgs(sensor_msgs::ImageConstPtr &limgmsg, 
		       sensor_msgs::ImageConstPtr &rimgmsg,
		       std::vector<OdomConstPtr> &vodommsgs);

    void clearMsgs(void);

    // for message callback if needed
    void leftImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void rightImageCallback(const sensor_msgs::ImageConstPtr& msg);
    //void imuCallback(const sensor_msgs::ImuConstPtr& msg);
    void odomCallback(const OdomConstPtr& msg);

    //
    inline Status getStatus(void) {return _status;}

    double getImageDelaySec(void) const {return _imageMsgDelaySec;}

private:
    double _imageMsgDelaySec;  // image message delay to odom message, in seconds
    std::queue<sensor_msgs::ImageConstPtr> _limageMsgQueue;
    std::queue<sensor_msgs::ImageConstPtr> _rimageMsgQueue;
    //std::queue<sensor_msgs::ImuConstPtr> _imuMsgQueue;
    std::queue<OdomConstPtr> _odomMsgQueue;
    //ros::Time _imuMsgTimeStart;
    ros::Time _odomMsgTimeStart;
    Status _status;
    int _dataUnsyncCnt;
};

}

#endif // MSGSYNCHRONIZER_H
