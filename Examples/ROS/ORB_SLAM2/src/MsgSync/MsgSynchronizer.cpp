#include "MsgSynchronizer.h"

namespace ORBVIO
{

MsgSynchronizer::MsgSynchronizer(const double& imagedelay):
    _imageMsgDelaySec(imagedelay), _status(NOTINIT),
    _dataUnsyncCnt(0)
{
    printf("image delay set as %.1fms\n",_imageMsgDelaySec*1000);
}

MsgSynchronizer::~MsgSynchronizer()
{

}


//bool MsgSynchronizer::getRecentMsgs(sensor_msgs::ImageConstPtr &imgmsg, std::vector<sensor_msgs::ImuConstPtr> &vimumsgs)
bool MsgSynchronizer::getRecentMsgs(sensor_msgs::ImageConstPtr &limgmsg, 
		   sensor_msgs::ImageConstPtr &rimgmsg,
		   std::vector<OdomConstPtr> &vodommsgs)
{
    if(_status == NOTINIT || _status == INIT)
    {
        //ROS_INFO("synchronizer not inited");
        return false;
    }
    if(_limageMsgQueue.empty())
    {
        //ROS_INFO("no left image stored in queue currently.");
        return false;
    }
    if(_rimageMsgQueue.empty())
    {
        //ROS_INFO("no right image stored in queue currently.");
        return false;
    }
    if(_odomMsgQueue.empty())
    {
        //ROS_WARN("no odom message stored, shouldn't");
        return false;
    }

    {
        sensor_msgs::ImageConstPtr limsg;
        sensor_msgs::ImageConstPtr rimsg;
        //sensor_msgs::ImuConstPtr bmsg;
	OdomConstPtr bmsg;

        //
        limsg = _limageMsgQueue.back();
	rimsg = _rimageMsgQueue.back();
        //bmsg = _imuMsgQueue.front();
	bmsg = _odomMsgQueue.front();

        // Check dis-continuity, tolerance 3 seconds
        if(limsg->header.stamp.toSec()-_imageMsgDelaySec + 3.0 < bmsg->header.stamp.toSec() )
        {
            ROS_ERROR("Data dis-continuity, > 3 seconds. Buffer cleared");
            clearMsgs();
            return false;
        }

        //
        limsg = _limageMsgQueue.front();
        rimsg = _rimageMsgQueue.front();
        //bmsg = _imuMsgQueue.back();
	bmsg = _odomMsgQueue.back();

        // Check dis-continuity, tolerance 3 seconds
        if(limsg->header.stamp.toSec()-_imageMsgDelaySec > bmsg->header.stamp.toSec() + 3.0)
        {
            ROS_ERROR("Data dis-continuity, > 3 seconds. Buffer cleared");
            clearMsgs();
            return false;
        }

        // Wait until the odom packages totolly com
        if(_limageMsgQueue.size()<10 && _odomMsgQueue.size() < 15/*_imuMsgQueue.size()<15*/
           && limsg->header.stamp.toSec()-_imageMsgDelaySec>bmsg->header.stamp.toSec() )
        {
            //ROS_WARN_STREAM("here return, last odom time "<<);
            return false;

        }

    }

    // get image message
    limgmsg = _limageMsgQueue.front();
    _limageMsgQueue.pop();
    rimgmsg = _rimageMsgQueue.front();
    _rimageMsgQueue.pop();

    // clear odom message vector, and push all odom messages whose timestamp is earlier than image message
    //vimumsgs.clear();
    vodommsgs.clear();
    while(true)
    {
        // if no more odom messages, stop loop
        //if(_imuMsgQueue.empty())
	if(_odomMsgQueue.empty())
            break;

        // consider delay between image and odom serial
        //sensor_msgs::ImuConstPtr tmpimumsg = _imuMsgQueue.front();
	OdomConstPtr tmpodommsg = _odomMsgQueue.front();
        //if(tmpimumsg->header.stamp.toSec() < imgmsg->header.stamp.toSec() - _imageMsgDelaySec)
	if(tmpodommsg->header.stamp.toSec() < limgmsg->header.stamp.toSec() - _imageMsgDelaySec)
        {
            // add to odom message vector
            //vimumsgs.push_back(tmpimumsg);
	    vodommsgs.push_back(tmpodommsg);
            //_imuMsgQueue.pop();
	    _odomMsgQueue.pop();

            _dataUnsyncCnt = 0;
        }
        else
        {
            if(_dataUnsyncCnt++>10)
            {
                _dataUnsyncCnt = 0;
                //_imuMsgQueue = std::queue<sensor_msgs::ImuConstPtr>();
                clearMsgs();
                ROS_ERROR("data unsynced many times, reset sync");
                return false;
            }
            // stop loop
            break;
        }
    }

    // the camera fps 20Hz, imu message 100Hz. so there should be not more than 5 imu messages between images
    //if(vimumsgs.size()>10)
    //    ROS_WARN("%lu imu messages between images, note",vimumsgs.size());
    //if(vimumsgs.size()==0)
    //    ROS_ERROR("no imu message between images!");
    if(vodommsgs.size()>10)
        ROS_WARN("%lu odom messages between images, note",vodommsgs.size());
    if(vodommsgs.size()==0)
        ROS_ERROR("no odom message between images!");

    return true;
}

// void MsgSynchronizer::addImuMsg(const sensor_msgs::ImuConstPtr &imumsg)
// {
//     if(_imageMsgDelaySec>=0) {
//         _imuMsgQueue.push(imumsg);
//         if(_status == NOTINIT)
//         {
//             _imuMsgTimeStart = imumsg->header.stamp;
//             _status = INIT;
//         }
//     }
//     else {
//         // if there's no image messages, don't add image
//         if(_status == NOTINIT)
//             return;
//         else if(_status == INIT)
//         {
//             // ignore all image messages with no imu messages between them
//             // only add below images
//             if(imumsg->header.stamp.toSec() + _imageMsgDelaySec > _imuMsgTimeStart.toSec())
//             {
//                 _imuMsgQueue.push(imumsg);
//                 _status = NORMAL;
//             }
//         }
//         else
//         {
//             // push message into queue
//             _imuMsgQueue.push(imumsg);
//         }
//     }
// 
// 
// }

void MsgSynchronizer::addOdomMsg(const OdomConstPtr &odommsg)
{
    if(_imageMsgDelaySec>=0) {
        _odomMsgQueue.push(odommsg);
        if(_status == NOTINIT)
        {
            _odomMsgTimeStart = odommsg->header.stamp;
            _status = INIT;
        }
    }
    else {
        // if there's no image messages, don't add image
        if(_status == NOTINIT)
            return;
        else if(_status == INIT)
        {
            // ignore all image messages with no odom messages between them
            // only add below images
            if(odommsg->header.stamp.toSec() + _imageMsgDelaySec > _odomMsgTimeStart.toSec())
            {
                _odomMsgQueue.push(odommsg);
                _status = NORMAL;
            }
        }
        else
        {
            // push message into queue
            _odomMsgQueue.push(odommsg);
        }
    }


}

void MsgSynchronizer::addLeftImageMsg(const sensor_msgs::ImageConstPtr &limgmsg)
{
    if(_imageMsgDelaySec >= 0) {
        // if there's no odom messages, don't add image
        if(_status == NOTINIT)
            return;
        else if(_status == INIT)
        {
            // ignore all image messages with no odom messages between them
            // only add below images
            if(limgmsg->header.stamp.toSec() - _imageMsgDelaySec > _odomMsgTimeStart.toSec())
            {
                _limageMsgQueue.push(limgmsg);
                _status = NORMAL;
            }
        }
        else
        {
            // push message into queue
            _limageMsgQueue.push(limgmsg);
        }
    }
    else {  // start by image message
        if(_status == NOTINIT)
        {
            _odomMsgTimeStart = limgmsg->header.stamp;
            _status = INIT;
        }
        else
        {   // no image data if there's no odom message
            _limageMsgQueue.push(limgmsg);
        }

    }
}

void MsgSynchronizer::addRightImageMsg(const sensor_msgs::ImageConstPtr &rimgmsg)
{
    if(_imageMsgDelaySec >= 0) {
        // if there's no odom messages, don't add image
        if(_status == NOTINIT)
            return;
        else if(_status == INIT)
        {
            // ignore all image messages with no odom messages between them
            // only add below images
            if(rimgmsg->header.stamp.toSec() - _imageMsgDelaySec > _odomMsgTimeStart.toSec())
            {
                _rimageMsgQueue.push(rimgmsg);
                _status = NORMAL;
            }
        }
        else
        {
            // push message into queue
            _rimageMsgQueue.push(rimgmsg);
        }
    }
    else {  // start by image message
        if(_status == NOTINIT)
        {
            _odomMsgTimeStart = rimgmsg->header.stamp;
            _status = INIT;
        }
        else
        {   // no image data if there's no odom message
            _rimageMsgQueue.push(rimgmsg);
        }

    }
}

void MsgSynchronizer::leftImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    addLeftImageMsg(msg);
}

void MsgSynchronizer::rightImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    addRightImageMsg(msg);
}

// void MsgSynchronizer::imuCallback(const sensor_msgs::ImuConstPtr &msg)
// {
//     addImuMsg(msg);
// }

void MsgSynchronizer::odomCallback(const OdomConstPtr& msg)
{
    addOdomMsg(msg);
}

void MsgSynchronizer::clearMsgs(void)
{
//     _imuMsgQueue = std::queue<sensor_msgs::ImuConstPtr>();
    _odomMsgQueue = std::queue<OdomConstPtr>();
    _limageMsgQueue = std::queue<sensor_msgs::ImageConstPtr>();
    _rimageMsgQueue = std::queue<sensor_msgs::ImageConstPtr>();
//    while(!_imageMsgQueue.empty())
//    {
//        _imageMsgQueue.pop();
//    }
//    while(!_imuMsgQueue.empty())
//    {
//        _imuMsgQueue.pop();
//    }
}

}
