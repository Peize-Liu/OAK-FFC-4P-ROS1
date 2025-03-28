#pragma once

#include <deque>
#include <thread>
#include <type_traits>
#include <typeinfo>
#include <vector>
#include <map>

#include "boost/make_shared.hpp"
#include "boost/shared_ptr.hpp"
#include "camera_info_manager/camera_info_manager.h"
#include "depthai/depthai.hpp"
#include "image_transport/image_transport.h"
#include "ros/console.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

namespace dai {

namespace ros {

namespace StdMsgs = std_msgs;
namespace ImageMsgs = sensor_msgs;
using ImagePtr = ImageMsgs::ImagePtr;
namespace rosOrigin = ::ros;

template <class RosMsg>
class SyncBridgePublisher {
   public:
    using ConvertFunc = std::function<void(std::shared_ptr<dai::ImgFrame>, std::deque<RosMsg>&)>;

    using CustomPublisher = typename std::
        conditional<std::is_same<RosMsg, ImageMsgs::Image>::value, std::shared_ptr<image_transport::Publisher>, std::shared_ptr<rosOrigin::Publisher> >::type;

    SyncBridgePublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                    rosOrigin::NodeHandle nh,
                    std::map<std::string, std::string> rosTopic,
                    ConvertFunc converter,
                    int queueSize,
                    std::string cameraParamUri = "",
                    bool lazyPublisher = true);

    SyncBridgePublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                    rosOrigin::NodeHandle nh,
                    std::map<std::string, std::string> rosTopics,
                    ConvertFunc converter,
                    int queueSize,
                    ImageMsgs::CameraInfo cameraInfoData,
                    bool lazyPublisher = true);

    /**
     * Tag Dispacher function to to overload the Publisher to ImageTransport Publisher
     */
    void advertise(int queueSize, std::true_type);

    /**
     * Tag Dispacher function to to overload the Publisher to use Default ros::Publisher
     */
    void advertise(int queueSize, std::false_type);

    SyncBridgePublisher(const SyncBridgePublisher& other);

    void addPublisherCallback();

    void publishHelper(std::shared_ptr<dai::ImgFrame> inData, std::string camera_name);

    void startPublisherThread();

    ~SyncBridgePublisher();

   private:
    /**
     * adding this callback will allow you to still be able to consume
     * the data for other processing using get() function .
     */
    void daiCallback(std::string name, std::shared_ptr<ADatatype> data);

    static const std::string LOG_TAG;
    std::shared_ptr<dai::DataOutputQueue> _daiMessageQueue;
    ConvertFunc _converter;

    rosOrigin::NodeHandle _nh;
    std::map<std::string, std::shared_ptr<rosOrigin::Publisher>> _cameraInfoPublisher;

    image_transport::ImageTransport _it;
    ImageMsgs::CameraInfo _cameraInfoData;
    std::map<std::string, CustomPublisher> _rosPublisher;

    std::thread _readingThread;
    std::map<std::string, std::string> _rosTopics;
    std::string _camInfoFrameId, _cameraParamUri;
    std::map<std::string, std::unique_ptr<camera_info_manager::CameraInfoManager>> _camInfoManager;
    bool _isCallbackAdded = false;
    bool _isImageMessage = false;  // used to enable camera info manager
    bool _lazyPublisher = true;
};

template <class RosMsg>
const std::string SyncBridgePublisher<RosMsg>::LOG_TAG = "SyncBridgePublisher";

template <class RosMsg>
SyncBridgePublisher<RosMsg>::SyncBridgePublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                                                 rosOrigin::NodeHandle nh,
                                                 std::map<std::string, std::string> rosTopics,
                                                 ConvertFunc converter,
                                                 int queueSize,
                                                 std::string cameraParamUri,
                                                 bool lazyPublisher)
    : _daiMessageQueue(daiMessageQueue),
      _converter(converter),
      _nh(nh),
      _it(_nh),
      _rosTopics(rosTopics),
      _cameraParamUri(cameraParamUri),
      _lazyPublisher(lazyPublisher) {
    // ROS_DEBUG_STREAM_NAMED(LOG_TAG, "Publisher Type : " << typeid(CustomPublisher).name());
    advertise(queueSize, std::is_same<RosMsg, ImageMsgs::Image>{});
}

template <class RosMsg>
SyncBridgePublisher<RosMsg>::SyncBridgePublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                                                 rosOrigin::NodeHandle nh,
                                                 std::map<std::string, std::string> rosTopics,
                                                 ConvertFunc converter,
                                                 int queueSize,
                                                 ImageMsgs::CameraInfo cameraInfoData,
                                                 bool lazyPublisher)
    : _daiMessageQueue(daiMessageQueue),
      _nh(nh),
      _converter(converter),
      _it(_nh),
      _cameraInfoData(cameraInfoData),
      _rosTopics(rosTopics),
      _lazyPublisher(lazyPublisher) {
    // ROS_DEBUG_STREAM_NAMED(LOG_TAG, "Publisher Type : " << typeid(CustomPublisher).name());
    advertise(queueSize, std::is_same<RosMsg, ImageMsgs::Image>{});
}

template <class RosMsg>
void SyncBridgePublisher<RosMsg>::advertise(int queueSize, std::false_type) {
    for (auto& rosTopic: _rosTopics) {
        _rosPublisher[rosTopic.first] = std::make_shared<rosOrigin::Publisher>(_nh.advertise<RosMsg>(rosTopic.second, queueSize));
    }
}

template <class RosMsg>
void SyncBridgePublisher<RosMsg>::advertise(int queueSize, std::true_type) {
    for (auto& rosTopic: _rosTopics) {
        auto cam_name = rosTopic.first;
        auto topic = rosTopic.second;
        if(!cam_name.empty()) {
            _isImageMessage = true;
            _camInfoManager[cam_name] = std::make_unique<camera_info_manager::CameraInfoManager>(rosOrigin::NodeHandle{_nh, cam_name}, cam_name, _cameraParamUri);
            if(_cameraParamUri.empty()) {
                _camInfoManager[cam_name]->setCameraInfo(_cameraInfoData);
            }
            // _cameraInfoPublisher[cam_name] = std::make_shared<rosOrigin::Publisher>(_nh.advertise<ImageMsgs::CameraInfo>(cam_name + "/camera_info", queueSize));
            _cameraInfoPublisher.insert({cam_name, std::make_shared<rosOrigin::Publisher>(_nh.advertise<ImageMsgs::CameraInfo>(cam_name + "/camera_info", queueSize))});
        }
        _rosPublisher[cam_name] = std::make_shared<image_transport::Publisher>(_it.advertise(topic, queueSize));
    }
}

template <class RosMsg>
SyncBridgePublisher<RosMsg>::SyncBridgePublisher(const SyncBridgePublisher& other) {
    _daiMessageQueue = other._daiMessageQueue;
    _nh = other._nh;
    _converter = other._converter;
    _rosTopics = other._rosTopics;
    _it = other._it;
    for (auto& rosTopic: _rosTopics) {
        auto cam_name = rosTopic.first;
        _rosPublisher[cam_name] = CustomPublisher(other._rosPublisher.at(cam_name));

        if(other._isImageMessage) {
            _isImageMessage = true;
            _camInfoManager[cam_name] = std::make_unique<camera_info_manager::CameraInfoManager>(std::move(other._camInfoManager.at(cam_name)));
            _cameraInfoPublisher[cam_name] = rosOrigin::Publisher(other._cameraInfoPublisher.at(cam_name));
        }
    }
}

template <class RosMsg>
void SyncBridgePublisher<RosMsg>::daiCallback(std::string name, std::shared_ptr<ADatatype> data) {
    auto daiDataPtr = std::dynamic_pointer_cast<dai::MessageGroup>(data);
    for (auto& rosTopic: _rosTopics) {
        auto pkg = daiDataPtr->get<dai::ImgFrame>(rosTopic.first);
        publishHelper(pkg, rosTopic.first);
    }
}

template <class RosMsg>
void SyncBridgePublisher<RosMsg>::startPublisherThread() {
    if(_isCallbackAdded) {
        std::runtime_error(
            "addPublisherCallback() function adds a callback to the"
            "depthai which handles the publishing so no need to start"
            "the thread using startPublisherThread() ");
    }

    _readingThread = std::thread([&]() {
        int messageCounter = 0;
        while(rosOrigin::ok()) {
            // auto daiDataPtr = _daiMessageQueue->get<dai::MessageGroup>();
            auto daiDataPtr = _daiMessageQueue->tryGet<dai::MessageGroup>();
            if(daiDataPtr == nullptr) {
                messageCounter++;
                if(messageCounter > 2000000) {
                    messageCounter = 0;
                }
                continue;
            }

            if(messageCounter != 0) {
                messageCounter = 0;
            }
            for (auto& rosTopic: _rosTopics) {
                auto pkg = daiDataPtr->get<dai::ImgFrame>(rosTopic.first);
                publishHelper(pkg, rosTopic.first);
            }
        }
    });
}

template <class RosMsg>
void SyncBridgePublisher<RosMsg>::addPublisherCallback() {
    _daiMessageQueue->addCallback(std::bind(&SyncBridgePublisher<RosMsg>::daiCallback, this, std::placeholders::_1, std::placeholders::_2));
    _isCallbackAdded = true;
}

template <class RosMsg>
void SyncBridgePublisher<RosMsg>::publishHelper(std::shared_ptr<dai::ImgFrame> inDataPtr, std::string camera_name) {
    std::deque<RosMsg> opMsgs;

    int infoSubCount = 0, mainSubCount = 0;
    if(_isImageMessage) {
        infoSubCount = _cameraInfoPublisher.at(camera_name)->getNumSubscribers();
    }
    mainSubCount = _rosPublisher.at(camera_name)->getNumSubscribers();

    if(!_lazyPublisher || (mainSubCount > 0 || infoSubCount > 0)) {
        _converter(inDataPtr, opMsgs);

        while(opMsgs.size()) {
            RosMsg currMsg = opMsgs.front();
            if(mainSubCount > 0) {
                _rosPublisher.at(camera_name)->publish(currMsg);
            }

            if(infoSubCount > 0) {
                // if (_isImageMessage){
                //     _camInfoFrameId = curr.header.frame_id
                // }
                auto localCameraInfo = _camInfoManager.at(camera_name)->getCameraInfo();
                localCameraInfo.header.seq = currMsg.header.seq;
                localCameraInfo.header.stamp = currMsg.header.stamp;
                localCameraInfo.header.frame_id = currMsg.header.frame_id;
                _cameraInfoPublisher.at(camera_name)->publish(localCameraInfo);
            }
            opMsgs.pop_front();
        }
    }
}

template <class RosMsg>
SyncBridgePublisher<RosMsg>::~SyncBridgePublisher() {
    if(_readingThread.joinable()) _readingThread.join();
}

}  // namespace ros

namespace rosBridge = ros;

}  // namespace dai
