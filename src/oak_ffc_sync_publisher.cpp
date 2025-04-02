#include <map>
#include <vector>

#include "camera_info_manager/camera_info_manager.h"
#include "ros/node_handle.h"
#include "sensor_msgs/Image.h"
#include "stereo_msgs/DisparityImage.h"

// Inludes common necessary includes for development using depthai library
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/DisparityConverter.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/ImuConverter.hpp"
#include "SyncBridgePublisher.hpp"

#define FPS 10

std::map<std::string, std::tuple<dai::node::MonoCamera::Properties::SensorResolution, int, int>> mono_res_opts = {
    {"400p", {dai::node::MonoCamera::Properties::SensorResolution::THE_400_P, 640, 400}},
    {"480p", {dai::node::MonoCamera::Properties::SensorResolution::THE_480_P, 640, 480}},
    {"720p", {dai::node::MonoCamera::Properties::SensorResolution::THE_720_P, 1280, 720}},
    {"800p", {dai::node::MonoCamera::Properties::SensorResolution::THE_800_P, 1280, 800}},
    {"1200p", {dai::node::MonoCamera::Properties::SensorResolution::THE_1200_P, 1920, 1200}}
};

std::map<std::string, std::tuple<dai::node::ColorCamera::Properties::SensorResolution, int, int>> color_res_opts = {
    {"720p", {dai::node::ColorCamera::Properties::SensorResolution::THE_720_P, 1280, 720}},
    {"800p", {dai::node::ColorCamera::Properties::SensorResolution::THE_800_P, 1280, 800}},
    {"1080p", {dai::node::ColorCamera::Properties::SensorResolution::THE_1080_P, 1920, 1080}},
    {"1200p", {dai::node::ColorCamera::Properties::SensorResolution::THE_1200_P, 1920, 1200}},
    {"4k", {dai::node::ColorCamera::Properties::SensorResolution::THE_4_K, 3840, 2160}}
    // {"5mp": dai::node::ColorCamera::Properties::SensorResolution::THE_5_MP},
    // {"12mp": dai::node::ColorCamera::Properties::SensorResolution::THE_12_MP},
    // {"48mp": dai::node::ColorCamera::Properties::SensorResolution::THE_48_MP}
};

bool auto_exposure_enable;
int exposure_time;
int iso;

std::map<std::string, dai::CameraBoardSocket> cam_socket_opts = {
    {"CAM_A", dai::CameraBoardSocket::CAM_A},
    {"CAM_B", dai::CameraBoardSocket::CAM_B},
    {"CAM_C", dai::CameraBoardSocket::CAM_C},
    {"CAM_D", dai::CameraBoardSocket::CAM_D}
};

std::tuple<dai::Pipeline, int, int >createPipeline(std::string camera_type, std::string resolution) {
    dai::Pipeline pipeline;
    int width, height;
    auto sync = pipeline.create<dai::node::Sync>();
    sync->setSyncThreshold(std::chrono::milliseconds(50));
    auto xSyncOut = pipeline.create<dai::node::XLinkOut>();
    xSyncOut->setStreamName("sync");
    sync->out.link(xSyncOut->input);

    for (const auto& opts: cam_socket_opts) {
        // auto xout = pipeline.create<dai::node::XLinkOut>();
        // xout->setStreamName(opts.first);
        if (camera_type == "color") {
            dai::node::ColorCamera::Properties::SensorResolution rgbResolution;
            auto camera = pipeline.create<dai::node::ColorCamera>();
            auto it = color_res_opts.find(resolution);
            if (it != color_res_opts.end()) {
                std::tie(rgbResolution, width, height) = it->second;
                camera->setResolution(rgbResolution);
            }
            // camera->isp.link(xout->input);
            camera->setBoardSocket(opts.second);
            camera->setFps(FPS);

            if (auto_exposure_enable) {
                camera->initialControl.setAutoExposureEnable();
            } else {
                camera->initialControl.setManualExposure(exposure_time, iso);
            }

            camera->isp.link(sync->inputs[opts.first]);
        } else {
            dai::node::MonoCamera::Properties::SensorResolution monoResolution;
            auto camera = pipeline.create<dai::node::MonoCamera>();
            auto it = mono_res_opts.find(resolution);
            if (it != mono_res_opts.end()) {
                std::tie(monoResolution, width, height) = it->second;
                camera->setResolution(monoResolution);
            }
            camera->out.link(sync->inputs[opts.first]);
            camera->setBoardSocket(opts.second);
            camera->setFps(FPS);
            camera->initialControl.setFrameSyncMode(dai::CameraControl::FrameSyncMode::INPUT);

            if (auto_exposure_enable) {
                camera->initialControl.setAutoExposureEnable();
            } else {
                camera->initialControl.setManualExposure(exposure_time, iso);
            }
        }
    }
    auto imu = pipeline.create<dai::node::IMU>();
    auto xImuOut = pipeline.create<dai::node::XLinkOut>();
    xImuOut->setStreamName("imu");
    imu->enableIMUSensor(dai::IMUSensor::ROTATION_VECTOR, 400);
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(10);
    imu->out.link(xImuOut->input);

    auto script = pipeline.create<dai::node::Script>();
    script->setProcessor(dai::ProcessorType::LEON_CSS);
    script->setScript(std::string(R"(# coding=utf-8
import time
import GPIO

# Script static arguments
fps = )") + std::to_string(FPS) + std::string(R"(
calib = Device.readCalibration2().getEepromData()
prodName  = calib.productName
boardName = calib.boardName
boardRev  = calib.boardRev

node.warn(f'Product name  : {prodName}')
node.warn(f'Board name    : {boardName}')
node.warn(f'Board revision: {boardRev}')

revision = -1
# Very basic parsing here, TODO improve
if len(boardRev) >= 2 and boardRev[0] == 'R':
    revision = int(boardRev[1])
node.warn(f'Parsed revision number: {revision}')

# Defaults for OAK-FFC-4P older revisions (<= R5)
GPIO_FSIN_2LANE = 41  # COM_AUX_IO2
GPIO_FSIN_4LANE = 40
GPIO_FSIN_MODE_SELECT = 6  # Drive 1 to tie together FSIN_2LANE and FSIN_4LANE

if revision >= 6:
    GPIO_FSIN_2LANE = 41  # still COM_AUX_IO2, no PWM capable
    GPIO_FSIN_4LANE = 42  # also not PWM capable
    GPIO_FSIN_MODE_SELECT = 38  # Drive 1 to tie together FSIN_2LANE and FSIN_4LANE
# Note: on R7 GPIO_FSIN_MODE_SELECT is pulled up, driving high isn't necessary (but fine to do)

# GPIO initialization
GPIO.setup(GPIO_FSIN_2LANE, GPIO.OUT)
GPIO.write(GPIO_FSIN_2LANE, 0)

GPIO.setup(GPIO_FSIN_4LANE, GPIO.IN)

GPIO.setup(GPIO_FSIN_MODE_SELECT, GPIO.OUT)
GPIO.write(GPIO_FSIN_MODE_SELECT, 1)

period = 1 / fps
active = 0.001

node.warn(f'FPS: {fps}  Period: {period}')

withInterrupts = False
if withInterrupts:
    node.critical(f'[TODO] FSYNC with timer interrupts (more precise) not implemented')
else:
    overhead = 0.003  # Empirical, TODO add thread priority option!
    while True:
        GPIO.write(GPIO_FSIN_2LANE, 1)
        time.sleep(active)
        GPIO.write(GPIO_FSIN_2LANE, 0)
        time.sleep(period - active - overhead)
)"));
    return std::make_tuple(pipeline, width, height);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "oak_ffc_sync");
    ros::NodeHandle pnh("~");

    dai::Pipeline pipeline;
    int width, height;
    std::string tfPrefix, camera_type = "color";
    std::string cameraParamUri;
    int badParams = 0;
    std::string resolution = "1200p";

    badParams += !pnh.getParam("camera_param_uri", cameraParamUri);
    badParams += !pnh.getParam("tf_prefix", tfPrefix);
    badParams += !pnh.getParam("camera_type", camera_type);
    badParams += !pnh.getParam("resolution", resolution);
    badParams += !pnh.getParam("auto_exposure_enable", auto_exposure_enable);
    badParams += !pnh.getParam("exposure_time", exposure_time);
    badParams += !pnh.getParam("iso", iso);

    if(badParams > 0) {
        std::cout << " Bad parameters -> " << badParams << std::endl;
        throw std::runtime_error("Couldn't find %d of the parameters");
    }

    std::tie(pipeline, width, height) = createPipeline(camera_type, resolution);

    dai::Device device(pipeline);

    auto calibrationHandler = device.readCalibration();

    auto boardName = calibrationHandler.getEepromData().boardName;
    if(height > 480 && boardName == "OAK-D-LITE") {
        width = 640;
        height = 480;
    }

    std::string color_url = cameraParamUri + "/" + "ffc_cam.yaml";

    auto imuQueue = device.getOutputQueue("imu", FPS, false);
    dai::rosBridge::ImuConverter imuConverter(tfPrefix + "_imu_frame", dai::ros::ImuSyncMethod::COPY, 0, 0, 0, 0, true);
    dai::rosBridge::BridgePublisher<sensor_msgs::Imu, dai::IMUData> imuPublish(
        imuQueue,
        pnh,
        std::string("imu"),
        std::bind(&dai::rosBridge::ImuConverter::toRosMsg, &imuConverter, std::placeholders::_1, std::placeholders::_2),
        FPS
    );
    imuPublish.addPublisherCallback();

    std::map<std::string, std::string> rosTocips = {
        {"CAM_A", "CAM_A/image"},
        {"CAM_B", "CAM_B/image"},
        {"CAM_C", "CAM_C/image"},
        {"CAM_D", "CAM_D/image"}
    };

    auto syncQueue = device.getOutputQueue("sync", FPS, false);
    std::unique_ptr<dai::rosBridge::ImageConverter> converter = std::make_unique<dai::rosBridge::ImageConverter>(tfPrefix + "_rgb_camera_optical_frame", true);
    try {
        auto cameraInfo = converter->calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A, width, height);
        dai::rosBridge::SyncBridgePublisher<sensor_msgs::Image> camPublish(
                        syncQueue,
                        pnh,
                        rosTocips,
                        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, converter.get(), std::placeholders::_1, std::placeholders::_2),
                        FPS,
                        cameraInfo);
        camPublish.startPublisherThread();
    } catch(const std::exception& e) {
        std::cerr << "\033[31m" << "[warning] The camera is not calibrated, please calibrate it before use!" << "\033[0m" << '\n';
        dai::rosBridge::SyncBridgePublisher<sensor_msgs::Image> camPublish(
                        syncQueue,
                        pnh,
                        rosTocips,
                        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, converter.get(), std::placeholders::_1, std::placeholders::_2),
                        FPS,
                        color_url);
        camPublish.startPublisherThread();
    }

    ros::spin();
    return 0;
}
