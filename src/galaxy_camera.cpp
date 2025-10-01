//
// Created by qiayuan on 6/27/20.
//

#include <galaxy_camera.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <utility>

namespace {

std::string ToLowerCopy(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

struct PixelFormatOption {
    const char* key;
    const char* canonical;
    int64_t value;
};

struct PixelFormatSelection {
    int64_t value;
    std::string canonical;
    bool recognized;
};

PixelFormatSelection ResolvePixelFormat(const std::string& requested) {
    static const std::array<PixelFormatOption, 3> kPixelFormats = {{
            {"bayerrg8", "BayerRG8", GX_PIXEL_FORMAT_BAYER_RG8},
            {"bayerrg12", "BayerRG12", GX_PIXEL_FORMAT_BAYER_RG12},
            {"bayerrg16", "BayerRG16", GX_PIXEL_FORMAT_BAYER_RG16},
    }};

    std::string key = ToLowerCopy(requested);
    for (const auto& option : kPixelFormats) {
        if (key == option.key) {
            return {option.value, option.canonical, true};
        }
    }

    return {GX_PIXEL_FORMAT_BAYER_RG8, "BayerRG8", key.empty() || key == "bayerrg8"};
}

}  // namespace

namespace galaxy_camera {

    GalaxyCamera::GalaxyCamera(const ros::NodeHandle &nh,
                               image_transport::CameraPublisher &pub,
                               sensor_msgs::CameraInfo info,
                               uint32_t height, uint32_t width,
                               uint32_t step,
                               uint32_t offset_x, uint32_t offset_y,
                               const std::string &encoding,
                               const std::string &camera_ip,
                               const std::string &camera_pixel_format) {
        nh_ = nh;
        pub_ = pub;
        info_ = std::move(info);
        image_.height = height;
        image_.width = width;
        image_.step = step;                  // expected width * 3 (bgr8/rgb8)
        image_.data.resize(height * step);
        image_.encoding = encoding;          // e.g., bgr8
        img = new char[height * width * 3];
        raw8_buf = new char[height * width];
        assert(GXInitLib() == GX_STATUS_SUCCESS); // Initializes the library.
        uint32_t device_num = 0;
        GXUpdateDeviceList(&device_num, 1000);
        assert(device_num == 1); // TODO add multi camera support.
        // Opens the device.
        GX_OPEN_PARAM open_param;
        open_param.accessMode = GX_ACCESS_EXCLUSIVE;
        open_param.openMode = GX_OPEN_IP;
        open_param.pszContent = (char *)camera_ip.c_str();
        //open_param.openMode = GX_OPEN_INDEX;
        //open_param.pszContent = (char *) "1";
        
        // Get handle
        assert(GXOpenDevice(&open_param, &dev_handle_) == GX_STATUS_SUCCESS);
        ROS_INFO("Camera Opened with IP %s", camera_ip.c_str());

        PixelFormatSelection selection = ResolvePixelFormat(camera_pixel_format);
        if (!selection.recognized) {
            if (!camera_pixel_format.empty()) {
                ROS_WARN("Unknown camera pixel format '%s'. Falling back to %s.",
                         camera_pixel_format.c_str(), selection.canonical.c_str());
            } else {
                ROS_INFO("No camera pixel format provided. Using default %s.",
                         selection.canonical.c_str());
            }
        }

        GX_STATUS status = GXSetEnum(dev_handle_, GX_ENUM_PIXEL_FORMAT, selection.value);
        if (status != GX_STATUS_SUCCESS) {
            if (selection.value != GX_PIXEL_FORMAT_BAYER_RG8) {
                ROS_WARN("Failed to set pixel format '%s' (status=%d). Trying BayerRG8 instead.",
                         selection.canonical.c_str(), status);
                status = GXSetEnum(dev_handle_, GX_ENUM_PIXEL_FORMAT, GX_PIXEL_FORMAT_BAYER_RG8);
                if (status == GX_STATUS_SUCCESS) {
                    selection = {GX_PIXEL_FORMAT_BAYER_RG8, "BayerRG8", true};
                } else {
                    ROS_ERROR("Failed to set fallback pixel format BayerRG8 (status=%d).", status);
                }
            } else {
                ROS_ERROR("Failed to set pixel format '%s' (status=%d).", selection.canonical.c_str(), status);
            }
        }

        requested_pixel_format_ = selection.canonical;
        ROS_INFO("Camera pixel format set to %s", requested_pixel_format_.c_str());

        assert(GXSetInt(dev_handle_, GX_INT_WIDTH, width) == GX_STATUS_SUCCESS);
        assert(GXSetInt(dev_handle_, GX_INT_HEIGHT, height) == GX_STATUS_SUCCESS);
        assert(GXSetInt(dev_handle_, GX_INT_OFFSET_X, offset_x) == GX_STATUS_SUCCESS);
        assert(GXSetInt(dev_handle_, GX_INT_OFFSET_Y, offset_y) == GX_STATUS_SUCCESS);

        GXRegisterCaptureCallback(dev_handle_,
                                  nullptr,
                                  GalaxyCamera::onFrameCB);
        GXStreamOn(dev_handle_);
        ROS_INFO("Stream On.");

        // write config to camera
        ROS_INFO("Writing parameters to camera...");
        writeConfig();
        ROS_INFO("Done.");
    }

    void GalaxyCamera::onFrameCB(GX_FRAME_CALLBACK_PARAM *pFrame) {
        if (pFrame->status == GX_FRAME_STATUS_SUCCESS) {

            // If device outputs 12/16-bit Bayer, convert to Raw8 then demosaic to RGB24
            if (pFrame->nPixelFormat == GX_PIXEL_FORMAT_BAYER_RG12 ||
                pFrame->nPixelFormat == GX_PIXEL_FORMAT_BAYER_RG16 ||
                pFrame->nPixelFormat == GX_PIXEL_FORMAT_BAYER_BG12 ||
                pFrame->nPixelFormat == GX_PIXEL_FORMAT_BAYER_BG16 ||
                pFrame->nPixelFormat == GX_PIXEL_FORMAT_BAYER_GR12 ||
                pFrame->nPixelFormat == GX_PIXEL_FORMAT_BAYER_GR16 ||
                pFrame->nPixelFormat == GX_PIXEL_FORMAT_BAYER_GB12 ||
                pFrame->nPixelFormat == GX_PIXEL_FORMAT_BAYER_GB16) {
                // Most Daheng 12-bit streams are left-aligned in 16-bit container
                DxRaw16toRaw8((void*)pFrame->pImgBuf, (void*)raw8_buf,
                              pFrame->nWidth, pFrame->nHeight, DX_BIT_4_11);
                // Demosaic with RGGB pattern, publish as bgr8 by using BGR channel order
                DxRaw8toRGB24Ex((void*)raw8_buf, (void*)img,
                                pFrame->nWidth, pFrame->nHeight,
                                RAW2RGB_NEIGHBOUR, BAYERRG, false, DX_ORDER_BGR);
            } else {
                // Assume Raw8 Bayer input; demosaic directly
                DxRaw8toRGB24Ex((void*)pFrame->pImgBuf, (void*)img,
                                pFrame->nWidth, pFrame->nHeight,
                                RAW2RGB_NEIGHBOUR, BAYERRG, false, DX_ORDER_BGR);
            }
            memcpy((char *) (&image_.data[0]), img, image_.step * image_.height);
            ros::Time now = ros::Time().now();
            image_.header.stamp = now;
            info_.header.stamp = now;
            pub_.publish(image_, info_);
        }
    }


    void GalaxyCamera::writeConfig() {
        double frame_rate,
                exposure_max, exposure_min, exposure_value,
                gain_min, gain_max, gain_value,
                black_value, white_value;
        bool exposure_auto, gain_auto, black_auto, white_auto;
        int white_selector;

        // get parameters
        nh_.param("frame_rate", frame_rate, 30.0);

        nh_.param("exposure_auto", exposure_auto, false);
        nh_.param("exposure_min", exposure_min, 20.0);
        nh_.param("exposure_max", exposure_max, 100000.0);
        nh_.param("exposure_value", exposure_value, 50000.0);

        nh_.param("gain_auto", gain_auto, false);
        nh_.param("gain_min", gain_min, 0.0);
        nh_.param("gain_max", gain_max, 24.0);
        nh_.param("gain_value", gain_value, 0.0);

        nh_.param("black_auto", black_auto, false);
        nh_.param("black_value", black_value, 2.0);

        nh_.param("white_auto", white_auto, false);
        nh_.param("white_selector", white_selector, 0);
        nh_.param("white_value", white_value, 1.0);


        // write to camera
        // Frame Rate
        GXSetEnum(dev_handle_, GX_ENUM_ACQUISITION_FRAME_RATE_MODE,
                  GX_ACQUISITION_FRAME_RATE_MODE_ON);
        GXSetFloat(dev_handle_, GX_FLOAT_ACQUISITION_FRAME_RATE, frame_rate);

        // Exposure
        if (exposure_auto) {
            GXSetFloat(dev_handle_, GX_FLOAT_AUTO_EXPOSURE_TIME_MAX, exposure_max);
            GXSetFloat(dev_handle_, GX_FLOAT_AUTO_EXPOSURE_TIME_MIN, exposure_min);
            GXSetEnum(dev_handle_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS);

        } else {
            GXSetEnum(dev_handle_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);
            GXSetFloat(dev_handle_, GX_FLOAT_EXPOSURE_TIME, exposure_value);
        }

        // Gain
        if (gain_auto) {
            GXSetFloat(dev_handle_, GX_FLOAT_AUTO_GAIN_MIN, gain_min);
            GXSetFloat(dev_handle_, GX_FLOAT_AUTO_GAIN_MAX, gain_max);
            GXSetEnum(dev_handle_, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_CONTINUOUS);
        } else {
            GXSetEnum(dev_handle_, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_OFF);
            GXSetFloat(dev_handle_, GX_FLOAT_GAIN, gain_value);
        }

        // Black level
        if (black_auto) {
            GXSetEnum(dev_handle_, GX_ENUM_BLACKLEVEL_AUTO, GX_BLACKLEVEL_AUTO_CONTINUOUS);
        } else {
            GXSetEnum(dev_handle_, GX_ENUM_BLACKLEVEL_AUTO, GX_BLACKLEVEL_AUTO_OFF);
            GXSetFloat(dev_handle_, GX_FLOAT_BLACKLEVEL, black_value);
        }
        // Balance White
        switch (white_selector) {
            case 0:
                GXSetEnum(dev_handle_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED);
                break;
            case 1:
                GXSetEnum(dev_handle_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_GREEN);
                break;
            case 2:
                GXSetEnum(dev_handle_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE);
                break;
        }
        if (white_auto) {
            GXSetEnum(dev_handle_, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);
        } else {
            GXSetEnum(dev_handle_, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_OFF);
            GXSetFloat(dev_handle_, GX_FLOAT_BALANCE_RATIO, white_value);
        }
    }

    GalaxyCamera::~GalaxyCamera() {
        GXStreamOff(dev_handle_);
        GXUnregisterCaptureCallback(dev_handle_);
        GXCloseDevice(dev_handle_);
        GXCloseLib();
        if (img) { delete[] img; img = nullptr; }
        if (raw8_buf) { delete[] raw8_buf; raw8_buf = nullptr; }
    }

    char *GalaxyCamera::img;
    char *GalaxyCamera::raw8_buf;
    sensor_msgs::Image GalaxyCamera::image_;
    image_transport::CameraPublisher GalaxyCamera::pub_;
    sensor_msgs::CameraInfo GalaxyCamera::info_;
}
