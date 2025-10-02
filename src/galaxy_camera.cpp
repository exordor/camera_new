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
        double configured_frame_rate = 0.0;
        nh_.param("frame_rate", configured_frame_rate, 5.0);
        if (configured_frame_rate <= 0.0) {
            configured_frame_rate = 5.0;
        }
        const double frame_period_ms = 1000.0 / configured_frame_rate;

        int resend_timeout_param = 500;
        nh_.param("gev_resend_timeout_ms", resend_timeout_param, 500);
        if (resend_timeout_param <= 0) {
            resend_timeout_param = 500;
        }
        int64_t resend_timeout_ms = static_cast<int64_t>(resend_timeout_param);

        int packet_timeout_param = -1;
        nh_.param("gev_packet_timeout_ms", packet_timeout_param, -1);
        int64_t packet_timeout_ms = packet_timeout_param > 0 ?
                static_cast<int64_t>(packet_timeout_param) : 1000;
        if (packet_timeout_ms < 200) {
            packet_timeout_ms = 200;
        }

        int block_timeout_param = -1;
        nh_.param("gev_block_timeout_ms", block_timeout_param, -1);
        int64_t base_block_timeout_ms = block_timeout_param > 0 ?
                static_cast<int64_t>(block_timeout_param) :
                static_cast<int64_t>(frame_period_ms * 5.0);
        if (base_block_timeout_ms < 1000) {
            base_block_timeout_ms = 1000;
        } else if (base_block_timeout_ms > 60000) {
            base_block_timeout_ms = 60000;
        }

        int packet_delay_param = 0;
        nh_.param("gev_packet_delay_us", packet_delay_param, 0);
        int64_t packet_delay = packet_delay_param >= 0 ?
                static_cast<int64_t>(packet_delay_param) : 0;

        int socket_buffer_param = -1;
        nh_.param("gev_socket_buffer_kb", socket_buffer_param, -1);
        int64_t socket_buffer_kb = socket_buffer_param > 0 ?
                static_cast<int64_t>(socket_buffer_param) : 8192;

        ROS_INFO("Target frame rate: %.2f fps (period %.2f ms)",
                 configured_frame_rate, frame_period_ms);
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

        // Configure GigE Vision stream parameters for better resilience under network load
        ROS_INFO("Configuring GigE Vision stream parameters...");
        
        // Enable packet resend mode for lost packets (critical when sharing network with LiDAR)
        GX_STATUS gx_status;
        gx_status = GXSetEnum(dev_handle_, GX_DS_ENUM_RESEND_MODE, GX_DS_RESEND_MODE_ON);
        if (gx_status == GX_STATUS_SUCCESS) {
            ROS_INFO("✓ Packet resend enabled");

            gx_status = GXSetInt(dev_handle_, GX_DS_INT_RESEND_TIMEOUT, resend_timeout_ms);
            if (gx_status == GX_STATUS_SUCCESS) {
                ROS_INFO("✓ Resend timeout set to %lldms",
                         static_cast<long long>(resend_timeout_ms));
            } else {
                ROS_WARN("Could not set resend timeout (status=%d)", gx_status);
            }

            gx_status = GXSetInt(dev_handle_, GX_DS_INT_PACKET_TIMEOUT, packet_timeout_ms);
            if (gx_status == GX_STATUS_SUCCESS) {
                ROS_INFO("✓ Packet timeout set to %lldms",
                         static_cast<long long>(packet_timeout_ms));
            } else {
                ROS_WARN("Could not set packet timeout (status=%d)", gx_status);
            }

            int64_t payload_size = 0;
            int64_t negotiated_packet_size = 0;
            int64_t estimated_packets = 0;
            int64_t target_wait_packets = 0;
            GX_INT_RANGE wait_packet_range{};

            if (GXGetInt(dev_handle_, GX_INT_PAYLOAD_SIZE, &payload_size) != GX_STATUS_SUCCESS) {
                payload_size = 0;
            }

            if (GXGetInt(dev_handle_, GX_INT_GEV_PACKETSIZE, &negotiated_packet_size) != GX_STATUS_SUCCESS ||
                negotiated_packet_size <= 0) {
                negotiated_packet_size = 1500;
            }

            if (payload_size > 0 && negotiated_packet_size > 0) {
                estimated_packets = (payload_size + negotiated_packet_size - 1) / negotiated_packet_size;
                int64_t safety_margin = std::max<int64_t>(estimated_packets / 10, 64);
                target_wait_packets = estimated_packets + safety_margin;
            }

            if (GXGetIntRange(dev_handle_, GX_DS_INT_MAX_WAIT_PACKET_COUNT, &wait_packet_range) == GX_STATUS_SUCCESS) {
                if (target_wait_packets == 0) {
                    int64_t default_wait = static_cast<int64_t>(1000);
                    if (default_wait < wait_packet_range.nMin) {
                        default_wait = wait_packet_range.nMin;
                    } else if (default_wait > wait_packet_range.nMax) {
                        default_wait = wait_packet_range.nMax;
                    }
                    target_wait_packets = default_wait;
                } else {
                    if (target_wait_packets < wait_packet_range.nMin) {
                        target_wait_packets = wait_packet_range.nMin;
                    } else if (target_wait_packets > wait_packet_range.nMax) {
                        target_wait_packets = wait_packet_range.nMax;
                    }
                }
            } else if (target_wait_packets == 0) {
                target_wait_packets = 1000;
            }

            gx_status = GXSetInt(dev_handle_, GX_DS_INT_MAX_WAIT_PACKET_COUNT, target_wait_packets);
            if (gx_status == GX_STATUS_SUCCESS) {
                ROS_INFO("✓ Max wait packet count set to %lld (payload=%lld bytes, packet=%lld bytes, estimated packets=%lld)",
                         static_cast<long long>(target_wait_packets),
                         static_cast<long long>(payload_size),
                         static_cast<long long>(negotiated_packet_size),
                         static_cast<long long>(estimated_packets));
            } else {
                ROS_WARN("Could not set max wait packet count (status=%d)", gx_status);
            }

            int64_t block_timeout_ms = base_block_timeout_ms;
            if (estimated_packets > 0 && packet_delay > 0) {
                int64_t assembly_time_ms = (packet_delay * estimated_packets) / 1000;
                if (assembly_time_ms > block_timeout_ms) {
                    int64_t extra_margin = static_cast<int64_t>(frame_period_ms * 2.0);
                    if (extra_margin < 200) {
                        extra_margin = 200;
                    }
                    block_timeout_ms = assembly_time_ms + extra_margin;
                    if (block_timeout_ms > 60000) {
                        block_timeout_ms = 60000;
                    }
                }
            }

            gx_status = GXSetInt(dev_handle_, GX_DS_INT_BLOCK_TIMEOUT, block_timeout_ms);
            if (gx_status == GX_STATUS_SUCCESS) {
                ROS_INFO("✓ Block timeout set to %lldms",
                         static_cast<long long>(block_timeout_ms));
            } else {
                ROS_WARN("Could not set block timeout (status=%d)", gx_status);
            }

            gx_status = GXSetInt(dev_handle_, GX_DS_INT_SOCKET_BUFFER_SIZE, socket_buffer_kb);
            if (gx_status == GX_STATUS_SUCCESS) {
                ROS_INFO("✓ Socket buffer size set to %lldKB",
                         static_cast<long long>(socket_buffer_kb));
            } else {
                ROS_WARN("Could not set socket buffer size (status=%d)", gx_status);
            }
        } else {
            ROS_WARN("Could not enable packet resend (status=%d) - may lose frames under network load", gx_status);
        }
        
        // Set inter-packet delay to reduce burst load (microseconds)
        // This spaces out packets to reduce collisions with LiDAR traffic
        // Inter-packet delay (microseconds) sourced from ROS parameters
        gx_status = GXSetInt(dev_handle_, GX_INT_GEV_PACKETDELAY, packet_delay);
        if (gx_status == GX_STATUS_SUCCESS) {
            if (packet_delay == 0) {
                ROS_INFO("✓ Inter-packet delay disabled (0µs)");
            } else {
                ROS_INFO("✓ Inter-packet delay set to %lldµs",
                         static_cast<long long>(packet_delay));
            }
        } else {
            ROS_WARN("Could not set packet delay (status=%d)", gx_status);
        }
        
        // Additional stream buffer tuning
        // Increase the number of frame buffers to handle packet arrival variance
        gx_status = GXSetInt(dev_handle_, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, 64);
        if (gx_status == GX_STATUS_SUCCESS) {
            ROS_INFO("✓ Stream buffer count set to 64");
        }
        
        // Increase the size of each transfer buffer
        gx_status = GXSetInt(dev_handle_, GX_DS_INT_STREAM_TRANSFER_SIZE, 256 * 1024);
        if (gx_status == GX_STATUS_SUCCESS) {
            ROS_INFO("✓ Stream transfer size set to 256KB");
        }
        
        // Set max packet size (may help with fragmentation)
        gx_status = GXSetInt(dev_handle_, GX_INT_GEV_PACKETSIZE, 1500);
        if (gx_status == GX_STATUS_SUCCESS) {
            ROS_INFO("✓ Packet size set to 1500 bytes");
        }
        
        ROS_INFO("GigE Vision stream configuration complete.");

        GXRegisterCaptureCallback(dev_handle_,
                                  nullptr,
                                  GalaxyCamera::onFrameCB);
        GXStreamOn(dev_handle_);
        ROS_INFO("Stream On.");
        
        // Query and log initial stream statistics
        int64_t resend_packet_count = 0;
        
        if (GXGetInt(dev_handle_, GX_DS_INT_RESEND_PACKET_COUNT, &resend_packet_count) == GX_STATUS_SUCCESS) {
            ROS_INFO("Initial resend packet count: %lld", (long long)resend_packet_count);
        } else {
            ROS_WARN("Could not query resend packet count - SDK may not support this feature");
        }

        // write config to camera
        ROS_INFO("Writing parameters to camera...");
        writeConfig();
        ROS_INFO("Done.");
    }

    void GalaxyCamera::onFrameCB(GX_FRAME_CALLBACK_PARAM *pFrame) {
        // Enhanced diagnostic logging for frame status
        static int frame_count = 0;
        static int success_count = 0;
        static int reported_incomplete_count = 0;
        static int dropped_incomplete_count = 0;
        static int recovered_count = 0;
        static ros::Time last_log = ros::Time::now();
        static int64_t total_expected_bytes = 0;
        static int64_t total_received_bytes = 0;

        frame_count++;

        int64_t expected_size = static_cast<int64_t>(pFrame->nWidth) * static_cast<int64_t>(pFrame->nHeight);
        switch (pFrame->nPixelFormat) {
            case GX_PIXEL_FORMAT_BAYER_RG12:
            case GX_PIXEL_FORMAT_BAYER_RG16:
            case GX_PIXEL_FORMAT_BAYER_BG12:
            case GX_PIXEL_FORMAT_BAYER_BG16:
            case GX_PIXEL_FORMAT_BAYER_GR12:
            case GX_PIXEL_FORMAT_BAYER_GR16:
            case GX_PIXEL_FORMAT_BAYER_GB12:
            case GX_PIXEL_FORMAT_BAYER_GB16:
                expected_size *= 2;
                break;
            default:
                break;
        }

        total_expected_bytes += expected_size;
        total_received_bytes += pFrame->nImgSize;

        auto maybe_flush_counters = [&]() {
            if ((ros::Time::now() - last_log).toSec() < 1.0) {
                return;
            }

            double data_loss_percent = 0.0;
            if (total_expected_bytes > 0) {
                double lost = static_cast<double>(total_expected_bytes - total_received_bytes);
                if (lost < 0.0) {
                    lost = 0.0;
                }
                data_loss_percent = 100.0 * lost / static_cast<double>(total_expected_bytes);
            }

            if (reported_incomplete_count > 0) {
                ROS_WARN("Frame status: Total=%d, Published=%d, SDKIncomplete=%d (Recovered=%d, Dropped=%d) | Data loss: %.1f%%",
                         frame_count, success_count, reported_incomplete_count, recovered_count, dropped_incomplete_count, data_loss_percent);
            } else {
                ROS_INFO("Frame status: Total=%d, Published=%d, SDKIncomplete=%d",
                         frame_count, success_count, reported_incomplete_count);
            }

            frame_count = 0;
            success_count = 0;
            reported_incomplete_count = 0;
            dropped_incomplete_count = 0;
            recovered_count = 0;
            total_expected_bytes = 0;
            total_received_bytes = 0;
            last_log = ros::Time::now();
        };

        const bool payload_missing = (pFrame->nImgSize < expected_size);
        const bool is_incomplete_status = (pFrame->status == GX_FRAME_STATUS_INCOMPLETE);

        if (pFrame->status != GX_FRAME_STATUS_SUCCESS) {
            reported_incomplete_count++;

            if (payload_missing || !is_incomplete_status) {
                dropped_incomplete_count++;

                if (dropped_incomplete_count == 1) {
                    const int64_t missing_bytes = expected_size - pFrame->nImgSize;
                    double missing_percent = 0.0;
                    if (expected_size > 0) {
                        missing_percent = 100.0 * static_cast<double>(missing_bytes) / static_cast<double>(expected_size);
                    }
                    ROS_ERROR("=== INCOMPLETE FRAME DETAILS ===");
                    ROS_ERROR("Frame ID: %lld", (long long)pFrame->nFrameID);
                    ROS_ERROR("Frame status: %d (GX_FRAME_STATUS_INCOMPLETE=-1)", pFrame->status);
                    ROS_ERROR("Image size received: %zu bytes", pFrame->nImgSize);
                    ROS_ERROR("Expected size: %lld bytes (Width=%d, Height=%d)",
                             (long long)expected_size, pFrame->nWidth, pFrame->nHeight);
                    ROS_ERROR("Bytes missing: %lld (%.1f%%)",
                             (long long)missing_bytes,
                             missing_percent);
                    ROS_ERROR("Pixel format: 0x%x", pFrame->nPixelFormat);
                    ROS_ERROR("Timestamp: %lld", (long long)pFrame->nTimestamp);
                    ROS_ERROR("===============================");
                }

                maybe_flush_counters();
                return;
            }

            recovered_count++;
            ROS_WARN_THROTTLE(5.0, "Frame %lld marked incomplete by SDK but payload complete; publishing anyway (recovered=%d)",
                              (long long)pFrame->nFrameID,
                              recovered_count);
        }

        success_count++;

        // If device outputs 12/16-bit Bayer, convert to Raw8 then demosaic to RGB24
        if (pFrame->nPixelFormat == GX_PIXEL_FORMAT_BAYER_RG12 ||
            pFrame->nPixelFormat == GX_PIXEL_FORMAT_BAYER_RG16 ||
            pFrame->nPixelFormat == GX_PIXEL_FORMAT_BAYER_BG12 ||
            pFrame->nPixelFormat == GX_PIXEL_FORMAT_BAYER_BG16 ||
            pFrame->nPixelFormat == GX_PIXEL_FORMAT_BAYER_GR12 ||
            pFrame->nPixelFormat == GX_PIXEL_FORMAT_BAYER_GR16 ||
            pFrame->nPixelFormat == GX_PIXEL_FORMAT_BAYER_GB12 ||
            pFrame->nPixelFormat == GX_PIXEL_FORMAT_BAYER_GB16) {
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

        maybe_flush_counters();
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
