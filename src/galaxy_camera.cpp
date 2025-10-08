//
// Created by qiayuan on 6/27/20.
//

#include <galaxy_camera.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <sstream>
#include <utility>
#include <vector>
#include <fstream>
#include <chrono>

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

const char* FrameStatusName(GX_FRAME_STATUS status) {
    switch (status) {
        case GX_FRAME_STATUS_SUCCESS:
            return "SUCCESS";
        case GX_FRAME_STATUS_INCOMPLETE:
            return "INCOMPLETE";
        case GX_FRAME_STATUS_INVALID_IMAGE_INFO:
            return "INVALID_INFO";
        default:
            return "UNKNOWN";
    }
}

struct StreamCounters {
    int64_t delivered_frame_count = -1;
    int64_t lost_frame_count = -1;
    int64_t incomplete_frame_count = -1;
    int64_t missing_blockid_count = -1;
    int64_t delivered_packet_count = -1;
    int64_t resend_packet_count = -1;
    int64_t rescued_packet_count = -1;
    int64_t unexpected_packet_count = -1;
    int64_t resend_command_count = -1;
};

int64_t SafeDelta(int64_t current, int64_t previous) {
    if (current < 0 || previous < 0) {
        return -1;
    }
    int64_t delta = current - previous;
    if (delta < 0) {
        delta = current;
    }
    return delta;
}

// Network interface statistics structure
struct NetworkInterfaceStats {
    int64_t rx_packets = 0;
    int64_t rx_bytes = 0;
    int64_t rx_errors = 0;
    int64_t rx_dropped = 0;
    int64_t tx_packets = 0;
    int64_t tx_bytes = 0;
    int64_t tx_errors = 0;
    int64_t tx_dropped = 0;
};

// Read network interface statistics from /sys/class/net/
NetworkInterfaceStats ReadNetworkStats(const std::string& interface_name) {
    NetworkInterfaceStats stats;
    
    auto read_stat = [&](const std::string& stat_name) -> int64_t {
        std::string path = "/sys/class/net/" + interface_name + "/statistics/" + stat_name;
        std::ifstream file(path);
        if (!file.is_open()) {
            return -1;
        }
        int64_t value = 0;
        file >> value;
        return value;
    };
    
    stats.rx_packets = read_stat("rx_packets");
    stats.rx_bytes = read_stat("rx_bytes");
    stats.rx_errors = read_stat("rx_errors");
    stats.rx_dropped = read_stat("rx_dropped");
    stats.tx_packets = read_stat("tx_packets");
    stats.tx_bytes = read_stat("tx_bytes");
    stats.tx_errors = read_stat("tx_errors");
    stats.tx_dropped = read_stat("tx_dropped");
    
    return stats;
}

// Detect network interface from camera IP
std::string DetectNetworkInterface(const std::string& camera_ip) {
    // Try common interface names
    std::vector<std::string> common_interfaces = {"eth0", "eth1", "enp0s31f6", "eno1", "enp2s0", "enp3s0"};
    
    for (const auto& iface : common_interfaces) {
        std::string path = "/sys/class/net/" + iface;
        std::ifstream test(path + "/operstate");
        if (test.is_open()) {
            std::string state;
            test >> state;
            if (state == "up") {
                return iface;
            }
        }
    }
    
    return "eth0";  // fallback
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
        
        // Detect network interface for monitoring
        network_interface_ = DetectNetworkInterface(camera_ip);
        ROS_INFO("Detected network interface for camera: %s", network_interface_.c_str());
        
        image_.height = height;
        image_.width = width;
        image_.step = step;                  // expected width * 3 (bgr8/rgb8)
        image_.data.resize(height * step);
        image_.encoding = encoding;          // e.g., bgr8
        // Note: img buffer removed - now converting directly to image_.data in processing thread
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

        int packet_size_param = -1;
        nh_.param("gev_packet_size", packet_size_param, -1);
        int64_t packet_size = packet_size_param > 0 ?
                static_cast<int64_t>(packet_size_param) : 1500;
        if (packet_size < 576) {
            packet_size = 576;  // Minimum safe UDP packet size
            ROS_WARN("Packet size too small, set to minimum 576 bytes");
        } else if (packet_size > 9000) {
            packet_size = 9000;  // Maximum for jumbo frames
            ROS_WARN("Packet size too large, capped at 9000 bytes");
        }

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
        
        // Set max packet size (configurable via ROS parameter)
        // Query the supported packet size range from the camera
        GX_INT_RANGE packet_size_range;
        if (GXGetIntRange(dev_handle_, GX_INT_GEV_PACKETSIZE, &packet_size_range) == GX_STATUS_SUCCESS) {
            ROS_INFO("Camera packet size range: %lld - %lld bytes",
                     static_cast<long long>(packet_size_range.nMin),
                     static_cast<long long>(packet_size_range.nMax));
            
            // Clamp to camera's supported range
            if (packet_size < packet_size_range.nMin) {
                packet_size = packet_size_range.nMin;
                ROS_WARN("Requested packet size below camera minimum, adjusted to %lld bytes",
                         static_cast<long long>(packet_size));
            } else if (packet_size > packet_size_range.nMax) {
                packet_size = packet_size_range.nMax;
                ROS_WARN("Requested packet size above camera maximum, adjusted to %lld bytes",
                         static_cast<long long>(packet_size));
            }
        } else {
            ROS_WARN("Could not query packet size range from camera");
        }
        
        gx_status = GXSetInt(dev_handle_, GX_INT_GEV_PACKETSIZE, packet_size);
        if (gx_status == GX_STATUS_SUCCESS) {
            ROS_INFO("✓ Packet size set to %lld bytes", static_cast<long long>(packet_size));
            
            // Verify the actual packet size set
            int64_t actual_packet_size = 0;
            if (GXGetInt(dev_handle_, GX_INT_GEV_PACKETSIZE, &actual_packet_size) == GX_STATUS_SUCCESS) {
                if (actual_packet_size != packet_size) {
                    ROS_WARN("Camera adjusted packet size to %lld bytes (requested %lld)",
                             static_cast<long long>(actual_packet_size),
                             static_cast<long long>(packet_size));
                }
            }
        } else {
            ROS_ERROR("Failed to set packet size to %lld bytes (status=%d)",
                      static_cast<long long>(packet_size), gx_status);
        }
        
        ROS_INFO("GigE Vision stream configuration complete.");

        GXRegisterCaptureCallback(dev_handle_,
                                  this,
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
        
        // ========== START PROCESSING THREAD FOR ZERO-COPY PASS-THROUGH ==========
        int max_queue_depth_param = 4;
        nh_.param("processing_queue_depth", max_queue_depth_param, 4);
        max_queue_depth_ = static_cast<size_t>(max_queue_depth_param);
        
        stop_processing_ = false;
        processing_thread_ = std::thread(&GalaxyCamera::ProcessingLoop, this);
        
        ROS_INFO("✓ Processing thread started (max queue depth: %zu)", max_queue_depth_);
        ROS_INFO("✓ Zero-copy pass-through mode enabled");
    }

    // ========== ZERO-COPY PASS-THROUGH IMPLEMENTATION ==========
    
    void GalaxyCamera::EnqueueFrameJob(FrameJob job) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        
        if (frame_queue_.size() >= max_queue_depth_) {
            // Queue full - drop oldest frame
            frame_queue_.pop_front();
            dropped_due_to_queue_++;
            
            if (dropped_due_to_queue_ % 10 == 1) {
                ROS_WARN("Processing queue full! Dropped %zu frames total. "
                         "Consider increasing queue_depth or optimizing processing.",
                         dropped_due_to_queue_);
            }
        }
        
        frame_queue_.push_back(std::move(job));
        queue_cv_.notify_one();
    }
    
    void GalaxyCamera::ProcessingLoop() {
        ROS_INFO("Processing thread started (thread ID: %lu)", pthread_self());
        
        while (ros::ok() && !stop_processing_) {
            FrameJob job;
            
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                queue_cv_.wait_for(lock, std::chrono::milliseconds(100),
                                   [this] { return !frame_queue_.empty() || stop_processing_; });
                
                if (stop_processing_) {
                    break;
                }
                
                if (frame_queue_.empty()) {
                    continue;
                }
                
                job = std::move(frame_queue_.front());
                frame_queue_.pop_front();
            }
            
            // ========== PROCESSING (runs in separate thread) ==========
            auto processing_start = std::chrono::high_resolution_clock::now();
            auto conversion_start = processing_start;
            
            // Convert image format
            if (job.pixel_format == GX_PIXEL_FORMAT_BAYER_RG12 ||
                job.pixel_format == GX_PIXEL_FORMAT_BAYER_RG16 ||
                job.pixel_format == GX_PIXEL_FORMAT_BAYER_BG12 ||
                job.pixel_format == GX_PIXEL_FORMAT_BAYER_BG16 ||
                job.pixel_format == GX_PIXEL_FORMAT_BAYER_GR12 ||
                job.pixel_format == GX_PIXEL_FORMAT_BAYER_GR16 ||
                job.pixel_format == GX_PIXEL_FORMAT_BAYER_GB12 ||
                job.pixel_format == GX_PIXEL_FORMAT_BAYER_GB16) {
                
                DxRaw16toRaw8((void*)job.raw_payload.data(), (void*)raw8_buf,
                              job.width, job.height, DX_BIT_4_11);
                DxRaw8toRGB24Ex((void*)raw8_buf, (void*)(&image_.data[0]),
                                job.width, job.height,
                                RAW2RGB_NEIGHBOUR, BAYERRG, false, DX_ORDER_BGR);
            } else {
                DxRaw8toRGB24Ex((void*)job.raw_payload.data(), (void*)(&image_.data[0]),
                                job.width, job.height,
                                RAW2RGB_NEIGHBOUR, BAYERRG, false, DX_ORDER_BGR);
            }
            
            auto conversion_end = std::chrono::high_resolution_clock::now();
            
            // Publish to ROS
            auto publish_start = conversion_end;
            image_.header.stamp = job.arrival_time;
            info_.header.stamp = job.arrival_time;
            pub_.publish(image_, info_);
            auto publish_end = std::chrono::high_resolution_clock::now();
            
            // Calculate timing
            std::chrono::duration<double, std::milli> conversion_duration = 
                conversion_end - conversion_start;
            std::chrono::duration<double, std::milli> publish_duration = 
                publish_end - publish_start;
            std::chrono::duration<double, std::milli> total_duration = 
                publish_end - processing_start;
            
            AccumulateProcessingStatistics(total_duration.count(),
                                          conversion_duration.count(),
                                          publish_duration.count());
        }
        
        ROS_INFO("Processing thread terminated");
    }
    
    void GalaxyCamera::AccumulateProcessingStatistics(double total_ms,
                                                      double conversion_ms,
                                                      double publish_ms) {
        std::lock_guard<std::mutex> lock(processing_stats_mutex_);
        
        processing_stats_.total_processing_time_ms += total_ms;
        processing_stats_.total_conversion_time_ms += conversion_ms;
        processing_stats_.total_publish_time_ms += publish_ms;
        
        if (total_ms > processing_stats_.max_processing_time_ms) {
            processing_stats_.max_processing_time_ms = total_ms;
        }
        if (total_ms < processing_stats_.min_processing_time_ms) {
            processing_stats_.min_processing_time_ms = total_ms;
        }
        
        processing_stats_.processed_frames++;
    }
    
    GalaxyCamera::ProcessingStatistics GalaxyCamera::ConsumeProcessingStatistics() {
        std::lock_guard<std::mutex> lock(processing_stats_mutex_);
        ProcessingStatistics stats = processing_stats_;
        
        // Reset for next period
        processing_stats_ = ProcessingStatistics();
        
        return stats;
    }

    void GalaxyCamera::onFrameCB(GX_FRAME_CALLBACK_PARAM *pFrame) {
        // ========== START: Callback Processing Time Measurement ==========
        auto callback_start_time = std::chrono::high_resolution_clock::now();
        
        static int frame_count = 0;
        static int success_count = 0;
        static int reported_incomplete_count = 0;
        static int dropped_incomplete_count = 0;
        static int recovered_count = 0;
        static ros::Time last_log = ros::Time::now();
        static int64_t total_expected_bytes = 0;
        static int64_t total_received_bytes = 0;
        
        // ========== Network Interface Statistics Monitoring ==========
        static NetworkInterfaceStats last_net_stats{};
        static NetworkInterfaceStats net_stats_delta{};
        static bool net_stats_initialized = false;
        static ros::Time last_net_stats_time = ros::Time::now();
        
        // ========== Callback Processing Time Statistics ==========
        static double total_callback_time_ms = 0.0;
        static double max_callback_time_ms = 0.0;
        static double min_callback_time_ms = 1e9;
        static int callback_count = 0;

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
        
        // Get self pointer early for use in lambda functions
        GalaxyCamera *self = static_cast<GalaxyCamera *>(pFrame->pUserParam);

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
            
            // Calculate average callback processing time
            double avg_callback_time_ms = 0.0;
            double avg_conversion_time_ms = 0.0;
            double avg_publish_time_ms = 0.0;
            if (callback_count > 0) {
                avg_callback_time_ms = total_callback_time_ms / callback_count;
            }
            
            // Get processing thread statistics
            ProcessingStatistics proc_stats;
            if (self) {
                proc_stats = self->ConsumeProcessingStatistics();
            }
            
            if (proc_stats.processed_frames > 0) {
                avg_conversion_time_ms = proc_stats.total_conversion_time_ms / proc_stats.processed_frames;
                avg_publish_time_ms = proc_stats.total_publish_time_ms / proc_stats.processed_frames;
            }

            // ========== COMPREHENSIVE BOTTLENECK DIAGNOSTICS ==========
            ROS_INFO("========== BOTTLENECK ANALYSIS REPORT ==========");
            
            // 1. Frame Processing Summary
            if (reported_incomplete_count > 0) {
                ROS_WARN("[1] FRAME STATUS: Total=%d, Published=%d, SDKIncomplete=%d (Recovered=%d, Dropped=%d) | Data loss: %.1f%%",
                         frame_count, success_count, reported_incomplete_count, recovered_count, dropped_incomplete_count, data_loss_percent);
            } else {
                ROS_INFO("[1] FRAME STATUS: Total=%d, Published=%d, SDKIncomplete=%d | Data loss: %.1f%%",
                         frame_count, success_count, reported_incomplete_count, data_loss_percent);
            }
            
            // 2. Network Interface Statistics
            if (net_stats_delta.rx_packets >= 0) {
                double net_rx_mbps = (net_stats_delta.rx_bytes * 8.0) / 1000000.0;
                ROS_INFO("[2] NETWORK INTERFACE (%s): RX packets=%lld (%.2f MB, %.1f Mbps), errors=%lld, dropped=%lld",
                         self ? self->network_interface_.c_str() : "unknown",
                         (long long)net_stats_delta.rx_packets,
                         net_stats_delta.rx_bytes / 1000000.0,
                         net_rx_mbps,
                         (long long)net_stats_delta.rx_errors,
                         (long long)net_stats_delta.rx_dropped);
                
                if (net_stats_delta.rx_errors > 0 || net_stats_delta.rx_dropped > 0) {
                    ROS_ERROR("[2] ⚠️  NETWORK LAYER ISSUES DETECTED! Errors=%lld, Dropped=%lld - CHECK NETWORK HARDWARE/DRIVERS",
                              (long long)net_stats_delta.rx_errors,
                              (long long)net_stats_delta.rx_dropped);
                }
            } else {
                ROS_WARN("[2] NETWORK INTERFACE: Statistics unavailable");
            }
            
            // 3. ROS Callback Processing Performance
            size_t queue_size = 0;
            size_t queue_drops = 0;
            if (self) {
                std::lock_guard<std::mutex> lock(self->queue_mutex_);
                queue_size = self->frame_queue_.size();
                queue_drops = self->dropped_due_to_queue_;
            }
            
            ROS_INFO("[3] CALLBACK (SDK->Queue): Avg=%.2f ms, Min=%.2f ms, Max=%.2f ms | Queue: %zu/%zu frames",
                     avg_callback_time_ms, min_callback_time_ms, max_callback_time_ms,
                     queue_size, self ? self->max_queue_depth_ : 0);
            
            if (proc_stats.processed_frames > 0) {
                double avg_processing_ms = proc_stats.total_processing_time_ms / proc_stats.processed_frames;
                ROS_INFO("[4] PROCESSING THREAD: Avg=%.2f ms (Conversion=%.2f ms, Publish=%.2f ms), Min=%.2f ms, Max=%.2f ms (frames=%d)",
                         avg_processing_ms, avg_conversion_time_ms, avg_publish_time_ms,
                         proc_stats.min_processing_time_ms, proc_stats.max_processing_time_ms,
                         proc_stats.processed_frames);
                
                if (avg_processing_ms > 30.0) {
                    ROS_WARN("[4] ⚠️  Processing thread is slow (%.2f ms) - may impact throughput", avg_processing_ms);
                }
            }
            
            if (queue_drops > 0) {
                ROS_ERROR("[3] ⚠️  QUEUE OVERFLOW! Dropped %zu frames due to full queue. Increase processing_queue_depth parameter.", queue_drops);
            }
            
            if (avg_callback_time_ms > 5.0) {
                ROS_WARN("[3] ⚠️  SDK callback is slow (%.2f ms) - should be < 5ms for zero-copy pass-through", avg_callback_time_ms);
            } else {
                ROS_INFO("[3] ✓ SDK callback fast (%.2f ms) - zero-copy pass-through working well", avg_callback_time_ms);
            }
            
            ROS_INFO("================================================");

            frame_count = 0;
            success_count = 0;
            reported_incomplete_count = 0;
            dropped_incomplete_count = 0;
            recovered_count = 0;
            total_expected_bytes = 0;
            total_received_bytes = 0;
            total_callback_time_ms = 0.0;
            max_callback_time_ms = 0.0;
            min_callback_time_ms = 1e9;
            callback_count = 0;
            last_log = ros::Time::now();
        };

        const bool payload_missing = (pFrame->nImgSize < expected_size);
        const bool is_incomplete_status = (pFrame->status == GX_FRAME_STATUS_INCOMPLETE);

        auto log_network_diagnostics = [&]() {
            std::ostringstream diag;
            diag << "Frame " << static_cast<long long>(pFrame->nFrameID)
                 << " status=" << FrameStatusName(pFrame->status) << '(' << pFrame->status << ')'
                 << " ts=" << static_cast<long long>(pFrame->nTimestamp)
                 << " bytes=" << static_cast<unsigned long long>(pFrame->nImgSize) << '/' << expected_size;

            if (payload_missing) {
                diag << " missing=" << (expected_size - static_cast<int64_t>(pFrame->nImgSize));
            }

            static int64_t last_frame_id_seen = -1;
            int64_t frame_gap = 0;
            if (last_frame_id_seen >= 0) {
                frame_gap = static_cast<int64_t>(pFrame->nFrameID) - last_frame_id_seen - 1;
            }
            last_frame_id_seen = static_cast<int64_t>(pFrame->nFrameID);

            diag << " gap=" << frame_gap;

            if (!self) {
                diag << " net=unavailable";
                ROS_INFO_STREAM(diag.str());
                if (frame_gap > 0) {
                    ROS_WARN_STREAM_THROTTLE(1.0, "Detected frame gap of "
                                                  << frame_gap << " block(s) before frame "
                                                  << static_cast<long long>(pFrame->nFrameID));
                }
                return;
            }

            auto value_to_string = [](int64_t value) -> std::string {
                return value >= 0 ? std::to_string(value) : std::string("n/a");
            };

            struct CounterBinding {
                int feature;
                int64_t StreamCounters::*member;
                const char *label;
            };

            constexpr double kCounterPollIntervalSec = 0.25;
            constexpr size_t kBindingCount = 9;

            static const std::array<CounterBinding, kBindingCount> kBindings = {{
                    {GX_DS_INT_DELIVERED_FRAME_COUNT, &StreamCounters::delivered_frame_count, "delivered_frames"},
                    {GX_DS_INT_LOST_FRAME_COUNT, &StreamCounters::lost_frame_count, "lost_frames"},
                    {GX_DS_INT_INCOMPLETE_FRAME_COUNT, &StreamCounters::incomplete_frame_count, "incomplete_frames"},
                    {GX_DS_INT_MISSING_BLOCKID_COUNT, &StreamCounters::missing_blockid_count, "missing_blockid"},
                    {GX_DS_INT_DELIVERED_PACKET_COUNT, &StreamCounters::delivered_packet_count, "delivered_packets"},
                    {GX_DS_INT_RESEND_PACKET_COUNT, &StreamCounters::resend_packet_count, "resend_packets"},
                    {GX_DS_INT_RESCUED_PACKED_COUNT, &StreamCounters::rescued_packet_count, "rescued_packets"},
                    {GX_DS_INT_UNEXPECTED_PACKED_COUNT, &StreamCounters::unexpected_packet_count, "unexpected_packets"},
                    {GX_DS_INT_RESEND_COMMAND_COUNT, &StreamCounters::resend_command_count, "resend_commands"},
            }};

            static std::array<bool, kBindingCount> disabled_features{};
            static StreamCounters last_stream_counters{};
            static StreamCounters cached_delta{};
            static bool stream_counters_initialized = false;
            static bool cached_delta_valid = false;
            static ros::Time last_counter_poll;

            const ros::Time now = ros::Time::now();
            const bool force_poll = payload_missing ||
                                    frame_gap > 0 ||
                                    (pFrame->status != GX_FRAME_STATUS_SUCCESS);
            const bool initial_poll = !stream_counters_initialized;
            const bool interval_elapsed = last_counter_poll.isZero() ||
                                          (now - last_counter_poll).toSec() >= kCounterPollIntervalSec;
            bool polled_this_frame = false;
            bool updated_this_poll = false;

            if (force_poll || initial_poll || interval_elapsed) {
                polled_this_frame = true;
                last_counter_poll = now;

                StreamCounters current;
                bool any_success = false;
                std::vector<const char *> newly_unsupported;
                std::vector<std::pair<const char *, GX_STATUS>> failed;
                StreamCounters previous = last_stream_counters;

                for (size_t i = 0; i < kBindingCount; ++i) {
                    const auto &binding = kBindings[i];
                    if (disabled_features[i]) {
                        continue;
                    }

                    int64_t value = 0;
                    GX_STATUS status = GXGetInt(self->dev_handle_, binding.feature, &value);
                    if (status == GX_STATUS_SUCCESS) {
                        current.*(binding.member) = value;
                        any_success = true;
                    } else {
                        current.*(binding.member) = -1;
                        if (status == GX_STATUS_NOT_IMPLEMENTED) {
                            disabled_features[i] = true;
                            newly_unsupported.push_back(binding.label);
                        } else {
                            failed.emplace_back(binding.label, status);
                        }
                    }
                }

                if (!newly_unsupported.empty()) {
                    std::ostringstream warn_stream;
                    warn_stream << "Stream counter(s) unavailable: ";
                    for (size_t i = 0; i < newly_unsupported.size(); ++i) {
                        if (i) {
                            warn_stream << ", ";
                        }
                        warn_stream << newly_unsupported[i];
                    }
                    ROS_WARN("%s", warn_stream.str().c_str());
                }

                if (!failed.empty()) {
                    std::ostringstream warn_stream;
                    warn_stream << "Failed to query stream counters: ";
                    for (size_t i = 0; i < failed.size(); ++i) {
                        if (i) {
                            warn_stream << ", ";
                        }
                        warn_stream << failed[i].first << "(status=" << failed[i].second << ")";
                    }
                    ROS_WARN_THROTTLE(5.0, "%s", warn_stream.str().c_str());
                }

                if (any_success) {
                    StreamCounters delta;
                    if (stream_counters_initialized) {
                        delta.delivered_frame_count =
                                SafeDelta(current.delivered_frame_count, previous.delivered_frame_count);
                        delta.lost_frame_count =
                                SafeDelta(current.lost_frame_count, previous.lost_frame_count);
                        delta.incomplete_frame_count =
                                SafeDelta(current.incomplete_frame_count, previous.incomplete_frame_count);
                        delta.missing_blockid_count =
                                SafeDelta(current.missing_blockid_count, previous.missing_blockid_count);
                        delta.delivered_packet_count =
                                SafeDelta(current.delivered_packet_count, previous.delivered_packet_count);
                        delta.resend_packet_count =
                                SafeDelta(current.resend_packet_count, previous.resend_packet_count);
                        delta.rescued_packet_count =
                                SafeDelta(current.rescued_packet_count, previous.rescued_packet_count);
                        delta.unexpected_packet_count =
                                SafeDelta(current.unexpected_packet_count, previous.unexpected_packet_count);
                        delta.resend_command_count =
                                SafeDelta(current.resend_command_count, previous.resend_command_count);
                    } else {
                        delta = current;
                    }

                    cached_delta = delta;
                    cached_delta_valid = true;
                    last_stream_counters = current;
                    stream_counters_initialized = true;

                    std::ostringstream raw_stream;
                    raw_stream << "GX counters updated: delivered=" << value_to_string(current.delivered_frame_count)
                               << ", lost=" << value_to_string(current.lost_frame_count)
                               << ", incomplete=" << value_to_string(current.incomplete_frame_count)
                               << ", missingId=" << value_to_string(current.missing_blockid_count)
                               << ", delivered_packets=" << value_to_string(current.delivered_packet_count)
                               << ", resend_packets=" << value_to_string(current.resend_packet_count)
                               << ", rescued_packets=" << value_to_string(current.rescued_packet_count)
                               << ", unexpected_packets=" << value_to_string(current.unexpected_packet_count)
                               << ", resend_commands=" << value_to_string(current.resend_command_count);
                    ROS_INFO_STREAM(raw_stream.str());
                    updated_this_poll = true;
                }

            }

            StreamCounters delta_display = cached_delta_valid ? cached_delta : StreamCounters{};

            diag << " frames(+"
                 << value_to_string(delta_display.delivered_frame_count)
                 << ", lost=" << value_to_string(delta_display.lost_frame_count)
                 << ", incomplete=" << value_to_string(delta_display.incomplete_frame_count)
                 << ", missingId=" << value_to_string(delta_display.missing_blockid_count)
                 << ')';

            diag << " packets(+"
                 << value_to_string(delta_display.delivered_packet_count)
                 << ", resend=" << value_to_string(delta_display.resend_packet_count)
                 << ", rescued=" << value_to_string(delta_display.rescued_packet_count)
                 << ", unexpected=" << value_to_string(delta_display.unexpected_packet_count)
                 << ')';

            diag << " resendCmd=" << value_to_string(delta_display.resend_command_count);

            if (cached_delta_valid) {
                diag << " totals(delivered=" << value_to_string(last_stream_counters.delivered_frame_count)
                     << ", lost=" << value_to_string(last_stream_counters.lost_frame_count)
                     << ", incomplete=" << value_to_string(last_stream_counters.incomplete_frame_count)
                     << ", missingId=" << value_to_string(last_stream_counters.missing_blockid_count)
                     << ", delivered_packets=" << value_to_string(last_stream_counters.delivered_packet_count)
                     << ", resend_packets=" << value_to_string(last_stream_counters.resend_packet_count)
                     << ", rescued_packets=" << value_to_string(last_stream_counters.rescued_packet_count)
                     << ", unexpected_packets=" << value_to_string(last_stream_counters.unexpected_packet_count)
                     << ", resend_commands=" << value_to_string(last_stream_counters.resend_command_count)
                     << ')';
            }

            if (!cached_delta_valid) {
                diag << " [no-counter-data]";
            } else if (updated_this_poll) {
                diag << " [polled]";
            } else if (polled_this_frame) {
                diag << " [poll-failed]";
            } else {
                diag << " [cached]";
            }

            ROS_INFO_STREAM(diag.str());

            if (frame_gap > 0) {
                ROS_WARN_STREAM_THROTTLE(1.0, "Detected frame gap of "
                                              << frame_gap << " block(s) before frame "
                                              << static_cast<long long>(pFrame->nFrameID)
                                              << " (missingBlockId delta="
                                              << value_to_string(delta_display.missing_blockid_count) << ")");
            }
        };

        log_network_diagnostics();

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
                    ROS_ERROR("Image size received: %lld bytes", (long long)pFrame->nImgSize);
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
        
        // ========== ZERO-COPY PASS-THROUGH: Just copy raw data and enqueue ==========
        auto copy_start = std::chrono::high_resolution_clock::now();
        
        // Create frame job with minimal copying
        FrameJob job;
        job.raw_payload.resize(pFrame->nImgSize);
        std::memcpy(job.raw_payload.data(), pFrame->pImgBuf, pFrame->nImgSize);
        job.width = pFrame->nWidth;
        job.height = pFrame->nHeight;
        job.pixel_format = pFrame->nPixelFormat;
        job.image_size = pFrame->nImgSize;
        job.frame_id = pFrame->nFrameID;
        job.arrival_time = ros::Time::now();
        
        auto copy_end = std::chrono::high_resolution_clock::now();
        
        // Enqueue for processing in separate thread
        if (self) {
            self->EnqueueFrameJob(std::move(job));
        }
        
        // ========== Network Statistics Collection (periodic) ==========
        ros::Time now = job.arrival_time;
        if (self && (now - last_net_stats_time).toSec() >= 1.0) {
            NetworkInterfaceStats current_net_stats = ReadNetworkStats(self->network_interface_);
            
            if (net_stats_initialized) {
                net_stats_delta.rx_packets = SafeDelta(current_net_stats.rx_packets, last_net_stats.rx_packets);
                net_stats_delta.rx_bytes = SafeDelta(current_net_stats.rx_bytes, last_net_stats.rx_bytes);
                net_stats_delta.rx_errors = SafeDelta(current_net_stats.rx_errors, last_net_stats.rx_errors);
                net_stats_delta.rx_dropped = SafeDelta(current_net_stats.rx_dropped, last_net_stats.rx_dropped);
                net_stats_delta.tx_packets = SafeDelta(current_net_stats.tx_packets, last_net_stats.tx_packets);
                net_stats_delta.tx_bytes = SafeDelta(current_net_stats.tx_bytes, last_net_stats.tx_bytes);
            } else {
                net_stats_delta = current_net_stats;
                net_stats_initialized = true;
            }
            
            last_net_stats = current_net_stats;
            last_net_stats_time = now;
        }
        
        // ========== END: Callback Processing Time Measurement ==========
        auto callback_end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> callback_duration = callback_end_time - callback_start_time;
        double callback_time_ms = callback_duration.count();
        
        total_callback_time_ms += callback_time_ms;
        if (callback_time_ms > max_callback_time_ms) {
            max_callback_time_ms = callback_time_ms;
        }
        if (callback_time_ms < min_callback_time_ms) {
            min_callback_time_ms = callback_time_ms;
        }
        callback_count++;

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
        // Stop processing thread first
        ROS_INFO("Stopping processing thread...");
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            stop_processing_ = true;
        }
        queue_cv_.notify_all();
        
        if (processing_thread_.joinable()) {
            processing_thread_.join();
            ROS_INFO("Processing thread stopped");
        }
        
        // Then stop camera
        GXStreamOff(dev_handle_);
        GXUnregisterCaptureCallback(dev_handle_);
        GXCloseDevice(dev_handle_);
        GXCloseLib();
        // Note: img buffer removed - no longer needed
        if (raw8_buf) { delete[] raw8_buf; raw8_buf = nullptr; }
        
        if (dropped_due_to_queue_ > 0) {
            ROS_WARN("Total frames dropped due to queue full: %zu", dropped_due_to_queue_);
        }
    }

    // Static member definitions (img removed)
    char *GalaxyCamera::raw8_buf;
    sensor_msgs::Image GalaxyCamera::image_;
    image_transport::CameraPublisher GalaxyCamera::pub_;
    sensor_msgs::CameraInfo GalaxyCamera::info_;
}
