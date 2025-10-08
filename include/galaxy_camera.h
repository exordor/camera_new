//
// Created by qiayuan on 6/27/20.
//

#ifndef SRC_GALAXY_CAMERA_INCLUDE_GALAXY_CAMERA_H_
#define SRC_GALAXY_CAMERA_INCLUDE_GALAXY_CAMERA_H_

#include "libgxiapi/GxIAPI.h"
#include "libgxiapi/DxImageProc.h"
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <vector>


namespace galaxy_camera {
    class GalaxyCamera {
    public:
        GalaxyCamera(const ros::NodeHandle &nh,
                     image_transport::CameraPublisher &pub,
                     sensor_msgs::CameraInfo info,
                     uint32_t height, uint32_t width, uint32_t step,
                     uint32_t offset_x, uint32_t offset_y,
                     const std::string &encoding,
                     const std::string &camera_ip,
                     const std::string &camera_pixel_format);

        ~GalaxyCamera();

        static sensor_msgs::Image image_;

    private:

        void writeConfig();

        GX_DEV_HANDLE dev_handle_{};

        int last_channel_ = 0;
        ros::NodeHandle nh_;
        
        std::string network_interface_;

        // Temporary buffer for Raw16 -> Raw8 conversion (used in processing thread)
        static char *raw8_buf;
        static image_transport::CameraPublisher pub_;
        static sensor_msgs::CameraInfo info_;

        std::string requested_pixel_format_;

        struct FrameJob {
            std::vector<uint8_t> raw_payload;
            int width = 0;
            int height = 0;
            int pixel_format = 0;
            size_t image_size = 0;
            int64_t frame_id = 0;
            ros::Time arrival_time;
        };

        struct ProcessingStatistics {
            double total_processing_time_ms = 0.0;
            double total_conversion_time_ms = 0.0;
            double total_publish_time_ms = 0.0;
            double max_processing_time_ms = 0.0;
            double min_processing_time_ms = std::numeric_limits<double>::max();
            int processed_frames = 0;
        };

        void EnqueueFrameJob(FrameJob job);
        void ProcessingLoop();
        void AccumulateProcessingStatistics(double total_ms,
                                            double conversion_ms,
                                            double publish_ms);
        ProcessingStatistics ConsumeProcessingStatistics();

        static void GX_STDC onFrameCB(GX_FRAME_CALLBACK_PARAM *pFrame);

        std::thread processing_thread_;
        std::mutex queue_mutex_;
        std::condition_variable queue_cv_;
        std::deque<FrameJob> frame_queue_;
        bool stop_processing_ = false;
        size_t max_queue_depth_ = 4;
        size_t dropped_due_to_queue_ = 0;

        std::mutex processing_stats_mutex_;
        ProcessingStatistics processing_stats_;

    };
}

#endif //SRC_GALAXY_CAMERA_INCLUDE_GALAXY_CAMERA_H_
