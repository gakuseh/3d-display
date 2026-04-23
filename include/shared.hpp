#pragma once

#include <cstdint>

#include <gtk/gtk.h>
#include <glibmm.h>

#include <opencv2/videoio.hpp>
#include <opencv2/objdetect/face.hpp>
#include <opencv2/core/types.hpp>

#include <boost/asio.hpp>
#include <boost/process.hpp>

#include <mutex>
#include <atomic>
#include <thread>
#include <fstream>
#include <iostream>
#include <queue>

namespace shared_vars {
    inline float PUPILLARY_DISTANCE_INCHES = 2.440945;
    inline int BUFFER_SIZE = 30.0;

    extern GtkApplication* app;
    extern GtkWidget* main_window;

    extern GtkBuilder *builder;

    extern GdkPaintable* main_webcam_paintable;
    extern GdkPaintable* second_webcam_paintable;
    extern std::mutex main_webcam_paintable_mutex;
    extern cv::VideoCapture main_webcam_capture;
    extern cv::VideoCapture second_webcam_capture;
    extern Glib::Dispatcher webcam_dispatcher;
    extern cv::Ptr<cv::FaceDetectorYN> face_detector_pointer;
    extern cv::Rect main_bounding_box;
    extern cv::Rect second_bounding_box;

    extern GtkPicture* main_webcam_image;
    extern GtkPicture* second_webcam_image;
    extern GtkPicture* main_fov_webcam_image;
    extern GtkPicture* second_fov_webcam_image;

    extern GtkStack* stack_widget;

    extern GtkEditable* qr_code_distance_editable;
    extern GtkEditable* lenticule_density_editable;
    extern GtkEditable* green_red_line_distance_editable;
    extern GtkEditable* main_horizontal_offset_editable;
    extern GtkEditable* main_vertical_offset_editable;
    extern GtkEditable* second_horizontal_offset_editable;
    extern GtkEditable* second_vertical_offset_editable;

    extern std::thread cv_process_thread;
    extern bool is_current_cv_action_face;
    extern bool is_doing_second_camera_qr_calibration;
    extern std::atomic<bool> do_cv_thread_run;

    extern boost::asio::io_context io_context;
    extern boost::asio::ip::tcp::socket renderer_socket;
    extern boost::asio::ip::tcp::endpoint endpoint;
    extern boost::asio::ip::tcp::acceptor acceptor;

    extern bool is_renderer_active;

    extern int BUFFER_SIZE;
    extern std::queue<double> left_eye_horizontal_angle_buffer;
    extern std::queue<double> right_eye_horizontal_angle_buffer;
    extern std::queue<double> left_eye_vertical_angle_buffer;
    extern std::queue<double> right_eye_vertical_angle_buffer;
    extern double left_eye_horizontal_angle_buffer_sum;
    extern double right_eye_horizontal_angle_buffer_sum;
    extern double left_eye_vertical_angle_buffer_sum;
    extern double right_eye_vertical_angle_buffer_sum;

    extern std::queue<std::tuple<float, float>> left_eye_uv_buffer;
    extern std::queue<std::tuple<float, float>> right_eye_uv_buffer;

    extern float left_eye_u_buffer_sum;
    extern float left_eye_v_buffer_sum;

    extern float right_eye_u_buffer_sum;
    extern float right_eye_v_buffer_sum;



    extern boost::process::child* renderer_program;

    void listen_for_renderer_socket_and_call_dispatcher(); // Run this in a new thread, because socket accept blocks.
}

namespace working_parameters {
    extern float qr_code_distance;
    extern float qr_code_width_proportion;
    extern float qr_code_height_proportion;
    extern float lenticule_density;
    extern float green_to_red_line_distance;
}

namespace parameters {
    extern float main_camera_horizontal_intrinsic_parameter;
    extern float main_camera_vertical_intrinsic_parameter;
    extern float pixels_per_lens;
    extern float main_camera_horizontal_offset_inches;
    extern float main_camera_vertical_offset_inches;
    extern float display_density_ppi;
    extern float second_camera_horizontal_intrinsic_parameter;
    extern float second_camera_vertical_intrinsic_parameter;
    extern float second_camera_horizontal_offset_inches;
    extern float second_camera_vertical_offset_inches;
}