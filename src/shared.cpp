#include "shared.hpp"

namespace shared_vars {
    GtkApplication* app = nullptr;
    GtkWidget* main_window = nullptr;

    GdkPaintable* webcam_paintable = nullptr;

    std::mutex webcam_paintable_mutex;
    cv::VideoCapture webcam_capture;
    Glib::Dispatcher webcam_dispatcher;
    cv::Ptr<cv::FaceDetectorYN> face_detector_pointer;
    cv::Rect bounding_box(0, 0, 0, 0);

    GtkPicture* main_webcam_image = nullptr;
    GtkPicture* fov_webcam_image = nullptr;

    GtkStack* stack_widget = nullptr;

    GtkEditable* qr_code_distance_editable = nullptr;
    GtkEditable* lenticule_density_editable = nullptr;
    GtkEditable* index_of_refraction_editable = nullptr;
    GtkEditable* green_red_line_distance_editable = nullptr;
    GtkEditable* horizontal_displacement_editable = nullptr;
    GtkEditable* vertical_displacement_editable = nullptr;

    std::thread cv_process_thread;
    bool is_current_cv_action_face = true;
    std::atomic<bool> do_cv_thread_run{true};

    boost::asio::io_context io_context;
    boost::asio::ip::tcp::socket renderer_socket(io_context);
    boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address(boost::asio::ip::address_v4(2130706433)), 42842);    
    boost::asio::ip::tcp::acceptor acceptor(io_context);

    bool is_renderer_active = false;

    GtkBuilder *builder = nullptr;

    int BUFFER_SIZE = 5;
    std::queue<double> left_eye_horizontal_angle_buffer;
    std::queue<double> right_eye_horizontal_angle_buffer;
    std::queue<double> left_eye_vertical_angle_buffer;
    std::queue<double> right_eye_vertical_angle_buffer;
    double left_eye_horizontal_angle_buffer_sum = 0.0;
    double right_eye_horizontal_angle_buffer_sum = 0.0;
    double left_eye_vertical_angle_buffer_sum = 0.0;
    double right_eye_vertical_angle_buffer_sum = 0.0;

    boost::process::child* renderer_program = nullptr;
}

void shared_vars::listen_for_renderer_socket_and_call_dispatcher() {
    shared_vars::acceptor.accept(shared_vars::renderer_socket);
    
    
    // Renderer is now connected, load settings, enable flag
    shared_vars::is_renderer_active = true;

    // Check if settings file exists
    std::ifstream save_file("calibration_settings.txt");
    if (save_file.is_open()) {
        std::string line;
        
        try {
            std::getline(save_file, line);
            parameters::camera_horizontal_intrinsic_parameter = std::stof(line);
            std::getline(save_file, line);
            parameters::camera_vertical_intrinsic_parameter = std::stof(line);
            std::getline(save_file, line);
            parameters::pixels_per_lens = std::stof(line);
            std::getline(save_file, line);
            parameters::index_of_refraction = std::stof(line);
        } catch (const std::invalid_argument& e) {
            std::cerr << "Invalid settings file: " << e.what() << std::endl;
        }

        save_file.close();

        // Send these settings to the renderer
        boost::asio::write(shared_vars::renderer_socket, boost::asio::buffer({(int64_t)2}));
        boost::asio::write(shared_vars::renderer_socket, boost::asio::buffer({(float_t)parameters::pixels_per_lens}));
        boost::asio::write(shared_vars::renderer_socket, boost::asio::buffer({(float_t)parameters::index_of_refraction}));
    }
}

namespace working_parameters {
    float qr_code_distance = 0;
    float qr_code_width_proportion = 0;
    float qr_code_height_proportion = 0;
    float lenticule_density = 0;
    float green_to_red_line_distance = 0;
}

namespace parameters {
    float camera_horizontal_intrinsic_parameter;
    float camera_vertical_intrinsic_parameter;
    float pixels_per_lens = 0;
    float index_of_refraction = 1.5;
}