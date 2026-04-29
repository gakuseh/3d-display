#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>
#include <cmath>

#include <boost/asio.hpp>

#include <gtk/gtk.h>
#include <glibmm.h>

#include <opencv2/core/mat.hpp>

#include "shared.hpp"
#include "gtk_signal_data.hpp"
#include "cv_actions.hpp"
#include "event_handlers.hpp"

GdkPaintable* cv_mat_to_paintable(const cv::Mat& mat) {
    cv::Mat rgb_mat;

    // Convert BGR to RGB (OpenCV uses BGR, GTK expects RGB)
    cv::cvtColor(mat, rgb_mat, cv::COLOR_BGR2RGB);

    // Create GBytes from cv::Mat data
    GBytes* bytes = g_bytes_new(rgb_mat.data, rgb_mat.total() * rgb_mat.elemSize());

    // Create GdkTexture from bytes
    GdkTexture* texture = gdk_memory_texture_new(
        rgb_mat.cols,              // width
        rgb_mat.rows,              // height
        GDK_MEMORY_R8G8B8,         // format (RGB, 8 bits per channel)
        bytes,                     // data
        rgb_mat.step[0]            // stride
    );

    GdkPaintable *paintable = GDK_PAINTABLE(texture);

    g_bytes_unref(bytes);

    return paintable;
}

void get_3d_coordinates_single_camera(
    const std::tuple<float, float>& left_eye_uv,
    const std::tuple<float, float>& right_eye_uv,
    std::tuple<float, float, float>& left_eye_position_out,
    std::tuple<float, float, float>& right_eye_position_out
    )
{
    float l_u = std::get<0>(left_eye_uv);
    float l_v = std::get<1>(left_eye_uv);

    float r_u = std::get<0>(right_eye_uv);
    float r_v = std::get<1>(right_eye_uv);

    //std::cout << std::string("DEBUG: Left UV: ") + "(" + std::to_string(l_u) + ", " + std::to_string(l_v) + ")" + "Right UV: " + "(" + std::to_string(r_u) + ", " + std::to_string(r_v) + ")" << std::endl;

    // Calculate left eye and right eye depth

    // Vectors pointing to left and right eyes
    float l_vector_x = (0.5 - l_u)/parameters::main_camera_horizontal_intrinsic_parameter;
    float l_vector_y = (0.5 - l_v)/parameters::main_camera_vertical_intrinsic_parameter;
    float l_vector_z = 1;
    float l_vector_mag = static_cast<float>(std::sqrt(std::pow(l_vector_x, 2) + std::pow(l_vector_y, 2) + std::pow(l_vector_z, 2)));

    float r_vector_x = (0.5 - r_u)/parameters::main_camera_horizontal_intrinsic_parameter;
    float r_vector_y = (0.5 - r_v)/parameters::main_camera_vertical_intrinsic_parameter;
    float r_vector_z = 1;
    float r_vector_mag = static_cast<float>(std::sqrt(std::pow(r_vector_x, 2) + std::pow(r_vector_y, 2) + std::pow(r_vector_z, 2)));

    // Calculations based on https://www.desmos.com/3d/gzap1niwhb
    // Assumptions: 1) Eyes are distance PUPILLARY_DISTANCE_INCHES apart
    //              2) Eyes line on a plane, whose normal vector points from the
    //                 origin to the midpoint between the two eyes. This implies:
    //              3) Eyes are both equally distant to the origin.

    // Scale vectors so that they are unit vectors

    l_vector_x /= l_vector_mag;
    l_vector_y /= l_vector_mag;
    l_vector_z /= l_vector_mag;

    r_vector_x /= r_vector_mag;
    r_vector_y /= r_vector_mag;
    r_vector_z /= r_vector_mag;

    float unit_vector_dot_product = l_vector_x * r_vector_x + l_vector_y * r_vector_y + l_vector_z * r_vector_z;

    float distance_to_eyes = shared_vars::PUPILLARY_DISTANCE_INCHES/std::sqrt(2.0 * (1 - unit_vector_dot_product));

    left_eye_position_out = std::tuple(
        l_vector_x * distance_to_eyes + parameters::main_camera_horizontal_offset_inches,
        l_vector_y * distance_to_eyes + parameters::main_camera_vertical_offset_inches,
        l_vector_z * distance_to_eyes + parameters::main_camera_z_offset_inches
        );

    right_eye_position_out = std::tuple(
        r_vector_x * distance_to_eyes + parameters::main_camera_horizontal_offset_inches,
        r_vector_y * distance_to_eyes + parameters::main_camera_vertical_offset_inches,
        r_vector_z * distance_to_eyes + parameters::main_camera_z_offset_inches
        );
}

void get_closest_point_from_two_lines(
    const std::tuple<float, float, float>& main_vector,
    const std::tuple<float, float, float>& main_camera_position,
    const std::tuple<float, float, float>& second_vector,
    const std::tuple<float, float, float>& second_camera_position,
    std::tuple<float, float, float>& closest_point)
{
    /*
    lines are functions M(s) = main_vector * s + main_camera_position
                        S(t) = second_vector * t + second_camera_position

    so let W(j) = M(j) - S(j) for a given j. W(j) is vector pointing from one
    point on line M to the other point on line S

    so as we change j we should get to the point where W vector is perpendicular
    to both M and S. that's when W is the shortest, and so is the closest point.

    so basically solve for j, by finding j s.t. W(j) dot main_vector = W(j) dot second_vector = 0
    */


    float main_x = std::get<0>(main_vector);
    float main_y = std::get<1>(main_vector);
    float main_z = std::get<2>(main_vector);

    float main_origin_x = std::get<0>(main_camera_position);
    float main_origin_y = std::get<1>(main_camera_position);
    float main_origin_z = std::get<2>(main_camera_position);

    float second_x = std::get<0>(second_vector);
    float second_y = std::get<1>(second_vector);
    float second_z = std::get<2>(second_vector);

    float second_origin_x = std::get<0>(second_camera_position);
    float second_origin_y = std::get<1>(second_camera_position);
    float second_origin_z = std::get<2>(second_camera_position);

    float wx = main_origin_x - second_origin_x;
    float wy = main_origin_y - second_origin_y;
    float wz = main_origin_z - second_origin_z;

    float a = main_x*main_x   + main_y*main_y   + main_z*main_z;
    float b = main_x*second_x + main_y*second_y + main_z*second_z;
    float c = second_x*second_x + second_y*second_y + second_z*second_z;
    float d = main_x*wx + main_y*wy + main_z*wz;
    float e = second_x*wx + second_y*wy + second_z*wz;

    float t = (b*e - c*d) / (a*c - b*b);
    float s = (a*e - b*d) / (a*c - b*b);

    float closest_main_x = main_origin_x + t * main_x;
    float closest_main_y = main_origin_y + t * main_y;
    float closest_main_z = main_origin_z + t * main_z;

    float closest_second_x = second_origin_x + s * second_x;
    float closest_second_y = second_origin_y + s * second_y;
    float closest_second_z = second_origin_z + s * second_z;

    closest_point = std::tuple<float, float, float>((closest_main_x+closest_second_x)/2.0, (closest_main_y+closest_second_y)/2.0, (closest_main_z+closest_second_z)/2.0);
}

void get_3d_coordinates_two_cameras(
    const std::tuple<float, float>& main_left_eye_uv,
    const std::tuple<float, float>& main_right_eye_uv,
    const std::tuple<float, float>& second_left_eye_uv,
    const std::tuple<float, float>& second_right_eye_uv,
    const std::tuple<float, float, float>& main_camera_position,
    const std::tuple<float, float, float>& second_camera_position,
    const float main_camera_horizontal_intrinsic_parameter,
    const float main_camera_vertical_intrinsic_parameter,
    const float second_camera_horizontal_intrinsic_parameter,
    const float second_camera_vertical_intrinsic_parameter,
    std::tuple<float, float, float>& left_eye_position_out,
    std::tuple<float, float, float>& right_eye_position_out
    ) {

    std::cout << "Main Left eye uv: " << std::get<0>(main_left_eye_uv) << ", " << std::get<1>(main_left_eye_uv) << std::endl;
    std::cout << "Main Right eye uv: " << std::get<0>(main_right_eye_uv) << ", " << std::get<1>(main_right_eye_uv) << std::endl;
    std::cout << "Second Left eye uv: " << std::get<0>(second_left_eye_uv) << ", " << std::get<1>(second_left_eye_uv) << std::endl;
    std::cout << "Second Right eye uv: " << std::get<0>(second_right_eye_uv) << ", " << std::get<1>(second_right_eye_uv) << std::endl;
    std::cout << std::endl;

    float main_l_u = std::get<0>(main_left_eye_uv);
    float main_l_v = std::get<1>(main_left_eye_uv);
    auto main_left_vector_x = (0.5 - main_l_u)/main_camera_horizontal_intrinsic_parameter;
    auto main_left_vector_y = (0.5 - main_l_v)/main_camera_vertical_intrinsic_parameter;
    auto main_left_vector_z = 1.0;
    auto main_left_vector = std::tuple<float, float, float>(
        main_left_vector_x, main_left_vector_y, main_left_vector_z);

    float second_l_u = std::get<0>(second_left_eye_uv);
    float second_l_v = std::get<1>(second_left_eye_uv);
    auto second_left_vector_x = (0.5 - second_l_u)/second_camera_horizontal_intrinsic_parameter;
    auto second_left_vector_y = (0.5 - second_l_v)/second_camera_vertical_intrinsic_parameter;
    auto second_left_vector_z = 1.0;
    auto second_left_vector = std::tuple<float, float, float>(
        second_left_vector_x, second_left_vector_y, second_left_vector_z);

    get_closest_point_from_two_lines(
        main_left_vector,
        main_camera_position,
        second_left_vector,
        second_camera_position,
        left_eye_position_out);

    float main_r_u = std::get<0>(main_right_eye_uv);
    float main_r_v = std::get<1>(main_right_eye_uv);
    auto main_right_vector_x = (0.5 - main_r_u)/main_camera_horizontal_intrinsic_parameter;
    auto main_right_vector_y = (0.5 - main_r_v)/main_camera_vertical_intrinsic_parameter;
    auto main_right_vector_z = 1.0;
    auto main_right_vector = std::tuple<float, float, float>(
        main_right_vector_x, main_right_vector_y, main_right_vector_z);

    float second_r_u = std::get<0>(second_right_eye_uv);
    float second_r_v = std::get<1>(second_right_eye_uv);
    auto second_right_vector_x = (0.5 - second_r_u)/second_camera_horizontal_intrinsic_parameter;
    auto second_right_vector_y = (0.5 - second_r_v)/second_camera_vertical_intrinsic_parameter;
    auto second_right_vector_z = 1.0;
    auto second_right_vector = std::tuple<float, float, float>(
        second_right_vector_x, second_right_vector_y, second_right_vector_z);

    get_closest_point_from_two_lines(
        main_right_vector,
        main_camera_position,
        second_right_vector,
        second_camera_position,
        right_eye_position_out);
}

void request_cv_process_update() {
    while (shared_vars::do_cv_thread_run) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000/60));

        // Lock the mutex
        shared_vars::main_webcam_paintable_mutex.lock();

        cv::Mat main_webcam_output_image;
        cv::Mat second_webcam_output_image;

        if (!shared_vars::is_current_cv_action_face) {
            // Do QR Code

            if (!shared_vars::is_doing_second_camera_qr_calibration) {
                cv_actions::detect_qr(shared_vars::main_webcam_capture, main_webcam_output_image, working_parameters::qr_code_width_proportion, working_parameters::qr_code_height_proportion);
            } else {
                cv_actions::detect_qr(shared_vars::second_webcam_capture, second_webcam_output_image, working_parameters::qr_code_width_proportion, working_parameters::qr_code_height_proportion);
            }
            //std::cout << "DEBUG: QR COde width proportion, QR code height proportion: " + std::to_string(working_parameters::qr_code_width_proportion) + " " + std::to_string(working_parameters::qr_code_height_proportion) << std::endl;
        } else {
            if (!shared_vars::second_webcam_capture.isOpened()) {
                // Only one camera

                // Get eye coordinates as pixels on the image. Coordinates should
                // have the origin in the top left of the image.
                std::tuple<float, float> new_left_eye_uv;
                std::tuple<float, float> new_right_eye_uv;

                bool did_detect_face = cv_actions::detect_face(shared_vars::face_detector_pointer, shared_vars::main_webcam_capture, shared_vars::main_bounding_box, main_webcam_output_image, new_left_eye_uv, new_right_eye_uv);

                // If the face detection did not work, then continue onto next loop
                if (did_detect_face) {
                    // Smoothing

                    shared_vars::left_eye_uv_buffer.push(new_left_eye_uv);
                    shared_vars::right_eye_uv_buffer.push(new_right_eye_uv);

                    shared_vars::left_eye_u_buffer_sum += std::get<0>(new_left_eye_uv);
                    shared_vars::left_eye_v_buffer_sum += std::get<1>(new_left_eye_uv);

                    shared_vars::right_eye_u_buffer_sum += std::get<0>(new_right_eye_uv);
                    shared_vars::right_eye_v_buffer_sum += std::get<1>(new_right_eye_uv);

                    if (shared_vars::left_eye_uv_buffer.size() > shared_vars::BUFFER_SIZE) {
                        std::tuple<float, float> removed_tuple = shared_vars::left_eye_uv_buffer.front();
                        shared_vars::left_eye_u_buffer_sum -= std::get<0>(removed_tuple);
                        shared_vars::left_eye_v_buffer_sum -= std::get<1>(removed_tuple);
                        shared_vars::left_eye_uv_buffer.pop();
                    }

                    if (shared_vars::right_eye_uv_buffer.size() > shared_vars::BUFFER_SIZE) {
                        std::tuple<float, float> removed_tuple = shared_vars::right_eye_uv_buffer.front();
                        shared_vars::right_eye_u_buffer_sum -= std::get<0>(removed_tuple);
                        shared_vars::right_eye_v_buffer_sum -= std::get<1>(removed_tuple);
                        shared_vars::right_eye_uv_buffer.pop();
                    }

                    auto left_eye_uv = std::tuple(
                        shared_vars::left_eye_u_buffer_sum/shared_vars::left_eye_uv_buffer.size(),
                        shared_vars::left_eye_v_buffer_sum/shared_vars::left_eye_uv_buffer.size()
                        );
                    auto right_eye_uv = std::tuple(
                        shared_vars::right_eye_u_buffer_sum/shared_vars::right_eye_uv_buffer.size(),
                        shared_vars::right_eye_v_buffer_sum/shared_vars::right_eye_uv_buffer.size()
                        );

                    // Check if new UV are too far from average
                    float left_squared_dist = (std::get<0>(left_eye_uv) - std::get<0>(new_left_eye_uv))*(std::get<0>(left_eye_uv) - std::get<0>(new_left_eye_uv)) + (std::get<1>(left_eye_uv) - std::get<1>(new_left_eye_uv))*(std::get<1>(left_eye_uv) - std::get<1>(new_left_eye_uv));
                    float right_squared_dist = (std::get<0>(right_eye_uv) - std::get<0>(new_right_eye_uv))*(std::get<0>(right_eye_uv) - std::get<0>(new_right_eye_uv)) + (std::get<1>(right_eye_uv) - std::get<1>(new_right_eye_uv))*(std::get<1>(right_eye_uv) - std::get<1>(new_right_eye_uv));

                    if (left_squared_dist > 0.0025 || right_squared_dist > 0.0025) {
                        shared_vars::left_eye_uv_buffer = std::queue<std::tuple<float, float>>();
                        shared_vars::right_eye_uv_buffer = std::queue<std::tuple<float, float>>();
                        shared_vars::left_eye_u_buffer_sum = 0;
                        shared_vars::right_eye_u_buffer_sum = 0;
                        shared_vars::left_eye_v_buffer_sum = 0;
                        shared_vars::right_eye_v_buffer_sum = 0;
                    }

                    std::tuple<float, float, float> left_eye_position;
                    std::tuple<float, float, float> right_eye_position;

                    get_3d_coordinates_single_camera(left_eye_uv, right_eye_uv, left_eye_position, right_eye_position);

                    float l_3d_x = std::get<0>(left_eye_position);
                    float l_3d_y = std::get<1>(left_eye_position);
                    float l_3d_z = std::get<2>(left_eye_position);

                    float r_3d_x = std::get<0>(right_eye_position);
                    float r_3d_y = std::get<1>(right_eye_position);
                    float r_3d_z = std::get<2>(right_eye_position);

                    if (shared_vars::is_renderer_active) {
                        //std::cout << "DEBUG: renderer is active" << std::endl;

                        std::vector<int64_t> request_code;
                        request_code.push_back((int64_t)4);

                        try {
                            boost::asio::write(shared_vars::renderer_socket, boost::asio::buffer({static_cast<int64_t>(4)}));
                            boost::asio::write(shared_vars::renderer_socket, boost::asio::buffer({static_cast<float_t>(l_3d_x), static_cast<float_t>(l_3d_y), static_cast<float_t>(l_3d_z)}));
                            boost::asio::write(shared_vars::renderer_socket, boost::asio::buffer({static_cast<float_t>(r_3d_x), static_cast<float_t>(r_3d_y), static_cast<float_t>(r_3d_z)}));
                        } catch (const boost::system::system_error& e) {
                            if (e.code() == boost::asio::error::broken_pipe ||
                                e.code() == boost::asio::error::connection_reset ||
                                e.code() == boost::asio::error::eof) {
                                std::cout << "Socket disconnected: " << e.what() << std::endl;
                                shared_vars::is_renderer_active = false;

                                g_application_quit(G_APPLICATION(shared_vars::app));
                            }
                        }
                    }
                }
            } else {
                // Two cameras

                std::tuple<float, float> main_left_eye_uv;
                std::tuple<float, float> main_right_eye_uv;

                std::tuple<float, float> second_left_eye_uv;
                std::tuple<float, float> second_right_eye_uv;

                bool did_detect_face = cv_actions::detect_face(
                    shared_vars::face_detector_pointer,
                    shared_vars::main_webcam_capture,
                    shared_vars::main_bounding_box,
                    main_webcam_output_image,
                    main_left_eye_uv,
                    main_right_eye_uv
                    );

                did_detect_face = cv_actions::detect_face(
                    shared_vars::face_detector_pointer,
                    shared_vars::second_webcam_capture,
                    shared_vars::second_bounding_box,
                    second_webcam_output_image,
                    second_left_eye_uv,
                    second_right_eye_uv
                    ) && did_detect_face;

                auto main_camera_position = std::tuple(
                    parameters::main_camera_horizontal_offset_inches,
                    parameters::main_camera_vertical_offset_inches,
                    parameters::main_camera_z_offset_inches);

                auto second_camera_position = std::tuple(
                    parameters::second_camera_horizontal_offset_inches,
                    parameters::second_camera_vertical_offset_inches,
                    parameters::second_camera_z_offset_inches);

                std::tuple<float, float, float> left_eye_position;
                std::tuple<float, float, float> right_eye_position;

                get_3d_coordinates_two_cameras(
                    main_left_eye_uv,
                    main_right_eye_uv,
                    second_left_eye_uv,
                    second_right_eye_uv,
                    main_camera_position,
                    second_camera_position,
                    parameters::main_camera_horizontal_intrinsic_parameter,
                    parameters::main_camera_vertical_intrinsic_parameter,
                    parameters::second_camera_horizontal_intrinsic_parameter,
                    parameters::second_camera_vertical_intrinsic_parameter,
                    left_eye_position,
                    right_eye_position
                    );

                //std::cout << std::get<0>(left_eye_position)  << "," << std::get<1>(left_eye_position) << "," << std::get<2>(left_eye_position) << "," << std::get<0>(right_eye_position)  << "," << std::get<1>(right_eye_position) << "," << std::get<2>(right_eye_position) << std::endl;

                if (!std::isnan(std::get<0>(left_eye_position))) {
                    boost::asio::write(shared_vars::renderer_socket, boost::asio::buffer({static_cast<int64_t>(4)}));
                    boost::asio::write(shared_vars::renderer_socket, boost::asio::buffer({std::get<0>(left_eye_position), std::get<1>(left_eye_position), std::get<2>(left_eye_position)}));
                    boost::asio::write(shared_vars::renderer_socket, boost::asio::buffer({std::get<0>(right_eye_position), std::get<1>(right_eye_position), std::get<2>(right_eye_position)}));
                }
            }
        }


        if (!main_webcam_output_image.empty()) {
            GdkPaintable* new_main_webcam_paintable = cv_mat_to_paintable(main_webcam_output_image);

            if (shared_vars::main_webcam_paintable) {
                g_object_unref(shared_vars::main_webcam_paintable);
            }

            shared_vars::main_webcam_paintable = new_main_webcam_paintable;
        }

        // Do the same thing if second webcam is online
        if (!second_webcam_output_image.empty()) {
            GdkPaintable* new_second_webcam_paintable = cv_mat_to_paintable(second_webcam_output_image);

            if (shared_vars::second_webcam_paintable) {
                g_object_unref(shared_vars::second_webcam_paintable);
            }

            shared_vars::second_webcam_paintable = new_second_webcam_paintable;
        }

        // Unlock mutex
        shared_vars::main_webcam_paintable_mutex.unlock();

        // Notify the main thread to update the UI
        shared_vars::webcam_dispatcher.emit();
    }
}

void handle_webcam_dispatch() {
    shared_vars::main_webcam_paintable_mutex.lock();

    gtk_picture_set_paintable(shared_vars::main_webcam_image, shared_vars::main_webcam_paintable);

    if (shared_vars::second_webcam_capture.isOpened()) {
        gtk_picture_set_paintable(shared_vars::second_webcam_image, shared_vars::second_webcam_paintable);
    }

    if (!shared_vars::is_current_cv_action_face) {
        gtk_picture_set_paintable(shared_vars::main_fov_webcam_image, shared_vars::main_webcam_paintable);
        gtk_picture_set_paintable(shared_vars::second_fov_webcam_image, shared_vars::second_webcam_paintable);
    }

    shared_vars::main_webcam_paintable_mutex.unlock();
}

// Signal handler for the button click
static void
on_hello_button_clicked (GtkWidget *widget,
                         gpointer   data)
{
    g_print ("Hello from UI file!\n");
}

static void
activate (GtkApplication *app,
          void        *_)
{
    GError *error = NULL;

    // Create a builder and load the UI file
    shared_vars::builder = gtk_builder_new ();
    if (!gtk_builder_add_from_file (shared_vars::builder, "ui/main.ui", &error)) {
      g_critical ("Error loading UI file: %s", error->message);
      g_error_free (error);
      return;
    }


    // Get the main window from the builder
    shared_vars::main_window = GTK_WIDGET (gtk_builder_get_object (shared_vars::builder, "main_window"));
    gtk_window_set_application (GTK_WINDOW (shared_vars::main_window), app);

    // Get the horizontal_offset_diagram picture widget and set its file
    if (GtkWidget *horizontal_offset_diagram = GTK_WIDGET (gtk_builder_get_object (shared_vars::builder, "horizontal_offset_diagram"))) {
        GFile *image_file = g_file_new_for_path("images/horizontal-offset-diagram.png");
        gtk_picture_set_file(GTK_PICTURE(horizontal_offset_diagram), image_file);
        g_object_unref(image_file);
    } else {
        g_warning("Could not find diagram picture widget");
    }

    // Get the vertical_offset_diagram picture widget and set its file
    if (GtkWidget *vertical_offset_diagram = GTK_WIDGET (gtk_builder_get_object (shared_vars::builder, "vertical_offset_diagram"))) {
        GFile *image_file = g_file_new_for_path("images/vertical-offset-diagram.png");
        gtk_picture_set_file(GTK_PICTURE(vertical_offset_diagram), image_file);
        g_object_unref(image_file);
    } else {
        g_warning("Could not find diagram picture widget");
    }

    // Set up webcam image variables
    shared_vars::main_webcam_image = GTK_PICTURE(gtk_builder_get_object (shared_vars::builder, "main_webcam_image"));
    shared_vars::second_webcam_image = GTK_PICTURE(gtk_builder_get_object (shared_vars::builder, "second_webcam_image"));
    shared_vars::main_fov_webcam_image = GTK_PICTURE(gtk_builder_get_object (shared_vars::builder, "main_fov_webcam_image"));
    shared_vars::second_fov_webcam_image = GTK_PICTURE(gtk_builder_get_object (shared_vars::builder, "second_fov_webcam_image"));

    // Set up video capture
    shared_vars::main_webcam_capture.open(0);
    if (!shared_vars::main_webcam_capture.isOpened()) {
        std::cerr << "Error: Could not open webcam." << std::endl;
    }

    // Check if second webcam is also available
    shared_vars::second_webcam_capture.open(2);
    if (shared_vars::second_webcam_capture.isOpened())
    {
        std::cout << "Second webcam detected" << std::endl;
    }

    // Get a frame and set up bounding box
    cv::Mat first_frame;
    shared_vars::main_webcam_capture >> first_frame;
    if (first_frame.empty()) {
        std::cerr << "Error: Could not capture initial frame from webcam." << std::endl;
    } else {
        shared_vars::main_bounding_box = cv::Rect(0, 0, first_frame.cols, first_frame.rows);
    }

    // Set up face detector
    shared_vars::face_detector_pointer = cv::FaceDetectorYN::create("models/face_detector_model.onnx", "", cv::Size(1, 1), 0.9, 0.3, 1);

    // Connect the dispatcher signal to the handler
    shared_vars::webcam_dispatcher.connect([]() {
        handle_webcam_dispatch();
    });

    shared_vars::cv_process_thread = std::thread(request_cv_process_update);

    // Get the CSS provider
    GtkCssProvider* css_provider = gtk_css_provider_new();
    gtk_css_provider_load_from_path(css_provider, "./ui/style.css");

    gtk_style_context_add_provider_for_display(gtk_widget_get_display(shared_vars::main_window),
                                              GTK_STYLE_PROVIDER(css_provider),
                                              GTK_STYLE_PROVIDER_PRIORITY_USER);

    // Set the stack pointer
    shared_vars::stack_widget = GTK_STACK(gtk_builder_get_object(shared_vars::builder, "main_stack"));

    // Connect signal for buttons
    GtkWidget *calibrate_button = GTK_WIDGET(gtk_builder_get_object(shared_vars::builder, "calibrate_button"));
    GtkWidget *main_fov_calibration_capture_button = GTK_WIDGET(gtk_builder_get_object(shared_vars::builder, "main_fov_calibration_capture_button"));
    GtkWidget *second_fov_calibration_capture_button = GTK_WIDGET(gtk_builder_get_object(shared_vars::builder, "second_fov_calibration_capture_button"));
    GtkWidget *display_density_continue_button = GTK_WIDGET(gtk_builder_get_object(shared_vars::builder, "display_density_continue_button"));
    GtkWidget *measurements_continue_button = GTK_WIDGET(gtk_builder_get_object(shared_vars::builder, "measurements_continue_button"));
    GtkWidget *change_object_button = GTK_WIDGET(gtk_builder_get_object(shared_vars::builder, "change_button"));
    GtkWidget *main_horizontal_offset_continue_button = GTK_WIDGET(gtk_builder_get_object(shared_vars::builder, "main_horizontal_offset_continue_button"));
    GtkWidget *main_vertical_offset_continue_button = GTK_WIDGET(gtk_builder_get_object(shared_vars::builder, "main_vertical_offset_continue_button"));
    GtkWidget *second_horizontal_offset_continue_button = GTK_WIDGET(gtk_builder_get_object(shared_vars::builder, "second_horizontal_offset_continue_button"));
    GtkWidget *second_vertical_offset_continue_button = GTK_WIDGET(gtk_builder_get_object(shared_vars::builder, "second_vertical_offset_continue_button"));

    g_signal_connect(calibrate_button, "clicked", G_CALLBACK(event_handlers::on_calibrate_button_clicked), NULL);
    g_signal_connect(main_fov_calibration_capture_button, "clicked", G_CALLBACK(event_handlers::on_main_fov_calibration_capture_clicked), NULL);
    g_signal_connect(second_fov_calibration_capture_button, "clicked", G_CALLBACK(event_handlers::on_second_fov_calibration_capture_clicked), NULL);
    g_signal_connect(display_density_continue_button, "clicked", G_CALLBACK(event_handlers::on_display_density_continue_clicked), NULL);
    g_signal_connect(measurements_continue_button, "clicked", G_CALLBACK(event_handlers::on_measurements_continue_clicked), NULL);
    g_signal_connect(change_object_button, "clicked", G_CALLBACK(event_handlers::on_change_object_clicked), NULL);
    g_signal_connect(main_horizontal_offset_continue_button, "clicked", G_CALLBACK(event_handlers::on_main_horizontal_offset_continue_clicked), NULL);
    g_signal_connect(main_vertical_offset_continue_button, "clicked", G_CALLBACK(event_handlers::on_main_vertical_offset_continue_clicked), NULL);
    g_signal_connect(second_horizontal_offset_continue_button, "clicked", G_CALLBACK(event_handlers::on_second_horizontal_offset_continue_clicked), NULL);
    g_signal_connect(second_vertical_offset_continue_button, "clicked", G_CALLBACK(event_handlers::on_second_vertical_offset_continue_clicked), NULL);

    // Set up the editable pointers
    shared_vars::qr_code_distance_editable = GTK_EDITABLE(gtk_builder_get_object(shared_vars::builder, "qr_code_distance_entry"));
    shared_vars::lenticule_density_editable = GTK_EDITABLE(gtk_builder_get_object(shared_vars::builder, "lenticule_density_entry"));
    shared_vars::green_red_line_distance_editable = GTK_EDITABLE(gtk_builder_get_object(shared_vars::builder, "green_red_line_distance_entry"));
    shared_vars::main_horizontal_offset_editable = GTK_EDITABLE(gtk_builder_get_object(shared_vars::builder, "main_horizontal_offset_entry"));
    shared_vars::main_vertical_offset_editable = GTK_EDITABLE(gtk_builder_get_object(shared_vars::builder, "main_vertical_offset_entry"));
    shared_vars::second_horizontal_offset_editable = GTK_EDITABLE(gtk_builder_get_object(shared_vars::builder, "second_horizontal_offset_entry"));
    shared_vars::second_vertical_offset_editable = GTK_EDITABLE(gtk_builder_get_object(shared_vars::builder, "second_vertical_offset_entry"));


    // Show the window
    gtk_window_present (GTK_WINDOW (shared_vars::main_window));

    // Start the renderer
    shared_vars::acceptor.open(shared_vars::endpoint.protocol());
    shared_vars::acceptor.set_option(boost::asio::ip::tcp::acceptor::reuse_address(true));
    shared_vars::acceptor.bind(shared_vars::endpoint);
    shared_vars::acceptor.listen(1);

    std::thread t(shared_vars::listen_for_renderer_socket_and_call_dispatcher);
    t.detach();

    shared_vars::renderer_program = new boost::process::child("renderer");

}

static void deactivate(GtkApplication *app, void *data) {
    std::cout << "Deactivate triggered. Cleaning up memory." << std::endl;
    shared_vars::do_cv_thread_run = false;

    std::cout << "Joining thread, waiting for thread end" << std::endl;
    shared_vars::cv_process_thread.join();
    std::cout << "Thread ended" << std::endl;

    std::cout << "Releasing webcam" << std::endl;
    shared_vars::main_webcam_capture.release();

    std::cout << "Tell renderer to quit" << std::endl;

    if (shared_vars::is_renderer_active && shared_vars::renderer_socket.is_open()) {
        boost::asio::write(shared_vars::renderer_socket, boost::asio::buffer({(int64_t)5}));
    } else {
        std::cout << "Renderer already inactive, skipping quit message." << std::endl;
    }
}

int
main (int    argc,
      char **argv)
{
    GMainLoop *main_loop;
    main_loop = g_main_loop_new(NULL, FALSE);

    int status;

    shared_vars::app = gtk_application_new ("org.gtk.example", G_APPLICATION_DEFAULT_FLAGS);
    g_signal_connect (shared_vars::app, "activate", G_CALLBACK (activate), NULL);
    g_signal_connect (shared_vars::app, "shutdown", G_CALLBACK (deactivate), NULL);

    status = g_application_run (G_APPLICATION (shared_vars::app), argc, argv);

    std::cout << "Main loop has exited. Line 241. Trying unref" << std::endl;

    g_object_unref (shared_vars::app);

    std::cout << "Successfully unrefed app." << std::endl;

    return status;
}
