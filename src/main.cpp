#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>

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

void request_cv_process_update() {
    while (shared_vars::do_cv_thread_run) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000/60));

        // Lock the mutex
        shared_vars::webcam_paintable_mutex.lock();

        cv::Mat output_image;

        if (!shared_vars::is_current_cv_action_face) { 
            // Do QR Code
            cv_actions::detect_qr(shared_vars::webcam_capture, output_image, working_parameters::qr_code_width_proportion, working_parameters::qr_code_height_proportion);
            //std::cout << "DEBUG: QR COde width proportion, QR code height proportion: " + std::to_string(working_parameters::qr_code_width_proportion) + " " + std::to_string(working_parameters::qr_code_height_proportion) << std::endl;
        } else {
            // Get eye coordinates as pixels on the image. Coordinates should
            // have the origin in the top left of the image.
            std::tuple<float, float> left_eye_uv;
            std::tuple<float, float> right_eye_uv;

            bool did_detect_face = cv_actions::detect_face(shared_vars::face_detector_pointer, shared_vars::webcam_capture, shared_vars::bounding_box, output_image, left_eye_uv, right_eye_uv);

            // If the face detection did not work, then continue onto next loop
            if (did_detect_face) {
                // std::cout << std::endl; //DEBUG
                float l_u = std::get<0>(left_eye_uv);
                float l_v = std::get<1>(left_eye_uv);

                float r_u = std::get<0>(right_eye_uv);
                float r_v = std::get<1>(right_eye_uv);

                //std::cout << std::string("DEBUG: Left UV: ") + "(" + std::to_string(l_u) + ", " + std::to_string(l_v) + ")" + "Right UV: " + "(" + std::to_string(r_u) + ", " + std::to_string(r_v) + ")" << std::endl;

                // Calculate left eye and right eye depth

                // Vectors pointing to left and right eyes
                float l_vector_x = (0.5 - l_u)/parameters::camera_horizontal_intrinsic_parameter;
                float l_vector_y = (0.5 - l_v)/parameters::camera_vertical_intrinsic_parameter;
                float l_vector_z = 1;
                float l_vector_mag = static_cast<float>(std::sqrt(std::pow(l_vector_x, 2) + std::pow(l_vector_y, 2) + std::pow(l_vector_z, 2)));

                float r_vector_x = (0.5 - r_u)/parameters::camera_horizontal_intrinsic_parameter;
                float r_vector_y = (0.5 - r_v)/parameters::camera_vertical_intrinsic_parameter;
                float r_vector_z = 1;
                float r_vector_mag = static_cast<float>(std::sqrt(std::pow(r_vector_x, 2) + std::pow(r_vector_y, 2) + std::pow(r_vector_z, 2)));

                // Angle between the left and right vectors
                float cosine_theta = (l_vector_x * r_vector_x + l_vector_y * r_vector_y + l_vector_z * r_vector_z)/(l_vector_mag * r_vector_mag);
                float theta = std::acos(cosine_theta);

                // std::cout << "DEBUG: Horizontal intrinsic parameter " << std::to_string(parameters::camera_horizontal_intrinsic_parameter) << std::endl;
                // std::cout << "DEBUG: Vertical intrinsic parameter " << std::to_string(parameters::camera_vertical_intrinsic_parameter) << std::endl;
                // std::cout << std::string("DEBUG: Left Vector: ") + "(" + std::to_string(l_vector_x) + ", " + std::to_string(l_vector_y) + ", " + std::to_string(l_vector_z) + ")";
                // std::cout << std::string(" Right Vector: ") + "(" + std::to_string(r_vector_x) + ", " + std::to_string(r_vector_y) + ", " + std::to_string(r_vector_z)+ ")" << std::endl;
                //
                // std::cout << "DEBUG: Distance between two eyes, UV: " + std::to_string(std::sqrt(std::pow(l_u - r_u, 2) + std::pow(l_v - r_v, 2))) << std::endl;
                // std::cout << "DEBUG: Angle between left and right vectors: " + std::to_string(theta * 180/3.1415) + " degrees" << std::endl;

                float both_eye_distance = shared_vars::PUPILLARY_DISTANCE_INCHES/2.0/std::sin(theta/2);

                // Depth is z axis
                // Derived from similar triangles
                float l_3d_z = both_eye_distance / std::sqrt(std::pow(l_vector_x, 2) + 1);
                float r_3d_z = both_eye_distance / std::sqrt(std::pow(r_vector_x, 2) + 1);

                // Using eye coordinates, estimate eye positions in 3D space. Origin
                // should be the center of the screen.

                // From Chapter 43 of MIT Visionbook, equation 43.1, converting 2D
                // homogenous point to 3D point is the formula
                // depth for that pixel * inverse of intrinsic matrix * 2d point
                //
                // Axes are calculated correctly so that you can just plug the
                // coords directly into Godot. Origin is the center of the scene.
                // When facing the display, right is positive x, up is positive y,
                // and out of the screen is positive z.
                float l_3d_x = (0.5 - l_u)/parameters::camera_horizontal_intrinsic_parameter*l_3d_z;
                float l_3d_y = (0.5 - l_v)/parameters::camera_vertical_intrinsic_parameter*l_3d_z;

                float r_3d_x = (0.5 - r_u)/parameters::camera_horizontal_intrinsic_parameter*r_3d_z;
                float r_3d_y = (0.5 - r_v)/parameters::camera_vertical_intrinsic_parameter*r_3d_z;

                l_3d_y += parameters::camera_vertical_offset_inches;
                r_3d_y += parameters::camera_vertical_offset_inches;

                l_3d_x += parameters::camera_horizontal_offset_inches;
                r_3d_x += parameters::camera_horizontal_offset_inches;

                // std::cout << "Left eye position, inches: ("
                //     + std::to_string(l_3d_x)  + ", " + std::to_string(l_3d_y) + ", " + std::to_string(l_3d_z)
                //     + "). Right eye position, inches: ("
                //     + std::to_string(r_3d_x)  + ", " + std::to_string(r_3d_y) + ", " + std::to_string(r_3d_z)
                // + ")" << std::endl;


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
        }

        // Convert to GdkPaintable
        GdkPaintable* new_paintable = cv_mat_to_paintable(output_image);

        
        if (shared_vars::webcam_paintable) {
            g_object_unref(shared_vars::webcam_paintable);
        }

        // Update the shared paintable
        shared_vars::webcam_paintable = new_paintable;

        // Unlock mutex
        shared_vars::webcam_paintable_mutex.unlock();

        // Notify the main thread to update the UI
        shared_vars::webcam_dispatcher.emit();
    }
}

void handle_webcam_dispatch() {  
    shared_vars::webcam_paintable_mutex.lock();

    gtk_picture_set_paintable(shared_vars::main_webcam_image, shared_vars::webcam_paintable);
    gtk_picture_set_paintable(shared_vars::fov_webcam_image, shared_vars::webcam_paintable);

    shared_vars::webcam_paintable_mutex.unlock();
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
    shared_vars::fov_webcam_image = GTK_PICTURE(gtk_builder_get_object (shared_vars::builder, "fov_webcam_image"));

    // Set up video capture
    shared_vars::webcam_capture.open(0);
    if (!shared_vars::webcam_capture.isOpened()) {
        std::cerr << "Error: Could not open webcam." << std::endl;
    }

    // Get a frame and set up bounding box
    cv::Mat first_frame;
    shared_vars::webcam_capture >> first_frame;
    if (first_frame.empty()) {
        std::cerr << "Error: Could not capture initial frame from webcam." << std::endl;
    } else {
        shared_vars::bounding_box = cv::Rect(0, 0, first_frame.cols, first_frame.rows);
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
    GtkWidget *fov_calibration_capture_button = GTK_WIDGET(gtk_builder_get_object(shared_vars::builder, "fov_calibration_capture_button"));
    GtkWidget *display_density_continue_button = GTK_WIDGET(gtk_builder_get_object(shared_vars::builder, "display_density_continue_button"));
    GtkWidget *measurements_continue_button = GTK_WIDGET(gtk_builder_get_object(shared_vars::builder, "measurements_continue_button"));
    GtkWidget *change_object_button = GTK_WIDGET(gtk_builder_get_object(shared_vars::builder, "change_button"));
    GtkWidget *horizontal_offset_continue_button = GTK_WIDGET(gtk_builder_get_object(shared_vars::builder, "horizontal_offset_continue_button"));
    GtkWidget *vertical_offset_continue_button = GTK_WIDGET(gtk_builder_get_object(shared_vars::builder, "vertical_offset_continue_button"));

    g_signal_connect(calibrate_button, "clicked", G_CALLBACK(event_handlers::on_calibrate_button_clicked), NULL);
    g_signal_connect(fov_calibration_capture_button, "clicked", G_CALLBACK(event_handlers::on_fov_calibration_capture_clicked), NULL);
    g_signal_connect(display_density_continue_button, "clicked", G_CALLBACK(event_handlers::on_display_density_continue_clicked), NULL);
    g_signal_connect(measurements_continue_button, "clicked", G_CALLBACK(event_handlers::on_measurements_continue_clicked), NULL);
    g_signal_connect(change_object_button, "clicked", G_CALLBACK(event_handlers::on_change_object_clicked), NULL);
    g_signal_connect(horizontal_offset_continue_button, "clicked", G_CALLBACK(event_handlers::on_horizontal_offset_continue_clicked), NULL);
    g_signal_connect(vertical_offset_continue_button, "clicked", G_CALLBACK(event_handlers::on_vertical_offset_continue_clicked), NULL);

    // Set up the entry pointers
    shared_vars::qr_code_distance_editable = GTK_EDITABLE(gtk_builder_get_object(shared_vars::builder, "qr_code_distance_entry"));
    shared_vars::lenticule_density_editable = GTK_EDITABLE(gtk_builder_get_object(shared_vars::builder, "lenticule_density_entry"));
    shared_vars::index_of_refraction_editable = GTK_EDITABLE(gtk_builder_get_object(shared_vars::builder, "index_of_refraction_entry"));
    shared_vars::green_red_line_distance_editable = GTK_EDITABLE(gtk_builder_get_object(shared_vars::builder, "green_red_line_distance_entry"));
    shared_vars::horizontal_offset_editable = GTK_EDITABLE(gtk_builder_get_object(shared_vars::builder, "horizontal_offset_entry"));
    shared_vars::vertical_offset_editable = GTK_EDITABLE(gtk_builder_get_object(shared_vars::builder, "vertical_offset_entry"));


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
    shared_vars::webcam_capture.release();

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
