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

        // Lock the mutex
        shared_vars::webcam_paintable_mutex.lock();

        cv::Mat output_image;

        if (!shared_vars::is_current_cv_action_face) { 
            // Do QR Code
            cv_actions::detect_qr(shared_vars::webcam_capture, output_image, working_parameters::qr_code_width_proportion, working_parameters::qr_code_height_proportion);
        } else {
            // Run action
            std::tuple<double, double> left_eye_position_proportion_from_center;
            std::tuple<double, double> right_eye_position_proportion_from_center;
            bool result = cv_actions::detect_face(shared_vars::face_detector_pointer, shared_vars::bounding_box, shared_vars::webcam_capture, output_image, left_eye_position_proportion_from_center, right_eye_position_proportion_from_center);
            
            if (result) {
                double left_eye_horizontal_angle = std::get<0>(left_eye_position_proportion_from_center) * (parameters::webcam_fov_deg / 2.0f);
                double left_eye_vertical_angle = std::get<1>(left_eye_position_proportion_from_center) * (parameters::webcam_fov_deg / 2.0f);
                double right_eye_horizontal_angle = std::get<0>(right_eye_position_proportion_from_center) * (parameters::webcam_fov_deg / 2.0f);
                double right_eye_vertical_angle = std::get<1>(right_eye_position_proportion_from_center) * (parameters::webcam_fov_deg / 2.0f);

                shared_vars::left_eye_horizontal_angle_buffer_sum += left_eye_horizontal_angle;
                shared_vars::left_eye_horizontal_angle_buffer.push(left_eye_horizontal_angle);
                shared_vars::left_eye_vertical_angle_buffer_sum += left_eye_vertical_angle;
                shared_vars::left_eye_vertical_angle_buffer.push(left_eye_vertical_angle);
                shared_vars::right_eye_horizontal_angle_buffer_sum += right_eye_horizontal_angle;
                shared_vars::right_eye_horizontal_angle_buffer.push(right_eye_horizontal_angle);
                shared_vars::right_eye_vertical_angle_buffer_sum += right_eye_vertical_angle;
                shared_vars::right_eye_vertical_angle_buffer.push(right_eye_vertical_angle);

                if (shared_vars::left_eye_horizontal_angle_buffer.size() > shared_vars::BUFFER_SIZE) {
                    shared_vars::left_eye_horizontal_angle_buffer_sum -= shared_vars::left_eye_horizontal_angle_buffer.front();
                    shared_vars::left_eye_horizontal_angle_buffer.pop();
                }

                if (shared_vars::left_eye_vertical_angle_buffer.size() > shared_vars::BUFFER_SIZE) {
                    shared_vars::left_eye_vertical_angle_buffer_sum -= shared_vars::left_eye_vertical_angle_buffer.front();
                    shared_vars::left_eye_vertical_angle_buffer.pop();
                }

                if (shared_vars::right_eye_horizontal_angle_buffer.size() > shared_vars::BUFFER_SIZE) {
                    shared_vars::right_eye_horizontal_angle_buffer_sum -= shared_vars::right_eye_horizontal_angle_buffer.front();
                    shared_vars::right_eye_horizontal_angle_buffer.pop();
                }

                if (shared_vars::right_eye_vertical_angle_buffer.size() > shared_vars::BUFFER_SIZE) {
                    shared_vars::right_eye_vertical_angle_buffer_sum -= shared_vars::right_eye_vertical_angle_buffer.front();
                    shared_vars::right_eye_vertical_angle_buffer.pop();
                }



                if (shared_vars::is_renderer_active) {
                    std::vector<int64_t> request_code;
                    request_code.push_back((int64_t)4);
                    
                    try {
                        boost::asio::write(shared_vars::renderer_socket, boost::asio::buffer(request_code));
                    } catch (const boost::system::system_error& e) {
                        if (e.code() == boost::asio::error::broken_pipe || 
                            e.code() == boost::asio::error::connection_reset ||
                            e.code() == boost::asio::error::eof) {
                            std::cout << "Socket disconnected: " << e.what() << std::endl;
                            shared_vars::is_renderer_active = false;

                            g_application_quit(G_APPLICATION(shared_vars::app));
                        }
                    }

                    double avg_left_eye_horizontal_angle = shared_vars::left_eye_horizontal_angle_buffer_sum / shared_vars::left_eye_horizontal_angle_buffer.size();
                    double avg_left_eye_vertical_angle = shared_vars::left_eye_vertical_angle_buffer_sum / shared_vars::left_eye_vertical_angle_buffer.size();
                    double avg_right_eye_horizontal_angle = shared_vars::right_eye_horizontal_angle_buffer_sum / shared_vars::right_eye_horizontal_angle_buffer.size();
                    double avg_right_eye_vertical_angle = shared_vars::right_eye_vertical_angle_buffer_sum / shared_vars::right_eye_vertical_angle_buffer.size();
                    

                    std::vector<double_t> message;
                    message.push_back(avg_left_eye_horizontal_angle);
                    message.push_back(avg_left_eye_vertical_angle);
                    message.push_back(avg_right_eye_horizontal_angle);
                    message.push_back(avg_right_eye_vertical_angle);

                    try {
                        boost::asio::write(shared_vars::renderer_socket, boost::asio::buffer(message));
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

        std::this_thread::sleep_for(std::chrono::milliseconds(1000/60));
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
    GtkCssProvider *css_provider;
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
    css_provider = gtk_css_provider_new();
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

    g_signal_connect(calibrate_button, "clicked", G_CALLBACK(event_handlers::on_calibrate_button_clicked), NULL);
    g_signal_connect(fov_calibration_capture_button, "clicked", G_CALLBACK(event_handlers::on_fov_calibration_capture_clicked), NULL);
    g_signal_connect(display_density_continue_button, "clicked", G_CALLBACK(event_handlers::on_display_density_continue_clicked), NULL);
    g_signal_connect(measurements_continue_button, "clicked", G_CALLBACK(event_handlers::on_measurements_continue_clicked), NULL);
    g_signal_connect(change_object_button, "clicked", G_CALLBACK(event_handlers::on_change_object_clicked), NULL);

    // Set up the entry pointers
    shared_vars::qr_code_distance_editable = GTK_EDITABLE(gtk_builder_get_object(shared_vars::builder, "qr_code_distance_entry"));
    shared_vars::lenticule_density_editable = GTK_EDITABLE(gtk_builder_get_object(shared_vars::builder, "lenticule_density_entry"));
    shared_vars::index_of_refraction_editable = GTK_EDITABLE(gtk_builder_get_object(shared_vars::builder, "index_of_refraction_entry"));
    shared_vars::green_red_line_distance_editable = GTK_EDITABLE(gtk_builder_get_object(shared_vars::builder, "green_red_line_distance_entry"));
    shared_vars::horizontal_displacement_editable = GTK_EDITABLE(gtk_builder_get_object(shared_vars::builder, "horizontal_displacement_entry"));
    shared_vars::vertical_displacement_editable = GTK_EDITABLE(gtk_builder_get_object(shared_vars::builder, "vertical_displacement_entry"));


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
