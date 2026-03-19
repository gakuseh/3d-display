#include "event_handlers.hpp"

// Parameters used to calculate other final parameters
// These do not need to be saved

void event_handlers::on_calibrate_button_clicked (GtkWidget *widget, gpointer _)
{
    // Switch to the calibration stack first
    shared_vars::is_current_cv_action_face = false;
    gtk_stack_set_visible_child_name(shared_vars::stack_widget, "fov_calibration_box");
}

void event_handlers::on_fov_calibration_capture_clicked(GtkWidget *widget, gpointer _)
{
    gtk_stack_set_visible_child_name(shared_vars::stack_widget, "measurements_calibration_box");
    shared_vars::is_current_cv_action_face = true;
}

void event_handlers::on_measurements_continue_clicked(GtkWidget *widget, gpointer _)
{
    std::string qr_code_distance_input(gtk_editable_get_chars(shared_vars::qr_code_distance_editable, 0, -1));
    bool was_parse_successful = false;

    try {
        working_parameters::qr_code_distance = std::stof(qr_code_distance_input);
        was_parse_successful = true;
    } catch (const std::invalid_argument& e) {
        std::cerr << "Invalid input for QR code distance: " << e.what() << std::endl;
    }

    if (!was_parse_successful) return;

    // Get the lenticule density

    std::string lenticule_density_input(gtk_editable_get_chars(shared_vars::lenticule_density_editable, 0, -1));
    was_parse_successful = false;

    try {
        working_parameters::lenticule_density = std::stof(lenticule_density_input);
        was_parse_successful = true;
    } catch (const std::invalid_argument& e) {
        std::cerr << "Invalid input for lenticule density: " << e.what() << std::endl;
    }

    if (!was_parse_successful) return;

    // Get the index of refraction
    std::string index_of_refraction_input(gtk_editable_get_chars(shared_vars::index_of_refraction_editable, 0, -1));
    was_parse_successful = false;

    try {
        parameters::index_of_refraction = std::stof(index_of_refraction_input);
        was_parse_successful = true;
    } catch (const std::invalid_argument& e) {
        std::cerr << "Invalid input for lenticule density: " << e.what() << std::endl;
    }

    if (!was_parse_successful) return;

    // Calculate intrinsic parameters
    parameters::camera_horizontal_intrinsic_parameter = working_parameters::qr_code_distance * working_parameters::qr_code_width_proportion / QR_CODE_WIDTH_INCH;
    parameters::camera_vertical_intrinsic_parameter = working_parameters::qr_code_distance * working_parameters::qr_code_height_proportion / QR_CODE_WIDTH_INCH;

    std::cout << "QR Code distance: " << working_parameters::qr_code_distance << " in." << std::endl; 
    std::cout << "Lenticule density: " << working_parameters::lenticule_density << " LPI" << std::endl;
    std::cout << "Index of refraction: " << parameters::index_of_refraction << std::endl;
    std::cout << "Horizontal intrinsic parameter:" << parameters::camera_horizontal_intrinsic_parameter << std::endl;
    std::cout << "Vertical intrinsic parameter:" << parameters::camera_vertical_intrinsic_parameter << std::endl;

    // Tell 3D renderer to display the measurement window
    std::vector<int64_t> message;
    message.push_back((int64_t)0);
    boost::asio::write(shared_vars::renderer_socket, boost::asio::buffer(message));

    // Switch to display density
    gtk_stack_set_visible_child_name(shared_vars::stack_widget, "display_density_calibration_box");
}

void event_handlers::on_display_density_continue_clicked(GtkWidget *widget, gpointer _)
{

    std::string green_to_red_line_distance_input(gtk_editable_get_chars(shared_vars::green_red_line_distance_editable, 0, -1));
    bool was_parse_successful = false;

    try {
        working_parameters::green_to_red_line_distance = std::stof(green_to_red_line_distance_input);
        was_parse_successful = true;
    } catch (const std::invalid_argument& e) {
        std::cerr << "Invalid input for green to red line distance: " << e.what() << std::endl;
    }

    if (!was_parse_successful) return;

    std::cout << "Distance from green to the red line: " << working_parameters::green_to_red_line_distance << " in." << std::endl;

    // Tell 3D renderer to hide the measurement window
    std::vector<int64_t> message;
    message.push_back((int64_t)1);
    boost::asio::write(shared_vars::renderer_socket, boost::asio::buffer(message));

    // Find pixels per lens, and send it to the 3D renderer
    // Also send the index of refraction
    parameters::pixels_per_lens = 500.0 / working_parameters::green_to_red_line_distance / working_parameters::lenticule_density;
    std::cout << "Event handlers.cpp. Line 84. pixels_per_lens is " << parameters::pixels_per_lens << std::endl;
    boost::asio::write(shared_vars::renderer_socket, boost::asio::buffer({(int64_t)2}));
    boost::asio::write(shared_vars::renderer_socket, boost::asio::buffer({(float_t)parameters::pixels_per_lens}));
    boost::asio::write(shared_vars::renderer_socket, boost::asio::buffer({(float_t)parameters::index_of_refraction}));

    // Write all parameters to a save file
    std::ofstream save_file("calibration_settings.txt");
    if (save_file.is_open()) {
        save_file << parameters::camera_horizontal_intrinsic_parameter << std::endl;
        save_file << parameters::camera_vertical_intrinsic_parameter << std::endl;
        save_file << parameters::pixels_per_lens << std::endl;
        save_file << parameters::index_of_refraction << std::endl;
        save_file.close();
    }

    // Switch to the main menu
    gtk_stack_set_visible_child_name(shared_vars::stack_widget, "main_box");
}

void on_new_object_selected(GObject* source_object, GAsyncResult* res, gpointer file_dialog_pointer) {
    GFile* file = gtk_file_dialog_open_finish(GTK_FILE_DIALOG(source_object), res, NULL);
    std::string file_pathname = g_file_get_path(file);
    g_object_unref(G_OBJECT(file_dialog_pointer));
    g_object_unref(G_OBJECT(file));

    // Send the file to the renderer through socket
    boost::asio::write(shared_vars::renderer_socket, boost::asio::buffer({(int64_t)6, (int64_t)file_pathname.length()}));
    boost::asio::write(shared_vars::renderer_socket, boost::asio::buffer(file_pathname));
}

void event_handlers::on_change_object_clicked(GtkWidget *widget, gpointer data) 
{
    GtkFileDialog* dialog = gtk_file_dialog_new();
    gtk_file_dialog_open(dialog, GTK_WINDOW(shared_vars::main_window), NULL, on_new_object_selected, dialog);
}