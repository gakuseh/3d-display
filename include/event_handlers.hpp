#pragma once

#include <gtk/gtk.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <boost/process.hpp>

#include "gtk_signal_data.hpp"
#include "shared.hpp"

const float QR_CODE_WIDTH_INCH = 4;

namespace event_handlers {
    void on_calibrate_button_clicked (GtkWidget *widget, gpointer data);
    void on_fov_calibration_capture_clicked(GtkWidget *widget, gpointer data);
    void on_display_density_continue_clicked(GtkWidget *widget, gpointer data);
    void on_horizontal_offset_continue_clicked(GtkWidget *widget, gpointer data);
    void on_vertical_offset_continue_clicked(GtkWidget *widget, gpointer data);
    void on_measurements_continue_clicked(GtkWidget *widget, gpointer data);
    void on_change_object_clicked(GtkWidget *widget, gpointer data);
}
