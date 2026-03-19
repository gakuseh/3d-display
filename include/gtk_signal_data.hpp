#pragma once

#include <gtk/gtk.h>
#include <glibmm.h>
#include <boost/process.hpp>

struct gtk_signal_data {
    boost::process::child *cv_process;
    boost::process::opstream *to_cv_pipe;
    boost::process::ipstream *from_cv_pipe;
    GtkWidget *main_webcam_image;
    GtkWidget *fov_webcam_image;
    bool main_webcam_image_set = false;
    bool fov_webcam_image_set = false;
    bool is_red = false;
    Glib::Dispatcher *dispatcher;
    GtkWidget *stack_widget;
    GtkBuilder *builder;

    GtkEditable *qr_code_distance_editable;
    GtkEditable *lenticule_density_editable;
    GtkEditable *green_red_line_distance_editable;
    GtkEditable *horizontal_offset_editable;
    GtkEditable *vertical_offset_editable;
};