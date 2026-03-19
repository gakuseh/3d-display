#include "cv_actions.hpp"

bool cv_actions::detect_face(
    cv::Ptr<cv::FaceDetectorYN>& face_model_pointer, 
    cv::VideoCapture &cap,
    cv::Rect& bounding_box,
    cv::Mat& output_image, 
    std::tuple<float, float>& left_eye_uv,
    std::tuple<float, float>& right_eye_uv
) {
    
    if (!cap.isOpened()) {
        return false; // Failed to open webcam
    }

    cap >> output_image; // Capture a full_frame
    if (output_image.empty()) {
        return false; // Failed to capture full_frame
    }

    if (bounding_box.width < 63 || bounding_box.height < 63) {
        // Minimum size for the face detector is 63x63, otherwise it crashes(?)

        bounding_box = cv::Rect(
            0,
            0,
            output_image.cols,
            output_image.rows
        ); // Reset search bounds
        return false;
    }


    face_model_pointer->setInputSize(bounding_box.size());

    // Search the sub_mat, which is only the area of the image within the bounding box.
    cv::Mat sub_mat = cv::Mat(output_image, bounding_box);
    cv::Mat output_array;

    face_model_pointer->detect(sub_mat, output_array);

    if (output_array.rows == 0) {
        bounding_box = cv::Rect(
            0,
            0,
            output_image.cols,
            output_image.rows
        ); // Reset search bounds
        return false; // No face detected
    }

    int face_x = (int)output_array.at<float>(0, 0) + bounding_box.x;
    int face_y = (int)output_array.at<float>(0, 1) + bounding_box.y;
    int face_width = (int)output_array.at<float>(0, 2);
    int face_height = (int)output_array.at<float>(0, 3);

    left_eye_uv = std::make_tuple(
        (output_array.at<float>(0, 6) + bounding_box.x) / output_image.cols,
        (output_array.at<float>(0, 7) + bounding_box.y) / output_image.rows
    );

    right_eye_uv = std::make_tuple(
        (output_array.at<float>(0, 4) + bounding_box.x) / output_image.cols,
        (output_array.at<float>(0, 5) + bounding_box.y) / output_image.rows
    );

    cv::Point left_eye_position_as_cv_point(
        (int)output_array.at<float>(0, 6) + bounding_box.x,
        (int)output_array.at<float>(0, 7) + bounding_box.y
    );
    cv::Point right_eye_position_as_cv_point(
        (int)output_array.at<float>(0, 4) + bounding_box.x,
        (int)output_array.at<float>(0, 5) + bounding_box.y
    );


    // Draw search bounds
    cv::rectangle(output_image, bounding_box, cv::Scalar(255, 0, 0), 2);

    // Draw face rectangle
    cv::rectangle(output_image, cv::Rect(
        face_x,
        face_y,
        face_width,
        face_height
    ), cv::Scalar(0, 255, 0), 2);

    // Draw eye positions
    cv::circle(output_image,  left_eye_position_as_cv_point, 5, cv::Scalar(255, 0, 0), -1);
    cv::circle(output_image, right_eye_position_as_cv_point, 5, cv::Scalar(255, 0, 0), -1);

    int new_bounding_box_x = std::clamp((int)(face_x + face_width/2 - face_width*SEARCH_AREA_SIZE/2), 0, output_image.cols);
    int new_bounding_box_y = std::clamp((int)(face_y + face_height/2 - face_height*SEARCH_AREA_SIZE/2), 0, output_image.rows);
    int new_bounding_box_width = std::clamp((int)(face_width*SEARCH_AREA_SIZE), 0, output_image.cols - new_bounding_box_x);
    int new_bounding_box_height = std::clamp((int)(face_height*SEARCH_AREA_SIZE), 0, output_image.rows - new_bounding_box_y);

    bounding_box = cv::Rect(
        new_bounding_box_x,
        new_bounding_box_y,
        new_bounding_box_width,
        new_bounding_box_height
    );

    return true;
}

bool cv_actions::detect_qr(
    cv::VideoCapture& cap,
    cv::Mat& out_frame,
    float& qr_code_width_proportion,
    float& qr_code_height_proportion
) {
    cv::QRCodeDetector detector;

    cap >> out_frame;

    cv::Mat output_points;

    detector.detect(out_frame, output_points);

    if (output_points.rows == 0) {
        // No QR Code detected
        return false;
    }

    // Output is like x, y, x, y, x, y
    // Start from top left and go clockwise

    int qr_code_x = (int)output_points.at<float>(0, 0);
    int qr_code_y = (int)output_points.at<float>(0, 1);
    int qr_code_width = (int)output_points.at<float>(0, 4) - qr_code_x;
    int qr_code_height = (int)output_points.at<float>(0, 5) - qr_code_y;

    // Draw QR code rectangle
    cv::rectangle(out_frame, cv::Rect(
        qr_code_x,
        qr_code_y,
        qr_code_width,
        qr_code_height
    ), cv::Scalar(0, 255, 0), 2);

    qr_code_width_proportion = qr_code_width  / (float)out_frame.cols;
    qr_code_height_proportion = qr_code_height / (float)out_frame.rows;

    return true;
}