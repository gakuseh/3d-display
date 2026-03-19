/*
CV Actions. Captures from webcam and processes them.
*/

#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect.hpp>
#include <cmath>
#include <tuple>


const float SEARCH_AREA_SIZE = 1.5f;

namespace cv_actions {
    // Gets a frame from the VideoCapture and tries to detect faces using the 
    // given face_model. If a face is detected, this function outputs the image 
    // Mat, as well as the coordinates of the two eyes on the image. The 
    // coordinates are relative to the origin at the top-left of the image.
    // A bounding box can also be provided. The bounding box will be mutated to
    // match the detected face.
    bool detect_face(
        cv::Ptr<cv::FaceDetectorYN>& face_model_pointer,
        cv::VideoCapture& cap,
        cv::Rect& bounding_box,
        cv::Mat& output_image,
        std::tuple<int, int>& left_eye_coord,
        std::tuple<int, int>& right_eye_coord
    );

    bool detect_qr(
        cv::VideoCapture& cap,
        cv::Mat& out_frame,
        float& qr_code_inverse_proportion
    );
}