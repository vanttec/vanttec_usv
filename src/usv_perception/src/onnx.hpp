//
// Created by ubuntu on 1/20/23.
//
#ifndef DETECT_NORMAL_YOLOV8_HPP
#define DETECT_NORMAL_YOLOV8_HPP
#include "common.hpp"
#include "fstream"
using namespace det;

class ONNX {
public:
    explicit ONNX(const std::string& engine_file_path);
    ~ONNX();

    void                 make_pipe(bool warmup = true);
    void                 copy_from_Mat(const cv::Mat& image);
    void                 copy_from_Mat(const cv::Mat& image, cv::Size& size);
    void                 letterbox(const cv::Mat& image, cv::Mat& out, cv::Size& size);
    void                 infer();
    void                 postprocess(std::vector<Object>& objs,
                                     float                score_thres = 0.25f,
                                     float                iou_thres   = 0.65f,
                                     int                  topk        = 100,
                                     int                  num_labels  = 80);
    static void          draw_objects(const cv::Mat&                                image,
                                      cv::Mat&                                      res,
                                      const std::vector<Object>&                    objs,
                                      const std::vector<std::string>&               CLASS_NAMES,
                                      const std::vector<std::vector<unsigned int>>& COLORS);
    int                  num_bindings;
    int                  num_inputs  = 0;
    int                  num_outputs = 0;
    std::vector<void*>   host_ptrs;
    std::vector<void*>   device_ptrs;

    PreParam pparam;
};

ONNX::ONNX(const std::string& engine_file_path) {}

ONNX::~ONNX() {}

void ONNX::make_pipe(bool warmup) {}

void ONNX::letterbox(const cv::Mat& image, cv::Mat& out, cv::Size& size) {}

void ONNX::copy_from_Mat(const cv::Mat& image) {}

void ONNX::copy_from_Mat(const cv::Mat& image, cv::Size& size) {}

void ONNX::infer() {}

void ONNX::postprocess(std::vector<Object>& objs, float score_thres, float iou_thres, int topk, int num_labels) {}

void ONNX::draw_objects(const cv::Mat&                                image,
                          cv::Mat&                                      res,
                          const std::vector<Object>&                    objs,
                          const std::vector<std::string>&               CLASS_NAMES,
                          const std::vector<std::vector<unsigned int>>& COLORS)
{}
#endif  // DETECT_NORMAL_YOLOV8_HPP
