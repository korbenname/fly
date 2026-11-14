#include "fc01_vision/tuning_window.hpp"

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace fc01_vision {
namespace {

int oddKernel(int value) {
  return (value % 2 == 0) ? value + 1 : value;
}

std::string shapeName(int mode) {
  switch (mode) {
    case 1:
      return "rect";
    case 2:
      return "circle";
    default:
      return "auto";
  }
}

}  // namespace

TuningWindow::~TuningWindow() {
  if (created_ && dirty_) {
    saveToFile();
  }
}

bool TuningWindow::create(const std::string& window_name,
                          const VisionPipeline& pipeline,
                          const std::string& save_path) {
  window_name_ = window_name;
  controls_name_ = window_name + " (sliders)";
  save_path_ = save_path;
  try {
    // Preview and controls are separate windows: trackbars are drawn below
    // the window image, so keeping them on the preview made it 1360 px tall
    // and the bottom half (the actual camera picture) fell off a 1080 px
    // screen.
    cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(controls_name_, cv::WINDOW_AUTOSIZE);
    addSliders(pipeline);
    // A thin gray strip: the trackbars make up the rest of the window height.
    cv::Mat placeholder(8, 420, CV_8UC3, cv::Scalar(40, 40, 40));
    cv::imshow(controls_name_, placeholder);
    // Preview top-left, sliders to its right; both fit on a 1080-px screen.
    cv::moveWindow(window_name_, 20, 20);
    cv::moveWindow(controls_name_, 700, 20);
    cv::waitKey(1);
    created_ = true;
  } catch (const cv::Exception& error) {
    failed_ = true;
    created_ = false;
    std::cerr << "tuning window disabled: " << error.what() << std::endl;
  }
  return created_;
}

void TuningWindow::addSliders(const VisionPipeline& pipeline) {
  const DetectorConfig& detector = pipeline.detectorConfig();
  HsvRange range;
  if (!detector.ranges.empty()) {
    range = detector.ranges.front();
  }
  const EstimatorConfig& estimator = pipeline.estimatorConfig();

  h_min_ = range.h_min;
  h_max_ = range.h_max;
  s_min_ = range.s_min;
  s_max_ = range.s_max;
  v_min_ = range.v_min;
  v_max_ = range.v_max;
  blur_ = detector.blur_kernel;
  morph_ = detector.morph_kernel;
  min_area_permille_ =
      std::max(1, static_cast<int>(detector.min_area_ratio * 1000.0));
  max_area_percent_ =
      std::max(1, static_cast<int>(detector.max_area_ratio * 100.0));
  min_width_ = detector.min_width_px;
  min_height_ = detector.min_height_px;
  circularity_percent_ =
      std::max(1, static_cast<int>(detector.circle_min_circularity * 100.0));
  diameter_cm_ =
      std::max(1, static_cast<int>(estimator.target_diameter_m * 100.0));
  reject_border_ = detector.reject_border_touch ? 1 : 0;
  shape_mode_ = detector.shape == "rect" ? 1 : (detector.shape == "circle" ? 2 : 0);

  const std::string& w = controls_name_;
  cv::createTrackbar("H min", w, &h_min_, 180);
  cv::createTrackbar("H max", w, &h_max_, 180);
  cv::createTrackbar("S min", w, &s_min_, 255);
  cv::createTrackbar("S max", w, &s_max_, 255);
  cv::createTrackbar("V min", w, &v_min_, 255);
  cv::createTrackbar("V max", w, &v_max_, 255);
  cv::createTrackbar("blur (odd)", w, &blur_, 15);
  cv::createTrackbar("morph (odd)", w, &morph_, 15);
  cv::createTrackbar("min area/1000", w, &min_area_permille_, 200);
  cv::createTrackbar("max area %", w, &max_area_percent_, 100);
  cv::createTrackbar("min width px", w, &min_width_, 200);
  cv::createTrackbar("min height px", w, &min_height_, 200);
  cv::createTrackbar("circle min %", w, &circularity_percent_, 100);
  cv::createTrackbar("target dia cm", w, &diameter_cm_, 200);
  cv::createTrackbar("reject border", w, &reject_border_, 1);
  cv::createTrackbar("shape 0auto 1rect 2circle", w, &shape_mode_, 2);

  last_h_min_ = h_min_;
  last_h_max_ = h_max_;
  last_s_min_ = s_min_;
  last_s_max_ = s_max_;
  last_v_min_ = v_min_;
  last_v_max_ = v_max_;
  last_blur_ = blur_;
  last_morph_ = morph_;
  last_min_area_ = min_area_permille_;
  last_max_area_ = max_area_percent_;
  last_min_width_ = min_width_;
  last_min_height_ = min_height_;
  last_circularity_ = circularity_percent_;
  last_diameter_ = diameter_cm_;
  last_reject_border_ = reject_border_;
  last_shape_ = shape_mode_;
  have_last_ = true;
}

bool TuningWindow::poll(VisionPipeline& pipeline) {
  if (!created_ || failed_) {
    return false;
  }
  const bool changed =
      h_min_ != last_h_min_ || h_max_ != last_h_max_ ||
      s_min_ != last_s_min_ || s_max_ != last_s_max_ ||
      v_min_ != last_v_min_ || v_max_ != last_v_max_ ||
      blur_ != last_blur_ || morph_ != last_morph_ ||
      min_area_permille_ != last_min_area_ ||
      max_area_percent_ != last_max_area_ ||
      min_width_ != last_min_width_ || min_height_ != last_min_height_ ||
      circularity_percent_ != last_circularity_ ||
      diameter_cm_ != last_diameter_ ||
      reject_border_ != last_reject_border_ || shape_mode_ != last_shape_;
  if (!changed) {
    return false;
  }

  last_h_min_ = h_min_;
  last_h_max_ = h_max_;
  last_s_min_ = s_min_;
  last_s_max_ = s_max_;
  last_v_min_ = v_min_;
  last_v_max_ = v_max_;
  last_blur_ = blur_;
  last_morph_ = morph_;
  last_min_area_ = min_area_permille_;
  last_max_area_ = max_area_percent_;
  last_min_width_ = min_width_;
  last_min_height_ = min_height_;
  last_circularity_ = circularity_percent_;
  last_diameter_ = diameter_cm_;
  last_reject_border_ = reject_border_;
  last_shape_ = shape_mode_;

  DetectorConfig detector = pipeline.detectorConfig();
  HsvRange range;
  if (!detector.ranges.empty()) {
    range = detector.ranges.front();
  }
  range.h_min = std::min(h_min_, h_max_);
  range.h_max = std::max(h_min_, h_max_);
  range.s_min = std::min(s_min_, s_max_);
  range.s_max = std::max(s_min_, s_max_);
  range.v_min = std::min(v_min_, v_max_);
  range.v_max = std::max(v_min_, v_max_);
  detector.ranges.clear();
  detector.ranges.push_back(range);
  detector.blur_kernel = oddKernel(blur_);
  detector.morph_kernel = oddKernel(morph_);
  detector.min_area_ratio =
      std::max(0.0001, min_area_permille_ / 1000.0);
  detector.max_area_ratio =
      std::min(1.0, std::max(0.001, max_area_percent_ / 100.0));
  detector.min_width_px = min_width_;
  detector.min_height_px = min_height_;
  detector.circle_min_circularity = circularity_percent_ / 100.0;
  detector.reject_border_touch = reject_border_ != 0;
  detector.shape = shapeName(shape_mode_);
  pipeline.setDetectorConfig(detector);

  EstimatorConfig estimator = pipeline.estimatorConfig();
  estimator.target_diameter_m = diameter_cm_ / 100.0;
  pipeline.setEstimatorConfig(estimator);

  dirty_ = true;
  saveToFile();
  return true;
}

void TuningWindow::saveToFile() {
  if (save_path_.empty() || !created_ || failed_) {
    return;
  }
  std::ostringstream out;
  out << "# Auto-saved by the fc01 vision tuning window.\n"
      << "# Merge these values into vision.yaml, or pass this file as an\n"
      << "# additional --params-file. The last slider change wins.\n"
      << "/**:\n"
      << "  ros__parameters:\n"
      << "    hsv_ranges:\n"
      << "      - \"" << std::min(h_min_, h_max_) << ","
      << std::max(h_min_, h_max_) << ","
      << std::min(s_min_, s_max_) << ","
      << std::max(s_min_, s_max_) << ","
      << std::min(v_min_, v_max_) << ","
      << std::max(v_min_, v_max_) << "\"\n"
      << "    blur_kernel: " << oddKernel(blur_) << "\n"
      << "    morph_kernel: " << oddKernel(morph_) << "\n";
  out << std::setprecision(6) << std::defaultfloat;
  out << "    min_area_ratio: "
      << std::max(0.0001, min_area_permille_ / 1000.0) << "\n"
      << "    max_area_ratio: "
      << std::min(1.0, std::max(0.001, max_area_percent_ / 100.0)) << "\n"
      << "    min_width_px: " << min_width_ << "\n"
      << "    min_height_px: " << min_height_ << "\n"
      << "    circle_min_circularity: " << circularity_percent_ / 100.0
      << "\n"
      << "    target_diameter_m: " << diameter_cm_ / 100.0 << "\n"
      << "    reject_border_touch: " << (reject_border_ ? "true" : "false")
      << "\n"
      << "    target_shape: \"" << shapeName(shape_mode_) << "\"\n";

  // Write-then-rename so a crash mid-write never leaves a truncated file.
  const std::string tmp_path = save_path_ + ".tmp";
  {
    std::ofstream file(tmp_path, std::ios::out | std::ios::trunc);
    if (!file) {
      std::cerr << "cannot write tuning values to " << tmp_path << std::endl;
      return;
    }
    file << out.str();
    if (!file.good()) {
      std::cerr << "failed to write tuning values to " << tmp_path
                << std::endl;
      return;
    }
  }
  if (std::rename(tmp_path.c_str(), save_path_.c_str()) != 0) {
    std::cerr << "cannot replace " << save_path_ << std::endl;
    return;
  }
  if (!save_logged_) {
    save_logged_ = true;
    std::cerr << "tuning values saved to " << save_path_ << std::endl;
  }
}

void TuningWindow::show(const cv::Mat& bgr) {
  if (!created_ || failed_ || bgr.empty()) {
    return;
  }
  try {
    cv::Mat display = bgr;
    if (bgr.cols < 640) {
      cv::resize(bgr, display, cv::Size(), 2.0, 2.0, cv::INTER_NEAREST);
    }
    cv::imshow(window_name_, display);
    const int key = cv::waitKey(1) & 0xFF;
    // 's' saves on demand; slider changes are also saved automatically.
    if ((key == 's' || key == 'S') && !save_path_.empty()) {
      dirty_ = true;
      saveToFile();
    }
  } catch (const cv::Exception& error) {
    failed_ = true;
    std::cerr << "tuning window disabled after error: " << error.what()
              << std::endl;
  }
}

}  // namespace fc01_vision
