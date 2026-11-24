#pragma once

#include <string>

#include <opencv2/core.hpp>

#include "fc01_vision/vision_pipeline.hpp"

namespace fc01_vision {

// OpenCV-highgui tuning window: the annotated camera preview plus sliders for
// the HSV threshold, blob filters and target size. Values are polled once per
// frame and pushed into the pipeline only when they change, so the detection
// result updates live without restarting the node.
//
// The preview and the sliders live in two separate windows. Attaching 15
// trackbars below the image made one ~1360 px tall window on a 1080 px
// screen, which pushed the camera picture off the bottom edge: the preview
// is now its own 640x480 window and the trackbars sit next to it.
//
// Must be created and driven from one thread (the image callback thread).
class TuningWindow {
 public:
  ~TuningWindow();

  // Creates the window and the trackbars from the pipeline's current config.
  // Returns false (and logs why) when no GUI is available.
  //
  // When `save_path` is non-empty, every slider change (and the final values
  // on destruction) is written to that file as a `/**` ros__parameters YAML,
  // so the tuning survives a restart. Empty disables saving.
  bool create(const std::string& window_name, const VisionPipeline& pipeline,
              const std::string& save_path = "");

  bool created() const { return created_; }

  // Reads every trackbar; applies the changes to `pipeline`.
  // Returns true when at least one slider moved since the last call.
  bool poll(VisionPipeline& pipeline);

  // Shows `bgr` (scaled up for readability) and pumps GUI events.
  void show(const cv::Mat& bgr);

 private:
  void addSliders(const VisionPipeline& pipeline);
  // Writes the current slider values to save_path_ (atomically).
  void saveToFile();

  std::string window_name_;
  // Separate window that carries all trackbars, so the preview window keeps
  // its natural 640x480 size and is never clipped by the slider row.
  std::string controls_name_;
  bool created_{false};
  bool failed_{false};
  bool have_last_{false};
  std::string save_path_;
  bool dirty_{false};
  bool save_logged_{false};

  // Trackbar storage; the addresses are handed to OpenCV and must stay valid.
  int h_min_{0};
  int h_max_{180};
  int s_min_{0};
  int s_max_{255};
  int v_min_{0};
  int v_max_{255};
  int blur_{5};
  int morph_{5};
  int min_area_permille_{2};
  int max_area_percent_{50};
  int min_width_{6};
  int min_height_{6};
  int circularity_percent_{85};
  int diameter_cm_{50};
  int reject_border_{1};
  int shape_mode_{0};

  int last_h_min_{-1};
  int last_h_max_{-1};
  int last_s_min_{-1};
  int last_s_max_{-1};
  int last_v_min_{-1};
  int last_v_max_{-1};
  int last_blur_{-1};
  int last_morph_{-1};
  int last_min_area_{-1};
  int last_max_area_{-1};
  int last_min_width_{-1};
  int last_min_height_{-1};
  int last_circularity_{-1};
  int last_diameter_{-1};
  int last_reject_border_{-1};
  int last_shape_{-1};
};

}  // namespace fc01_vision
