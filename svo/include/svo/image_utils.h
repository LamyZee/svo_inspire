#ifndef IMAGE_UTILES_H_
#define IMAGE_UTILES_H_

#include <opencv2/core.hpp>
#include <vector>

namespace svo{

namespace image{

int sampleBrightnessHistogram(const cv::Mat& raw_img,
                              std::vector<int>* histogram,
                              int* avg_pixel_val_ptr = nullptr);

float matchingHistogram(const std::vector<int>& hist_src,
                        const std::vector<int>& hist_tgt,
                        const float init_scale);

}// namespace image
}// namespace svo
#endif