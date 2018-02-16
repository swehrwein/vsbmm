#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

void draw(
    cv::Mat3b& frame,
    const std::vector<cv::Vec2f>& actual,
    const std::vector<std::pair<cv::Vec2f, bool>>& predicted,
    const std::vector<float>& colors,
    const int cmap = cv::COLORMAP_WINTER,
    const int maxLineLength=20);
