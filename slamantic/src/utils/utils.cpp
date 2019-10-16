//
// copyright Matthias Schoerghuber (AIT)
//

#include <slamantic/utils/utils.hpp>


cv::Mat slamantic::utils::getPatch(cv::Mat const& image, cv::Point2i const& point, size_t const& patchSize)
{
  cv::Mat patch = image(calcPatch(image, point, patchSize));
  return patch;
}
cv::Rect slamantic::utils::calcPatch(cv::Mat const& image, cv::Point2i const& point, size_t const& patchSize)
{
  // get patch arround keypoint
  int const   halfPatchSize = patchSize / 2;
  cv::Point2i pt1;
  pt1.x = std::max(0, point.x - halfPatchSize);
  pt1.y = std::max(0, point.y - halfPatchSize);
  cv::Point2f pt2;
  pt2.x = std::min(image.cols, point.x + halfPatchSize + 1);
  pt2.y = std::min(image.rows, point.y + halfPatchSize + 1);

  return cv::Rect(pt1, pt2);;
}
