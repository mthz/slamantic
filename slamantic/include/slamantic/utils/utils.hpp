//
// copyright Matthias Schoerghuber (AIT)
//

#ifndef SLAMANTIC_UTILS_HPP
#define SLAMANTIC_UTILS_HPP

#include <opencv2/core.hpp>

namespace slamantic
{
  namespace utils
  {

    /**
  * get image patch around point of size patchSize
  * @param image source image
  * @param point position of patch center
  * @param patchSize size of patch windwo
  * @return
  */
    cv::Rect calcPatch(cv::Mat const& image, cv::Point2i const& point, size_t const& patchSize);


    /**
      * get image patch around point of size patchSize
      * @param image source image
      * @param point position of patch center
      * @param patchSize size of patch windwo
      * @return
      */
    cv::Mat getPatch(cv::Mat const& image, cv::Point2i const& point, size_t const& patchSize);


    /**
     * convert rgb scalar to bgr scalar
     * @param rgb
     * @return
     */
    inline cv::Scalar to_bgr(cv::Scalar const& rgb)
    {
      return cv::Scalar(rgb[2], rgb[1], rgb[0]);
    }

  }
}
#endif //SLAMANTIC_UTILS_HPP
