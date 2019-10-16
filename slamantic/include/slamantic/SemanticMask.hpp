//
// copyright Matthias Schoerghuber (AIT)
//
#ifndef SLAMANTIC_SEMANTICMASK_HPP
#define SLAMANTIC_SEMANTICMASK_HPP

#include <slamantic/SemanticImage.hpp>

namespace slamantic
{

  /**
   * create a mask image for dynamic regions
   * a mask is a binary image with 0 = not reliable region, 255 = reliable
   */
  class SemanticMask
  {
    public:

      //< default layers for mask generation, referse to flags in Label
      enum class Layer
      {
          Reliable, Dynamic, Thing
      };

      /**
       * create a dynamic mask instance for semantic image
       * @param image
       */
      SemanticMask(SemanticImage const& image, Layer const& layer);

      /**
       * create dynamic mask from custom lambda function
       * @param image
       * @param pred
       */
      SemanticMask(SemanticImage const& image, std::function<bool(Label const&)> pred);

      /**
       * erode mask image to mark label boundaries as bad
       * @param size erosion kernel size
       */
      void erode(size_t const size = 3);

      /**
       * dilate mask image to mark label boundaries as bad
       * @param size dilation kernel size
       */
      void dilate(size_t const size = 3);

      /**
       * return if pt is marked as dynamic
       * @param pt
       * @return
       */
      bool isDynamic(cv::Point const& pt) const;

      /**
       * get mask image
       * @return
       */
      cv::Mat getMask() const;

    private:

      /**
       * fill mask image; iterate trough label image
       * @param image
       */
      void create(SemanticImage const& image, std::function<bool(Label const&)>& maskProperty);

      cv::Mat mMask;
  };
}

#endif //SLAMANTIC_SEMANTICMASK_HPP
