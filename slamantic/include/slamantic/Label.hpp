//
// copyright Matthias Schoerghuber (AIT)
//
#ifndef SLAMANTIC_LABEL_HPP
#define SLAMANTIC_LABEL_HPP

#include <opencv2/core.hpp>

#include <slamantic/common.hpp>

namespace slamantic
{

  typedef int LabelId;

  LabelId const MAX_LABEL_ID = 128; //< maximum number of labels; used for accelerated histogramm

  /**
   * Label definition strcture
   */
  struct Label
  {
    // returns maximum number of labels, is as well invalid labeld id
    static LabelId constexpr max() { return MAX_LABEL_ID; }
    static LabelId constexpr invalid() { return MAX_LABEL_ID; }

    std::string name; //< name of the label
    cv::Vec3b   color         = cv::Vec3b(0, 0, 0);  //< color of label
    bool    isThing        = false; //< if background or foreground
    bool    enabled        = false; //< if label is enabled
    bool    isDynamic      = false; //< if labels is a dynamic content
    LabelId id             = invalid(); //< local unique guranteed id
    bool    isInstance     = false;//< if label can be an unique instance
    bool    isUnReliable   = false; //< label can provide relaible 3D points
    double  dynamicsFactor = 0; //< dynamic factor of label [-1,1]
  };

  typedef std::vector<Label> Labels; //< default label container

  /**
   * create legend image of semantic image
   * @param labels
   * @param columns num of columns
   * @param column_width  pixel width of column
   * @param row_height pixel height of column
   * @return
   */
  cv::Mat createLabelBar(Labels const& labels,
                         size_t const columns = 1,
                         size_t const column_width = 100,
                         size_t const row_height = 50);


  std::ostream& operator<<(std::ostream& out, const Label& label);
}

#endif //SLAMANTIC_LABEL_HPP
