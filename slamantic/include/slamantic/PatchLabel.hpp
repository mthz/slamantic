//
// copyright Matthias Schoerghuber (AIT)
//
#ifndef SLAMANTIC_LABELSTATISTIC_HPP
#define SLAMANTIC_LABELSTATISTIC_HPP

#include <slamantic/SemanticImage.hpp>
#include <slamantic/utils/utils.hpp>

namespace slamantic
{

  class PatchLabel
  {
    public:

      PatchLabel(slamantic::SemanticImage& imLabel, const cv::Point2f& point, size_t const& size);

      /**
       * get most dominant label
       * @return
       */
      LabelId getLabel() const;

      /**
       * get most dominant label with ratio test to second
       * @param ratio
       * @return labelId of dominant, invalid if ratio test failed
       */
      LabelId getLabelByRatio(float const ratio = 0.8) const;

      /**
       * return true if labelId is present in patch
       * @param labelId
       * @return
       */
      bool contains(const slamantic::LabelId& labelId);

    private:

      PatchLabel() = delete;

      /**
       * holding struct to store label information within patch
       */
      struct PatchLabelStatistic
      {
        PatchLabelStatistic() {}
        PatchLabelStatistic(size_t const& id) : id(id) {}

        LabelId id    = Label::invalid(); //< labeld id
        size_t  count = 0; //< number of pixels from this label within the patch

        bool operator<(PatchLabelStatistic const& other) const
        {
          return count < other.count;
        }
      };


      /**
       * compute label histogram for patch
       * @param imLabel
       */
      void computeStatistics(SemanticImage const& imLabel);


      const cv::Point2f                mPoint;
      const size_t                     mPatchSize = 3;
      std::vector<PatchLabelStatistic> mLabelHistogram;
  };

}

#endif