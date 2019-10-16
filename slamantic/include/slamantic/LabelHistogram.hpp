//
// copyright Matthias Schoerghuber (AIT)
//
#ifndef VSLAMFW_LABELHISTOGRAM_HPP
#define VSLAMFW_LABELHISTOGRAM_HPP

#include <map>

#include <slamantic/Label.hpp>

namespace slamantic
{

  /**
   * compute a histogram of multiple labels
   */
  class LabelHistogram
  {
    public:
      LabelHistogram() {}
      LabelHistogram(const LabelHistogram&) = delete;
      LabelHistogram& operator=(const LabelHistogram&) = delete;

      /**
       * add label to histogram
       * @param id
       */
      void insert(slamantic::LabelId const& id);

      /**
       * clear historgram
       */
      void clear();

      /**
       * return number of unique labels
       * @return
       */
      size_t size() const;

      /**
       * return number of inserted labels
       * @return
       */
      size_t size_total() const;

      /**
       * get the label with the most occurrences
       * @return pari with label and occurrences
       */
      std::pair<slamantic::LabelId, size_t> max_element() const;

      /**
       * get complete histogram data
       * @return
       */
      std::map<slamantic::LabelId, size_t> const& histogram() const;

      void print(std::ostream& out) const;

    private:

      std::map<slamantic::LabelId, size_t> mHistogram; //< id and label cnt map

      size_t mNumTotal = 0; //< number of datapoints in histogram

  };

}

#endif //VSLAMFW_LABELHISTOGRAM_HPP
