//
// copyright Matthias Schoerghuber (AIT)
//
#ifndef VSLAMFW_SEMANTICDESCRIPTOR_HPP
#define VSLAMFW_SEMANTICDESCRIPTOR_HPP

#include <opencv2/core.hpp>

#include <slamantic/Label.hpp>
#include <slamantic/SemanticImage.hpp>

namespace slamantic
{

  /**
   * Semantic descriptor consisting of a single label with a probability
   */
  struct SingleSemanticDescriptor
  {
    explicit SingleSemanticDescriptor() : labelId(Label::invalid()), probability(0.0) {}
    explicit SingleSemanticDescriptor(LabelId const& labelId) : labelId(labelId), probability(1.0) {}
    explicit SingleSemanticDescriptor(LabelId const& labelId, double probability)
      : labelId(labelId), probability(probability) {}

    LabelId labelId     = Label::invalid();  //< id of label
    double  probability = 0.0;  //< probability of being labelId
    double labelDynamicsFactor = 0.0; //< cache dynamics factor value to prevent cumbersome read out later
  };

  /**
   * Descriptor with single label and its probability with aggregation support
   */
  struct SingleSemanticDescriptorAgg: public SingleSemanticDescriptor
  {
    using SingleSemanticDescriptor::SingleSemanticDescriptor;

    size_t totalLabelCnt = 1;
    size_t labelIdCnt    = 1;

    /**
     * initialize aggregation
     * @param labelId id of label
     * @param labelCnt number of label occurences
     * @param totalLabelCnt number of total measurements
     */
    void init(LabelId const& labelId, size_t const& labelCnt, size_t const& totalLabelCnt)
    {
      DCHECK(totalLabelCnt > 0);
      DCHECK(labelCnt <= totalLabelCnt);
      SingleSemanticDescriptorAgg::labelId       = labelId;
      SingleSemanticDescriptorAgg::labelIdCnt    = labelCnt;
      SingleSemanticDescriptorAgg::totalLabelCnt = totalLabelCnt;
      SingleSemanticDescriptorAgg::probability   = static_cast<double>(labelIdCnt) / static_cast<double>(totalLabelCnt);
    }

    /**
     * update label probability with new measurementent
     * @param measuredLabelId
     */
    void update(LabelId const& measuredLabelId)
    {
      if(measuredLabelId == labelId)
      {
        labelIdCnt++;
      }
      totalLabelCnt++;
      probability = static_cast<double>(labelIdCnt) / static_cast<double>(totalLabelCnt);
    }
  };

  template<int N>
  class SemanticDescriptor
  {
    public:

      void compute(SemanticImage const& image, cv::Point2f const& pt)
      {

      }

      static double distance(SemanticDescriptor const& a, SemanticDescriptor const& b)
      {
        for(auto const& a_elem : a.mData)
        {
          for(auto const& b_elem : b.mData)
          {
            if(a_elem.id == b_elem.id)
            {

            }
          }
        }
        return 0;
      }

    private:

      struct TData
      {
        size_t id;
        double area = 0;
      };

      Eigen::Array<TData, N, 1> mData;
  };

}


#endif //VSLAMFW_SEMANTICDESCRIPTOR_HPP
