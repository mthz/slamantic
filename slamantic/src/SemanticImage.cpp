//
// copyright Matthias Schoerghuber (AIT)
//

#include <slamantic/SemanticImage.hpp>
#include <slamantic/SemanticMask.hpp>

slamantic::SemanticImagePtr
slamantic::create(const slamantic::SemanticImageType& type, const slamantic::SemanticPtr& pSemantic, cv::Mat const& img)
{
  DCHECK(pSemantic);
  DCHECK(!img.empty());
  switch(type)
  {
    case SemanticImageType::ID:
      return std::make_shared<SemanticImageId>(img, pSemantic);
    case SemanticImageType::COLOR:
      return std::make_shared<SemanticImageRGB>(img, pSemantic);
    default:
      CHECK(false);
  }
  return nullptr;
}

void slamantic::SemanticImage::computeDynamicMask(const int erosion_size)
{
  // create mask only once
  if(mDynamicMask.empty())
  {
    // mask of unreliable areas which are not things
    slamantic::SemanticMask dynamicMask(*this, [&](Label const& label)
    {
      return label.isDynamic;
    });

    dynamicMask.erode(erosion_size);

    mDynamicMask = dynamicMask.getMask();
  }
}

bool slamantic::SemanticImage::isDynamic(cv::Point2f const& pt) const
{
  DCHECK(!mDynamicMask.empty());
  return mDynamicMask.at<std::uint8_t>(pt) == 0;
}
float slamantic::SemanticImage::getProbabilityAt(cv::Point const& pt) const
{
  if(mProbabilities.empty())
  {
    return 1.0;
  }

  return mProbabilities.at<float>(pt);
}
