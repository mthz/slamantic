//
// copyright Matthias Schoerghuber (AIT)
//
#include <slamantic/utils/utils.hpp>
#include <slamantic/SemanticMask.hpp>

void
slamantic::SemanticMask::create(const slamantic::SemanticImage& image, std::function<bool(Label const&)>& maskProperty)
{
//  TIMED_SCOPE(timer, "mask");
  mMask = cv::Mat(image.getImage().size(), CV_8U, 255);
  cv::Mat maskThing = cv::Mat(image.getImage().size(), CV_8U, 255);

  for(int y = 0; y < mMask.rows; y++)
  {
    for(int x = 0; x < mMask.cols; x++)
    {
      cv::Point const pt(x, y);
      Label const& label = image.getLabelAt(pt);
      if(maskProperty(label))
      {
        maskThing.at<std::uint8_t>(pt) = 0;
      }
    }
  }

  mMask = mMask & maskThing;
}

cv::Mat slamantic::SemanticMask::getMask() const
{
  return mMask;
}
bool slamantic::SemanticMask::isDynamic(cv::Point const& pt) const
{
  return mMask.at<uint8_t>(pt) > 0;
}
void slamantic::SemanticMask::dilate(const size_t size)
{
  cv::Mat kernel = cv::getStructuringElement(cv::MorphShapes::MORPH_RECT, cv::Size(size, size));
  cv::dilate(mMask, mMask, kernel, cv::Point(-1, -1));
}
void slamantic::SemanticMask::erode(const size_t size)
{
  cv::Mat kernel = cv::getStructuringElement(cv::MorphShapes::MORPH_RECT, cv::Size(size, size));
  cv::erode(mMask, mMask, kernel, cv::Point(-1, -1));
}
slamantic::SemanticMask::SemanticMask(const slamantic::SemanticImage& image, Layer const& layer)
{
  std::function<bool(Label const&)> maskProperty;
  switch(layer)
  {
    case Layer::Dynamic:
      maskProperty = [](Label const& label)
      { return label.isDynamic; };
      break;
    case Layer::Reliable:
      maskProperty = [](Label const& label)
      { return label.isUnReliable; };
      break;
    case Layer::Thing:
      maskProperty = [](Label const& label)
      { return label.isThing; };
      break;
  }
  create(image, maskProperty);
}

slamantic::SemanticMask::SemanticMask(SemanticImage const& image, std::function<bool(Label const&)> pred)
{
  create(image, pred);
}
