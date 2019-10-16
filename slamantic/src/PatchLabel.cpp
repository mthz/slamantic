//
// copyright Matthias Schoerghuber (AIT)
//
#include <slamantic/PatchLabel.hpp>
void slamantic::PatchLabel::computeStatistics(const slamantic::SemanticImage& imLabel)
{
  // compute histogram of patch
  cv::Rect patch = utils::calcPatch(imLabel.getImage(), mPoint, mPatchSize);

  // speed up histogram calculation by histogram in array with labelId access
  mLabelHistogram.resize(Label::max());
  for(size_t i = 0; i < MAX_LABEL_ID; i++)
  {
    mLabelHistogram[i].id = i;
  }

  // count labels
  for(int iy = patch.y; iy < patch.y + patch.height; iy++)
  {
    for(int ix = patch.x; ix < patch.x + patch.width; ix++)
    {
      LabelId id = imLabel.getLabelIdAt(cv::Point2i(ix, iy));
      LOG_IF(id == Label::invalid(), WARNING) << "invalid label read from image";
      mLabelHistogram[id].count++;
    }
  }

  // remove non found labels of statistic
  auto fzero = std::remove_if(mLabelHistogram.begin(), mLabelHistogram.end(), [](PatchLabelStatistic const& stat)
  { return stat.count == 0; });
  mLabelHistogram.erase(fzero, mLabelHistogram.end());

  std::sort(mLabelHistogram.rbegin(), mLabelHistogram.rend());
}
bool slamantic::PatchLabel::contains(const slamantic::LabelId& labelId)
{
  auto it = std::find_if(mLabelHistogram.begin(), mLabelHistogram.end(), [labelId](PatchLabelStatistic const& stat)
  {
    return stat.id == labelId;
  });
  return it != mLabelHistogram.end();
}
slamantic::LabelId slamantic::PatchLabel::getLabelByRatio(const float ratio) const
{
  if(mLabelHistogram.empty())
  {
    return Label::invalid();
  }
  if(mLabelHistogram.size() == 1)
  {
    return mLabelHistogram.front().id;
  }

  float numBest       = static_cast<float>(mLabelHistogram[0].count);
  float numSecondBest = static_cast<float>(mLabelHistogram[1].count);

  bool ratioTest = numBest * ratio >= numSecondBest;

  return ratioTest ? mLabelHistogram[0].id : Label::invalid();
}
slamantic::LabelId slamantic::PatchLabel::getLabel() const
{
  if(mLabelHistogram.empty())
  {
    return Label::invalid();
  }
  return mLabelHistogram[0].id;
}
slamantic::PatchLabel::PatchLabel(slamantic::SemanticImage& imLabel, const cv::Point2f& point, size_t const& size)
  : mPoint(point), mPatchSize(size)
{
  DCHECK(mPatchSize % 2 == 1); // must be odd
  computeStatistics(imLabel);
}
