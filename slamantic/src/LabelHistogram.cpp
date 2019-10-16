//
// copyright Matthias Schoerghuber (AIT)
//
#include <slamantic/LabelHistogram.hpp>

void slamantic::LabelHistogram::insert(slamantic::LabelId const& id) {
  DCHECK(id != slamantic::Label::invalid());
  mNumTotal++;
  if(mHistogram.find(id) == mHistogram.end())
  {
    mHistogram[id] = 1;
  }
  else
  {
    mHistogram[id]++;
  }
}
void slamantic::LabelHistogram::clear() {
  mNumTotal = 0;
  mHistogram.clear();
}
size_t slamantic::LabelHistogram::size() const {
  return mHistogram.size();
}
size_t slamantic::LabelHistogram::size_total() const {
  return mNumTotal;
}
std::pair<slamantic::LabelId, size_t> slamantic::LabelHistogram::max_element() const {
  auto max = std::max_element(mHistogram.begin(),
                              mHistogram.end(),
                              [](std::pair<slamantic::LabelId, size_t> const& a,
                                 std::pair<slamantic::LabelId, size_t> const& b)
                              { return a.second < b.second; });
  return *max;
}
std::map<slamantic::LabelId, size_t> const& slamantic::LabelHistogram::histogram() const {
  return mHistogram;
}
void slamantic::LabelHistogram::print(std::ostream& out) const {
  for(auto const& elem : mHistogram)
  {
    out << elem.first << ": " << elem.second << "\n";
  }
  out << std::endl;
}
