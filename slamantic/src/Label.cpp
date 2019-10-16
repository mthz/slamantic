//
// copyright Matthias Schoerghuber (AIT)
//

#include <opencv2/imgproc.hpp>

#include <slamantic/Label.hpp>
#include <slamantic/utils/utils.hpp>


std::ostream& slamantic::operator<<(std::ostream& out, const slamantic::Label& label)
{
  out << label.name << " (id: " << label.id;
  if(label.isThing)
  {
    out << ", thing";
  }
  if(label.isDynamic)
  {
    out << ", dynamic";
  }

  out << ", color: " << static_cast<unsigned int>(label.color[0]) << "/" << static_cast<unsigned int>(label.color[1])
      << "/" << static_cast<unsigned int>(label.color[2]) << ", dynamicFactor: " << label.dynamicsFactor;
  out << ")";
  return out;
}


cv::Mat slamantic::createLabelBar(const slamantic::Labels& labels,
                                  const size_t columns,
                                  const size_t column_width,
                                  const size_t row_height)
{
  CHECK(row_height > 0);
  CHECK(column_width > 0);
  if(labels.empty() || columns < 1)
  {
    return cv::Mat();
  }

  size_t rows = ceil(static_cast<double>(labels.size()) / static_cast<double>(columns));


  size_t width  = column_width * columns;
  size_t height = row_height * rows;

  cv::Mat img(height, width, CV_8UC3, cv::Scalar(0, 0, 0));

  size_t     labelIdx = 0;
  for(size_t r        = 0; r < rows; r++)
  {
    for(size_t c = 0; c < columns; c++)
    {
      cv::Mat area = img(cv::Rect(c * column_width, r * row_height, column_width, row_height));
      Label const& label = labels[labelIdx];
      area = utils::to_bgr(label.color);
      cv::putText(area,
                  std::to_string(label.id) + " " + label.name,
                  cv::Point2i(5, row_height / 1.65),
                  0,
                  0.5,
                  cv::Scalar(255, 255, 255));
      labelIdx++;
      if(labelIdx >= labels.size())
      {
        return img;
      }
    }
  }

  return img;
}