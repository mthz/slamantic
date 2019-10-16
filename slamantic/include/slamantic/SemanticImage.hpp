//
// copyright Matthias Schoerghuber (AIT)
//
#ifndef SLAMANTIC_SEMANTICIMAGE_HPP
#define SLAMANTIC_SEMANTICIMAGE_HPP

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <slamantic/Label.hpp>
#include <slamantic/Semantic.hpp>

namespace slamantic
{
  /**
   * container for semantic image; stores label image and associate label configuration
   * abstract base class; specializations for color or id based image
   */
  class SemanticImage
  {
    public:

      virtual ~SemanticImage() = default;

      /**
       * get label image
       * @return
       */
      cv::Mat getImage() const
      {
        return mImage;
      }

      /**
       * get associated semantic information
       * @return
       */
      SemanticPtr getSemantic() const
      {
        return mpSemantic;
      }

      /**
       * get label at position
       * @param pt position
       * @return
       */
      virtual LabelId getLabelIdAt(cv::Point const& pt) const = 0;


      Label const& getLabelAt(cv::Point const& pt) const
      {
        DCHECK(mpSemantic);
        return mpSemantic->getLabel(getLabelIdAt(pt));
      }

      virtual SemanticImageType getType() const = 0;

      /**
       * compute mask image for isDynamic function()
       */
      void computeDynamicMask(int const erosion_size = 3);

      cv::Mat getDynamicMask() const
      {
        return mDynamicMask;
      }

      /**
       * return true if position pt is dynamic
       * @return
       */
      bool isDynamic(cv::Point2f const& pt) const;

      /**
       * set probability image; must be same size as labeling image
       * @param probabilities
       */
      void setProbabilities(cv::Mat const& probabilities)
      {
        DCHECK(probabilities.rows == mImage.rows);
        DCHECK(probabilities.cols == mImage.cols);
        DCHECK(probabilities.type() == CV_32FC1);
        mProbabilities = probabilities;
      }

      /**
       * get the probabilty image
       * @return
       */
      cv::Mat getProbabilities()
      {
        return mProbabilities;
      }

      /**
       * get probability at pt [0,1]
       * @param pt
       * @return probabilty or  1 if no prob. image is set
       */
      float getProbabilityAt(cv::Point const& pt) const;


    protected:
      SemanticImage() = default;
      SemanticImage(cv::Mat const& image, SemanticPtr const& pSemantic) : mImage(image), mpSemantic(pSemantic) {}
      SemanticImage(std::string const& filename, SemanticPtr const& pSemantic, int load_flag) : mpSemantic(pSemantic)
      {
        mImage = cv::imread(filename, load_flag);
      }
      cv::Mat     mImage;
      SemanticPtr mpSemantic = nullptr;

      cv::Mat mDynamicMask;

      cv::Mat mProbabilities;
  };

  typedef std::shared_ptr<SemanticImage> SemanticImagePtr;

  /**
   * specialiaztion for id images
   * must be 8-bit 1 channel images
   */
  class SemanticImageId: public SemanticImage
  {
    public:

      SemanticImageId(cv::Mat const& image, SemanticPtr const& pSemantic = nullptr) : SemanticImage(image, pSemantic)
      {
        CHECK(image.type() == CV_8U) << "invalid image type";;
      }

      SemanticImageId(std::string const& filename, SemanticPtr const& pSemantic = nullptr) : SemanticImage(filename,
                                                                                                           pSemantic,
                                                                                                           cv::ImreadModes::IMREAD_GRAYSCALE)
      {
        CHECK(mImage.type() == CV_8U) << "invalid image type";;
      }

      LabelId getLabelIdAt(cv::Point const& pt) const
      {
        return mImage.at<std::uint8_t>(pt);
      }

      SemanticImageType getType() const
      {
        return ID;
      }
  };


  /**
   * specialiaztion for color images
   * must be 8-bit 1 channel images
   */
  class SemanticImageRGB: public SemanticImage
  {
    public:
      SemanticImageRGB(cv::Mat const& image, SemanticPtr const& pSemantic = nullptr) : SemanticImage(image, pSemantic)
      {
        CHECK(mImage.type() == CV_8UC3) << "invalid image type";
      }

      SemanticImageRGB(std::string const& filename, SemanticPtr const& pSemantic = nullptr) : SemanticImage(filename,
                                                                                                            pSemantic,
                                                                                                            cv::ImreadModes::IMREAD_UNCHANGED)
      {
        CHECK(mImage.type() == CV_8UC3) << "invalid image type";
      }

      LabelId getLabelIdAt(cv::Point const& pt) const
      {
        cv::Vec3b bgr = mImage.at<cv::Vec3b>(pt);
        cv::Vec3b rgb(bgr[2], bgr[1], bgr[0]);
        LabelId   id  = mpSemantic->getLabelIdByColor(rgb);
        return id;
      }

      SemanticImageType getType() const
      {
        return COLOR;
      }

    private:
  };

  /**
   * semantic image factory method
   * @param type type of source image
   * @param pSemantic associated semantic
   * @param img source image
   * @return
   */
  SemanticImagePtr create(SemanticImageType const& type, SemanticPtr const& pSemantic, cv::Mat const& img);

};

#endif //SLAMANTIC_SEMANTICIMAGE_HPP
