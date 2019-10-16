//
// copyright Matthias Schoerghuber (AIT)
//
#ifndef SLAMANTIC_SEMANTIC_HPP
#define SLAMANTIC_SEMANTIC_HPP

#include <slamantic/common.hpp>
#include <slamantic/Label.hpp>

namespace slamantic
{

  /**
   * enum depicting the source type / content of the semantic image
   */
  enum SemanticImageType
  {
    COLOR, //< source type is RGB color image
    ID //< source is image id
  };

  /**
   * class which holds the semantic definition of labels
   */
  class Semantic
  {
    public:

      typedef struct _TOptions
      {
        std::string       algorithm; //< name of semantic algorithm
        SemanticImageType type = SemanticImageType::ID; //< source type of semantic image
        bool              showLabelBar = false; //< flag to show label abar

        void read(cv::FileNode const& fn);
      } TOptions;

      /**
       * return all loaded labels
       * @return
       */
      Labels const& getLabels() const
      {
        return mLabels;
      }

      /**
       * return all loaded labels
       * @return
       */
      Labels& getLabels()
      {
        return mLabels;
      }

      /**
       * return options
       * @return
       */
      TOptions const& getOptions() const
      {
        return mOptions;
      }

      /**
       * get label label from index
       * @param idx
       * @return
       */
      Label const& getLabel(LabelId const& id) const;

      /**
       * get label from color
       * @param vec
       * @return
       */
      Label const& getLabelByColor(cv::Vec3b const& vec) const;

      /**
       * get label id from color
       * @param vec
       * @return
       */
      LabelId getLabelIdByColor(cv::Vec3b const& vec) const;

      LabelId getLabelIdByName(std::string const& name) const;

      /**
       * load semantic definition from yaml
       * @param yaml
       */
      void loadFromYaml(std::string const& yaml);

      /**
       * load labels
       * @param yaml filename to yaml
       */
      void loadLabelsDefinition(cv::FileNode const& fn);

      /**
       * get all labels which are defined as instances
       * @return
       */
      std::vector<LabelId> const& getInstanceLabels() const;

    private:

      std::vector<Label>   mLabels;
      std::vector<LabelId> mInstanceLabels;
      TOptions             mOptions;

  };


  typedef std::shared_ptr<Semantic> SemanticPtr;

}

#endif //SLAMANTIC_SEMANTIC_HPP
