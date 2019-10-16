//
// copyright Matthias Schoerghuber (AIT)
//
#include "slamantic/Semantic.hpp"


void slamantic::Semantic::_TOptions::read(cv::FileNode const& fn)
{
  if(fn.empty())
  {
    return;
  }
  std::string type_str = fn["type"];
  if(type_str == "id")
  {
    type = SemanticImageType::ID;
  }
  else if(type_str == "rgb")
  {
    type = SemanticImageType::COLOR;
  }
  else
  {
    LOG(FATAL) << type << " is not a valid semantic image type, [id,rgb]";
  }

  fn["algorithm"] >> algorithm;

  int tmp;
  fn["showLabelBar"] >> tmp;
  showLabelBar = static_cast<bool>(tmp);
}

const slamantic::Label& slamantic::Semantic::getLabel(const slamantic::LabelId& id) const
{
  return mLabels[id];
}

const slamantic::Label& slamantic::Semantic::getLabelByColor(cv::Vec3b const& vec) const
{
  return getLabel(getLabelIdByColor(vec));
}


slamantic::LabelId slamantic::Semantic::getLabelIdByColor(cv::Vec3b const& vec) const
{
  for(auto const& label : mLabels)
  {
    if(vec[0] - label.color[0] == 0 && vec[1] - label.color[1] == 0 && vec[2] - label.color[2] == 0)
    {
      return label.id;
    }
  }
  return Label::invalid();
}

void slamantic::Semantic::loadFromYaml(std::string const& yaml)
{
  cv::FileStorage fs(yaml, cv::FileStorage::READ);
  CHECK(fs.isOpened());

  loadLabelsDefinition(fs["labels"]);
  mOptions.read(fs["semantic"]);
}
const std::vector<slamantic::LabelId>& slamantic::Semantic::getInstanceLabels() const
{
  return mInstanceLabels;
}

void slamantic::Semantic::loadLabelsDefinition(cv::FileNode const& fn)
{
  if(fn.empty())
  {
    return;
  }

  mLabels.clear();
  mLabels.resize(MAX_LABEL_ID);

  cv::FileNodeIterator it     = fn.begin();
  cv::FileNodeIterator it_end = fn.end();

  LabelId max_label_id_in_definition = 0;
  for(; it != it_end; ++it)
  {
    Label       label;
    std::string tmp_bool;
    int         tmp_int;

    (*it)["name"] >> label.name;
    (*it)["isThing"] >> tmp_bool;
    label.isThing = tmp_bool == "true";

    cv::Vec3i tmp_color;
    (*it)["color"] >> tmp_color;
    label.color = cv::Vec3b(tmp_color[0], tmp_color[1], tmp_color[2]);

    (*it)["enabled"] >> tmp_bool;
    label.enabled = tmp_bool == "true";

    (*it)["id"] >> tmp_int;
    label.id = static_cast<LabelId>(tmp_int);
    CHECK(label.id < MAX_LABEL_ID) << "slamantic lib is configured to support maximum " << MAX_LABEL_ID << " labels";
    CHECK(mLabels[label.id].id == Label::invalid()) << " multiple definition for id " << label.id;
    max_label_id_in_definition = std::max(max_label_id_in_definition, label.id);

    (*it)["dynamicFactor"] >> label.dynamicsFactor;
    CHECK_BOUNDS(label.dynamicsFactor, -1, 1);

    (*it)["isInstance"] >> tmp_bool;
    label.isInstance = tmp_bool == "true";

    (*it)["isUnReliable"] >> tmp_bool;
    label.isUnReliable = tmp_bool == "true";

    LOG(INFO) << label;

    if(label.isInstance)
    {
      mInstanceLabels.push_back(label.id);
    }

    mLabels[label.id] = label;
  }

  mLabels.resize(max_label_id_in_definition + 1);
  LOG(INFO) << "number of labels: " << mLabels.size();
  // check if there are undefined labels within label id range 0 to max_label_id_in..
  for(LabelId i = 0; i < max_label_id_in_definition; i++)
  {
    CHECK(mLabels[i].id == i) << "invalid label definition for id " << i;
  }

}
slamantic::LabelId slamantic::Semantic::getLabelIdByName(std::string const& name) const
{
  auto const& it = std::find_if(mLabels.begin(), mLabels.end(), [name](Label const& l)
  {
    return l.name == name;
  });
  return it != mLabels.end() ? it->id : Label::invalid();
}
