#include "gtest/gtest.h"

#include <tinyxml2.h>

#include "config.h"
#include "urdf_parser/urdf_parser.h"

// These tests validate the performance of the urdf parser, which is used to create 
// urdf::ModelInterface objects from urdf files

using namespace urdf;

const std::string urdf_directory = SOURCE_DIRECTORY "/robot-models/";

std::vector<std::string> GetTestUrdfFiles()
{
    std::vector<std::string> test_urdf_files;
    test_urdf_files.push_back("mini_cheetah");
    test_urdf_files.push_back("mini_cheetah_leg");
    test_urdf_files.push_back("four_bar");
    test_urdf_files.push_back("six_bar");
    test_urdf_files.push_back("planar_leg_linkage");
    test_urdf_files.push_back("revolute_rotor_chain");
    test_urdf_files.push_back("mit_humanoid_leg");
    return test_urdf_files;
}

class ParserTest : public ::testing::TestWithParam<std::string>
{
};

INSTANTIATE_TEST_SUITE_P(ParserTest, ParserTest, ::testing::ValuesIn(GetTestUrdfFiles()));

TEST_P(ParserTest, createModelFromUrdfFile)
{
    std::shared_ptr<ModelInterface> model = parseURDFFile(urdf_directory + GetParam() + ".urdf");
    ASSERT_TRUE(model != nullptr);
}

struct ParentLinkTestData
{
    std::string urdf_file;
    std::map<std::string, std::string> links_and_parents;
};

std::vector<ParentLinkTestData> GetLinksAndParents()
{
    std::vector<ParentLinkTestData> datas;

    ParentLinkTestData four_bar_data;
    four_bar_data.urdf_file = "four_bar";
    four_bar_data.links_and_parents.insert(std::make_pair("link1", "base_link"));
    four_bar_data.links_and_parents.insert(std::make_pair("link2", "link1"));
    four_bar_data.links_and_parents.insert(std::make_pair("link3", "base_link"));
    datas.push_back(four_bar_data);

    ParentLinkTestData mini_cheetah_leg_data;
    mini_cheetah_leg_data.urdf_file = "mini_cheetah_leg";
    mini_cheetah_leg_data.links_and_parents.insert(std::make_pair("abduct", "base"));
    mini_cheetah_leg_data.links_and_parents.insert(std::make_pair("abduct_rotor", "base"));
    mini_cheetah_leg_data.links_and_parents.insert(std::make_pair("thigh", "abduct"));
    mini_cheetah_leg_data.links_and_parents.insert(std::make_pair("hip_rotor", "abduct"));
    mini_cheetah_leg_data.links_and_parents.insert(std::make_pair("shank", "thigh"));
    mini_cheetah_leg_data.links_and_parents.insert(std::make_pair("knee_rotor", "thigh"));
    datas.push_back(mini_cheetah_leg_data);

    return datas;
}
class ParentLinkTest : public ::testing::TestWithParam<ParentLinkTestData>
{
protected:
    ParentLinkTest()
    {
        model_ = urdf::parseURDFFile(urdf_directory + GetParam().urdf_file + ".urdf");
    }
    std::shared_ptr<urdf::ModelInterface> model_;
};

INSTANTIATE_TEST_SUITE_P(ParentLinkTest, ParentLinkTest, ::testing::ValuesIn(GetLinksAndParents()));

TEST_P(ParentLinkTest, parent)
{
    for (const auto &link_and_parent : GetParam().links_and_parents)
    {
        const std::string &link_name = link_and_parent.first;
        const std::string &parent_name = link_and_parent.second;
        ASSERT_EQ(parent_name, model_->getLink(link_name)->getParent()->name);
    }
}

struct ChildrenLinksTestData
{
    std::string urdf_file;
    std::map<std::string, std::vector<std::string>> links_and_children;
};

std::vector<ChildrenLinksTestData> GetLinksAndChildren()
{
    std::vector<ChildrenLinksTestData> datas;

    ChildrenLinksTestData four_bar_data;
    four_bar_data.urdf_file = "four_bar";
    four_bar_data.links_and_children.insert(std::make_pair("base_link", std::vector<std::string>{"link1", "link3"}));
    four_bar_data.links_and_children.insert(std::make_pair("link1", std::vector<std::string>{"link2"}));
    four_bar_data.links_and_children.insert(std::make_pair("link2", std::vector<std::string>{}));
    four_bar_data.links_and_children.insert(std::make_pair("link3", std::vector<std::string>{}));
    datas.push_back(four_bar_data);

    ChildrenLinksTestData mini_cheetah_leg_data;
    mini_cheetah_leg_data.urdf_file = "mini_cheetah_leg";
    mini_cheetah_leg_data.links_and_children.insert(std::make_pair("base", std::vector<std::string>{"abduct", "abduct_rotor"}));
    mini_cheetah_leg_data.links_and_children.insert(std::make_pair("abduct", std::vector<std::string>{"thigh", "hip_rotor"}));
    mini_cheetah_leg_data.links_and_children.insert(std::make_pair("abduct_rotor", std::vector<std::string>{}));
    mini_cheetah_leg_data.links_and_children.insert(std::make_pair("thigh", std::vector<std::string>{"shank", "knee_rotor"}));
    mini_cheetah_leg_data.links_and_children.insert(std::make_pair("hip_rotor", std::vector<std::string>{}));
    mini_cheetah_leg_data.links_and_children.insert(std::make_pair("shank", std::vector<std::string>{}));
    mini_cheetah_leg_data.links_and_children.insert(std::make_pair("knee_rotor", std::vector<std::string>{}));
    datas.push_back(mini_cheetah_leg_data);

    return datas;
}

class ChildrenLinksTest : public ::testing::TestWithParam<ChildrenLinksTestData>
{
protected:
    ChildrenLinksTest()
    {
        model_ = urdf::parseURDFFile(urdf_directory + GetParam().urdf_file + ".urdf");
    }
    std::shared_ptr<urdf::ModelInterface> model_;
};

INSTANTIATE_TEST_SUITE_P(ChildrenLinksTest, ChildrenLinksTest,
                         ::testing::ValuesIn(GetLinksAndChildren()));

TEST_P(ChildrenLinksTest, children)
{
    for (const auto &[link_name , children_names] : GetParam().links_and_children)
    {
        ASSERT_EQ(children_names.size(), model_->getLink(link_name)->child_links.size());
        for (const auto &child_name : children_names)
        {
            bool found_child = false;
            for (const auto &child_link : model_->getLink(link_name)->child_links)
            {
                if (child_link->name == child_name)
                {
                    found_child = true;
                    break;
                }
            }
            ASSERT_TRUE(found_child);
        }
    }
}

struct LoopLinksTestData
{
    std::string urdf_file;
    std::map<std::string, std::vector<std::string>> links_and_loops_links;
};

std::vector<LoopLinksTestData> GetLinksAndLoopLinks()
{
    std::vector<LoopLinksTestData> datas;

    LoopLinksTestData four_bar_data;
    four_bar_data.urdf_file = "four_bar";
    four_bar_data.links_and_loops_links.insert(std::make_pair("base_link",
                                                              std::vector<std::string>{}));
    four_bar_data.links_and_loops_links.insert(std::make_pair("link1",
                                                              std::vector<std::string>{}));
    four_bar_data.links_and_loops_links.insert(std::make_pair("link2",
                                                              std::vector<std::string>{"link3"}));
    four_bar_data.links_and_loops_links.insert(std::make_pair("link3",
                                                              std::vector<std::string>{"link1"}));
    datas.push_back(four_bar_data);

    LoopLinksTestData mini_cheetah_leg_data;
    mini_cheetah_leg_data.urdf_file = "mini_cheetah_leg";
    mini_cheetah_leg_data.links_and_loops_links.insert(std::make_pair("base", std::vector<std::string>{}));
    mini_cheetah_leg_data.links_and_loops_links.insert(std::make_pair("abduct", std::vector<std::string>{"abduct_rotor"}));
    mini_cheetah_leg_data.links_and_loops_links.insert(std::make_pair("abduct_rotor", std::vector<std::string>{"abduct"}));
    mini_cheetah_leg_data.links_and_loops_links.insert(std::make_pair("thigh", std::vector<std::string>{"hip_rotor"}));
    mini_cheetah_leg_data.links_and_loops_links.insert(std::make_pair("hip_rotor", std::vector<std::string>{"thigh"}));
    mini_cheetah_leg_data.links_and_loops_links.insert(std::make_pair("shank", std::vector<std::string>{"knee_rotor"}));
    mini_cheetah_leg_data.links_and_loops_links.insert(std::make_pair("knee_rotor", std::vector<std::string>{"shank"}));
    datas.push_back(mini_cheetah_leg_data);

    return datas;
}

class LoopLinksTest : public ::testing::TestWithParam<LoopLinksTestData>
{
protected:
    LoopLinksTest()
    {
        model_ = urdf::parseURDFFile(urdf_directory + GetParam().urdf_file + ".urdf");
    }
    std::shared_ptr<urdf::ModelInterface> model_;
};

INSTANTIATE_TEST_SUITE_P(LoopLinksTest, LoopLinksTest,
                         ::testing::ValuesIn(GetLinksAndLoopLinks()));

TEST_P(LoopLinksTest, loop_links)
{
    for (const auto &[link_name, loop_link_names] : GetParam().links_and_loops_links)
    {
        ASSERT_EQ(loop_link_names.size(), model_->getLink(link_name)->loop_links.size());
        for (const auto &loop_link_name : loop_link_names)
        {
            bool found_loop_link = false;
            for (const auto &loop_link : model_->getLink(link_name)->loop_links)
            {
                if (loop_link->name == loop_link_name)
                {
                    found_loop_link = true;
                    break;
                }
            }
            ASSERT_TRUE(found_loop_link);
        }
    }
}

using StrPair = std::pair<std::string, std::string>;
struct NearestCommonAncestorTestData
{
    std::string urdf_file;
    std::map<std::string, std::string> constraints_and_nearest_common_ancestors;
};

std::vector<NearestCommonAncestorTestData> GetConstraintsAndNearestCommonAncestors()
{
    std::vector<NearestCommonAncestorTestData> datas;

    NearestCommonAncestorTestData four_bar_data;
    four_bar_data.urdf_file = "four_bar";
    four_bar_data.constraints_and_nearest_common_ancestors.insert(std::make_pair("constraint1",
                                                                                 "base_link"));
    datas.push_back(four_bar_data);

    NearestCommonAncestorTestData mini_cheetah_leg_data;
    mini_cheetah_leg_data.urdf_file = "mini_cheetah_leg";
    mini_cheetah_leg_data.constraints_and_nearest_common_ancestors.insert(
        std::make_pair("abduct_transmission", "base"));
    mini_cheetah_leg_data.constraints_and_nearest_common_ancestors.insert(
        std::make_pair("hip_transmission", "abduct"));
    mini_cheetah_leg_data.constraints_and_nearest_common_ancestors.insert(
        std::make_pair("knee_transmission", "thigh"));
    datas.push_back(mini_cheetah_leg_data);

    return datas;
}

class NearestCommonAncestorTest : public ::testing::TestWithParam<NearestCommonAncestorTestData>
{
protected:
    NearestCommonAncestorTest()
    {
        model_ = urdf::parseURDFFile(urdf_directory + GetParam().urdf_file + ".urdf");
    }
    std::shared_ptr<urdf::ModelInterface> model_;
};

INSTANTIATE_TEST_SUITE_P(NearestCommonAncestorTest, NearestCommonAncestorTest,
                         ::testing::ValuesIn(GetConstraintsAndNearestCommonAncestors()));

TEST_P(NearestCommonAncestorTest, nearest_common_ancestor)
{
    for (const auto &[constraint_name, nearest_common_ancestor_name] :
         GetParam().constraints_and_nearest_common_ancestors)
    {
        urdf::ConstraintConstSharedPtr constraint = model_->getConstraint(constraint_name);
        ASSERT_EQ(constraint->nearest_common_ancestor_name, nearest_common_ancestor_name);
    }
}

// TODO(@MatthewChignoli): Add these features, then add the tests back in
// TEST(parser, combined_parse)
// {
//     std::vector<std::string> files;
//     files.push_back(urdf_directory + "mini_cheetah_base.urdf");
//     files.push_back(urdf_directory + "mini_cheetah_fr_leg.urdf");
//     files.push_back(urdf_directory + "mini_cheetah_fl_leg.urdf");
//     files.push_back(urdf_directory + "mini_cheetah_hr_leg.urdf");
//     files.push_back(urdf_directory + "mini_cheetah_hl_leg.urdf");
//     std::shared_ptr<ModelInterface> combined_model = parseURDFFiles(files, false);

//     std::shared_ptr<ModelInterface> model = parseURDFFile(urdf_directory + "mini_cheetah.urdf");

//     // Verify the models have the same size and root
//     ASSERT_EQ(combined_model->links_.size(), model->links_.size());
//     ASSERT_EQ(combined_model->joints_.size(), model->joints_.size());
//     ASSERT_EQ(combined_model->constraints_.size(), model->constraints_.size());
//     ASSERT_EQ(combined_model->getRoot()->name, model->getRoot()->name);

//     // Verify that the links are the same
//     for (const auto &link : combined_model->links_)
//     {
//         ASSERT_TRUE(model->getLink(link->name) != nullptr);
//     }
//     for (const auto &link : model->links_)
//     {
//         ASSERT_TRUE(combined_model->getLink(link->name) != nullptr);
//     }

//     // Verify that the joints are the same
//     for (const auto &joint : combined_model->joints_)
//     {
//         ASSERT_TRUE(model->getJoint(joint.second->name) != nullptr);
//     }
//     for (const auto &joint : model->joints_)
//     {
//         ASSERT_TRUE(combined_model->getJoint(joint.second->name) != nullptr);
//     }

//     // Verify that the constraints are the same
//     for (const auto &constraint : combined_model->constraints_)
//     {
//         ASSERT_TRUE(model->getConstraint(constraint.second->name) != nullptr);
//     }
//     for (const auto &constraint : model->constraints_)
//     {
//         ASSERT_TRUE(combined_model->getConstraint(constraint.second->name) != nullptr);
//     }
// }

class ExporterTest : public ::testing::TestWithParam<std::string>
{
    void SetUp() override
    {
        urdf_file_name = urdf_directory + GetParam() + ".urdf";
        exported_file_name = urdf_directory + GetParam() + "_exported.urdf";
    }

    void TearDown() override
    {
        std::remove(exported_file_name.c_str());
    }

protected:
    std::string urdf_file_name;
    std::string exported_file_name;
};

INSTANTIATE_TEST_SUITE_P(ExporterTest, ExporterTest, ::testing::ValuesIn(GetTestUrdfFiles()));

// TODO(@MatthewChignoli): This test does not make sure the exported file is valid/matches the original
TEST_P(ExporterTest, parseAndExport)
{
    std::shared_ptr<ModelInterface> model = parseURDFFile(urdf_file_name);
    ASSERT_TRUE(model != nullptr);
    tinyxml2::XMLDocument *xml_doc = exportURDF(model);
    ASSERT_TRUE(xml_doc != nullptr);
    xml_doc->SaveFile(exported_file_name.c_str());
    delete xml_doc;
}
