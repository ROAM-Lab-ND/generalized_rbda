#include "gtest/gtest.h"

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

struct LinkOrderTestdata
{
    std::string urdf_file;
    std::vector<std::string> link_order;
};

std::vector<LinkOrderTestdata> GetLinkOrders()
{
    std::vector<LinkOrderTestdata> datas;

    LinkOrderTestdata four_bar_data;
    four_bar_data.urdf_file = "four_bar";
    four_bar_data.link_order.push_back("base_link");
    four_bar_data.link_order.push_back("link1");
    four_bar_data.link_order.push_back("link2");
    four_bar_data.link_order.push_back("link3");
    datas.push_back(four_bar_data);

    LinkOrderTestdata mini_cheetah_leg_data;
    mini_cheetah_leg_data.urdf_file = "mini_cheetah_leg";
    mini_cheetah_leg_data.link_order.push_back("base");
    mini_cheetah_leg_data.link_order.push_back("abduct");
    mini_cheetah_leg_data.link_order.push_back("abduct_rotor");
    mini_cheetah_leg_data.link_order.push_back("thigh");
    mini_cheetah_leg_data.link_order.push_back("hip_rotor");
    mini_cheetah_leg_data.link_order.push_back("shank");
    mini_cheetah_leg_data.link_order.push_back("knee_rotor");
    datas.push_back(mini_cheetah_leg_data);

    LinkOrderTestdata mit_humanoid_leg_data;
    mit_humanoid_leg_data.urdf_file = "mit_humanoid_leg";
    mit_humanoid_leg_data.link_order.push_back("base");
    mit_humanoid_leg_data.link_order.push_back("hip_rz_link");
    mit_humanoid_leg_data.link_order.push_back("hip_rz_rotor");
    mit_humanoid_leg_data.link_order.push_back("hip_rx_link");
    mit_humanoid_leg_data.link_order.push_back("hip_rx_rotor");
    mit_humanoid_leg_data.link_order.push_back("hip_ry_link");
    mit_humanoid_leg_data.link_order.push_back("hip_ry_rotor");
    mit_humanoid_leg_data.link_order.push_back("knee_link");
    mit_humanoid_leg_data.link_order.push_back("knee_rotor");
    mit_humanoid_leg_data.link_order.push_back("ankle_rotor");
    mit_humanoid_leg_data.link_order.push_back("ankle_link");
    datas.push_back(mit_humanoid_leg_data);

    return datas;
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
    for (const auto &[link_name, children_names] : GetParam().links_and_children)
    {
        ASSERT_EQ(children_names.size(), model_->getLink(link_name)->child_links.size());
        for (const auto &child_name : children_names)
        {
            bool found_child = false;
            for (urdf::LinkConstSharedPtr child_link : model_->getLink(link_name)->child_links)
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

struct SupportingChainsTestData
{
    std::string urdf_file;
    std::map<std::string, std::vector<std::string>> links_and_supporting_chains;
};

std::vector<SupportingChainsTestData> GetLinksAndSupportingChains()
{
    std::vector<SupportingChainsTestData> datas;

    SupportingChainsTestData four_bar_data;
    four_bar_data.urdf_file = "four_bar";
    four_bar_data.links_and_supporting_chains.insert(std::make_pair("link1", std::vector<std::string>{"link1"}));
    four_bar_data.links_and_supporting_chains.insert(std::make_pair("link2", std::vector<std::string>{"link1", "link2"}));
    four_bar_data.links_and_supporting_chains.insert(std::make_pair("link3", std::vector<std::string>{"link3"}));
    datas.push_back(four_bar_data);

    SupportingChainsTestData mini_cheetah_leg_data;
    mini_cheetah_leg_data.urdf_file = "mini_cheetah_leg";
    mini_cheetah_leg_data.links_and_supporting_chains.insert(std::make_pair("abduct", std::vector<std::string>{"abduct"}));
    mini_cheetah_leg_data.links_and_supporting_chains.insert(std::make_pair("abduct_rotor", std::vector<std::string>{"abduct_rotor"}));
    mini_cheetah_leg_data.links_and_supporting_chains.insert(std::make_pair("thigh", std::vector<std::string>{"abduct", "thigh"}));
    mini_cheetah_leg_data.links_and_supporting_chains.insert(std::make_pair("hip_rotor", std::vector<std::string>{"abduct", "hip_rotor"}));
    mini_cheetah_leg_data.links_and_supporting_chains.insert(std::make_pair("shank", std::vector<std::string>{"abduct", "thigh", "shank"}));
    mini_cheetah_leg_data.links_and_supporting_chains.insert(std::make_pair("knee_rotor", std::vector<std::string>{"abduct", "thigh", "knee_rotor"}));
    datas.push_back(mini_cheetah_leg_data);

    return datas;
}

class SupportingChainsTest : public ::testing::TestWithParam<SupportingChainsTestData>
{
protected:
    SupportingChainsTest()
    {
        model_ = urdf::parseURDFFile(urdf_directory + GetParam().urdf_file + ".urdf");
    }
    std::shared_ptr<urdf::ModelInterface> model_;
};

INSTANTIATE_TEST_SUITE_P(SupportingChainsTest, SupportingChainsTest,
                         ::testing::ValuesIn(GetLinksAndSupportingChains()));

TEST_P(SupportingChainsTest, supporting_chains)
{
    for (const auto &[link_name, supporting_chain_names] : GetParam().links_and_supporting_chains)
    {
        // Build the supporting chain from the urdf model
        const std::string root_name = model_->getRoot()->name;
        std::string name = link_name;
        std::vector<LinkConstSharedPtr> supporting_chain;
        while (name != root_name)
        {
            supporting_chain.insert(supporting_chain.begin(), model_->getLink(name));
            name = model_->getLink(name)->getParent()->name;
        }

        ASSERT_EQ(supporting_chain_names.size(), supporting_chain.size());
        for (size_t i = 0; i < supporting_chain_names.size(); ++i)
        {
            ASSERT_EQ(supporting_chain_names[i], supporting_chain[i]->name);
        }
    }
}

struct NeighborsTestData
{
    std::string urdf_file;
    std::map<std::string, std::vector<std::string>> links_and_neighbors;
};

std::vector<NeighborsTestData> GetLinksAndNeighbors()
{
    std::vector<NeighborsTestData> datas;

    NeighborsTestData four_bar_data;
    four_bar_data.urdf_file = "four_bar";
    four_bar_data.links_and_neighbors.insert(std::make_pair("base_link", std::vector<std::string>{"link1", "link3"}));
    four_bar_data.links_and_neighbors.insert(std::make_pair("link1", std::vector<std::string>{"link2"}));
    four_bar_data.links_and_neighbors.insert(std::make_pair("link2", std::vector<std::string>{"link3"}));
    four_bar_data.links_and_neighbors.insert(std::make_pair("link3", std::vector<std::string>{"link1"}));
    datas.push_back(four_bar_data);

    NeighborsTestData mini_cheetah_leg_data;
    mini_cheetah_leg_data.urdf_file = "mini_cheetah_leg";
    mini_cheetah_leg_data.links_and_neighbors.insert(std::make_pair("base", std::vector<std::string>{"abduct", "abduct_rotor"}));
    mini_cheetah_leg_data.links_and_neighbors.insert(std::make_pair("abduct", std::vector<std::string>{"hip_rotor", "thigh", "abduct_rotor"}));
    mini_cheetah_leg_data.links_and_neighbors.insert(std::make_pair("abduct_rotor", std::vector<std::string>{"abduct"}));
    mini_cheetah_leg_data.links_and_neighbors.insert(std::make_pair("thigh", std::vector<std::string>{"shank", "knee_rotor", "hip_rotor"}));
    mini_cheetah_leg_data.links_and_neighbors.insert(std::make_pair("hip_rotor", std::vector<std::string>{"thigh"}));
    mini_cheetah_leg_data.links_and_neighbors.insert(std::make_pair("shank", std::vector<std::string>{"knee_rotor"}));
    mini_cheetah_leg_data.links_and_neighbors.insert(std::make_pair("knee_rotor", std::vector<std::string>{"shank"}));
    datas.push_back(mini_cheetah_leg_data);

    return datas;
}

class NeighborsTest : public ::testing::TestWithParam<NeighborsTestData>
{
protected:
    NeighborsTest()
    {
        model_ = urdf::parseURDFFile(urdf_directory + GetParam().urdf_file + ".urdf");
    }
    std::shared_ptr<urdf::ModelInterface> model_;
};

INSTANTIATE_TEST_SUITE_P(NeighborsTest, NeighborsTest,
                         ::testing::ValuesIn(GetLinksAndNeighbors()));

TEST_P(NeighborsTest, neighbors)
{
    // NOTE: The order of the neighbors matters

    for (const auto &[link_name, neighbors_names] : GetParam().links_and_neighbors)
    {
        std::vector<LinkConstSharedPtr> neighbors;
        for (const LinkConstSharedPtr child : model_->getLink(link_name)->child_links)
        {
            neighbors.push_back(child);
        }
        for (const LinkConstSharedPtr loop_link : model_->getLink(link_name)->loop_links)
        {
            neighbors.push_back(loop_link);
        }

        ASSERT_EQ(neighbors_names.size(), neighbors.size());
        int i = 0;
        for (const LinkConstSharedPtr &neighbor : neighbors)
        {
            ASSERT_EQ(neighbors_names[i], neighbor->name);
            i++;
        }
    }
}

class ClustersTest : public ::testing::TestWithParam<std::string>
{
protected:
    ClustersTest()
    {
        model_ = urdf::parseURDFFile(urdf_directory + GetParam() + ".urdf");
    }
    std::shared_ptr<urdf::ModelInterface> model_;
};

INSTANTIATE_TEST_SUITE_P(ClustersTest, ClustersTest, ::testing::ValuesIn(GetTestUrdfFiles()));

TEST_P(ClustersTest, parents)
{
    using LinkPtr = std::shared_ptr<Link>;
    using ClusterPtr = std::shared_ptr<Cluster>;

    std::vector<LinkPtr> links;
    model_->getLinks(links);

    for (const LinkPtr &link : links)
    {
        if (link->getParent() == nullptr)
            continue;

        LinkPtr parent = link->getParent();

        ClusterPtr cluster_containing_link =
            model_->clusters_[model_->containing_cluster_[link->name]];
        ClusterPtr cluster_containing_parent =
            model_->clusters_[model_->containing_cluster_[parent->name]];
        
        ClusterPtr parent_cluster_of_cluster_containing_link = cluster_containing_link->getParent();

        ASSERT_TRUE(cluster_containing_parent == cluster_containing_link ||
                    cluster_containing_parent == parent_cluster_of_cluster_containing_link);
    }
}

TEST_P(ClustersTest, children)
{
    // TODO(@MatthewChignoli): Remove these kinds of aliases
    using LinkPtr = std::shared_ptr<Link>;
    using ClusterPtr = std::shared_ptr<Cluster>;

    std::vector<LinkPtr> links;
    model_->getLinks(links);

    for (const LinkPtr &link : links)
    {
        ClusterSharedPtr cluster_containing_link =
            model_->clusters_[model_->containing_cluster_[link->name]];

        std::vector<ClusterPtr> child_clusters = cluster_containing_link->child_clusters;

        for (const LinkConstSharedPtr child_link : link->child_links)
        {
            ClusterSharedPtr cluster_containing_child_link =
                model_->clusters_[model_->containing_cluster_[child_link->name]];

            ASSERT_TRUE(cluster_containing_child_link == cluster_containing_link ||
                        std::find(child_clusters.begin(), child_clusters.end(), cluster_containing_child_link) != child_clusters.end());
        }
    }
}

// TODO(@MatthewChignoli): Add this test back in
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


// TODO(@MatthewChignoli): Add this test back in
// class ExporterTest : public ::testing::TestWithParam<std::string>
// {
//     void SetUp() override
//     {
//         urdf_file_name = urdf_directory + GetParam() + ".urdf";
//         exported_file_name = urdf_directory + GetParam() + "_exported.urdf";
//     }

//     void TearDown() override
//     {
//         std::remove(exported_file_name.c_str());
//     }

// protected:
//     std::string urdf_file_name;
//     std::string exported_file_name;
// };

// INSTANTIATE_TEST_SUITE_P(ExporterTest, ExporterTest, ::testing::ValuesIn(GetTestUrdfFiles()));

// TEST_P(ExporterTest, parseAndExport)
// {
//     std::shared_ptr<ModelInterface> model = parseURDFFile(urdf_file_name);
//     ASSERT_TRUE(model != nullptr);
//     tinyxml2::XMLDocument *xml_doc = exportURDF(model);
//     ASSERT_TRUE(xml_doc != nullptr);
//     xml_doc->SaveFile(exported_file_name);
//     delete xml_doc;
// }
