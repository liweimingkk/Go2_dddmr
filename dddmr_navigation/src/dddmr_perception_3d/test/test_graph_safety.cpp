#include "perception_3d/dynamic_graph.h"
#include "perception_3d/static_graph.h"

#include <gtest/gtest.h>

TEST(StaticGraphSafety, CopiesOwnStorageWithoutAliasing)
{
  perception_3d::StaticGraph original;
  original.allocateGraph(2);
  edge_t edge{1U, 0.5F};
  original.insertEdgeInNode(0U, edge);
  original.setPenality(0U, 2.0F);

  perception_3d::StaticGraph copy = original;
  copy.clear();

  EXPECT_EQ(original.getSize(), 2U);
  EXPECT_EQ(original.getEdge(0U).size(), 1U);
  EXPECT_FLOAT_EQ(original.getNodeWeight(0U), 2.0F);
  EXPECT_EQ(copy.getSize(), 0U);
}

TEST(StaticGraphSafety, MissingQueriesDoNotMutateGraph)
{
  perception_3d::StaticGraph graph;
  graph.allocateGraph(1);
  EXPECT_TRUE(graph.getEdge(99U).empty());
  EXPECT_FLOAT_EQ(graph.getNodeWeight(99U), 0.0F);
  EXPECT_EQ(graph.getSize(), 1U);
}

TEST(DynamicGraphSafety, AllocatesExactlyGroundSize)
{
  perception_3d::DynamicGraph graph;
  graph.initial(2U, 10.0);
  EXPECT_EQ(graph.getdGraphSize(), 2U);
  EXPECT_DOUBLE_EQ(graph.getValue(0U), 10.0);
  EXPECT_DOUBLE_EQ(graph.getValue(1U), 10.0);
  EXPECT_DOUBLE_EQ(graph.getValue(2U), 0.0);
  EXPECT_EQ(graph.getdGraphSize(), 2U);
}

TEST(DynamicGraphSafety, InvalidWritesDoNotCreateNodes)
{
  perception_3d::DynamicGraph graph;
  graph.initial(1U, 10.0);
  graph.setValue(4U, 1.0);
  graph.clearValue(5U, 9.0);
  EXPECT_EQ(graph.getdGraphSize(), 1U);
  graph.setValue(0U, 3.0);
  EXPECT_DOUBLE_EQ(graph.getValue(0U), 3.0);
}
