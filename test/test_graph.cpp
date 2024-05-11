#include <sodf/graph/directed_graph.h>

#include <gtest/gtest.h>

using namespace sodf;
using namespace sodf::graph;

TEST(DirectedGraph, findShortestPath)
{
  enum nodes
  {
    A,
    B,
    C,
    D,
    E
  };

  std::vector<DirectedGraph::Edge> edges{ DirectedGraph::Edge(A, C), DirectedGraph::Edge(B, B),
                                          DirectedGraph::Edge(B, D), DirectedGraph::Edge(B, E),
                                          DirectedGraph::Edge(C, B), DirectedGraph::Edge(C, D),
                                          DirectedGraph::Edge(D, E), DirectedGraph::Edge(E, A),
                                          DirectedGraph::Edge(E, B) };
  std::vector<int> weights{ 1, 2, 1, 2, 7, 3, 1, 1, 1 };

  std::deque<int> path;

  DirectedGraph dg(std::move(edges), std::move(weights));
  bool rc = dg.findShortestPath(A, E, path);

  EXPECT_EQ(true, rc);
  EXPECT_EQ(4, path.size());
  EXPECT_EQ(0, path[0]);
  EXPECT_EQ(2, path[1]);
  EXPECT_EQ(3, path[2]);
  EXPECT_EQ(4, path[3]);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
