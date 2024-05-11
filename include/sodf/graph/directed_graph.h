#ifndef DIRECTED_GRAPH_H_
#define DIRECTED_GRAPH_H_

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/property_maps/constant_property_map.hpp>

namespace sodf {
namespace graph {

class DirectedGraph
{
public:
  typedef std::pair<int, int> Edge;
  using G = boost::adjacency_list<boost::listS, boost::vecS, boost::directedS, boost::no_property,
                                  boost::property<boost::edge_weight_t, int>>;
  using V = G::vertex_descriptor;
  using E = G::edge_descriptor;

  DirectedGraph(std::vector<Edge>&& edges, std::vector<int>&& weights);

  bool findShortestPath(int start, int end, std::deque<int>& path);

  void saveToDOTFile(const std::string& path, const std::vector<std::string>& node_names);

protected:
private:
  std::vector<Edge> edges;
  std::vector<int> weights;

  G g;

  boost::property_map<G, boost::edge_weight_t>::type weightmap;
  std::unique_ptr<std::vector<boost::default_color_type>> colors;
  std::unique_ptr<std::vector<V>> _pred;
  std::unique_ptr<std::vector<size_t>> _dist;
  std::vector<V> p;

  V* predmap;
  std::size_t* distmap;
};

}  // namespace graph
}  // namespace sodf

#endif  // DIRECTED_GRAPH_H_
