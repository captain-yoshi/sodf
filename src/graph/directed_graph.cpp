#include <sodf/graph/directed_graph.h>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/config.hpp>

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/tuple/tuple.hpp>

#include <fstream>

namespace sodf {
namespace graph {

namespace {

struct my_visitor : boost::default_dijkstra_visitor
{
  using base = boost::default_dijkstra_visitor;
  struct done
  {
  };

  my_visitor(DirectedGraph::V vd, size_t& visited) : destination(vd), visited(visited)
  {
  }

  void finish_vertex(DirectedGraph::V v, DirectedGraph::G const& g)
  {
    ++visited;

    if (v == destination)
      throw done{};

    base::finish_vertex(v, g);
  }

private:
  DirectedGraph::V destination;
  size_t& visited;
};

}  // namespace

DirectedGraph::DirectedGraph(std::vector<Edge>&& edges, std::vector<int>&& weights)
  : edges(std::move(edges)), weights(std::move(weights))
{
  g = G(this->edges.begin(), this->edges.end(), this->weights.begin(), 0);
  weightmap = boost::get(boost::edge_weight, g);
  p = std::vector<V>(num_vertices(g));
  colors = std::make_unique<std::vector<boost::default_color_type>>(num_vertices(g), boost::default_color_type{});
  _pred = std::make_unique<std::vector<V>>(num_vertices(g), g.null_vertex());
  _dist = std::make_unique<std::vector<size_t>>(num_vertices(g), -1ull);

  predmap = _pred->data();  // interior properties: boost::get(boost::vertex_predecessor, g);
  distmap = _dist->data();  // interior properties: boost::get(boost::vertex_distance, g);
}

bool DirectedGraph::findShortestPath(int start, int end, std::deque<int>& path)
{
  V start_vertex = vertex(start, g);
  V end_vertex = vertex(end, g);

  size_t visited = 0;

  my_visitor vis{ end_vertex, visited };

  try
  {
    std::cout << "Searching from #" << start_vertex << " to #" << end_vertex << "...\n";
    boost::dijkstra_shortest_paths(g, start_vertex,
                                   boost::visitor(vis)
                                       .color_map(colors->data())
                                       .distance_map(distmap)
                                       .predecessor_map(predmap)
                                       .weight_map(weightmap));

    std::cout << "No path found\n";
    return false;
  }
  catch (my_visitor::done const&)
  {
    std::cout << visited << std::endl;
    std::cout << num_vertices(g) << std::endl;
    std::cout << "Percentage skipped: " << (100.0 * (num_vertices(g) - visited) / num_vertices(g)) << "%\n";
  }

  size_t distance = distmap[end_vertex];
  std::cout << "Distance from #" << start_vertex << " to #" << end_vertex << ": " << distance << "\n";

  if (distance != size_t(-1))
  {
    // std::deque<V> path_v;
    path.push_front(end_vertex);
    for (V current = end_vertex; current != g.null_vertex() && predmap[current] != current && current != start_vertex;)
    {
      path.push_front(predmap[current]);
      current = predmap[current];
    }

    std::cout << "Path from #" << start_vertex << " to #" << end_vertex << ": ";
    // std::copy(path.begin(), path.end(), std::ostream_iterator<V>(std::cout, ", "));
    std::cout << end_vertex << "\n";

    return true;
  }
  return false;
}

void DirectedGraph::saveToDOTFile(const std::string& path, const std::vector<std::string>& node_names)
{
  std::ofstream dot_file(path);

  dot_file << "digraph D {\n"
           << "  rankdir=LR\n"
           << "  size=\"4,3\"\n"
           << "  ratio=\"fill\"\n"
           << "  edge[style=\"bold\"]\n"
           << "  node[shape=\"circle\"]\n";

  boost::graph_traits<G>::edge_iterator ei, ei_end;
  for (tie(ei, ei_end) = boost::edges(g); ei != ei_end; ++ei)
  {
    boost::graph_traits<G>::edge_descriptor e = *ei;
    boost::graph_traits<G>::vertex_descriptor u = source(e, g), v = target(e, g);

    if (u < node_names.size() && v < node_names.size())
    {
      std::cout << "node names size is smaller then the graph nodes" << std::endl;
      return;
    }

    dot_file << node_names[u] << " -> " << node_names[v] << "[label=\"" << get(weightmap, e) << "\"";
    if (p[v] == u)
      dot_file << ", color=\"black\"";
    else
      dot_file << ", color=\"grey\"";
    dot_file << "]";
  }
  dot_file << "}";
  dot_file.close();
}

}  // namespace graph
}  // namespace sodf
