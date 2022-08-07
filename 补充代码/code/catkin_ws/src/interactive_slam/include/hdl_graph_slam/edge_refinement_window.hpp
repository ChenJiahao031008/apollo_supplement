#ifndef EDGE_REFINEMENT_WINDOW_HPP
#define EDGE_REFINEMENT_WINDOW_HPP

#include <mutex>
#include <atomic>
#include <thread>
#include <memory>
#include <random>
#include <boost/optional.hpp>

#include <imgui.h>
#include <hdl_graph_slam/robust_kernels.hpp>
#include <hdl_graph_slam/registration_methods.hpp>
#include <hdl_graph_slam/view/keyframe_view.hpp>
#include <hdl_graph_slam/view/interactive_graph_view.hpp>

namespace g2o {
class EdgeSE3;
}

namespace hdl_graph_slam {

struct EdgeInfo {
public:
  EdgeInfo(g2o::EdgeSE3* edge);

  bool operator<(const EdgeInfo& rhs) const;

  bool operator>(const EdgeInfo& rhs) const;

  double error() const;

  double score() const;

  void update();

public:
  g2o::EdgeSE3* edge;

  int begin;
  int end;
  int num_evaluations;
};

class EdgeRefinementWindow {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeRefinementWindow(std::shared_ptr<InteractiveGraphView>& graph);
  ~EdgeRefinementWindow();

  void draw_ui();

  void draw_gl(glk::GLSLShader& shader);

  void show();

  void start();

  void close();

private:
  void apply_robust_kernel();

  void refinement();
  void refinement_task();

private:
  bool show_window;
  std::shared_ptr<InteractiveGraphView>& graph;

  std::mt19937 mt;
  std::atomic_bool running;
  std::thread refinement_thread;

  std::mutex edges_mutex;
  std::vector<EdgeInfo> edges;
  g2o::EdgeSE3* inspected_edge;

  RegistrationMethods registration_method;
  RobustKernels robust_kernel;

  int optimization_cycle;
  int optimization_count;
};

}  // namespace hdl_graph_slam

#endif
