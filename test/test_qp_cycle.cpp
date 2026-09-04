// Real QP-cycle gate (A3): the OSQP backend must solve at least one actual
// condensed MPC cycle built from a real reference.  Registered only when
// LINEAR_MPC_WITH_OSQP is ON.
#include <Eigen/Dense>
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

#include "linear_mpc_controller/mpc/qp_problem.hpp"

using namespace linear_mpc_controller;

namespace
{
std::vector<TrackPoint> straightLine(double length, double v, double ds)
{
  std::vector<TrackPoint> pts;
  for (double s = 0.0; s <= length + 1e-9; s += ds) {
    TrackPoint p;
    p.s = s; p.x = s; p.y = 0.0; p.yaw = 0.0; p.kappa = 0.0; p.v = v;
    pts.push_back(p);
  }
  return pts;
}
}  // namespace

int main()
{
  MpcParams params;
  params.N = 25;
  auto traj = straightLine(20.0, 0.5, 0.05);
  assert(traj.size() > 100);

  CondensedMpcProblem problem(params, traj, 0.0);
  Eigen::Vector4d x0(0.05, 0.1, 0.0, 0.0);  // small lateral/heading error, at rest
  Eigen::MatrixXd H;
  Eigen::VectorXd q, l, u;
  Eigen::MatrixXd C;
  const int nv = problem.build(x0, H, q, C, l, u);
  assert(nv == 2 * params.N);

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(H);
  std::cout << "[gate] H min_eig=" << es.eigenvalues().minCoeff()
    << " max_eig=" << es.eigenvalues().maxCoeff() << " nv=" << nv << std::endl;
  std::cout << "[gate] H finite=" << H.allFinite() << " q finite=" << q.allFinite()
    << " C finite=" << C.allFinite() << std::endl;
  std::cout << "[gate] C rows=" << C.rows() << " cols=" << C.cols()
    << " l size=" << l.size() << " u size=" << u.size() << std::endl;

  auto solver = makeDefaultSolver(4000, params.qp_abs_tol, params.qp_rel_tol);
  auto sol = solver->solve(H, q, C, l, u, Eigen::VectorXd::Zero(nv));

  std::cout << "[gate] qp status=" << static_cast<int>(sol.status)
    << " iterations=" << sol.iterations
    << " time_us=" << sol.solve_time_us << std::endl;

  if (sol.status != QpSolution::Status::kSolved &&
    sol.status != QpSolution::Status::kApproximate)
  {
    std::cout << "[gate] FAIL: QP did not solve" << std::endl;
    return 1;
  }
  if (sol.u.size() != nv || !sol.u.allFinite()) {
    std::cout << "[gate] FAIL: non-finite solution" << std::endl;
    return 1;
  }
  std::cout << "[gate] PASS: real QP cycle solved" << std::endl;
  return 0;
}
