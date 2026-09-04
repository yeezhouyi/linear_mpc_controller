// OSQP-backed dense QP solver (U3, KTD12).  The condensed MPC problem is
// small (n = 2N vars), so we re-setup the workspace per solve instead of
// maintaining osqp_update_data bookkeeping; qp_time_us in the diagnostics
// reports the real cost of that choice.
#include "linear_mpc_controller/mpc/qp_problem.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <osqp.h>

#include <Eigen/Sparse>

#include <vector>

namespace linear_mpc_controller
{
namespace
{

c_int toCsc(const Eigen::MatrixXd & dense, bool upper_only, std::vector<c_float> & values,
  std::vector<c_int> & row_idx, std::vector<c_int> & col_ptr)
{
  const c_int rows = static_cast<c_int>(dense.rows());
  const c_int cols = static_cast<c_int>(dense.cols());
  col_ptr.assign(cols + 1, 0);
  for (c_int j = 0; j < cols; ++j) {
    for (c_int i = 0; i < rows; ++i) {
      if (upper_only && i > j) continue;
      const double v = dense(i, j);
      if (v == 0.0) continue;
      values.push_back(v);
      row_idx.push_back(i);
    }
    col_ptr[j + 1] = static_cast<c_int>(values.size());
  }
  return cols;
}

class OsqpSolver : public QpSolver
{
public:
  OsqpSolver(int max_iter, double abs_tol, double rel_tol)
  : max_iter_(max_iter), abs_tol_(abs_tol), rel_tol_(rel_tol) {}

  QpSolution solve(const Eigen::MatrixXd & H, const Eigen::VectorXd & q,
    const Eigen::MatrixXd & C, const Eigen::VectorXd & l, const Eigen::VectorXd & u,
    const Eigen::VectorXd & warm_start) override
  {
    QpSolution sol;
    const auto t0 = std::chrono::steady_clock::now();

    std::vector<c_float> px, ax;
    std::vector<c_int> pi, pp, ai, ap;
    toCsc(H.selfadjointView<Eigen::Upper>(), true, px, pi, pp);
    toCsc(C, false, ax, ai, ap);
    const c_int n = static_cast<c_int>(H.cols());
    const c_int m = static_cast<c_int>(C.rows());

    fprintf(stderr, "[osqp_solver] n=%d m=%d nnzP=%d nnzA=%d\n", (int)n, (int)m, (int)px.size(), (int)ax.size());
    std::cout << "[osqp_solver] n=" << n << " m=" << m << " nnzP=" << px.size() << " nnzA=" << ax.size() << std::endl;
    std::vector<c_float> qv(q.data(), q.data() + q.size());
    std::vector<c_float> lv(l.data(), l.data() + l.size());
    std::vector<c_float> uv(u.data(), u.data() + u.size());
    // OSQP 0.6.2 + IEEE inf in l/u corrupts the heap on this vendor build;
    // saturate to large finite bounds instead (|v| > 1e20 means inf).
    for (auto & v : lv) if (v < -1e20) v = -1e20;
    for (auto & v : uv) if (v > 1e20) v = 1e20;

    // CSC self-check: catch inconsistent col_ptr before OSQP copies data
    auto validate = [](const std::vector<c_int> & p, const std::vector<c_int> & i,
      const std::vector<c_float> & x, c_int cols) {
      if (static_cast<c_int>(p.size()) != cols + 1) return "p size mismatch";
      if (p.front() != 0 || p.back() != static_cast<c_int>(x.size())) return "p ends mismatch";
      if (static_cast<c_int>(i.size()) != static_cast<c_int>(x.size())) return "i size mismatch";
      for (c_int j = 0; j < cols; ++j)
        if (p[j] > p[j + 1]) return "p not monotonic";
      for (const auto v : x) if (!std::isfinite(v)) return "non-finite value";
      return "";
    };
    const char *errP = validate(pp, pi, px, n);
    const char *errA = validate(ap, ai, ax, static_cast<c_int>(C.cols()));
    std::cout << "[osqp_solver] validate P=" << (errP ? errP : "ok")
      << " A=" << (errA ? errA : "ok") << std::endl;

    OSQPData data{};
    data.n = n;
    data.m = m;
    data.P = csc_matrix(n, n, static_cast<c_int>(px.size()), px.data(), pi.data(), pp.data());
    data.A = csc_matrix(m, n, static_cast<c_int>(ax.size()), ax.data(), ai.data(), ap.data());
    data.q = qv.data();
    data.l = lv.data();
    data.u = uv.data();

    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.verbose = 0;
    settings.max_iter = max_iter_;
    settings.eps_abs = abs_tol_;
    settings.eps_rel = rel_tol_;
    settings.polish = 1;
    settings.warm_start = 1;

    OSQPWorkspace * work = nullptr;
        const c_int ret = osqp_setup(&work, &data, &settings);
    std::cout << "[osqp_solver] setup ret=" << ret << std::endl;
    fprintf(stderr, "[osqp_solver] setup ret=%d work=%p\n", (int)ret, (void*)work);
    if (ret != 0 || work == nullptr) {
      sol.status = QpSolution::Status::kFailed;
      return sol;
    }

    // warm start from the previous stacked solution when the layout matches
    if (warm_start.size() == static_cast<Eigen::Index>(n)) {
      std::vector<c_float> w(warm_start.data(), warm_start.data() + n);
      osqp_warm_start_x(work, w.data());
    }

    const c_int st = osqp_solve(work);
    const c_int status_val = (st == 0) ? work->info->status_val : st;
    const auto t1 = std::chrono::steady_clock::now();


    if (status_val != OSQP_SOLVED && status_val != OSQP_SOLVED_INACCURATE) {
      std::cout << "[osqp_solver] solve failed status_val=" << status_val
        << " iter=" << work->info->iter << std::endl;
    }

    if (status_val == OSQP_SOLVED || status_val == OSQP_SOLVED_INACCURATE) {
      sol.status = (status_val == OSQP_SOLVED) ? QpSolution::Status::kSolved :
        QpSolution::Status::kApproximate;
      sol.u = Eigen::VectorXd::Zero(n);
      for (c_int i = 0; i < n; ++i) {
        sol.u(i) = static_cast<double>(work->solution->x[i]);
      }
      sol.iterations = static_cast<int>(work->info->iter);
      sol.objective = static_cast<double>(work->info->obj_val);
      sol.pri_res = static_cast<double>(work->info->pri_res);
      sol.dua_res = static_cast<double>(work->info->dua_res);
    }

    osqp_cleanup(work);
    sol.solve_time_us =
      std::chrono::duration<double, std::micro>(t1 - t0).count();
    return sol;
  }

private:
  int max_iter_;
  double abs_tol_;
  double rel_tol_;
};

}  // namespace

std::unique_ptr<QpSolver> makeOsqpSolver(int max_iter, double abs_tol, double rel_tol)
{
  return std::make_unique<OsqpSolver>(max_iter, abs_tol, rel_tol);
}

}  // namespace linear_mpc_controller
