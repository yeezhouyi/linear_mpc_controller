#include "linear_mpc_controller/mpc/qp_problem.hpp"

#include <cmath>
#include <limits>

namespace linear_mpc_controller
{

namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();

TrackPoint sampleByArcRef(const std::vector<TrackPoint> & traj, double s)
{
  return sampleByArc(traj, s);
}

void buildLtvWindow(const std::vector<TrackPoint> & traj, double base_arc,
  const MpcParams & p, std::vector<Eigen::Matrix4d> & A_d,
  std::vector<Eigen::Matrix<double, 4, 2>> & B_d, std::vector<TrackPoint> & anchors)
{
  A_d.resize(p.N);
  B_d.resize(p.N);
  anchors.resize(p.N + 1);
  std::vector<double> arcs(p.N + 1);
  double s = base_arc;
  for (int j = 0; j <= p.N; ++j) {
    arcs[j] = s;
    if (s >= traj.back().s) break;
    const TrackPoint pt = sampleByArcRef(traj, s);
    s += p.Ts * std::max(pt.v, 0.0);
  }
  for (int j = p.N; j >= 0; --j) {
    if (arcs[j] >= traj.back().s || arcs[j] < 0.0) arcs[j] = traj.back().s;
  }
  for (int j = 0; j <= p.N; ++j) anchors[j] = sampleByArcRef(traj, arcs[j]);
  for (int j = 0; j < p.N; ++j) {
    Eigen::Matrix4d A_c;
    Eigen::Matrix<double, 4, 2> B_c;
    const TrackPoint & pt = anchors[j];
    linearContinuousMatrices(std::max(pt.v, 1e-9), pt.kappa, A_c, B_c);
    discretizeZoh(A_c, B_c, p.Ts, A_d[j], B_d[j]);
  }
}
}  // namespace

CondensedMpcProblem::CondensedMpcProblem(const MpcParams & p,
  const std::vector<TrackPoint> & traj, double base_arc)
: params_(p)
{
  buildLtvWindow(traj, base_arc, p, A_d_, B_d_, anchors_);
}

Eigen::Vector4d CondensedMpcProblem::referenceState(int k) const
{
  Eigen::Vector4d ref = Eigen::Vector4d::Zero();
  const TrackPoint & a = anchors_[k + 1];
  ref(kV) = a.v;
  ref(kOmega) = a.kappa * a.v;
  return ref;
}

int CondensedMpcProblem::build(const Eigen::Vector4d & x0, Eigen::MatrixXd & H,
  Eigen::VectorXd & q, Eigen::MatrixXd & C, Eigen::VectorXd & l, Eigen::VectorXd & u) const
{
  const int N = params_.N;
  const int nv = 2 * N;
  const int nx4 = 4 * N;

  // Prediction matrices F (4N x 4), G (4N x 2N)
  Eigen::MatrixXd F = Eigen::MatrixXd::Zero(nx4, 4);
  Eigen::MatrixXd G = Eigen::MatrixXd::Zero(nx4, nv);
  F.block<4, 4>(0, 0) = A_d_[0];
  G.block<4, 2>(0, 0) = B_d_[0];
  for (int k = 1; k < N; ++k) {
    F.block<4, 4>(4 * k, 0) = A_d_[k] * F.block<4, 4>(4 * (k - 1), 0);
    G.block(4 * k, 0, 4, 2 * k) = A_d_[k] * G.block(4 * (k - 1), 0, 4, 2 * k);
    G.block<4, 2>(4 * k, 2 * k) = B_d_[k];
  }

  // Block-diagonal weights
  Eigen::MatrixXd Qbar = Eigen::MatrixXd::Zero(nx4, nx4);
  for (int k = 0; k < N; ++k) {
    Qbar.block<4, 4>(4 * k, 4 * k) = (k == N - 1) ? params_.Q_F : params_.Q;
  }
  Eigen::MatrixXd Sbar = Eigen::MatrixXd::Zero(nv, nv);
  for (int k = 0; k < N; ++k) Sbar.block<2, 2>(2 * k, 2 * k) = params_.S;

  // Reference stack for predicted states 1..N
  Eigen::VectorXd Xref = Eigen::VectorXd::Zero(nx4);
  for (int k = 0; k < N; ++k) Xref.segment<4>(4 * k) = referenceState(k);

  const Eigen::VectorXd x_free = F * x0 - Xref;
  H = (G.transpose() * Qbar * G + Sbar).eval();
  q = G.transpose() * Qbar * x_free;

  // Constraints: rows l <= C z <= u
  const int n_rows = 8 * N;  // v lo/hi, omega lo/hi (4N) + accel box (4N)
  C = Eigen::MatrixXd::Zero(n_rows, nv);
  l = Eigen::VectorXd::Constant(n_rows, -kInf);
  u = Eigen::VectorXd::Constant(n_rows, kInf);

  int row = 0;
  auto add_row = [&](const Eigen::RowVectorXd & c, double lo, double hi) {
    C.row(row) = c;
    l(row) = lo;
    u(row) = hi;
    ++row;
  };
  for (int k = 0; k < N; ++k) {
    const Eigen::RowVectorXd Gv = G.row(4 * k + kV);
    const Eigen::RowVectorXd Gw = G.row(4 * k + kOmega);
    const double Fv = F(4 * k + kV, 0) * x0(0) + F(4 * k + kV, 1) * x0(1) +
      F(4 * k + kV, 2) * x0(2) + F(4 * k + kV, 3) * x0(3);
    const double Fw = F(4 * k + kOmega, 0) * x0(0) + F(4 * k + kOmega, 1) * x0(1) +
      F(4 * k + kOmega, 2) * x0(2) + F(4 * k + kOmega, 3) * x0(3);
    add_row(Gv, -kInf, params_.v_max - Fv);
    add_row(-Gv, -kInf, -params_.v_min + Fv);
    add_row(Gw, -kInf, params_.omega_max - Fw);
    add_row(-Gw, -kInf, params_.omega_max + Fw);
  }
  for (int k = 0; k < N; ++k) {
    Eigen::RowVectorXd ea = Eigen::RowVectorXd::Zero(nv);
    ea(2 * k) = 1.0;
    Eigen::RowVectorXd eal = Eigen::RowVectorXd::Zero(nv);
    eal(2 * k + 1) = 1.0;
    add_row(ea, -kInf, params_.a_max);
    add_row(-ea, -kInf, params_.a_max);
    add_row(eal, -kInf, params_.alpha_max);
    add_row(-eal, -kInf, params_.alpha_max);
  }
  return nv;
}

namespace
{
/// Solver used when OSQP is unavailable (all solves fail; model/fallback
/// tests still build and run).
class UnavailableSolver : public QpSolver
{
public:
  QpSolution solve(const Eigen::MatrixXd &, const Eigen::VectorXd &, const Eigen::MatrixXd &,
    const Eigen::VectorXd &, const Eigen::VectorXd &, const Eigen::VectorXd &) override
  {
    QpSolution sol;
    sol.status = QpSolution::Status::kFailed;
    return sol;
  }
};
}  // namespace

std::unique_ptr<QpSolver> makeDefaultSolver(int max_iter, double abs_tol, double rel_tol)
{
#ifndef LINEAR_MPC_HAVE_OSQP
  (void)max_iter;
  (void)abs_tol;
  (void)rel_tol;
  return std::make_unique<UnavailableSolver>();
#else
  // Forward-declared real factory implemented in the OSQP-enabled build.
  extern std::unique_ptr<QpSolver> makeOsqpSolver(int max_iter, double abs_tol, double rel_tol);
  return makeOsqpSolver(max_iter, abs_tol, rel_tol);
#endif
}

}  // namespace linear_mpc_controller
