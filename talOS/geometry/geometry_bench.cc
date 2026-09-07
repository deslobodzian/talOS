// Microbenchmark for talOS/geometry: prints ns/op per primitive so the
// "fast" claim stays a measurement. Each loop chains iteration i into
// i+1 (or perturbs by i) so the compiler cannot hoist the work out.
// Run: bazel run //talOS/geometry:geometry_bench -c opt

#include <Eigen/Dense>
#include <chrono>
#include <cstdio>

#include "talOS/geometry/lie.h"

namespace {

using Clock = std::chrono::steady_clock;

struct Timer {
  Clock::time_point start = Clock::now();
  double elapsed_ns = 0;
  void stop(int iters) {
    elapsed_ns =
        std::chrono::duration<double, std::nano>(Clock::now() - start).count() /
        iters;
  }
};

}  // namespace

int main() {
  using talos::geometry::SE3;
  using talos::geometry::SO3;
  constexpr int kIters = 1000000;
  volatile double sink = 0.0;

  const SO3<double> r = SO3<double>::Exp(Eigen::Vector3d(0.1, -0.2, 0.3));
  const Eigen::Vector3d w(0.1, -0.2, 0.3);
  const Eigen::Vector3d p(0.5, -1.5, 2.5);
  const SE3<double> x = SE3<double>::Exp(
      (Eigen::Matrix<double, 6, 1>() << 0.1, -0.2, 0.3, 1.0, 2.0, -1.0)
          .finished());
  Eigen::Matrix<double, 6, 1> tau;
  tau << 0.01, 0.02, -0.01, 0.1, 0.2, 0.3;

  {
    Timer t;
    SO3<double> acc = r;
    for (int i = 0; i < kIters; ++i) acc = acc * r;
    t.stop(kIters);
    sink += acc.rotationMatrix()(0, 0);
    std::printf("so3_compose   %8.2f ns/op\n", t.elapsed_ns);
  }
  {
    Timer t;
    Eigen::Matrix3d acc = Eigen::Matrix3d::Zero();
    for (int i = 0; i < kIters; ++i) {
      acc += SO3<double>::Exp(w + Eigen::Vector3d::Constant(i * 1e-12))
                 .rotationMatrix();
    }
    t.stop(kIters);
    sink += acc(0, 0);
    std::printf("so3_exp       %8.2f ns/op\n", t.elapsed_ns);
  }
  {
    Timer t;
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();
    SO3<double> ri = r;
    for (int i = 0; i < kIters; ++i) {
      ri = SO3<double>(ri.rotationMatrix() * r.rotationMatrix());
      acc += ri.Log();
    }
    t.stop(kIters);
    sink += acc(0);
    std::printf("so3_log       %8.2f ns/op\n", t.elapsed_ns);
  }
  {
    Timer t;
    Eigen::Matrix3d acc = Eigen::Matrix3d::Zero();
    for (int i = 0; i < kIters; ++i) {
      acc += SO3<double>::RightJacobianOfExp(
          w + Eigen::Vector3d::Constant(i * 1e-12));
    }
    t.stop(kIters);
    sink += acc(0, 0);
    std::printf("so3_jr        %8.2f ns/op\n", t.elapsed_ns);
  }
  {
    Timer t;
    SE3<double> acc = x;
    for (int i = 0; i < kIters; ++i) acc = acc * x;
    t.stop(kIters);
    sink += acc.translation()(0);
    std::printf("se3_compose   %8.2f ns/op\n", t.elapsed_ns);
  }
  {
    Timer t;
    Eigen::Matrix4d acc = Eigen::Matrix4d::Zero();
    for (int i = 0; i < kIters; ++i) {
      acc += SE3<double>::Exp(tau +
                              Eigen::Matrix<double, 6, 1>::Constant(i * 1e-12))
                 .matrix();
    }
    t.stop(kIters);
    sink += acc(0, 0);
    std::printf("se3_exp       %8.2f ns/op\n", t.elapsed_ns);
  }
  {
    Timer t;
    Eigen::Matrix<double, 6, 1> acc = Eigen::Matrix<double, 6, 1>::Zero();
    SE3<double> xi = x;
    for (int i = 0; i < kIters; ++i) {
      xi = xi * x;
      acc += xi.Log();
    }
    t.stop(kIters);
    sink += acc(0);
    std::printf("se3_log       %8.2f ns/op\n", t.elapsed_ns);
  }
  {
    Timer t;
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d pi = p;
    for (int i = 0; i < kIters; ++i) {
      pi = x.act(pi);
      acc += pi;
    }
    t.stop(kIters);
    sink += acc(0);
    std::printf("se3_act       %8.2f ns/op\n", t.elapsed_ns);
  }
  {
    Timer t;
    Eigen::Matrix<double, 6, 6> acc = Eigen::Matrix<double, 6, 6>::Zero();
    for (int i = 0; i < kIters / 2; ++i) {
      acc += SE3<double>::RightJacobianOfExp(
          tau + Eigen::Matrix<double, 6, 1>::Constant(i * 1e-12));
    }
    t.stop(kIters / 2);
    sink += acc(0, 0);
    std::printf("se3_jr        %8.2f ns/op\n", t.elapsed_ns);
  }
  return sink == 42.0 ? 1 : 0;
}
