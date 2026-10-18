#include <gtest/gtest.h>

#include <whole_body_posture_optimization/util/common.h>
#include <whole_body_posture_optimization/util/worker_thread_pool.h>
#include <whole_body_posture_optimization/solvers/grid_search_optimizer.h>

#include <voxblox/core/esdf_map.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <future>

using namespace whole_body_posture_optimization;

namespace {

// Note: the force-angle stability metric expects the contact hull points in
// clockwise order (viewed from above) for positive-stable results.
std::vector<Vector3<double>> squareSupportPolygon(double half_size) {
  return {
    {half_size, -half_size, 0.0},
    {-half_size, -half_size, 0.0},
    {-half_size, half_size, 0.0},
    {half_size, half_size, 0.0},
  };
}

}

TEST(ForceAngleStability, stableForComInsidePolygon) {
  auto contact_points = squareSupportPolygon(0.5);
  Vector3<double> com(0.0, 0.0, 0.2);
  std::vector<double> stabilities = computeForceAngleStabilitiesWithGravity(contact_points, com);
  ASSERT_EQ(contact_points.size(), stabilities.size());
  for (double stability: stabilities) {
    EXPECT_GT(stability, 0.0);
  }
  // By symmetry, all edges have the same stability
  for (size_t i = 1; i < stabilities.size(); ++i) {
    EXPECT_NEAR(stabilities[0], stabilities[i], 1e-9);
  }
}

TEST(ForceAngleStability, unstableForComOutsidePolygon) {
  auto contact_points = squareSupportPolygon(0.5);
  // COM far beyond the +x edge
  Vector3<double> com(1.0, 0.0, 0.2);
  std::vector<double> stabilities = computeForceAngleStabilitiesWithGravity(contact_points, com);
  double min_stability = *std::min_element(stabilities.begin(), stabilities.end());
  EXPECT_LT(min_stability, 0.0);
}

TEST(ForceAngleStability, stabilityDecreasesWithComHeight) {
  auto contact_points = squareSupportPolygon(0.5);
  std::vector<double> low = computeForceAngleStabilitiesWithGravity(contact_points, {0.2, 0.0, 0.1});
  std::vector<double> high = computeForceAngleStabilitiesWithGravity(contact_points, {0.2, 0.0, 1.0});
  EXPECT_GT(*std::min_element(low.begin(), low.end()),
            *std::min_element(high.begin(), high.end()));
}

TEST(Common, lorentzian) {
  EXPECT_DOUBLE_EQ(0.0, lorentzian(0.0, 0.2));
  EXPECT_GT(lorentzian(0.5, 0.2), lorentzian(0.1, 0.2));
  EXPECT_DOUBLE_EQ(lorentzian(-0.3, 0.2), lorentzian(0.3, 0.2));
}

TEST(Common, gaussianDistribution) {
  double peak = gaussianDistribution(0.0, 0.0, 1.0);
  EXPECT_NEAR(1.0 / std::sqrt(2 * M_PI), peak, 1e-9);
  EXPECT_GT(peak, gaussianDistribution(1.0, 0.0, 1.0));
  EXPECT_NEAR(gaussianDistribution(-1.0, 0.0, 1.0), gaussianDistribution(1.0, 0.0, 1.0), 1e-12);
}

TEST(Common, getIndexMapAndMapListToSubset) {
  std::vector<std::string> list = {"a", "b", "c", "d"};
  std::vector<std::string> subset = {"c", "a"};
  std::vector<std::size_t> indices = getIndexMap(list, subset);
  ASSERT_EQ(2u, indices.size());
  EXPECT_EQ(2u, indices[0]);
  EXPECT_EQ(0u, indices[1]);

  std::vector<double> values = {1.0, 2.0, 3.0, 4.0};
  std::vector<double> subset_values = mapListToSubset(values, indices);
  ASSERT_EQ(2u, subset_values.size());
  EXPECT_DOUBLE_EQ(3.0, subset_values[0]);
  EXPECT_DOUBLE_EQ(1.0, subset_values[1]);
}

TEST(Common, quatToEulerMatchesEigenForYaw) {
  double yaw = 0.7;
  Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  Eigen::Vector3d euler = quatToEuler(q, "zyx");
  EXPECT_NEAR(yaw, euler(0), 1e-9);
  EXPECT_NEAR(0.0, euler(1), 1e-9);
  EXPECT_NEAR(0.0, euler(2), 1e-9);
}

namespace {

// f(x) = (x0 - 0.5)^2 + (x1 + 0.25)^2
struct QuadraticFunctor {
  template<typename T>
  bool operator()(const T* const x, T* residual) const {
    residual[0] = (x[0] - T(0.5)) * (x[0] - T(0.5)) + (x[1] + T(0.25)) * (x[1] + T(0.25));
    return true;
  }
};

}

TEST(GridSearchOptimizer, findsCoarseMinimum) {
  auto cost_function = std::make_shared<ceres_nlopt_wrapper::CeresCostFunctionWrapper>(
      new ceres::AutoDiffCostFunction<QuadraticFunctor, 1, 2>(new QuadraticFunctor()));

  nlopt::opt opt(nlopt::GN_DIRECT, 2);
  opt.set_lower_bounds({-1.0, -1.0});
  opt.set_upper_bounds({1.0, 1.0});

  GridSearchOptimizer optimizer;
  // resolution_ default is 0.1
  optimizer.loadSettingsFromNlopt(opt);
  optimizer.setMinFunction(cost_function);

  std::vector<double> parameters;
  double minf;
  optimizer.optimize(parameters, minf);

  ASSERT_EQ(2u, parameters.size());
  EXPECT_NEAR(0.5, parameters[0], 0.1);
  EXPECT_NEAR(-0.25, parameters[1], 0.1);
  EXPECT_LT(minf, 0.02);
}

TEST(WorkerThreadPool, executesAllTasks) {
  WorkerThreadPool pool(4);
  const int num_tasks = 100;
  std::atomic<int> counter{0};
  std::vector<std::promise<void>> promises(num_tasks);
  for (int i = 0; i < num_tasks; ++i) {
    pool.addTask([&counter, &promises, i]() {
      counter++;
      promises[i].set_value();
    });
  }
  for (auto& promise: promises) {
    promise.get_future().wait();
  }
  EXPECT_EQ(num_tasks, counter.load());
}

namespace {

/// Floor at z = 0 and a 0.02 m plate with its top at z = 0.2 over x >= 0, free space between them.
/// Positive distances are multiplied by overestimate, as a projective map can report them.
sdf_contact_estimation::SdfModel openStepModel(double overestimate = 1.0)
{
  voxblox::EsdfMap::Config config;
  config.esdf_voxel_size = 0.05;
  config.esdf_voxels_per_side = 16;
  auto esdf = std::make_shared<voxblox::EsdfMap>(config);
  voxblox::Layer<voxblox::EsdfVoxel>* layer = esdf->getEsdfLayerPtr();
  for (int bx = -2; bx < 2; ++bx) {
    for (int by = -2; by < 2; ++by) {
      for (int bz = -1; bz < 2; ++bz) {
        auto block = layer->allocateBlockPtrByIndex(voxblox::BlockIndex(bx, by, bz));
        for (size_t i = 0; i < block->num_voxels(); ++i) {
          const voxblox::Point c = block->computeCoordinatesFromLinearIndex(i);
          const double plate_z = c.z() > 0.2 ? c.z() - 0.2 : (c.z() < 0.18 ? 0.18 - c.z() : -std::min(0.2 - c.z(), c.z() - 0.18));
          const double plate = c.x() >= 0 ? plate_z : std::hypot(c.x(), std::max(plate_z, 0.0));
          const double distance = std::min(static_cast<double>(c.z()), plate);
          block->getVoxelByLinearIndex(i).distance = static_cast<float>(distance > 0.0 ? overestimate * distance : distance);
          block->getVoxelByLinearIndex(i).observed = true;
        }
      }
    }
  }
  sdf_contact_estimation::SdfModel model;
  model.loadEsdf(esdf, 0.4f, false);
  return model;
}

}

TEST(Common, highestSurfaceFindsTheTopOfAnOpenStep) {
  const auto model = openStepModel();
  EXPECT_NEAR(0.2, highestSurface(model, 0.3, 0.0, 0.45, -0.3), 0.03);
  EXPECT_NEAR(0.0, highestSurface(model, -0.4, 0.0, 0.45, -0.3), 0.03);
}

TEST(Common, highestSurfaceSkipsTheUndersideOfAPlate) {
  // Starting between floor and plate, as the previous height on the floor does
  const auto model = openStepModel();
  EXPECT_NEAR(0.0, highestSurface(model, 0.3, 0.0, 0.15, -0.3), 0.03);
}

TEST(Common, highestSurfaceFindsAThinPlateUnderOverstatedDistances) {
  const auto model = openStepModel(1.3);
  EXPECT_NEAR(0.2, highestSurface(model, 0.3, 0.0, 0.6, -0.3), 0.03);
}

TEST(Common, highestSurfaceIsNaNWhenStartingInSolid) {
  const auto model = openStepModel();
  EXPECT_TRUE(std::isnan(highestSurface(model, -0.4, 0.0, -0.1, -0.3)));
}

TEST(Common, highestSurfaceIsNaNWithoutASurfaceInRange) {
  const auto model = openStepModel();
  EXPECT_TRUE(std::isnan(highestSurface(model, -0.4, 0.0, 0.5, 0.1)));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
