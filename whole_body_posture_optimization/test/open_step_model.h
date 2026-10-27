#pragma once

#include <sdf_contact_estimation/sdf/sdf_model.h>
#include <voxblox/core/esdf_map.h>

#include <algorithm>
#include <cmath>
#include <memory>

/// Floor at z = 0 and a 0.02 m plate with its top at z = 0.2 over x >= 0, free space between them.
/// Positive distances are multiplied by overestimate, as a projective map can report them.
inline sdf_contact_estimation::SdfModel openStepModel(double overestimate = 1.0)
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
