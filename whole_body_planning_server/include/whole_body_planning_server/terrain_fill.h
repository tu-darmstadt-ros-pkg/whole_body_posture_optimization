#ifndef WHOLE_BODY_PLANNING_SERVER_TERRAIN_FILL_H
#define WHOLE_BODY_PLANNING_SERVER_TERRAIN_FILL_H

// Fill of unobserved ESDF voxels around the robot with a 2.5D terrain prior, so
// that unknown space stops reading as free space (a drop) where the geometry
// says there is terrain. Works on a copy of the voxblox layer; observed voxels
// are never changed.

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <vector>

#include <Eigen/Geometry>
#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>

namespace whole_body_planning_server
{

namespace terrain_fill
{

using EsdfLayer = voxblox::Layer<voxblox::EsdfVoxel>;

/// Corridor along the robot's heading where the fill runs [m].
constexpr double kCorridorBack = 0.6;
constexpr double kCorridorAhead = 2.5;
constexpr double kCorridorHalfWidth = 0.6;
/// Vertical range around the base [m].
constexpr double kZBelow = 1.0;
constexpr double kZAbove = 1.2;
/// Footprint half extents [m] in the base frame. Columns under it without an
/// observed surface get the footprint plane (the sensor blind spot around the
/// tracks).
constexpr double kFootprintHalfLength = 0.4;
constexpr double kFootprintHalfWidth = 0.35;
/// Fully unknown columns behind an observed rising face (seen from the robot,
/// along its heading) get the face's top height for this depth [m].
constexpr double kFaceDepth = 0.4;
/// A face rises when its top lies at least kFaceMinRise [m] above the lowest
/// observed surface within kFaceWindow [m] before it on the same line.
constexpr double kFaceMinRise = 0.04;
constexpr double kFaceWindow = 0.3;

/// Top observed surface of one column, from the layer.
struct ColumnInfo {
  bool any_observed = false;
  std::optional<double> surface; // [m] height of the top observed surface
};

namespace detail
{
inline voxblox::GlobalIndex columnIndex( const EsdfLayer &layer, double x, double y, double z )
{
  const double inv = 1.0 / layer.voxel_size();
  return voxblox::GlobalIndex( static_cast<voxblox::LongIndexElement>( std::floor( x * inv ) ),
                               static_cast<voxblox::LongIndexElement>( std::floor( y * inv ) ),
                               static_cast<voxblox::LongIndexElement>( std::floor( z * inv ) ) );
}
} // namespace detail

/// Top surface of column (ix, iy) between voxel rows k_lo..k_hi: the highest
/// observed voxel with distance <= 0.6 voxel; its height is its centre minus
/// its distance.
inline ColumnInfo analyseColumn( const EsdfLayer &layer, voxblox::LongIndexElement ix,
                                 voxblox::LongIndexElement iy, voxblox::LongIndexElement k_lo,
                                 voxblox::LongIndexElement k_hi )
{
  ColumnInfo info;
  const double vs = layer.voxel_size();
  for ( voxblox::LongIndexElement k = k_hi; k >= k_lo; --k ) {
    const voxblox::EsdfVoxel *v = layer.getVoxelPtrByGlobalIndex( voxblox::GlobalIndex( ix, iy, k ) );
    if ( v == nullptr || !v->observed )
      continue;
    info.any_observed = true;
    if ( v->distance <= 0.6 * vs ) {
      info.surface = ( k + 0.5 ) * vs - v->distance;
      return info;
    }
  }
  return info;
}

/// Sets the unobserved voxels of column (ix, iy), rows k_lo..k_hi, to the
/// vertical signed distance to a surface at height h [m], clipped to
/// +-truncation.
inline void setColumn( EsdfLayer &layer, voxblox::LongIndexElement ix, voxblox::LongIndexElement iy,
                       voxblox::LongIndexElement k_lo, voxblox::LongIndexElement k_hi, double h,
                       double truncation )
{
  const double vs = layer.voxel_size();
  for ( voxblox::LongIndexElement k = k_lo; k <= k_hi; ++k ) {
    const double zc = ( k + 0.5 ) * vs;
    const voxblox::Point centre( ( ix + 0.5 ) * vs, ( iy + 0.5 ) * vs, zc );
    auto block = layer.allocateBlockPtrByCoordinates( centre );
    voxblox::EsdfVoxel &v = block->getVoxelByCoordinates( centre );
    if ( v.observed )
      continue;
    v.distance = static_cast<float>( std::clamp( zc - h, -truncation, truncation ) );
    v.observed = true;
    block->set_has_data( true );
  }
}

/// Copy of `in` with the unobserved voxels in the corridor around robot_pose
/// (base frame on the track bottom, x forward) filled: columns with an
/// observed surface below and above it, the footprint plane under the robot,
/// and the top of a rising face behind it.
inline std::shared_ptr<EsdfLayer> fill( const EsdfLayer &in, const Eigen::Isometry3d &robot_pose,
                                        double truncation )
{
  auto out = std::make_shared<EsdfLayer>( in );
  const double vs = in.voxel_size();
  const Eigen::Vector3d p = robot_pose.translation();
  const double yaw = std::atan2( robot_pose.linear()( 1, 0 ), robot_pose.linear()( 0, 0 ) );
  const Eigen::Vector2d fwd( std::cos( yaw ), std::sin( yaw ) );
  const Eigen::Vector2d side( -fwd.y(), fwd.x() );
  const voxblox::LongIndexElement k_lo = detail::columnIndex( in, 0, 0, p.z() - kZBelow ).z();
  const voxblox::LongIndexElement k_hi = detail::columnIndex( in, 0, 0, p.z() + kZAbove ).z();

  // Footprint plane: z(x, y) through the base origin, normal the base z axis.
  const Eigen::Vector3d n = robot_pose.linear().col( 2 );
  const auto plane = [&]( double x, double y ) {
    return p.z() - ( n.x() * ( x - p.x() ) + n.y() * ( y - p.y() ) ) / std::max( n.z(), 0.2 );
  };

  // Lines along the heading, one per voxel across the corridor; each line is a
  // sequence of distinct columns from kCorridorBack behind to kCorridorAhead.
  for ( double b = -kCorridorHalfWidth; b <= kCorridorHalfWidth + 1e-9; b += vs ) {
    std::vector<voxblox::GlobalIndex> cols;
    std::vector<double> along;
    for ( double a = -kCorridorBack; a <= kCorridorAhead + 1e-9; a += 0.5 * vs ) {
      const Eigen::Vector2d q = p.head<2>() + a * fwd + b * side;
      const voxblox::GlobalIndex c = detail::columnIndex( in, q.x(), q.y(), 0.0 );
      if ( !cols.empty() && cols.back().x() == c.x() && cols.back().y() == c.y() )
        continue;
      cols.push_back( c );
      along.push_back( a );
    }
    // Observed columns along the line ahead of the base: (along, surface height).
    std::vector<std::pair<double, double>> seen;
    std::optional<double> face_top;
    double face_along = 0.0;
    for ( size_t i = 0; i < cols.size(); ++i ) {
      const auto ix = cols[i].x(), iy = cols[i].y();
      const ColumnInfo info = analyseColumn( in, ix, iy, k_lo, k_hi );
      const double cx = ( ix + 0.5 ) * vs, cy = ( iy + 0.5 ) * vs;
      if ( info.surface ) {
        setColumn( *out, ix, iy, k_lo, k_hi, *info.surface, truncation );
        if ( along[i] >= 0.0 ) {
          double lowest = *info.surface;
          for ( const auto &s : seen )
            if ( s.first >= along[i] - kFaceWindow )
              lowest = std::min( lowest, s.second );
          if ( *info.surface >= lowest + kFaceMinRise ) {
            face_top = *info.surface;
            face_along = along[i];
          } else {
            face_top.reset();
          }
          seen.emplace_back( along[i], *info.surface );
        }
        continue;
      }
      const Eigen::Vector2d local( fwd.dot( Eigen::Vector2d( cx - p.x(), cy - p.y() ) ),
                                   side.dot( Eigen::Vector2d( cx - p.x(), cy - p.y() ) ) );
      const bool under = std::abs( local.y() ) <= kFootprintHalfWidth &&
                         std::abs( local.x() ) <= kFootprintHalfLength;
      if ( info.any_observed ) {
        // Observed free space without a surface is a real drop, except under
        // the robot, which stands there.
        face_top.reset();
        if ( under )
          setColumn( *out, ix, iy, k_lo, k_hi, plane( cx, cy ), truncation );
        continue;
      }
      if ( face_top && along[i] - face_along <= kFaceDepth ) {
        setColumn( *out, ix, iy, k_lo, k_hi, *face_top, truncation );
        continue;
      }
      if ( under )
        setColumn( *out, ix, iy, k_lo, k_hi, plane( cx, cy ), truncation );
    }
  }
  return out;
}

} // namespace terrain_fill
} // namespace whole_body_planning_server

#endif // WHOLE_BODY_PLANNING_SERVER_TERRAIN_FILL_H
