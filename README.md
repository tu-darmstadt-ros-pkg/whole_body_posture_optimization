# Whole body posture optimization

`whole_body_posture_optimization` optimizes flipper and arm postures along a path.
`whole_body_planning_server` plans and executes them as a `follow_path` action server.

## Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `publish_markers` | `true` | build and publish the debug markers on `~/markers` |

## Debug markers

`~/markers` (`visualization_msgs/MarkerArray`, e.g. `/whole_body_planning_server/markers`)
shows the optimized waypoints in the robot model frame. Each message starts with a DELETEALL.
Markers are built only while `~/markers` has a subscriber.
The planning server publishes the planned window (the newest 20 waypoints with continuous planning)
while a goal is active and publishes one DELETEALL when the goal succeeds, is canceled or is preempted.
While it holds during a goal it adds the `status` namespace. An aborted goal leaves only the `status` namespace,
naming why the robot stopped.

- `ghost_<k>`: robot at waypoint k, green when feasible, red when infeasible
- `planned_path`: optimized base positions, orange, failed waypoint red
- `reference_path`: requested path, grey
- `support_polygon`: support polygon edges, red to green by force angle stability
- `contacts`: contact points, yellow
- `com`: centre of mass and its projection to the ground, magenta
- `failed_pose`: requested pose of an infeasible waypoint, red arrow
- `terrain_surface` (planning server only): the planner's SDF zero level under the main tracks along the window,
  translucent blue, gaps where unknown
- `terrain_no_surface` (planning server only): red points 0.5 m below the base height where the known SDF has no
  surface that close, above it when the base lies in solid
- `status`: red sphere above the robot with the hold reason, `no map` (waiting for the first map),
  `no posture` (no support polygon) or `unstable` (stability below the limit) while an infeasible waypoint is retried.
  After an abort the same two, or `no plan`, `no trajectory`, `no timing`, `no controller`, `too slow`, `exec failed`,
  `follow failed` or `deviation`
