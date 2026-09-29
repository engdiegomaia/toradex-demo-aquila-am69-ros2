# Empty mount point for external model trees

This directory is **deliberately empty**. It is the default for the `sim`
service's `/maze/models` bind mount in `compose.host.yml`.

Compose has no conditional mount: the volume line either always exists or
never does. A default that points at a nonexistent path makes Compose
**create a directory** with that name; a default pointing at `/dev/null`
fails to mount. This directory is the default that always exists and never
brings anything with it.

When `MAZE_MODELS` is set in `.env`, it replaces this path and the maze mesh
appears. When it is not set, `/maze/models` stays empty, `GZ_SIM_RESOURCE_PATH`
points at a directory with no models, and nothing changes.

The `ros_maze_worlds` mesh is not vendored here because its upstream
`package.xml` declares `<license>TODO</license>`. See the header of
`ros2_ws/src/demo_simulation/worlds/quadruped_maze11.sdf` and
`docs/scenarios.md`.
