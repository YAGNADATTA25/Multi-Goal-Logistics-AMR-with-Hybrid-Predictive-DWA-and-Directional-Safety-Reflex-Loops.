# Gazebo model attribution

The assets below are from the [OSRF Gazebo model database](https://github.com/osrf/gazebo_models), distributed under the Creative Commons Attribution 3.0 Unported license. The upstream license is reproduced in LICENSE.gazebo_models. Original author credits are retained here and in the existing walker/model.config.

## Person mesh

- Upstream model: [person_walking](https://github.com/osrf/gazebo_models/tree/master/person_walking).
- Author: Marina Kollmitz; created with MakeHuman, as recorded by the upstream model.config.
- Existing project assets: walker/meshes/walking.dae and walker/materials/textures/.
- Changes in this integration: fix model URIs to use the existing walker folder; install the model assets; reference the mesh from two moving person models with simplified cylindrical collision geometry.
- The mixed world uses a fixed walking pose. It does not add skeletal animation.

## Four-wheel carts

- Upstream model: [cart_rigid_suspension](https://github.com/osrf/gazebo_models/tree/master/cart_rigid_suspension).
- Author: Steve Peters.
- Source model.sdf blob: f42d097a58f528cfdb1fbfa3487c856fd09a306d.
- The two cart definitions in ../worlds/warehouse_mixed.world retain the upstream chassis, wheels, joints, and inertial data.
- Changes: unique names and starting poses, a visible box payload with collision geometry, and the existing gazebo_ros_planar_move driver for each obstacle topic.
- Payload mass is represented by the chassis's existing lumped mass; this scene is for navigation testing rather than load-dynamics validation.
