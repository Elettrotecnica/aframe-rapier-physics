# Rapier JavaScript binding

[rapier.js](https://github.com/dimforge/rapier.js/) is the official JavaScript distribution of the Rapier engine.

## Contents

- [Considerations](#considerations-before-use)
- [Installation](#installation)
- [Basics](#basics)
- [System](#rapier-physics)
- [Components](#components)
  - [`rapier-body`](#rapier-body)
  - [`rapier-shape`](#rapier-shape)
  - [`rapier-constraint`](#rapier-constraint)
- [Using the Rapier API](#using-the-rapier-api)
- [Events](#events)
- [Statistics](#statistics)

## Considerations Before Use

The components implemented here provide a simple interface to
introduce physics in your WebXR experience. Rapier is a feature-rich
physics engine suitable for a variety of use-cases, but because of the
nature of WebXR, we have focused exclusively on that of physics in a
3D world. Even then, some features such as vehicle controllers or
other stuff that may be relevant in a 3D simulation have not been
integrated. This may change in the future or not, depending on demand
and contributions.

The physics system exposes handles to the World and EventQueue
objects, so an option is to plug into those in your custom components
to introduce new features. The components for body and shape also
provide a handle to their respective body and colliders and can be
used to extend the vanilla behavior.

## Installation

To include the library in your project, place it after the A-Frame one, for instance:
```
<head>
...
   <script src="https://aframe.io/releases/1.7.1/aframe.min.js"></script>
   ...
   <script type="module" src="../aframe-rapier.js"></script>
...
</head>
```

Notice the *type* attribute set to *module*. This is required because
Rapier must be initialized asyncronously.

The Rapier JavaScript bundle we use is currently hardcoded in the
system file. This may change in the future, but has the advantage that
one does not need anything else.

## Basics

To begin using the driver, add the `rapier-physics` system on the
`a-scene`. `debug: true` will enable wireframe debugging of physics
shapes/bodies.

```html
<a-scene rapier-physics="debug: true;">
  <!-- ... -->
</a-scene>
```

To create a physics body, both a `rapier-body` and at least one
`rapier-shape` component should be added to an entity.

```html
<!-- Fixed box -->
<a-box position="0 0.5 -5" width="3" height="1" depth="1" rapier-body="type: Fixed" rapier-shape></a-box>

<!-- Dynamic box -->
<a-box position="5 0.5 0" width="1" height="1" depth="1" rapier-body rapier-shape></a-box>
```

## System

### `rapier-physics`

This system must be set on the `a-scene` to enable physics in your
scene. A couple of options can be set to customize your simulation:

| Property    | Default     | Description                                                  |
| ----------- | ----------- | ------------------------------------------------------------ |
| gravity     | `0 -9.81 0` | Gravity in your simulation. By default this is similar to that on Earth in m/s^2.  |
| debug       | `false`     | When set, a wireframe will be drawn around collision shapes to make them visible. |
| stats       | `[]`        | Whether to output stats, and how to output them.  One or more of `console`, `events`, `panel`. See [Statistics](#statistics) |
| maxTimeStep | `100`       | Physics engines may produce strange behaviors when computing simulation steps too far in the future and Rapier seems to make no exception. This option caps the number of milliseconds of the physics step, so that if the simulation is stopped (e.g. when the browser canvas becomes hidden), resuming will not try to simulate too big a timeframe. Set to positive infinity to disable. |

## Components

### `rapier-body`

A `rapier-body` component may be added to any entity in a
scene. While having only a `rapier-body` will technically give you a
valid physics body in the scene, only after adding a `rapier-shape`
will your entity begin to collide with other objects.

| Property   | Default   | Description                                                  |
| -----------| ----------| ------------------------------------------------------------ |
| type       | `Dynamic` | Options: `Dynamic`, `Fixed`, `KinematicPositionBased` and `KinematicVelocityBased`. See [body types](https://rapier.rs/docs/user_guides/bevy_plugin/rigid_bodies/#rigid-body-type). |
| canSleep   | `true`    | Allow bodies to sleep when inactive. See [sleeping](https://rapier.rs/docs/user_guides/bevy_plugin/rigid_bodies/#sleeping) |
| ccdEnabled | `false`   | Enable [Continuous Collision Detection](https://rapier.rs/docs/user_guides/javascript/advanced_collision_detection/#continuous-collision-detection). |
| linearDamping  | `0`   | linear damping coefficient, affecting body's linear velocity. See [rigid_body_damping](https://rapier.rs/docs/user_guides/javascript/rigid_body_damping/) |
| angularDamping | `0`   | angular damping coefficient, affecting body's angular velocity. See [rigid_body_damping](https://rapier.rs/docs/user_guides/javascript/rigid_body_damping/) |

### `rapier-shape`

Any entity with a `rapier-body` component can also have 1 or more
`rapier-shape` components. The `rapier-shape` component is what
defines the collision shape of the entity. `rapier-shape` components
can be added and removed at any time. By default an appropriate
collision shape will be guessed automatically, but one could also
specify one manually.

| Property            | Dependencies                                              | Default                       | Description                                                                                                                               |
| ------------------- | --------------------------------------------------------- | ----------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------- |
| shape               | —                                                         | `ConvexHull`                        | Options: `Cuboid`, `Cylinder`, `Capsule`, `Cone`, `Ball`, `ConvexHull`, `TriMesh` and `Heigtfield`. see [Shapes](https://rapier.rs/docs/user_guides/javascript/colliders/#shapes). |
| fit                 | —                                                         | `true`                         | Flag deciding if we should compute the collision shape automatically |
| halfExtents         | `fit: false` and any shape other than `ConvexHull`, `TriMesh` or `Heightfield` | `0.5 0.5 0.5` | Set the halfExtents to use.                        |
| offset              | —                                                               | `0 0 0`                     | Offset of this shape with respect to the entity    |
| minHalfExtent       | `fit: true` and any shape other than `ConvexHull` or `TriMesh`  | `Number.NEGATIVE_INFINITY`  | The minimum value for any axis of the halfExtents. |
| maxHalfExtent       | `fit: true` and any shape other than `ConvexHull` or `TriMesh`  | `Number.NEGATIVE_INFINITY`  | The maximum value for any axis of the halfExtents. |
| radius              | `fit: false` and shape `Ball` or `Cylinder`                     | `1`                         | Set the radius for spheres.                        |
| axis                | —                                                               | `y`                         | Options: `x`, `y`, `z`. Override default axis for `Cylinder` shape|
| includeInvisible    | `fit: true`                                                     | `false`                     | Include also vertices from invisible meshes when computing the collision shape |
| sensor              | —                                                               | `false`                     | Should this shape behave as a sensor? See [collider types](https://rapier.rs/docs/user_guides/javascript/colliders/#collider-type) |
| contactSkin         | —                                                               | `0`                           | Additional thickness of the collision shape which can be useful to make the simulation more stable. Should be set small enough to not be noticeable. |
| density         | —                                                               | `0`                           | Material density, which can be used to compite the mass when not specified |
| friction         | —                                                               | `0.5`                           | Collider friction coefficient. Typically <= 1 |
| mass         | —                                                               | `1`                           | Mass of the collider. Mandatory > 0 for dynamic objects to work |
| restitution         | —                                                               | `0`                           | Collider bouncyness |
| scaleAutoUpdate     | —                                                               | `false`                           | Update the collision shape when the object's scale changes. Rapier does not support scaling the collision shape, so the entire collider must be recomputed. Use with caution. |
| emitCollisionEvents | —                                                               | `false`                           | Emit `collidestart` and `collideend` events when a collider starts/stops colliding with another one. See [events](#events). |
| enabled | —                                                               | `true`                           | When set to `false` disables all physics on this entity. |
| collisionGroups | —                                                       | `NaN`                           | When set to a number, controls the [collision groups](https://rapier.rs/docs/user_guides/javascript/colliders/#collider-type) for this entity |
| solverGroups | —                                                       | `NaN`                           | When set to a number, controls the [solver groups](https://rapier.rs/docs/user_guides/javascript/colliders/#collider-type) for this entity |
| heightSegments | `Heightfield` shape and `fit` set to `false` | `2` | The number of vertical segments for the Heightfield. `2` is the minimum required by Rapier and corresponds to `1` in three.js logics. |
| widthSegments | `Heightfield` shape and `fit` set to `false` | `2` | The number of horizontal segments for the Heightfield. `2` is the minimum required by Rapier and corresponds to `1` in three.js logics. |
| heightFieldScale | `Heightfield` shape and `fit: false` | `1 1 1` | The scale applied to the heightfield shape. This is not the same as the displacementScale, as it is used to control the size of the shape for all three dimensions. |
| heights | `Heightfield` shape and `fit` set to `false`| `[]` | The heights of the heightfield along its local y axis, provided as a matrix stored in column-major order. |


### `rapier-constraint`

The `rapier-constraint` component is used to bind `rapier-bodies`
together using hinges, fixed distances, or fixed attachment
points.

Example:

```html
<a-box id="other-box" rapier-body rapier-shape />
<a-box rapier-constraint="target: #other-box;" rapier-body rapier-shape />
```

| Property    | Dependencies                           | Default | Description                                                                               |
| ----------- | -------------------------------------- | ------- | ----------------------------------------------------------------------------------------- |
| type        | —                                      | `fixed`  | Options: `fixed`, `generic`, `prismatic`, `revolute`, `rope`, `spring`. See [Joints](https://rapier.rs/docs/user_guides/javascript/joints) |
| target      | —                                      | —       | Selector for a single entity to which current entity should be bound.                     |
| anchor1     | — | `null` | Point where the joint is attached on the first rigid-body affected by this joint. Expressed in the local-space of the rigid-body. |
| anchor2     | — | `null` | Point where the joint is attached on the second rigid-body affected by this joint. Expressed in the local-space of the rigid-body. |
| frame1      | `type: fixed` (optional) | `null` | The reference orientation of the joint wrt. the first rigid-body. |
| frame2      | `type: fixed` (optional) | `null` | The reference orientation of the joint wrt. the first rigid-body. |
| axis | `type: generic, prismatic, revolute` | `0 0 1` | Axis used to constrain the movement. |
| axesMask | `type: generic` | `null` | Mask representing the locked axes of the joint. See [JointAxesMask](https://rapier.rs/javascript3d/enums/JointAxesMask.html) |

## Using the Rapier API

Rapier is pretty well documented. One good starting point is the [JavaScript bindings documentation](https://rapier.rs/docs/user_guides/javascript/getting_started_js).

The full API documentation for JavaScript 3D is available [here](https://rapier.rs/javascript3d/index.html)

## Events

| event         | description                                                                                         |
| ------------- | --------------------------------------------------------------------------------------------------- |
| `body-loaded` | Fired by the entity when its physics body has been created.                                         |
| `collidestart`     | Fired when two shapes collide. `emitCollisionEvents: true` must be set on the `rapier-shape`.          |
| `collideend` | Fired when two shapes stop colliding. `emitCollisionEvents: true` must be set on the `rapier-shape`.   |

The collision event detail will contain the following information:

| Property  | Description  |
| --------- | ------------ |
| targetEl  | The entity we have touched. |
| contactPoint | The vec3 contact point on our entity |
| targetContactPoint | The vec3 contact point on the touched entity |
| impulse | Impulse at contact |

## Statistics

When enabled, the system will collect a few statistics about its
internals, which may be helpful in providing an estimate of the
complexity and performance of your physics scene.

The following statistics are available.  Each of these is refreshed
every 100 ticks (i.e. every 100 frames).

| Statistic  | Meaning                                                      |
| ---------- | ------------------------------------------------------------ |
| Static     | The number of static bodies being handled by the physics engine. |
| Dynamic    | The number of dynamic bodies being handled by the physics engine. |
| Kinematic(v) | The number of velocity-based kinematic bodies being handled by the physics engine. |
| Kinematic(p) | The number of position-based kinematic bodies being handled by the physics engine. |
| Colliders  | The total number of colliders in the simulation, which may be greater than the number of total bodies. |
| Manifolds  | A manifold represents a pair of bodies that are close to each other, but might have zero one or more actual contacts. We count all such potential contact pairs for every collider in the simulation. |
| Collisions | The number of current collisions between pairs of colliders. This means that the two colliders are in contact with each other. This is increased whenever a contact event is starting and decreased when this stops according to the physics engine. |
| Before     | The number of milliseconds per tick before invoking the physics engine.  Typically this is the time taken to synchronize the scene state into the physics engine, e.g. movements of kinematic bodies, or changes to physics shapes.<br />Median = median value in the last 100 ticks<br />90th % = 90th percentile value in the last 100 ticks<br />99th % = maximum recorded value over the last 100 ticks. |
| After      | The number of milliseconds per tick after invoking the physics engine.  Typically this is the time taken to synchronize the physics engine state into the scene, e.g. movements of dynamic bodies. This includes emitting and handling of collision events. <br />Reported as Median / 90th / 99th percentiles, as above. |
| Engine     | The number of milliseconds per tick actually running the physics engine.<br />Reported as Median / 90th / 99th percentiles, as above. |
| Total      | The total number of milliseconds of physics processing per tick: Before + Engine + After.  Reported as Median / 90th / 99th percentiles, as above. |

