# Rapier JavaScript binding

[rapier.js](https://github.com/dimforge/rapier.js/) is the official JavaScript distribution of the Rapier engine.

## Contents

- [Considerations](#considerations-before-use)
- [Installation](#installation)
- [Basics](#basics)
- [Components](#components)
  - [`rapier-body`](#rapier-body)
  - [`rapier-shape`](#rapier-shape)
  - [`rapier-constraint`](#rapier-constraint)
- [Using the Rapier API](#using-the-rapier-api)
- [Events](#events)
- [System Configuration](#system-configuration)

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

To create a physics body, both an `rapier-body` and at least one
`rapier-shape` component should be added to an entity.

```html
<!-- Fixed box -->
<a-box position="0 0.5 -5" width="3" height="1" depth="1" rapier-body="type: Fixed" rapier-shape></a-box>

<!-- Dynamic box -->
<a-box position="5 0.5 0" width="1" height="1" depth="1" rapier-body rapier-shape></a-box>
```

## Components

### `rapier-body`

An `rapier-body` component may be added to any entity in a
scene. While having only an `rapier-body` will technically give you a
valid physics body in the scene, only after adding an `rapier-shape`
will your entity begin to collide with other objects.

| Property   | Default   | Description                                                  |
| -----------| ----------| ------------------------------------------------------------ |
| type       | `Dynamic` | Options: `Dynamic`, `Fixed`, `KinematicPositionBased` and `KinematicVelocityBased`. See [body types](https://rapier.rs/docs/user_guides/bevy_plugin/rigid_bodies/#rigid-body-type). |
| canSleep   | `true`    | Allow bodies to sleep when inactive. See [sleeping](https://rapier.rs/docs/user_guides/bevy_plugin/rigid_bodies/#sleeping) |
| ccdEnabled | `false`   | Enable [Continuous Collision Detection](https://rapier.rs/docs/user_guides/javascript/advanced_collision_detection/#continuous-collision-detection). |

### `rapier-shape`

Any entity with an `rapier-body` component can also have 1 or more
`rapier-shape` components. The `rapier-shape` component is what
defines the collision shape of the entity. `rapier-shape` components
can be added and removed at any time. By default an appropriate
collision shape will be guessed automatically, but one could also
specify one manually.

| Property            | Dependencies                                              | Default                       | Description                                                                                                                               |
| ------------------- | --------------------------------------------------------- | ----------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------- |
| shape               | —                                                         | `ConvexHull`                        | Options: `Cuboid`, `Cylinder`, `Capsule`, `Cone`, `Ball`, `ConvexHull`, `TriMesh`. see [Shapes](https://rapier.rs/docs/user_guides/javascript/colliders/#shapes). |
| fit                 | —                                                         | `true`                         | Flag deciding if we should compute the collision shape automatically |
| halfExtents         | `fit: false` and any shape other than `ConvexHull` or `TriMesh` | `1 1 1`                     | Set the halfExtents to use.                        |
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
| emitCollisionEvents | —                                                               | `false`                           | Emit `collisionstart` and `collisionend` events when a collider starts/stops colliding with another one. See [events](#events). |
| enabled | —                                                               | `true`                           | When set to `false` disables all physics on this entity. |
| collisionGroups | —                                                       | `NaN`                           | When set to a number, controls the [collision groups](https://rapier.rs/docs/user_guides/javascript/colliders/#collider-type) for this entity |
| solverGroups | —                                                       | `NaN`                           | When set to a number, controls the [solver groups](https://rapier.rs/docs/user_guides/javascript/colliders/#collider-type) for this entity |

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
| type        | —                                      | `fixed`  | Options: `fixed`, `generic`, `prismatic`, `revolute`, `rope`, `spring` |
| target      | —                                      | —       | Selector for a single entity to which current entity should be bound.                     |
| anchor1     | — | `null` | Point where the joint is attached on the first rigid-body affected by this joint. Expressed in the local-space of the rigid-body. |
| anchor2     | — | `null` | Point where the joint is attached on the second rigid-body affected by this joint. Expressed in the local-space of the rigid-body. |
| frame1      | `type: fixed` (optional) | `0 0 0 1` | The reference orientation of the joint wrt. the first rigid-body. |
| frame2      | `type: fixed` (optional) | `0 0 0 1` | The reference orientation of the joint wrt. the first rigid-body. |
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

## System Configuration

| Property      | Default   | Description                                                  |
| ------------- | --------- | ------------------------------------------------------------ |
| debug         | `true`    | Whether to show wireframes for debugging.                    |
| gravity       | `0 -9.81 0`    | Force of gravity (in m/s^2).                                 |
| stats         |           | Where to output performance stats (if any), `panel`, `console`, `events` (or some combination). <br />- `panel` output stats to a panel similar to the A-Frame stats panel.<br />-`events` generates `physics-tick-timer` events, which can be processed externally.<br/> -`console`outputs stats to the console. |

## Statistics

The following statistics are available.  Each of these is refreshed
every 100 ticks (i.e. every 100 frames).

Some statistics follow the nomenclature from aframe-physics-system
they were lifted from and do not map 100% to Rapier concepts - but
they may nevertheless be helpful in providing an approximate estimate
of the complexity involved in a given physics scene.

| Statistic  | Meaning                                                      |
| ---------- | ------------------------------------------------------------ |
| Static     | The number of static bodies being handled by the physics engine. |
| Dynamic    | The number of dynamic bodies being handled by the physics engine. |
| Kinematic  | The number of kinematic bodies being handled by the physics engine. |
| Manifolds  | A manifold represents a pair of bodies that are close to each other, but might have zero one or more actual contacts. |
| Contacts   | The number of actual contacts between pairs of bodies.  There may be zero, one or multiple contacts per manifold (up to four - the physics engine discards any more than this, while always preserving the deepest contact point). |
| Collisions | The number of current collisions between pairs of bodies.  This means that the two bodies are in contact with each other (one or more contacts).<br />One would expect this number too be lower than the number of Manifolds, but that doesn't seem to consistently be the case.  This may indicate a bug, or may just indicate that we need to better understand & explain the exact meanings of these statistics. |
| Coll Keys  | An alternative measure of the number of current collisions between pairs of bodies, based on a distinct internal storage mechanism.<br />This seems to be consistently lower than Collisions, which may indicate a bug, or may just indicate that we need to better understand & explain the exact meanings of these statistics. |
| Before     | The number of milliseconds per tick before invoking the physics engine.  Typically this is the time taken to synchronize the scene state into the physics engine, e.g. movements of kinematic bodies, or changes to physics shapes.<br />Median = median value in the last 100 ticks<br />90th % = 90th percentile value in the last 100 ticks<br />99th % = maximum recorded value over the last 100 ticks. |
| After      | The number of milliseconds per tick after invoking the physics engine.  Typically this is the time taken to synchronize the physics engine state into the scene, e.g. movements of dynamic bodies.<br />Reported as Median / 90th / 99th percentiles, as above. |
| Engine     | The number of milliseconds per tick actually running the physics engine.<br />Reported as Median / 90th / 99th percentiles, as above. |
| Total      | The total number of milliseconds of physics processing per tick: Before + Engine + After.  Reported as Median / 90th / 99th percentiles, as above. |

