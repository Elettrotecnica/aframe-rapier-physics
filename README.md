# A-Frame Rapier Phsyics

This is a [Rapier](https://rapier.rs/) [A-Frame](https://aframe.io/)
integration implementing a physics system similar to the existing
[aframe-physics-system](https://github.com/c-frame/aframe-physics-system)
for the CANNON and Ammo drivers.

## Main Features

* Fixed, dynamic and kinematic bodies
* (Semi) automated collision shapes and compound shapes
* Physical joints
* Collision events

## Usage

To include the library in your project, place it after the A-Frame one, for instance:
```html
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

See the [docs](docs/) for a description of the system and components.

See the [examples](examples/) for various usages of the system.

## Motivation

Compared to existing tools for physics with A-Frame, this one has the
following advantages:

* based on a modern, actively-maintained library 
* official Javascript bindings by the library authors
* three.js-friendly. Constructs such as vectors and quaternions are intercompatible.

## Acknowledgments

As a starting point for my implementation, I have used the [Rapier
example from the three.js
repository](https://github.com/mrdoob/three.js/blob/master/examples/jsm/physics/RapierPhysics.js)

For the automated computing of collision shapes from THREE objects, a
lot of inspiration came from
[three-to-ammo](https://github.com/c-frame/three-to-ammo).

The examples and some related components have been adapted from those
in
[aframe-physics-system](https://github.com/c-frame/aframe-physics-system).

