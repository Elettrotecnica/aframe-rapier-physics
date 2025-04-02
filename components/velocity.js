/**
 * Velocity, in m/s.
 */
AFRAME.registerComponent('velocity', {
  schema: {type: 'vec3', default: { x: 0, y: 0, z: 0 }},

  tick: function (t, dt) {
    const velocity = this.data;
    const position = this.el.object3D.position;

    this.el.object3D.position.set(
      position.x + velocity.x * dt / 1000,
      position.y + velocity.y * dt / 1000,
      position.z + velocity.z * dt / 1000
    );
  }
});

//
// Local variables:
//    mode: javascript
//    js-indent-level: 2
//    indent-tabs-mode: nil
// End:
//
