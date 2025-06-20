/**
 * Force Pushable component.
 *
 * Applies behavior to the current entity such that cursor clicks will apply a
 * strong impulse, pushing the entity away from the viewer.
 *
 * Requires: physics
 */
(() => {
  const origin = new THREE.Vector3();
  const force = new THREE.Vector3();

  AFRAME.registerComponent('force-pushable', {
    dependencies: ['rapier-body'],
    schema: {
      force: { default: 10 }
    },
    init: function () {
      this.forcePush = this.forcePush.bind(this);
      this.bodyComponent = this.el.components['rapier-body'];
      this.children = [];
    },
    forcePush: function (e) {
      const body = this.bodyComponent.body;
      if (!body) return;

      const pusher = e.detail.cursorEl.object3D
      pusher.getWorldPosition(origin);

      const target = e.detail.intersection.point;

      force.
        copy(target).
        sub(origin).
        normalize().
        multiplyScalar(this.data.force);

      console.debug('force-pushable:', force);

      body.applyImpulseAtPoint(force, target, true);
    },
    play: function() {
      //
      // Click event from the cursor component will fire only on items
      // added by setObject3D. If this is an entity with children
      // entities, ensure these are also been set.
      //
      this.children.length = 0;
      this.children.push(...this.el.object3D.children);
      for (const child of this.children) {
        this.el.setObject3D(child.id, child);
      }
      this.el.addEventListener('click', this.forcePush);
    },
    pause: function() {
      for (const child of this.children) {
        this.el.removeObject3D(child.id);
      }
      this.children.length = 0;
      this.el.removeEventListener('click', this.forcePush);
    }
  });
})();

//
// Local variables:
//    mode: javascript
//    js-indent-level: 2
//    indent-tabs-mode: nil
// End:
//
