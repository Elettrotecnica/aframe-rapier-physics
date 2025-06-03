/**
 * @classdesc Can be used to include Rapier as a Physics engine into
 * `three.js` apps. The API can be initialized via:
 * ```js
 * const physics = await RapierPhysics();
 * ```
 * The component automatically imports Rapier from a CDN so make sure
 * to use the component with an active Internet connection.
 *
 * @name RapierPhysics
 * @class
 * @hideconstructor
 */
const RAPIER_PATH = 'https://cdn.skypack.dev/@dimforge/rapier3d-compat@0.17.3';

const getWorldPosition = (function () {
  const position = new THREE.Vector3();
  return function (object) {
    return object.getWorldPosition(position);
  };
})();

const getWorldScale = (function () {
  const scale = new THREE.Vector3();
  return function (object) {
    return object.getWorldScale(scale);
  };
})();

const getWorldQuaternion = (function () {
  const quaternion = new THREE.Quaternion();
  return function (object) {
    return object.getWorldQuaternion(quaternion);
  };
})();

/*
  This is useful when computing primitive collision shapes for models
  that do not have the origin at the center.
 */
const computeOffset = (function () {
  const b = new THREE.Box3();
  const v = new THREE.Vector3();
  return function (object) {
    object.updateMatrixWorld();
    b.setFromObject(object);
    b.getCenter(v);
    object.worldToLocal(v);
    v.divide(object.scale);
    return v;
  };
})();

class RapierDebugRenderer {
  mesh
  world

  constructor(scene, world) {
    this.world = world;
    this.mesh = new THREE.LineSegments(
      new THREE.BufferGeometry(),
      new THREE.LineBasicMaterial({ color: 0xffffff, vertexColors: true })
    );
    this.mesh.frustumCulled = false;
    scene.add(this.mesh);
  }

  update() {
    const { vertices, colors } = this.world.debugRender()
    this.mesh.geometry.setAttribute(
      'position',
      new THREE.BufferAttribute(vertices, 3)
    );
    this.mesh.geometry.setAttribute(
      'color',
      new THREE.BufferAttribute(colors, 4)
    );
    this.mesh.visible = true;
  }
}

let RAPIER = null;

async function RapierPhysics(options) {

  const scene = options.scene;
  const gravity = options.gravity;

  if ( RAPIER === null ) {

    RAPIER = await import( `${RAPIER_PATH}` );
    await RAPIER.init();

  }

  // Docs: https://rapier.rs/docs/api/javascript/JavaScript3D/

  const world = new RAPIER.World( gravity );

  const events = new RAPIER.EventQueue(true);

  const debugRenderer = options.debug ?
    new RapierDebugRenderer(scene, world) : null;

  const ZERO = new THREE.Vector3();

  const objects = [];
  const objectToBodyMap = new WeakMap();
  const bodyToObjectMap = new WeakMap();

  let numCollisions = 0;
  let numManifolds = 0;

  const _setOptions = (function() {
    const meshes = [];
    return function (object, options) {
      // fit = true: a single shape is automatically sized to bound all meshes within the entity.
      // fit = false: a single shape is sized manually. Requires halfExtents or radius.
      options.fit = options.hasOwnProperty('fit') ?
        options.fit : true;

      options.shape = options.shape || 'convexHull';

      options.minHalfExtent = options.hasOwnProperty('minHalfExtent') ?
        options.minHalfExtent : 0;

      options.maxHalfExtent = options.hasOwnProperty('maxHalfExtent') ?
        options.maxHalfExtent : Number.POSITIVE_INFINITY;

      options.axis = options.axis || 'y';

      options.includeInvisible = options.hasOwnProperty('includeInvisible') ?
        options.includeInvisible : false;

      if (!options.hasOwnProperty('halfExtents')) {
        options.halfExtents = { x: 1, y: 1, z: 1 };
      }

      if (isNaN(options.radius)) {
        options.radius = 0
      }

      const scale = getWorldScale(object);

      if (options.fit && scale.x === scale.y && scale.y === scale.z) {
        //
        // Here we try to guess the shape and shape parameters
        // directly from the geometries. We do this only for trivial
        // cases where the mesh is a single primitive. We can only
        // support non-deforming scale with this method.
        //
        meshes.length = 0;
        object.getObjectsByProperty('isMesh', true, meshes);
        if (meshes.length === 1) {
          const geometry = meshes[0].geometry;
          const parameters = geometry.parameters;

          options.halfExtents.x = parameters?.width !== undefined ?
            parameters.width / 2 : 0.5;
          options.halfExtents.y = parameters?.height !== undefined ?
            parameters.height / 2 : 0.5;
          options.halfExtents.z = parameters?.depth !== undefined ?
            parameters.depth / 2 : 0.5;

          options.fit = false;

          switch (geometry.type) {
          case 'PlaneGeometry':
            options.halfExtents.z = 0;
            options.shape = 'Cuboid';
            break;
          case 'BoxGeometry':
            options.shape = 'Cuboid';
            break;
          case 'SphereGeometry':
            options.radius = parameters.radius;
            options.shape = 'Ball';
            break;
          case 'CylinderGeometry':
            if (parameters.radiusTop ===
                parameters.radiusBottom) {
              options.radius = parameters.radiusTop;
              options.shape = 'Cylinder';
            }
            break;
          default:
            options.fit = true;
          }

        }
      }

      options.halfExtents.x *= scale.x;
      options.halfExtents.y *= scale.y;
      options.halfExtents.z *= scale.z;

      options.radius *= scale[options.axis];
    };
  })();

  // returns the bounding box for the geometries underneath `root`.
  const _computeBounds = (function() {
    const bounds = new THREE.Box3();
    return function (vertices) {
      let minX = +Infinity;
      let minY = +Infinity;
      let minZ = +Infinity;
      let maxX = -Infinity;
      let maxY = -Infinity;
      let maxZ = -Infinity;
      bounds.min.set(0, 0, 0);
      bounds.max.set(0, 0, 0);

      for (let i = 0; i < vertices.length; i+=3) {
        const x = vertices[i];
        const y = vertices[i + 1];
        const z = vertices[i + 2];
        if (x < minX) minX = x;
        if (y < minY) minY = y;
        if (z < minZ) minZ = z;
        if (x > maxX) maxX = x;
        if (y > maxY) maxY = y;
        if (z > maxZ) maxZ = z;
      }

      bounds.min.set(minX, minY, minZ);
      bounds.max.set(maxX, maxY, maxZ);
      return bounds;
    };
  })();

  const _computeHalfExtents = (function() {
    const halfExtents = new THREE.Vector3();
    return function(bounds, minHalfExtent, maxHalfExtent) {
      return halfExtents
        .subVectors(bounds.max, bounds.min)
        .multiplyScalar(0.5)
        .clampScalar(minHalfExtent, maxHalfExtent);
    };
  })();

  const _computeRadius = (function() {
    const center = new THREE.Vector3();
    return function(vertices, bounds) {
      let maxRadiusSq = 0;
      let { x: cx, y: cy, z: cz } = bounds.getCenter(center);

      for (let i = 0; i < vertices.length; i+=3) {
        const x = vertices[i];
        const y = vertices[i + 1];
        const z = vertices[i + 2];
        const dx = cx - x;
        const dy = cy - y;
        const dz = cz - z;
        maxRadiusSq = Math.max(maxRadiusSq, dx * dx + dy * dy + dz * dz);
      }

      return Math.sqrt(maxRadiusSq);
    };
  })();

  function createCuboidShape(object, options) {
    if (options.fit) {
      options.offset = computeOffset(object);

      const verticesAndIndexes = _getVerticesAndIndexes(object, options);
      const vertices = verticesAndIndexes.vertices;

      const halfExtents = _computeHalfExtents(
        _computeBounds(vertices),
        options.minHalfExtent,
        options.maxHalfExtent
      );
      options.halfExtents.x = halfExtents.x;
      options.halfExtents.y = halfExtents.y;
      options.halfExtents.z = halfExtents.z;
    }

    const sx = options.halfExtents.x;
    const sy = options.halfExtents.y;
    const sz = options.halfExtents.z;

    return RAPIER.ColliderDesc.cuboid( sx, sy, sz );
  }

  function createCylinderShape(object, options) {
    if (options.fit) {
      options.offset = computeOffset(object);

      const verticesAndIndexes = _getVerticesAndIndexes(object, options);
      const vertices = verticesAndIndexes.vertices;

      const bounds = _computeBounds(vertices);
      const halfExtents = _computeHalfExtents(
        bounds,
        options.minHalfExtent,
        options.maxHalfExtent
      );
      options.halfExtents.x = halfExtents.x;
      options.halfExtents.y = halfExtents.y;
      options.halfExtents.z = halfExtents.z;

      options.radius = _computeRadius(vertices, bounds);
    }

    switch (options.axis) {
    case 'y':
      return RAPIER.ColliderDesc.cylinder( options.halfExtents.y, options.radius );
    case 'x':
      return RAPIER.ColliderDesc.cylinder( options.halfExtents.x, options.radius );
    case 'z':
      return RAPIER.ColliderDesc.cylinder( options.halfExtents.z, options.radius );
    }

    return null;
  }

  function createCapsuleShape(object, options) {
    if (options.fit) {
      options.offset = computeOffset(object);

      const verticesAndIndexes = _getVerticesAndIndexes(object, options);
      const vertices = verticesAndIndexes.vertices;

      const bounds = _computeBounds(vertices);
      const halfExtents = _computeHalfExtents(
        bounds,
        options.minHalfExtent,
        options.maxHalfExtent
      );
      options.halfExtents.x = halfExtents.x;
      options.halfExtents.y = halfExtents.y;
      options.halfExtents.z = halfExtents.z;

      options.radius = _computeRadius(vertices, bounds);
    }

    switch (options.axis) {
    case 'y':
      return RAPIER.ColliderDesc.capsule( options.halfExtents.y, options.radius );
    case 'x':
      return RAPIER.ColliderDesc.capsule( options.halfExtents.x, options.radius );
    case 'z':
      return RAPIER.ColliderDesc.capsule( options.halfExtents.z, options.radius );
    }

    return null;
  }

  function createConeShape(object, options) {
    if (options.fit) {
      options.offset = computeOffset(object);

      const verticesAndIndexes = _getVerticesAndIndexes(object, options);
      const vertices = verticesAndIndexes.vertices;

      const bounds = _computeBounds(vertices);
      const halfExtents = _computeHalfExtents(
        bounds,
        options.minHalfExtent,
        options.maxHalfExtent
      );
      options.halfExtents.x = halfExtents.x;
      options.halfExtents.y = halfExtents.y;
      options.halfExtents.z = halfExtents.z;

      options.radius = _computeRadius(vertices, bounds);
    }

    switch (options.axis) {
    case 'y':
      return RAPIER.ColliderDesc.cone( options.halfExtents.y, options.radius );
    case 'x':
      return RAPIER.ColliderDesc.cone( options.halfExtents.x, options.radius );
    case 'z':
      return RAPIER.ColliderDesc.cone( options.halfExtents.z, options.radius );
    }

    return null;
  }

  function createBallShape(object, options) {
    if (options.fit) {
      options.offset = computeOffset(object);

      const verticesAndIndexes = _getVerticesAndIndexes(object, options);
      const vertices = verticesAndIndexes.vertices;

      const bounds = _computeBounds(vertices);
      options.radius = _computeRadius(vertices, bounds);
    }

    return RAPIER.ColliderDesc.ball( options.radius );
  }

  function createConvexHullShape(object, options) {
    const verticesAndIndexes = _getVerticesAndIndexes(object, options);
    const vertices = verticesAndIndexes.vertices;

    return RAPIER.ColliderDesc.convexHull( vertices );
  }

  function createTriMeshShape(object, options) {
    const verticesAndIndexes = _getVerticesAndIndexes(object, options);
    const vertices = verticesAndIndexes.vertices;
    // if the buffer is non-indexed, generate an index buffer
    const indexes = verticesAndIndexes.indexes.length === 0 ?
          Uint32Array.from( Array( parseInt( vertices.length / 3 ) ).keys() ) :
          verticesAndIndexes.indexes;

    return RAPIER.ColliderDesc.trimesh( vertices, indexes );
  }

  const _getVerticesAndIndexes = (function() {
    const vertex = new THREE.Vector3();
    const quaternion = new THREE.Quaternion();
    const verticesAndIndexes = {
      vertices: [],
      indexes: []
    };
    return function (object, options) {
      const vertices = verticesAndIndexes.vertices;
      const indexes = verticesAndIndexes.indexes;

      vertices.length = 0;
      indexes.length = 0;

      //
      // A Rapier shape is assumed to be at rotation 0, so before we
      // extract the vertices we undo any rotation on the object and
      // set it back at the end.
      //
      quaternion.copy(object.quaternion);
      object.quaternion.identity();

      let indexOffset = 0;

      object.traverse(mesh => {
        if (
          mesh.isMesh &&
            (options.includeInvisible || (mesh.el && mesh.el.object3D.visible) || mesh.visible)
        ) {
          const scale = getWorldScale(mesh);

          const geometry = mesh.geometry;
          const positionAttribute = geometry.getAttribute( 'position' );

          for (let i = 0; i < positionAttribute.count; i++) {
            vertex.fromBufferAttribute(positionAttribute, i);
            vertex.multiply(scale);
            vertices.push( vertex.x, vertex.y, vertex.z );
          }

          if (geometry.index) {
            const index = geometry.index;
            for ( let j = 0; j < index.count; ++ j ) {
              indexes.push( index.getX( j ) + indexOffset );
            }
            indexOffset += geometry.attributes.position.count;
          }
        }
      });

      object.quaternion.copy(quaternion);

      return verticesAndIndexes;
    };
  })();

  function getShape( object, options ) {
    _setOptions(object, options);

    switch (options.shape) {
    case 'Cuboid':
      return createCuboidShape(object, options);

    case 'Cylinder':
      return createCylinderShape(object, options);

    case 'Capsule':
      return createCapsuleShape(object, options);

    case 'Cone':
      return createConeShape(object, options);

    case 'Ball':
      return createBallShape(object, options);

    case 'ConvexHull':
      return createConvexHullShape(object, options);

    case 'TriMesh':
      return createTriMeshShape(object, options);

    }

    return null;
  }

  function createBody( object, options ) {
    let body = objectToBodyMap.get(object);
    if ( body ) {
      throw 'Cannot specify multiple bodies on one object.'
    }

    const desc = (() => {
      switch (options.type) {
      case 'Fixed':
        return RAPIER.RigidBodyDesc.fixed();
      case 'Dynamic':
        return RAPIER.RigidBodyDesc.dynamic();
      case 'KinematicVelocityBased':
        return RAPIER.RigidBodyDesc.kinematicVelocityBased();
      case 'KinematicPositionBased':
        return RAPIER.RigidBodyDesc.kinematicPositionBased();
      default:
        return RAPIER.RigidBodyDesc.dynamic();
      }
    })();

    desc.setCanSleep( options.canSleep );
    desc.setTranslation( ...getWorldPosition(object) );
    desc.setRotation( getWorldQuaternion(object) );
    desc.setCcdEnabled( options.ccdEnabled );

    body = world.createRigidBody( desc );

    objects.push(object);
    objectToBodyMap.set(object, body);
    bodyToObjectMap.set(body, object);

    return body;
  }

  function createCollider( object, options ) {
    const body = objectToBodyMap.get(object);
    if ( !body ) {
      throw 'No body defined on this object.'
    }

    const shape = getShape( object, options );

    if ( shape === null ) {
      throw 'Cannot compute shape.'
    }

    if (options.emitCollisionEvents) {
      shape.setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS);
    }

    shape.setSensor( options.sensor );
    shape.setContactSkin( options.contactSkin );
    shape.setDensity( options.density );
    shape.setFriction( options.friction );
    shape.setMass( options.mass );
    shape.setRestitution( options.restitution );
    if (!isNaN(options.collisionGroups)) {
      shape.setCollisionGroups(options.collisionGroups);
    }
    if (!isNaN(options.solverGroups)) {
      shape.setSolverGroups(options.solverGroups);
    }

    const collider = world.createCollider( shape, body );
    collider.setTranslationWrtParent( options.offset );
    collider.setEnabled( options.enabled );

    return collider;
  }

  function updateBody( object, options ) {
    const body = objectToBodyMap.get(object);
    if ( !body ) return;

    const bodyType = (() => {
      switch (options.type) {
      case 'Fixed':
        return RAPIER.RigidBodyType.Fixed;
      case 'Dynamic':
        return RAPIER.RigidBodyType.Dynamic;
      case 'KinematicVelocityBased':
        return RAPIER.RigidBodyType.KinematicVelocityBased;
      case 'KinematicPositionBased':
        return RAPIER.RigidBodyType.KinematicPositionBased;
      default:
        return options.mass > 0 ?
          RAPIER.RigidBodyType.Dynamic :
          RAPIER.RigidBodyType.Fixed;
      }
    })();
    body.setBodyType(bodyType, false);
    body.enableCcd( options.ccdEnabled );
  }

  function updateCollider(object, collider, options, rebuild = false) {
    let shape = collider.shape;

    if (rebuild || shape.type !== RAPIER.ShapeType[options.shape]) {
      shape = getShape(object, options).shape;
      collider.setShape(shape);
    }

    collider.setActiveEvents(options.emitCollisionEvents ?
                             RAPIER.ActiveEvents.COLLISION_EVENTS :
                             RAPIER.ActiveEvents.NONE);

    collider.setSensor( options.sensor );
    collider.setContactSkin( options.contactSkin );
    collider.setDensity( options.density );
    collider.setFriction( options.friction );
    collider.setMass( options.mass );
    collider.setRestitution( options.restitution );
    collider.setTranslationWrtParent( options.offset );
    collider.setEnabled( options.enabled );
    if (!isNaN(options.collisionGroups)) {
      collider.setCollisionGroups(options.collisionGroups);
    }
    if (!isNaN(options.solverGroups)) {
      collider.setSolverGroups(options.solverGroups);
    }
  }

  function removeBody( object ) {
    const body = objectToBodyMap.get(object);
    if ( !body ) return;

    bodyToObjectMap.delete(body);
    objectToBodyMap.delete(object);
    world.removeRigidBody(body);

    const index = objects.indexOf(object);
    objects.splice(index, 1);
  }

  function removeCollider( collider ) {
    world.removeCollider(collider);
  }

  const distanceVector = (function () {
    const v1 = new THREE.Vector3();
    const v2 = new THREE.Vector3();
    return function (object1, object2) {
      object1.getWorldPosition(v1);
      object2.getWorldPosition(v2);
      const dist = v1.distanceTo(v2);
      const dir = v2.sub(v1).normalize().multiplyScalar(dist);
      return dir;
    };
  })();

  function createConstraint(object1, object2, options) {
    const body1 = objectToBodyMap.get(object1);
    if (!body1) return;

    const body2 = objectToBodyMap.get(object2);
    if (!body2) return;

    let params;
    switch (options.type) {
    case 'fixed':
      params = RAPIER.JointData.fixed(
        object2.position, options.frame1,
        object1.position, options.frame2
      );
      break;
    case 'generic':
      const axeMask = RAPIER.JointAxesMask[options.axeMask];
      params = RAPIER.JointData.generic(
        options.anchor1, options.anchor2,
        options.axis, axeMask
      );
      break;
    case 'prismatic':
      if (isNaN(options.anchor1.x)) {
        Object.assign(options.anchor1, ZERO);
      }
      if (isNaN(options.anchor2.x)) {
        Object.assign(options.anchor2, ZERO);
      }
      params = RAPIER.JointData.prismatic(
        options.anchor1, options.anchor2,
        options.axis
      );
      if (options.limitsEnabled) {
        params.limitsEnabled = true;
        params.limits = [options.limitMin, options.limitMax];
      }
      break;
    case 'revolute':
      params = RAPIER.JointData.revolute(
        options.anchor1, options.anchor2,
        options.axis
      );
      break;
    case 'rope':
      params = RAPIER.JointData.rope(
        options.length, options.anchor1,
        options.anchor2
      );
      break;
    case 'spherical':
      params = RAPIER.JointData.spherical(
        options.anchor1, options.anchor2
      );
      break;
    case 'spring':
      if (isNaN(options.anchor1.x) && isNaN(options.anchor2.x)) {
        //
        // No anchor specified: the spring is assumed to be as long as
        // the distance between the two objects.
        //
        Object.assign(
          options.anchor1,
          distanceVector(object1, object2)
        );
      }
      if (isNaN(options.anchor1.x)) {
        Object.assign(options.anchor1, ZERO);
      }
      if (isNaN(options.anchor2.x)) {
        Object.assign(options.anchor2, ZERO);
      }
      params = RAPIER.JointData.spring(
        options.restLength, options.stiffness, options.damping,
        options.anchor1, options.anchor2
      );
      break;
    default:
      return null;
    }

    return world.createImpulseJoint(params, body1, body2, true);
  }

  function removeConstraint( constraint ) {
    world.removeImpulseJoint(constraint, true);
  }

  function getObjectFromCollider( collider ) {
    const body = collider.parent();
    const object = bodyToObjectMap.get(body);

    return object;
  }

  function syncBodyToObject( object, resetVelocities = false ) {
    const body = objectToBodyMap.get( object );

    if (resetVelocities) {
      body.setAngvel( ZERO );
      body.setLinvel( ZERO );
    }

    body.setTranslation( getWorldPosition(object) );
    body.setRotation( getWorldQuaternion(object) );
  }

  function wakeUpJointedBodies( body ) {
    const impulseJoints = world.impulseJoints;
    impulseJoints.forEachJointHandleAttachedToRigidBody(body.handle, (handle) => {
      const joint = impulseJoints.get(handle);
      joint.body1() === body ? joint.body2().wakeUp() : joint.body1().wakeUp();
    });
  }

  function step(timeDelta) {
    //
    // A kinematic body follows the object. Changes on the object are
    // applied to the body before the next simulation step.
    //
    world.forEachRigidBody((body) => {
      const object = bodyToObjectMap.get( body );
      if (!object) return;

      if (body.isKinematic()) {
        body.setNextKinematicTranslation( getWorldPosition(object) );
        body.setNextKinematicRotation( getWorldQuaternion(object) );
        wakeUpJointedBodies(body);
      }
    });

    world.timestep = timeDelta * 0.001;
    world.step(events);

    //
    // Active non-kinematic bodies apply their body changes on the
    // object after the simulation step.
    //
    world.forEachRigidBody((body) => {
      const object = bodyToObjectMap.get( body );
      if (!object) return;

      if (!body.isKinematic() && !body.isSleeping() && !body.isFixed() && body.isMoving()) {
        //
        // Detach and reattach so that the local position is the world
        // position: the body position is always in world coordinates.
        //
        const parent = object.parent;
        scene.attach( object );
        object.position.copy( body.translation() );
        object.quaternion.copy( body.rotation() );
        parent.attach( object );

        wakeUpJointedBodies(body);
      }
    });

    debugRenderer?.update();
  }

  return {
    /**
     * The physics world.
     *
     * @member
     * @name RapierPhysics#world
     */
    world: world,

    /**
     * The event queue to obtain collision events.
     *
     * @member
     * @name RapierPhysics#events
     */
    events: events,

    /**
     * Retrieves the THREE.Object3D associated with the Rapier handle.
     *
     * @method
     * @name RapierPhysics#getObjectFromCollider
     * @param {handle} the collider handle
     */
    getObjectFromCollider: getObjectFromCollider,

    /**
     * Creates the given object to this physics simulation.
     *
     * @method
     * @name RapierPhysics#createBody
     * @param {Object} object The object to create.
     */
    createBody: createBody,

    /**
     * Updates the given object in the physics simulation.
     *
     * @method
     * @name RapierPhysics#updateBody
     * @param {Object} object The object to update.
     */
    updateBody: updateBody,

    /**
     * Remove the given object from this physics simulation.
     *
     * @method
     * @name RapierPhysics#removeBody
     * @param {Object} object The object to remove.
     */
    removeBody: removeBody,

    /**
     * Creates the given object to this physics simulation.
     *
     * @method
     * @name RapierPhysics#createCollider
     * @param {Object} object The object to create.
     */
    createCollider: createCollider,

    /**
     * Updates the given object in the physics simulation.
     *
     * @method
     * @name RapierPhysics#updateCollider
     * @param {Object} object The object to update.
     */
    updateCollider: updateCollider,

    /**
     * Remove the given object from this physics simulation.
     *
     * @method
     * @name RapierPhysics#removeCollider
     * @param {Object} object The object to remove.
     */
    removeCollider: removeCollider,

    /**
     * Create a constraint between 2 objects
     *
     * @method
     * @name RapierPhysics#createConstraint
     * @param {Object} object1 Object 1 in te constraint.
     * @param {Object} object2 Object 2 in te constraint.
     * @param {Object} options Constraint options.
     */
    createConstraint: createConstraint,

    /**
     * Remove a joint constraint
     *
     * @method
     * @name RapierPhysics#removeConstraint
     * @param {Object} constraint
     */
    removeConstraint: removeConstraint,

    /**
     * Sync position and rotation of the given object onto its rigid
     * body which is part of the physics simulation. Optionally, this
     * will also reset linear and angular velocity, useful when
     * something should be teleported.
     *
     * @method
     * @name RapierPhysics#setObjectPosition
     * @param {Object} object The object we sync from.
     * @param {boolean} resetVelocities Flag deciding if velocities should be reset.
     */
    syncBodyToObject: syncBodyToObject,

    /**
     * Simulate physics at specified timestep. To be used in a tick handler.
     *
     * @method
     * @name RapierPhysics#step
     * @param timeDelta time delta - as received by A-Frame component tick handler.
     */
    step: step
  };

}

window.AFRAME.registerSystem('rapier-physics', {
  schema: {
    gravity: { type: 'vec3', default: {x: 0, y: -9.81, z: 0} },
    // If true, show wireframes around physics bodies.
    debug: { type: 'boolean', default: false },
    // Whether to output stats, and how to output them.  One or more of "console", "events", "panel"
    stats: {type: 'array', default: []},
    // When the animation freezes, we may end up with big timesteps
    // that the simulation will not handle well. Cap the timestep to
    // this many milliseconds.
    maxTimeStep: { type: 'number', default: 100 }
  },
  init: function () {
    const options = {
      'scene': this.el.object3D,
      'debug': this.data.debug,
      'gravity': this.data.gravity
    };

    (async () => {

      this.physics = await RapierPhysics(options);

      this.initStats();

      this.el.emit('physics-loaded');

    })();

  },
  initStats() {
    // Data used for performance monitoring.
    this.statsToConsole = this.data.stats.includes("console");
    this.statsToEvents = this.data.stats.includes("events");
    this.statsToPanel = this.data.stats.includes("panel");

    this.numCollisions = 0;
    this.numManifolds = 0;

    if (this.statsToConsole || this.statsToEvents || this.statsToPanel) {
      this.trackPerf = true;
      this.tickCounter = 0;

      this.statsTickData = {};
      this.statsBodyData = {};

      this.bodyTypeToStatsPropertyMap = {
        [RAPIER.RigidBodyType.Fixed] : "staticBodies",
        [RAPIER.RigidBodyType.KinematicVelocityBased] : "kinematicBodies",
        [RAPIER.RigidBodyType.KinematicPositionBased] : "kinematicBodies",
        [RAPIER.RigidBodyType.Dynamic] : "dynamicBodies"
      };

      this.countBodies = () => {
        const statsData = this.statsBodyData
        statsData.manifolds = this.numManifolds;
        statsData.collisions = this.numCollisions;
        statsData.collisionKeys = this.physics.world.colliders.len();
        statsData.staticBodies = 0
        statsData.kinematicBodies = 0
        statsData.dynamicBodies = 0

        this.physics.world.bodies.forEach((rigidBody) => {
          const bodyType = this.bodyTypeToStatsPropertyMap[rigidBody.bodyType()];
          this.statsBodyData[bodyType]++;
        });
      }

      const scene = this.el.sceneEl;
      scene.setAttribute("stats-collector", `inEvent: physics-tick-data;
                                             properties: before, after, engine, total;
                                             outputFrequency: 100;
                                             outEvent: physics-tick-summary;
                                             outputs: percentile__50, percentile__90, max`);
    }

    if (this.statsToPanel) {
      const scene = this.el.sceneEl;
      const space = "&nbsp&nbsp&nbsp"

      scene.setAttribute("stats-panel", "")
      scene.setAttribute("stats-group__bodies", `label: Physics Bodies`)
      scene.setAttribute("stats-row__b1", `group: bodies;
                                           event:physics-body-data;
                                           properties: staticBodies;
                                           label: Static`)
      scene.setAttribute("stats-row__b2", `group: bodies;
                                           event:physics-body-data;
                                           properties: dynamicBodies;
                                           label: Dynamic`)
      scene.setAttribute("stats-row__b3", `group: bodies;
                                             event:physics-body-data;
                                             properties: kinematicBodies;
                                             label: Kinematic`)
      scene.setAttribute("stats-row__b4", `group: bodies;
                                             event: physics-body-data;
                                             properties: manifolds;
                                             label: Manifolds`)
      scene.setAttribute("stats-row__b5", `group: bodies;
                                             event: physics-body-data;
                                             properties: collisions;
                                             label: Collisions`)
      scene.setAttribute("stats-row__b6", `group: bodies;
                                             event: physics-body-data;
                                             properties: collisionKeys;
                                             label: Coll Keys`)

      scene.setAttribute("stats-group__tick", `label: Physics Ticks: Median${space}90th%${space}99th%`)
      scene.setAttribute("stats-row__1", `group: tick;
                                          event:physics-tick-summary;
                                          properties: before.percentile__50,
                                                      before.percentile__90,
                                                      before.max;
                                          label: Before`)
      scene.setAttribute("stats-row__2", `group: tick;
                                          event:physics-tick-summary;
                                          properties: after.percentile__50,
                                                      after.percentile__90,
                                                      after.max;
                                          label: After`)
      scene.setAttribute("stats-row__3", `group: tick;
                                          event:physics-tick-summary;
                                          properties: engine.percentile__50,
                                                      engine.percentile__90,
                                                      engine.max;
                                          label: Engine`)
      scene.setAttribute("stats-row__4", `group: tick;
                                          event:physics-tick-summary;
                                          properties: total.percentile__50,
                                                      total.percentile__90,
                                                      total.max;
                                          label: Total`)
    }
  },
  tick: function (time, timeDelta) {
    if (!this.physics) return;

    // TODO: aframe-physics-system supported callbacks before/after
    // the physics step... not sure we need this.
    const beforeStartTime = performance.now();

    const engineStartTime = performance.now();

    this.physics.step(Math.min(timeDelta, this.data.maxTimeStep));

    const world = this.physics.world;
    const events = this.physics.events;

    // Review events
    this.numManifolds = 0;
    events.drainCollisionEvents((handle1, handle2, started) => {

      this.numCollisions+= started ? 1 : -1;
      const collider1 = world.getCollider(handle1);
      const collider2 = world.getCollider(handle2);
      const object1 = this.physics.getObjectFromCollider(collider1);
      const object2 = this.physics.getObjectFromCollider(collider2);
      const eventName = started ? 'collidestart' : 'collideend';
      world.narrowPhase.contactPair(handle1, handle2, (manifold, flipped) => {
        this.numManifolds++;

        const contactPoint1 = manifold.localContactPoint1();
        const contactPoint2 = manifold.localContactPoint2();
        const impulse = manifold.contactImpulse();

        if (collider1.activeEvents() !== RAPIER.ActiveEvents.NONE) {
          object1.el.emit(eventName, object1.el, {
            targetEl: object2.el,
            contactPoint: contactPoint1,
            targetContactPoint: contactPoint2,
            impulse: impulse
          });
        }
        if (collider1.activeEvents() !== RAPIER.ActiveEvents.NONE) {
          object2.el.emit(eventName, object2.el, {
            targetEl: object1.el,
            contactPoint: contactPoint2,
            targetContactPoint: contactPoint1,
            impulse: impulse
          });
        }
      });
    });

    const engineEndTime = performance.now();

    if (this.trackPerf) {
      const afterEndTime = performance.now();

      this.statsTickData.before = engineStartTime - beforeStartTime
      this.statsTickData.engine = engineEndTime - engineStartTime
      this.statsTickData.after = afterEndTime - engineEndTime
      this.statsTickData.total = afterEndTime - beforeStartTime

      this.el.emit("physics-tick-data", this.statsTickData)

      this.tickCounter++;

      if (this.tickCounter === 100) {

        this.countBodies();

        if (this.statsToConsole) {
          console.log("Physics body stats:", this.statsBodyData)
        }

        if (this.statsToEvents  || this.statsToPanel) {
          this.el.emit("physics-body-data", this.statsBodyData)
        }
        this.tickCounter = 0;
      }
    }
  }
});

window.AFRAME.registerComponent('rapier-body', {
  schema: {
    type: {
      type: 'string', default: 'Dynamic',
      oneOf: ['Fixed', 'Dynamic', 'KinematicVelocityBased', 'KinematicPositionBased']
    },
    canSleep: { type: 'boolean', default: true },
    ccdEnabled: { type: 'boolean', default: false }
  },
  init: function () {
    this.system = this.el.sceneEl.systems['rapier-physics'];
    this.options = Object.assign({}, this.data);
  },
  update: function () {
    if (!this.body) return;
    Object.assign(this.options, this.data);
    this.system.physics.updateBody(
      this.el.object3D,
      this.options
    );
  },
  createBody() {
    if (this.body) return;
    if (!this.system.physics) {
      this.el.sceneEl.addEventListener('physics-loaded', (evt) => {
        this.createBody();
      }, {once: true});
    } else {
      this.body = this.system.physics.createBody(
        this.el.object3D,
        this.options
      );
      this.el.emit('body-loaded');
    }
  },
  play: function () {
    this.createBody();
  },
  pause: function () {
    if (!this.body) return;
    this.system.physics.removeBody(this.el.object3D);
    this.body = null;
  }
});

window.AFRAME.registerComponent('rapier-shape', {
  dependencies: ['rapier-body'],
  multiple: true,
  schema: {
    fit: { type: 'boolean', default: true },
    sensor: { type: 'boolean', default: false },
    shape: {
      type: 'string',
      default: 'ConvexHull',
      oneOf: [
        'Cuboid',
        'Cylinder',
        'Capsule',
        'Cone',
        'Ball',
        'ConvexHull',
        'TriMesh'
      ]
    },
    offset: { type: 'vec3', default: { x: 0, y: 0, z: 0 } },
    halfExtents: { type: 'vec3', default: { x: 1, y: 1, z: 1 } },
    minHalfExtent: { type: 'number', default: 0.05 },
    maxHalfExtent: { type: 'number', default: Number.POSITIVE_INFINITY },
    radius: { type: 'number', default: 0 },
    includeInvisible: { type: 'boolean', default: false },
    contactSkin: { type: 'number', default: 0 },
    density: { type: 'number', default: 0 },
    friction: { type: 'number', default: 0.5 },
    mass: { type: 'number', default: 1 },
    restitution: { type: 'number', default: 0 },
    axis: { type: 'string', default: 'y' },
    scaleAutoUpdate: { type: 'boolean', default: false },
    emitCollisionEvents: { type: 'boolean', default: false },
    enabled: { type: 'boolean', default: true },
    collisionGroups: {type: 'number', default: NaN},
    solverGroups: {type: 'number', default: NaN}
  },
  init: function () {
    this.system = this.el.sceneEl.systems['rapier-physics'];
    this.bodyComponent = this.el.components['rapier-body'];

    this.options = Object.assign({}, this.data);

    const scale = getWorldScale(this.el.object3D);
    this.oldScale = new THREE.Vector3();
    this.oldScale.copy(scale);
  },
  update: function () {
    if (!this.collider) return;
    Object.assign(this.options, this.data);
    this.system.physics.updateCollider(
      this.el.object3D,
      this.collider,
      this.options,
      false
    );
  },
  createCollider() {
    if (this.collider) return;

    if (!this.bodyComponent.body) {
      this.el.addEventListener('body-loaded', () => {
        this.createCollider();
      }, {once: true});
    } else if (!this.el.object3DMap.mesh) {
      this.el.addEventListener('object3dset', (evt) => {
        if (evt.detail.type === 'mesh') {
          this.createCollider();
        }
      }, {once: true});
    } else {
      this.collider = this.system.physics.createCollider(
        this.el.object3D,
        this.options
      );
    }
  },
  play: function () {
    this.createCollider();
  },
  pause: function () {
    if (!this.collider) return;
    this.system.physics.removeCollider(this.collider);
    this.collider = null;
  },
  scaleHasChanged () {
    const scale = getWorldScale(this.el.object3D);
    const distance = scale.distanceToSquared(this.oldScale);
    const scaleChanged = distance > 0.0001;
    this.oldScale.copy(scale);
    return scaleChanged;
  },
  tick: function () {
    if (this.collider && this.data.scaleAutoUpdate) {
      if (this.scaleHasChanged()) {
        Object.assign(this.options, this.data);
        this.system.physics.updateCollider(
          this.el.object3D,
          this.collider,
          this.options,
          true
        );
      }
    }
  }
});

window.AFRAME.registerComponent('rapier-constraint', {
  dependencies: ['rapier-body'],
  multiple: true,
  schema: {
    type: {
      type: 'string', default: 'fixed',
      oneOf: ['fixed', 'generic', 'prismatic', 'revolute', 'rope', 'spring']
    },
    target: { type: 'selector' },
    anchor1: { type: 'vec3', default: null },
    anchor2: { type: 'vec3', default: null },
    frame1: { type: 'vec4', default: { x: 0, y: 0, z: 0, w: 1} },
    frame2: { type: 'vec4', default: { x: 0, y: 0, z: 0, w: 1} },
    axis: { type: 'vec3', default: { x: 0, y: 0, z: 1 } },
    length: { type: 'number', default: 0 },
    restLength: { type: 'number', default: 0 },
    stiffness: { type: 'number', default: 0 },
    damping: { type: 'number', default: 0 },
    limitsEnabled: { type: 'boolean', default: false },
    limitMin: { type: 'number', default: Number.NEGATIVE_INFINITY },
    limitMax: { type: 'number', default: Number.POSITIVE_INFINITY },
    axesMask: { type: 'string' }
  },
  init: function () {
    this.system = this.el.sceneEl.systems['rapier-physics'];
    this.createConstraint = this.createConstraint.bind(this);
    this.bodyComponent = this.el.components['rapier-body'];
  },
  createConstraint () {
    if (this.constraint) return;

    if (!this.data.target) return;

    if (!this.bodyComponent.body) {
      this.el.addEventListener('body-loaded', this.createConstraint, {once: true});
      return;
    }

    this.targetBodyComponent = this.data.target.components['rapier-body'];
    if (!this.targetBodyComponent || !this.targetBodyComponent.body) {
      this.data.target.addEventListener('body-loaded', this.createConstraint, {once: true});
      return;
    }

    const source = this.el.object3D;
    const target = this.data.target.object3D;
    const options = this.data;

    this.constraint = this.system.physics.createConstraint(source, target, options);
  },
  removeConstraint () {
    if (!this.constraint) return;
    this.system.physics.removeConstraint(this.constraint);
    this.constraint = null;
  },
  update: function () {
    if (this.isPlaying) {
      this.removeConstraint();
      this.createConstraint();
    }
  },
  play: function () {
    this.createConstraint();
  },
  pause: function () {
    this.removeConstraint();
  }
});

//
// Local variables:
//    mode: javascript
//    js-indent-level: 2
//    indent-tabs-mode: nil
// End:
//
