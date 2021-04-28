/*
 * A list of constants built-in for
 * the physics engine.
 */

export const REVISION = '1.0.9';

// BroadPhase
export const BR_NULL = 0;
export const BR_BRUTE_FORCE = 1;
export const BR_SWEEP_AND_PRUNE = 2;
export const BR_BOUNDING_VOLUME_TREE = 3;

// Body type
export const BODY_NULL = 0;
export const BODY_DYNAMIC = 1;
export const BODY_STATIC = 2;
export const BODY_KINEMATIC = 3;
export const BODY_GHOST = 4;

// Shape type
export const SHAPE_NULL = 0;
export const SHAPE_SPHERE = 1;
export const SHAPE_BOX = 2;
export const SHAPE_CYLINDER = 3;
export const SHAPE_PLANE = 4;
export const SHAPE_PARTICLE = 5;
export const SHAPE_TETRA = 6;

// Joint type
export const JOINT_NULL = 0;
export const JOINT_DISTANCE = 1;
export const JOINT_BALL_AND_SOCKET = 2;
export const JOINT_HINGE = 3;
export const JOINT_WHEEL = 4;
export const JOINT_SLIDER = 5;
export const JOINT_PRISMATIC = 6;

// AABB approximation
export const AABB_PROX = 0.005;