import { SHAPE_NULL } from '../constants';
import { printError } from '../core/Utils';
import { Vec3 } from '../math/Vec3';
import { Mat33 } from '../math/Mat33';
import { AABB } from '../math/AABB';

let count = 0;

function ShapeIdCount() {
	return count++;
}

/**
 * A shape is used to detect collisions of rigid bodies.
 *
 * @author saharan
 * @author lo-th
 */

export class Shape {
	type = SHAPE_NULL;
	Shape = true;
	// global identification of the shape should be unique to the shape.
	id = ShapeIdCount();
	// previous shape in parent rigid body. Used for fast interactions.
	prev = null;
	// next shape in parent rigid body. Used for fast interactions.
	next = null;
	// proxy of the shape used for broad-phase collision detection.
	proxy = null;
	// parent rigid body of the shape.
	parent = null;
	// linked list of the contacts with the shape.
	contactLink = null;
	// number of the contacts with the shape.
	numContacts = 0;
	// center of gravity of the shape in world coordinate system.
	position = new Vec3();
	// rotation matrix of the shape in world coordinate system.
	rotation = new Mat33();
	// axis-aligned bounding box of the shape.
	aabb = new AABB();

	/**
	 * Shape base class
	 * @param {Object} config
	 * @param {number} config.density
	 * @param {number} config.friction
	 * @param {number} config.restitution
	 * @param {number} config.belongsTo
	 * @param {number} config.collidesWith
	 * @param {Vec3} config.relativePosition
	 * @param {Mat33} config.relativeRotation
	 */
	constructor(config) {
		const {
			density,
			friction,
			restitution,
			belongsTo,
			collidesWith,
			relativePosition,
			relativeRotation
		} = config;

		// density of the shape.
		this.density = density;
		// coefficient of friction of the shape.
		this.friction = friction;
		// coefficient of restitution of the shape.
		this.restitution = restitution;
		// bits of the collision groups to which the shape belongs.
		this.belongsTo = belongsTo;
		// bits of the collision groups with which the shape collides.
		this.collidesWith = collidesWith;
		// position of the shape in parent's coordinate system.
		this.relativePosition = new Vec3().copy(relativePosition);
		// rotation matrix of the shape in parent's coordinate system.
		this.relativeRotation = new Mat33().copy(relativeRotation);
	}

	// Calculate the mass information of the shape.
	calculateMassInfo(out) {
		printError('Shape', 'Inheritance error.');
	}

	// Update the proxy of the shape.
	updateProxy() {
		printError('Shape', 'Inheritance error.');
	}
}