import { ManifoldPoint } from './ManifoldPoint';

/**
 * A contact manifold between two shapes.
 * @author saharan
 * @author lo-th
 */

export class ContactManifold {
	// The first rigid body.
	body1 = null;
	// The second rigid body.
	body2 = null;
	// The number of manifold points.
	numPoints = 0;
	// The manifold points.
	points = [
		new ManifoldPoint(),
		new ManifoldPoint(),
		new ManifoldPoint(),
		new ManifoldPoint()
	];

	// Reset the manifold.
	reset(shape1, shape2) {
		this.body1 = shape1.parent;
		this.body2 = shape2.parent;
		this.numPoints = 0;
	}

	//  Add a point into this manifold.
	/**
	 *
	 * @param {Vec3} pos
	 * @param {Vec3} norm
	 * @param {number} penetration
	 * @param {boolean} flip
	 */
	addPointVec(pos, norm, penetration, flip) {
		this.addPoint(pos.x, pos.y, pos.z, norm.x, norm.y, norm.z, penetration, flip);
	}

	/**
	 * Add a point into this manifold.
	 * @param {number} x
	 * @param {number} y
	 * @param {number} z
	 * @param {number} nx
	 * @param {number} ny
	 * @param {number} nz
	 * @param {number} penetration
	 * @param {boolean} flip
	 */
	addPoint(x, y, z, nx, ny, nz, penetration, flip) {
		const {
			points,
			body1,
			body2
		} = this;

		const p = points[this.numPoints++];

		p.position.set(x, y, z);
		p.localPoint1.sub(p.position, body1.position).applyMatrix3(body1.rotation);
		p.localPoint2.sub(p.position, body2.position).applyMatrix3(body2.rotation);

		p.normalImpulse = 0;
		p.normal.set(nx, ny, nz);

		if (flip) {
			p.normal.negate();
		}

		p.penetration = penetration;
		p.warmStarted = false;
	}
}
