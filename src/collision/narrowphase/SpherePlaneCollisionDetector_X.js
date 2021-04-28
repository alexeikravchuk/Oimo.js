import { CollisionDetector } from './CollisionDetector';
import { Vec3 } from '../../math/Vec3';

/**
 * A collision detector which detects collisions between two spheres.
 * @author saharan
 * @author lo-th
 */

export class SpherePlaneCollisionDetector extends CollisionDetector {
	n = new Vec3();
	p = new Vec3();

	constructor(flip) {
		super();
		this.flip = flip;
	}

	detectCollision(shape1, shape2, manifold) {
		const { n, p, flip } = this;

		const sphere = flip ? shape2 : shape1;
		const plane = flip ? shape1 : shape2;
		const rad = sphere.radius;

		n.sub(sphere.position, plane.position);

		n.x *= plane.normal.x;
		n.y *= plane.normal.y;
		n.z *= plane.normal.z;

		const len = Math.sqrt(n.lengthSq());

		if (len > 0 && len < rad) {
			n.copy(plane.normal).negate();

			p.copy(sphere.position).addScaledVector(n, rad);
			manifold.addPointVec(p, n, len - rad, flip);
		}
	}
}
