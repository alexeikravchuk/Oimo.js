import { CollisionDetector } from './CollisionDetector';
import { _Math } from '../../math/Math';
import { Vec3 } from '../../math/Vec3';

/**
 * A collision detector which detects collisions between two spheres.
 * @author saharan
 * @author lo-th
 */

export class SphereSphereCollisionDetector extends CollisionDetector {
	n = new Vec3();
	p = new Vec3();

	detectCollision(shape1, shape2, manifold) {
		const {
			n,
			p
		} = this;

		n.sub(shape2.position, shape1.position);
		const rad = shape1.radius + shape2.radius;
		let len = n.lengthSq();

		if (len > 0 && len < rad * rad) {
			len = _Math.sqrt(len);
			n.scaleEqual(1 / len);

			//n.normalize();
			p.copy(shape1.position).addScaledVector(n, shape1.radius);
			manifold.addPointVec(p, n, len - rad, false);
		}
	}
}
