import { CollisionDetector } from './CollisionDetector';
import { _Math } from '../../math/Math';

/**
 * A collision detector which detects collisions between two spheres.
 * @author saharan
 */

export class SphereSphereCollisionDetector extends CollisionDetector {
	detectCollision(shape1, shape2, manifold) {
		const p1 = shape1.position;
		const p2 = shape2.position;
		let dx = p2.x - p1.x;
		let dy = p2.y - p1.y;
		let dz = p2.z - p1.z;
		let len = dx * dx + dy * dy + dz * dz;
		const r1 = shape1.radius;
		const r2 = shape2.radius;
		const rad = r1 + r2;
		if (len > 0 && len < rad * rad) {
			len = _Math.sqrt(len);
			const invLen = 1 / len;
			dx *= invLen;
			dy *= invLen;
			dz *= invLen;
			manifold.addPoint(p1.x + dx * r1, p1.y + dy * r1, p1.z + dz * r1, dx, dy, dz, len - rad, false);
		}
	}
}
