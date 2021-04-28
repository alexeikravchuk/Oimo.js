import {
	SHAPE_PLANE,
	AABB_PROX
} from '../constants';
import { Shape } from './Shape';
import { _Math } from '../math/Math';
import { Vec3 } from '../math/Vec3';

/**
 * Plane shape.
 * @author lo-th
 */

export class Plane extends Shape {
	type = SHAPE_PLANE;

	constructor(config, normal) {
		super(config);

		// radius of the shape.
		this.normal = new Vec3(0, 1, 0);
	}


	volume() {
		return Number.MAX_VALUE;
	}

	calculateMassInfo(out) {
		out.mass = this.density;//0.0001;
		const inertia = 1;
		out.inertia.set(inertia, 0, 0, 0, inertia, 0, 0, 0, inertia);
	}

	updateProxy() {
		const p = AABB_PROX;

		const min = -_Math.INF;
		const max = _Math.INF;
		const n = this.normal;
		// The plane AABB is infinite, except if the normal is pointing along any axis
		this.aabb.set(
			n.x === -1 ? this.position.x - p : min, n.x === 1 ? this.position.x + p : max,
			n.y === -1 ? this.position.y - p : min, n.y === 1 ? this.position.y + p : max,
			n.z === -1 ? this.position.z - p : min, n.z === 1 ? this.position.z + p : max
		);

		if (this.proxy != null) {
			this.proxy.update();
		}
	}
}



