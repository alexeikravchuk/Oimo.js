import { SHAPE_PARTICLE } from '../constants';
import { Shape } from './Shape';

/**
 * A Particle shape
 * @author lo-th
 */

export class Particle extends Shape {
	type = SHAPE_PARTICLE;

	constructor(config, normal) {
		super(config);
	}

	volume() {
		return Number.MAX_VALUE;
	}

	calculateMassInfo(out) {
		const inertia = 0;
		out.inertia.set(inertia, 0, 0, 0, inertia, 0, 0, 0, inertia);
	}

	updateProxy() {
		const p = 0;//AABB_PROX;

		this.aabb.set(
			this.position.x - p, this.position.x + p,
			this.position.y - p, this.position.y + p,
			this.position.z - p, this.position.z + p
		);

		if (this.proxy != null) {
			this.proxy.update();
		}
	}
}
