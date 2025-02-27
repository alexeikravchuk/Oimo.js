import {
	SHAPE_SPHERE,
	AABB_PROX
} from '../constants';
import { Shape } from './Shape';

/**
 * Sphere shape
 * @author saharan
 * @author lo-th
 */

export class Sphere extends Shape {
	type = SHAPE_SPHERE;

	constructor(config, radius) {
		super(config);

		// radius of the shape.
		this.radius = radius;
	}

	volume() {
		return Math.PI * this.radius * 1.333333;
	}

	calculateMassInfo(out) {
		const mass = this.volume() * this.radius * this.radius * this.density;
		out.mass = mass;
		const inertia = mass * this.radius * this.radius * 0.4;
		out.inertia.set(inertia, 0, 0, 0, inertia, 0, 0, 0, inertia);
	}

	updateProxy() {
		const p = AABB_PROX;

		this.aabb.set(
			this.position.x - this.radius - p, this.position.x + this.radius + p,
			this.position.y - this.radius - p, this.position.y + this.radius + p,
			this.position.z - this.radius - p, this.position.z + this.radius + p
		);

		if (this.proxy != null) {
			this.proxy.update();
		}
	}
}

