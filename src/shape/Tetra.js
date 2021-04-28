import {
	SHAPE_TETRA,
	AABB_PROX
} from '../constants';
import { Shape } from './Shape';

/**
 * A tetra shape.
 * @author xprogram
 */

export class Tetra extends Shape {
	type = SHAPE_TETRA;

	constructor(config, p1, p2, p3, p4) {
		super(config);
		this.verts = [p1, p2, p3, p4];
		this.faces = [this.mtri(0, 1, 2), this.mtri(1, 2, 3), this.mtri(2, 3, 4), this.mtri(4, 0, 1)];
	}

	calculateMassInfo(out) {
		// I guess you could calculate box mass and split it
		// in half for the tetra...
		this.aabb.setFromPoints(this.verts);
		const p = this.aabb.elements;
		const x = p[3] - p[0];
		const y = p[4] - p[1];
		const z = p[5] - p[2];
		const mass = x * y * z * this.density;
		const divid = 1 / 12;
		out.mass = mass;
		out.inertia.set(
			mass * (2 * y * 2 * y + 2 * z * 2 * z) * divid, 0, 0,
			0, mass * (2 * x * 2 * x + 2 * z * 2 * z) * divid, 0,
			0, 0, mass * (2 * y * 2 * y + 2 * x * 2 * x) * divid
		);
	}

	updateProxy() {
		this.aabb.setFromPoints(this.verts);
		this.aabb.expandByScalar(AABB_PROX);

		if (this.proxy !== null) {
			this.proxy.update();
		}
	}

	mtri(a, b, c) {
		return {
			a: a,
			b: b,
			c: c
		};
	}
}
