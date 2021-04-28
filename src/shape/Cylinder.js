import {
	SHAPE_CYLINDER,
	AABB_PROX
} from '../constants';
import { Shape } from './Shape';
import { Vec3 } from '../math/Vec3';

/**
 * Cylinder shape
 * @author saharan
 * @author lo-th
 */

export class Cylinder extends Shape {
	type = SHAPE_CYLINDER;
	normalDirection = new Vec3();
	halfDirection = new Vec3();

	constructor(config, radius, height) {
		super(config);

		this.radius = radius;
		this.height = height;
		this.halfHeight = height * 0.5;

	}

	calculateMassInfo(out) {
		const rsq = this.radius * this.radius;
		const mass = Math.PI * rsq * this.height * this.density;
		const inertiaXZ = ((0.25 * rsq) + (0.0833 * this.height * this.height)) * mass;
		const inertiaY = 0.5 * rsq;
		out.mass = mass;
		out.inertia.set(inertiaXZ, 0, 0, 0, inertiaY, 0, 0, 0, inertiaXZ);
	}

	updateProxy() {
		const te = this.rotation.elements;
		let len,
			wx,
			hy,
			dz,
			xx,
			yy,
			zz,
			w,
			h,
			d,
			p;

		xx = te[1] * te[1];
		yy = te[4] * te[4];
		zz = te[7] * te[7];

		this.normalDirection.set(te[1], te[4], te[7]);
		this.halfDirection.scale(this.normalDirection, this.halfHeight);

		wx = 1 - xx;
		len = Math.sqrt(wx * wx + xx * yy + xx * zz);
		if (len > 0) {
			len = this.radius / len;
		}
		wx *= len;
		hy = 1 - yy;
		len = Math.sqrt(yy * xx + hy * hy + yy * zz);
		if (len > 0) {
			len = this.radius / len;
		}
		hy *= len;
		dz = 1 - zz;
		len = Math.sqrt(zz * xx + zz * yy + dz * dz);
		if (len > 0) {
			len = this.radius / len;
		}
		dz *= len;

		w = this.halfDirection.x < 0 ? -this.halfDirection.x : this.halfDirection.x;
		h = this.halfDirection.y < 0 ? -this.halfDirection.y : this.halfDirection.y;
		d = this.halfDirection.z < 0 ? -this.halfDirection.z : this.halfDirection.z;

		w = wx < 0 ? w - wx : w + wx;
		h = hy < 0 ? h - hy : h + hy;
		d = dz < 0 ? d - dz : d + dz;

		p = AABB_PROX;

		this.aabb.set(
			this.position.x - w - p, this.position.x + w + p,
			this.position.y - h - p, this.position.y + h + p,
			this.position.z - d - p, this.position.z + d + p
		);

		if (this.proxy != null) {
			this.proxy.update();
		}

	}
}
