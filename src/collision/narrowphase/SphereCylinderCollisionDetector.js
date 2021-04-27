import { CollisionDetector } from './CollisionDetector';
import { _Math } from '../../math/Math';

export class SphereCylinderCollisionDetector extends CollisionDetector {
	constructor(flip) {
		super();
		this.flip = flip;
	}

	detectCollision(shape1, shape2, manifold) {
		let s,
			c;
		if (this.flip) {
			s = shape2;
			c = shape1;
		} else {
			s = shape1;
			c = shape2;
		}

		const ps = s.position;
		const psx = ps.x;
		const psy = ps.y;
		const psz = ps.z;
		const pc = c.position;
		const pcx = pc.x;
		const pcy = pc.y;
		const pcz = pc.z;
		const dirx = c.normalDirection.x;
		const diry = c.normalDirection.y;
		const dirz = c.normalDirection.z;
		const rads = s.radius;
		const radc = c.radius;
		const rad2 = rads + radc;
		const halfh = c.halfHeight;
		let dx = psx - pcx;
		let dy = psy - pcy;
		let dz = psz - pcz;
		let dot = dx * dirx + dy * diry + dz * dirz;

		if (dot < -halfh - rads || dot > halfh + rads) {
			return;
		}
		var cx = pcx + dot * dirx;
		var cy = pcy + dot * diry;
		var cz = pcz + dot * dirz;
		var d2x = psx - cx;
		var d2y = psy - cy;
		var d2z = psz - cz;
		var len = d2x * d2x + d2y * d2y + d2z * d2z;
		if (len > rad2 * rad2) {
			return;
		}
		if (len > radc * radc) {
			len = radc / _Math.sqrt(len);
			d2x *= len;
			d2y *= len;
			d2z *= len;
		}
		if (dot < -halfh) {
			dot = -halfh;
		} else if (dot > halfh) {
			dot = halfh;
		}
		cx = pcx + dot * dirx + d2x;
		cy = pcy + dot * diry + d2y;
		cz = pcz + dot * dirz + d2z;
		dx = cx - psx;
		dy = cy - psy;
		dz = cz - psz;
		len = dx * dx + dy * dy + dz * dz;
		var invLen;
		if (len > 0 && len < rads * rads) {
			len = _Math.sqrt(len);
			invLen = 1 / len;
			dx *= invLen;
			dy *= invLen;
			dz *= invLen;
			///result.addContactInfo(psx+dx*rads,psy+dy*rads,psz+dz*rads,dx,dy,dz,len-rads,s,c,0,0,false);
			manifold.addPoint(psx + dx * rads, psy + dy * rads, psz + dz * rads, dx, dy, dz, len - rads, this.flip);
		}
	}
}
