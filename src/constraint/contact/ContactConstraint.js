import { Constraint } from '../Constraint';
import { ContactPointDataBuffer } from './ContactPointDataBuffer';
import { Vec3 } from '../../math/Vec3';
import { _Math } from '../../math/Math';

/**
 * ...
 * @author saharan
 */
export class ContactConstraint extends Constraint {
	// The coefficient of restitution of the constraint.
	restitution = NaN;

	// The coefficient of friction of the constraint.
	friction = NaN;
	p1 = null;
	p2 = null;
	lv1 = null;
	lv2 = null;
	av1 = null;
	av2 = null;
	i1 = null;
	i2 = null;

	tmp = new Vec3();
	tmpC1 = new Vec3();
	tmpC2 = new Vec3();

	tmpP1 = new Vec3();
	tmpP2 = new Vec3();

	tmplv1 = new Vec3();
	tmplv2 = new Vec3();
	tmpav1 = new Vec3();
	tmpav2 = new Vec3();

	num = 0;

	constructor(manifold) {
		super();
		// The contact manifold of the constraint.
		this.manifold = manifold;

		this.ps = manifold.points;
		const cs = this.cs = new ContactPointDataBuffer();
		cs.next = new ContactPointDataBuffer();
		cs.next.next = new ContactPointDataBuffer();
		cs.next.next.next = new ContactPointDataBuffer();
	}

	// Attach the constraint to the bodies.
	attach() {
		const {
			body1,
			body2
		} = this;

		this.p1 = body1.position;
		this.p2 = body2.position;
		this.lv1 = body1.linearVelocity;
		this.av1 = body1.angularVelocity;
		this.lv2 = body2.linearVelocity;
		this.av2 = body2.angularVelocity;
		this.i1 = body1.inverseInertia;
		this.i2 = body2.inverseInertia;
	}

	// Detach the constraint from the bodies.
	detach() {
		this.p1 = null;
		this.p2 = null;
		this.lv1 = null;
		this.lv2 = null;
		this.av1 = null;
		this.av2 = null;
		this.i1 = null;
		this.i2 = null;
	}

	preSolve(timeStep, invTimeStep) {
		let {
			body1,
			body2,
			manifold,
			cs,
			p1,
			p2,
			ps,
			av1,
			av2,
			lv1,
			lv2,
			tmp,
			tmpC1,
			tmpC2,
			tmpP1,
			tmpP2,
		} = this;

		const mass1 = body1.inverseMass;
		const mass2 = body2.inverseMass;
		const m1m2 = mass1 + mass2;

		this.num = manifold.numPoints;

		let p,
			rvn,
			len,
			norImp,
			norTar,
			sepV,
			i1,
			i2;

		for (let i = 0; i < this.num; i++) {
			p = ps[i];

			tmpP1.sub(p.position, p1);
			tmpP2.sub(p.position, p2);

			tmpC1.crossVectors(av1, tmpP1);
			tmpC2.crossVectors(av2, tmpP2);

			cs.norImp = p.normalImpulse;
			cs.tanImp = p.tangentImpulse;
			cs.binImp = p.binormalImpulse;

			cs.nor.copy(p.normal);

			tmp.set(
				(lv2.x + tmpC2.x) - (lv1.x + tmpC1.x),
				(lv2.y + tmpC2.y) - (lv1.y + tmpC1.y),
				(lv2.z + tmpC2.z) - (lv1.z + tmpC1.z)
			);

			rvn = _Math.dotVectors(cs.nor, tmp);

			cs.tan.set(
				tmp.x - rvn * cs.nor.x,
				tmp.y - rvn * cs.nor.y,
				tmp.z - rvn * cs.nor.z
			);

			len = _Math.dotVectors(cs.tan, cs.tan);

			if (len <= 0.04) {
				cs.tan.tangent(cs.nor);
			}

			cs.tan.normalize();

			cs.bin.crossVectors(cs.nor, cs.tan);

			cs.norU1.scale(cs.nor, mass1);
			cs.norU2.scale(cs.nor, mass2);

			cs.tanU1.scale(cs.tan, mass1);
			cs.tanU2.scale(cs.tan, mass2);

			cs.binU1.scale(cs.bin, mass1);
			cs.binU2.scale(cs.bin, mass2);

			cs.norT1.crossVectors(tmpP1, cs.nor);
			cs.tanT1.crossVectors(tmpP1, cs.tan);
			cs.binT1.crossVectors(tmpP1, cs.bin);

			cs.norT2.crossVectors(tmpP2, cs.nor);
			cs.tanT2.crossVectors(tmpP2, cs.tan);
			cs.binT2.crossVectors(tmpP2, cs.bin);

			i1 = this.i1;
			i2 = this.i2;

			cs.norTU1.copy(cs.norT1).applyMatrix3(i1, true);
			cs.tanTU1.copy(cs.tanT1).applyMatrix3(i1, true);
			cs.binTU1.copy(cs.binT1).applyMatrix3(i1, true);

			cs.norTU2.copy(cs.norT2).applyMatrix3(i2, true);
			cs.tanTU2.copy(cs.tanT2).applyMatrix3(i2, true);
			cs.binTU2.copy(cs.binT2).applyMatrix3(i2, true);

			tmpC1.crossVectors(cs.norTU1, tmpP1);
			tmpC2.crossVectors(cs.norTU2, tmpP2);
			tmp.add(tmpC1, tmpC2);
			cs.norDen = 1 / (m1m2 + _Math.dotVectors(cs.nor, tmp));

			tmpC1.crossVectors(cs.tanTU1, tmpP1);
			tmpC2.crossVectors(cs.tanTU2, tmpP2);
			tmp.add(tmpC1, tmpC2);
			cs.tanDen = 1 / (m1m2 + _Math.dotVectors(cs.tan, tmp));

			tmpC1.crossVectors(cs.binTU1, tmpP1);
			tmpC2.crossVectors(cs.binTU2, tmpP2);
			tmp.add(tmpC1, tmpC2);
			cs.binDen = 1 / (m1m2 + _Math.dotVectors(cs.bin, tmp));

			if (p.warmStarted) {
				norImp = p.normalImpulse;

				lv1.addScaledVector(cs.norU1, norImp);
				av1.addScaledVector(cs.norTU1, norImp);

				lv2.subScaledVector(cs.norU2, norImp);
				av2.subScaledVector(cs.norTU2, norImp);

				cs.norImp = norImp;
				cs.tanImp = 0;
				cs.binImp = 0;
				rvn = 0; // disable bouncing
			} else {
				cs.norImp = 0;
				cs.tanImp = 0;
				cs.binImp = 0;
			}


			if (rvn > -1) {
				rvn = 0;
			} // disable bouncing

			norTar = this.restitution * -rvn;
			sepV = -(p.penetration + 0.005) * invTimeStep * 0.05; // allow 0.5cm error
			if (norTar < sepV) {
				norTar = sepV;
			}
			cs.norTar = norTar;
			cs.last = i === this.num - 1;
			cs = cs.next;
		}
	}

	solve() {
		let {
			cs,
			friction,
			lv1,
			lv2,
			av1,
			av2,
			tmp,
			tmplv1,
			tmplv2,
			tmpav1,
			tmpav2
		} = this;
		tmplv1.copy(lv1);
		tmplv2.copy(lv2);
		tmpav1.copy(av1);
		tmpav2.copy(av2);

		let oldImp1,
			newImp1,
			oldImp2,
			newImp2,
			rvn,
			norImp,
			tanImp,
			binImp,
			max,
			len;

		while (true) {
			norImp = cs.norImp;
			tanImp = cs.tanImp;
			binImp = cs.binImp;
			max = -norImp * friction;

			tmp.sub(tmplv2, tmplv1);

			rvn = _Math.dotVectors(tmp, cs.tan) + _Math.dotVectors(tmpav2, cs.tanT2) - _Math.dotVectors(tmpav1, cs.tanT1);

			oldImp1 = tanImp;
			newImp1 = rvn * cs.tanDen;
			tanImp += newImp1;

			rvn = _Math.dotVectors(tmp, cs.bin) + _Math.dotVectors(tmpav2, cs.binT2) - _Math.dotVectors(tmpav1, cs.binT1);

			oldImp2 = binImp;
			newImp2 = rvn * cs.binDen;
			binImp += newImp2;

			// cone friction clamp
			len = tanImp * tanImp + binImp * binImp;
			if (len > max * max) {
				len = max / _Math.sqrt(len);
				tanImp *= len;
				binImp *= len;
			}

			newImp1 = tanImp - oldImp1;
			newImp2 = binImp - oldImp2;

			//

			tmp.set(
				cs.tanU1.x * newImp1 + cs.binU1.x * newImp2,
				cs.tanU1.y * newImp1 + cs.binU1.y * newImp2,
				cs.tanU1.z * newImp1 + cs.binU1.z * newImp2
			);

			tmplv1.addEqual(tmp);

			tmp.set(
				cs.tanTU1.x * newImp1 + cs.binTU1.x * newImp2,
				cs.tanTU1.y * newImp1 + cs.binTU1.y * newImp2,
				cs.tanTU1.z * newImp1 + cs.binTU1.z * newImp2
			);

			tmpav1.addEqual(tmp);

			tmp.set(
				cs.tanU2.x * newImp1 + cs.binU2.x * newImp2,
				cs.tanU2.y * newImp1 + cs.binU2.y * newImp2,
				cs.tanU2.z * newImp1 + cs.binU2.z * newImp2
			);

			tmplv2.subEqual(tmp);

			tmp.set(
				cs.tanTU2.x * newImp1 + cs.binTU2.x * newImp2,
				cs.tanTU2.y * newImp1 + cs.binTU2.y * newImp2,
				cs.tanTU2.z * newImp1 + cs.binTU2.z * newImp2
			);

			tmpav2.subEqual(tmp);

			// restitution part
			tmp.sub(tmplv2, tmplv1);

			rvn = _Math.dotVectors(tmp, cs.nor) + _Math.dotVectors(tmpav2, cs.norT2) - _Math.dotVectors(tmpav1, cs.norT1);

			oldImp1 = norImp;
			newImp1 = (rvn - cs.norTar) * cs.norDen;
			norImp += newImp1;
			if (norImp > 0) {
				norImp = 0;
			}

			newImp1 = norImp - oldImp1;

			tmplv1.addScaledVector(cs.norU1, newImp1);
			tmpav1.addScaledVector(cs.norTU1, newImp1);
			tmplv2.subScaledVector(cs.norU2, newImp1);
			tmpav2.subScaledVector(cs.norTU2, newImp1);

			cs.norImp = norImp;
			cs.tanImp = tanImp;
			cs.binImp = binImp;

			if (cs.last) {
				break;
			}
			cs = cs.next;
		}

		lv1.copy(tmplv1);
		lv2.copy(tmplv2);
		av1.copy(tmpav1);
		av2.copy(tmpav2);
	}

	postSolve() {
		let {
			cs,
			num,
			ps
		} = this;
		let p;

		while (num--) {
			p = ps[num];
			p.normal.copy(cs.nor);
			p.tangent.copy(cs.tan);
			p.binormal.copy(cs.bin);

			p.normalImpulse = cs.norImp;
			p.tangentImpulse = cs.tanImp;
			p.binormalImpulse = cs.binImp;
			p.normalDenominator = cs.norDen;
			p.tangentDenominator = cs.tanDen;
			p.binormalDenominator = cs.binDen;
			cs = cs.next;
		}
	}
}
