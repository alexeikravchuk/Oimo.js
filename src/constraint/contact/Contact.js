import { ContactLink } from './ContactLink';
import { ImpulseDataBuffer } from './ImpulseDataBuffer';
import { ContactManifold } from './ContactManifold';
import { ContactConstraint } from './ContactConstraint';
import { _Math } from '../../math/Math';

/**
 * A contact is a pair of shapes whose axis-aligned bounding boxes are overlapping.
 * @author saharan
 */

export class Contact {
	Contact = true;

	// The first shape.
	shape1 = null;
	// The second shape.
	shape2 = null;
	// The first rigid body.
	body1 = null;
	// The second rigid body.
	body2 = null;
	// The previous contact in the world.
	prev = null;
	// The next contact in the world.
	next = null;
	// Internal
	persisting = false;
	// Whether both the rigid bodies are sleeping or not.
	sleeping = false;
	// The collision detector between two shapes.
	detector = null;
	// The contact constraint of the contact.
	constraint = null;
	// Whether the shapes are touching or not.
	touching = false;
	// shapes is very close and touching
	close = false;
	dist = _Math.INF;

	constructor() {
		this.b1Link = new ContactLink(this);
		this.b2Link = new ContactLink(this);
		this.s1Link = new ContactLink(this);
		this.s2Link = new ContactLink(this);

		// The contact manifold of the contact.
		const manifold = this.manifold = new ContactManifold();

		this.buffer = [
			new ImpulseDataBuffer(),
			new ImpulseDataBuffer(),
			new ImpulseDataBuffer(),
			new ImpulseDataBuffer()
		];

		this.points = manifold.points;
		this.constraint = new ContactConstraint(manifold);
	}

	mixRestitution(restitution1, restitution2) {
		return _Math.sqrt(restitution1 * restitution2);
	}

	mixFriction(friction1, friction2) {
		return _Math.sqrt(friction1 * friction2);
	}

	/**
	 * Update the contact manifold.
	 */
	updateManifold() {
		const {
			constraint,
			detector,
			shape1,
			shape2,
			manifold,
			buffer,
			points
		} = this;

		constraint.restitution = this.mixRestitution(shape1.restitution, shape2.restitution);
		constraint.friction = this.mixFriction(shape1.friction, shape2.friction);
		let numBuffers = manifold.numPoints;
		let i = numBuffers;

		while (i--) {
			const b = buffer[i];
			const p = points[i];
			b.lp1X = p.localPoint1.x;
			b.lp1Y = p.localPoint1.y;
			b.lp1Z = p.localPoint1.z;
			b.lp2X = p.localPoint2.x;
			b.lp2Y = p.localPoint2.y;
			b.lp2Z = p.localPoint2.z;
			b.impulse = p.normalImpulse;
		}

		manifold.numPoints = 0;
		detector.detectCollision(shape1, shape2, manifold);
		const num = manifold.numPoints;

		if (num === 0) {
			this.touching = false;
			this.close = false;
			this.dist = _Math.INF;
			return;
		}

		if (this.touching || this.dist < 0.001) {
			this.close = true;
		}
		this.touching = true;
		i = num;

		while (i--) {
			const p = points[i];
			const {
				localPoint1: {
					x: lp1x,
					y: lp1y,
					z: lp1z
				},
				localPoint2: {
					x: lp2x,
					y: lp2y,
					z: lp2z
				}
			} = p;

			let index = -1;
			let minDistance = 0.0004;
			let j = numBuffers;

			while (j--) {
				const b = buffer[j];

				let dx = b.lp1X - lp1x;
				let dy = b.lp1Y - lp1y;
				let dz = b.lp1Z - lp1z;
				const distance1 = dx * dx + dy * dy + dz * dz;

				dx = b.lp2X - lp2x;
				dy = b.lp2Y - lp2y;
				dz = b.lp2Z - lp2z;
				const distance2 = dx * dx + dy * dy + dz * dz;

				if (distance1 < distance2) {
					if (distance1 < minDistance) {
						minDistance = distance1;
						index = j;
					}
				} else {
					if (distance2 < minDistance) {
						minDistance = distance2;
						index = j;
					}
				}

				if (minDistance < this.dist) {
					this.dist = minDistance;
				}

			}

			if (index !== -1) {
				const tmp = buffer[index];
				buffer[index] = buffer[--numBuffers];
				buffer[numBuffers] = tmp;
				p.normalImpulse = tmp.impulse;
				p.warmStarted = true;
			} else {
				p.normalImpulse = 0;
				p.warmStarted = false;
			}
		}
	}

	/**
	 * Attach the contact to the shapes.
	 * @param   shape1
	 * @param   shape2
	 */
	attach(shape1, shape2) {
		const {
			manifold,
			constraint,
			s1Link,
			s2Link,
			b1Link,
			b2Link
		} = this;

		this.shape1 = shape1;
		this.shape2 = shape2;
		const body1 = this.body1 = shape1.parent;
		const body2 = this.body2 = shape2.parent;

		manifold.body1 = body1;
		manifold.body2 = body2;
		constraint.body1 = body1;
		constraint.body2 = body2;
		constraint.attach();

		s1Link.shape = shape2;
		s1Link.body = body2;
		s2Link.shape = shape1;
		s2Link.body = body1;

		if (shape1.contactLink) {
			(s1Link.next = shape1.contactLink).prev = s1Link;
		} else {
			s1Link.next = null;
		}
		shape1.contactLink = s1Link;
		shape1.numContacts++;

		if (shape2.contactLink) {
			(s2Link.next = shape2.contactLink).prev = s2Link;
		} else {
			s2Link.next = null;
		}

		shape2.contactLink = s2Link;
		shape2.numContacts++;

		b1Link.shape = shape2;
		b1Link.body = body2;
		b2Link.shape = shape1;
		b2Link.body = body1;

		if (body1.contactLink != null) {
			(b1Link.next = body1.contactLink).prev = b1Link;
		} else {
			b1Link.next = null;
		}
		body1.contactLink = b1Link;
		body1.numContacts++;

		if (body2.contactLink != null) {
			(b2Link.next = body2.contactLink).prev = b2Link;
		} else {
			b2Link.next = null;
		}

		body2.contactLink = b2Link;
		body2.numContacts++;

		this.prev = null;
		this.next = null;

		this.persisting = true;
		this.sleeping = body1.sleeping && body2.sleeping;
		manifold.numPoints = 0;
	}

	/**
	 * Detach the contact from the shapes.
	 */
	detach() {
		const {
			s1Link,
			s2Link,
			b1Link,
			b2Link,
			shape1,
			shape2,
			body1,
			body2,
			manifold,
			constraint
		} = this;
		let {
			prev,
			next
		} = s1Link;

		if (prev !== null) {
			prev.next = next;
		}
		if (next !== null) {
			next.prev = prev;
		}
		if (shape1.contactLink === s1Link) {
			shape1.contactLink = next;
		}
		s1Link.prev = null;
		s1Link.next = null;
		s1Link.shape = null;
		s1Link.body = null;
		shape1.numContacts--;

		prev = s2Link.prev;
		next = s2Link.next;
		if (prev !== null) {
			prev.next = next;
		}
		if (next !== null) {
			next.prev = prev;
		}
		if (shape2.contactLink === s2Link) {
			shape2.contactLink = next;
		}
		s2Link.prev = null;
		s2Link.next = null;
		s2Link.shape = null;
		s2Link.body = null;
		shape2.numContacts--;

		prev = b1Link.prev;
		next = b1Link.next;
		if (prev !== null) {
			prev.next = next;
		}
		if (next !== null) {
			next.prev = prev;
		}
		if (body1.contactLink === b1Link) {
			body1.contactLink = next;
		}
		b1Link.prev = null;
		b1Link.next = null;
		b1Link.shape = null;
		b1Link.body = null;
		body1.numContacts--;

		prev = b2Link.prev;
		next = b2Link.next;
		if (prev !== null) {
			prev.next = next;
		}
		if (next !== null) {
			next.prev = prev;
		}
		if (body2.contactLink === b2Link) {
			body2.contactLink = next;
		}
		b2Link.prev = null;
		b2Link.next = null;
		b2Link.shape = null;
		b2Link.body = null;
		body2.numContacts--;

		manifold.body1 = null;
		manifold.body2 = null;
		constraint.body1 = null;
		constraint.body2 = null;
		constraint.detach();

		this.shape1 = null;
		this.shape2 = null;
		this.body1 = null;
		this.body2 = null;
	}
}
