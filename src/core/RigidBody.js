import {
	BODY_NULL,
	BODY_DYNAMIC,
	BODY_STATIC
} from '../constants';
import { printError } from './Utils';

import { MassInfo } from '../shape/MassInfo';

import { _Math } from '../math/Math';
import { Mat33 } from '../math/Mat33';
import { Quat } from '../math/Quat';
import { Vec3 } from '../math/Vec3';

/**
 * The class of rigid body.
 * Rigid body has the shape of a single or multiple collision processing,
 * I can set the parameters individually.
 * @author saharan
 * @author lo-th
 */

export class RigidBody {
	scale = 1;
	invScale = 1;

	// possible link to three Mesh;
	mesh = null;

	id = NaN;
	name = '';
	// The maximum number of shapes that can be added to a one rigid.
	// this.MAX_SHAPES = 64; //64;

	prev = null;
	next = null;

	// I represent the kind of rigid body.
	// Please do not change from the outside this variable.
	// If you want to change the type of rigid body, always
	// Please specify the type you want to set the arguments of setupMass method.
	type = BODY_NULL;

	massInfo = new MassInfo();

	newPosition = new Vec3();
	controlPos = false;
	newOrientation = new Quat();
	newRotation = new Vec3();
	currentRotation = new Vec3();
	controlRot = false;
	controlRotInTime = false;

	quaternion = new Quat();
	pos = new Vec3();


	// Is the translational velocity.
	linearVelocity = new Vec3();
	// Is the angular velocity.
	angularVelocity = new Vec3();

	//--------------------------------------------
	//  Please do not change from the outside this variables.
	//--------------------------------------------

	// It is a world that rigid body has been added.
	parent = null;
	contactLink = null;
	numContacts = 0;

	// An array of shapes that are included in the rigid body.
	shapes = null;
	// The number of shapes that are included in the rigid body.
	numShapes = 0;

	// It is the link array of joint that is connected to the rigid body.
	jointLink = null;
	// The number of joints that are connected to the rigid body.
	numJoints = 0;

	// It is the world coordinate of the center of gravity in the sleep just before.
	sleepPosition = new Vec3();
	// It is a quaternion that represents the attitude of sleep just before.
	sleepOrientation = new Quat();
	// I will show this rigid body to determine whether it is a rigid body static.
	isStatic = false;
	// I indicates that this rigid body to determine whether it is a rigid body dynamic.
	isDynamic = false;

	isKinematic = false;

	// It is a rotation matrix representing the orientation.
	rotation = new Mat33();

	//--------------------------------------------
	// It will be recalculated automatically from the shape, which is included.
	//--------------------------------------------

	// This is the weight.
	mass = 0;
	// It is the reciprocal of the mass.
	inverseMass = 0;
	// It is the inverse of the inertia tensor in the world system.
	inverseInertia = new Mat33();
	// It is the inertia tensor in the initial state.
	localInertia = new Mat33();
	// It is the inverse of the inertia tensor in the initial state.
	inverseLocalInertia = new Mat33();
	tmpInertia = new Mat33();

	// I indicates rigid body whether it has been added to the simulation Island.
	addedToIsland = false;
	// It shows how to sleep rigid body.
	allowSleep = true;
	// This is the time from when the rigid body at rest.
	sleepTime = 0;
	// I shows rigid body to determine whether it is a sleep state.
	sleeping = false;

	/**
	 *
	 * @param {Vec3} position
	 * @param {Quat} rotation
	 */
	constructor(position, rotation) {
		this.position = position || new Vec3();
		this.orientation = rotation || new Quat();
	}

	setParent(world) {
		this.parent = world;
		this.scale = this.parent.scale;
		this.invScale = this.parent.invScale;
		this.id = this.parent.numRigidBodies;
		if (!this.name) {
			this.name = this.id;
		}

		this.updateMesh();
	}

	/**
	 * I'll add a shape to rigid body.
	 * If you add a shape, please call the setupMass method to step up to the start of the next.
	 * @param {Box|Sphere|Cylinder|Plane} shape shape to Add
	 * @return void
	 */
	addShape(shape) {
		if (shape.parent) {
			printError('RigidBody', 'It is not possible that you add a shape which already has an associated body.');
		}

		if (this.shapes != null) {
			(this.shapes.prev = shape).next = this.shapes;
		}
		this.shapes = shape;
		shape.parent = this;
		if (this.parent) {
			this.parent.addShape(shape);
		}
		this.numShapes++;
	}

	/**
	 * I will delete the shape from the rigid body.
	 * If you delete a shape, please call the setupMass method to step up to the start of the next.
	 * @param {Box|Sphere|Cylinder|Plane} shape to delete
	 * @return void
	 */
	removeShape(shape) {
		if (shape.parent !== this) {
			return;
		}
		const prev = shape.prev;
		const next = shape.next;
		if (prev !== null) {
			prev.next = next;
		}
		if (next !== null) {
			next.prev = prev;
		}
		if (this.shapes === shape) {
			this.shapes = next;
		}
		shape.prev = null;
		shape.next = null;
		shape.parent = null;
		if (this.parent) {
			this.parent.removeShape(shape);
		}
		this.numShapes--;
	}

	remove() {
		this.dispose();
	}

	dispose() {
		this.parent.removeRigidBody(this);
	}

	checkContact(name) {
		this.parent.checkContact(this.name, name);
	}

	/**
	 * Calculates mass data (center of gravity, mass, moment inertia, etc...).
	 * If the parameter type is set to BODY_STATIC, the rigid body will be fixed to the space.
	 * If the parameter adjustPosition is set to true, the shapes' relative positions and
	 * the rigid body's position will be adjusted to the center of gravity.
	 * @param {number} type
	 * @param {boolean} adjustPosition
	 * @return void
	 */
	setupMass(type = BODY_STATIC, adjustPosition = true) {
		this.type = type;
		this.isDynamic = type === BODY_DYNAMIC;
		this.isStatic = type === BODY_STATIC;

		this.mass = 0;
		this.localInertia.set(0, 0, 0, 0, 0, 0, 0, 0, 0);

		const tmpM = new Mat33();
		const tmpV = new Vec3();

		for (let shape = this.shapes; shape !== null; shape = shape.next) {
			shape.calculateMassInfo(this.massInfo);
			const shapeMass = this.massInfo.mass;
			tmpV.addScaledVector(shape.relativePosition, shapeMass);
			this.mass += shapeMass;
			this.rotateInertia(shape.relativeRotation, this.massInfo.inertia, tmpM);
			this.localInertia.add(tmpM);

			// add offset inertia
			this.localInertia.addOffset(shapeMass, shape.relativePosition);

		}

		this.inverseMass = 1 / this.mass;
		tmpV.scaleEqual(this.inverseMass);

		if (adjustPosition) {
			this.position.add(tmpV);
			for (let shape = this.shapes; shape !== null; shape = shape.next) {
				shape.relativePosition.subEqual(tmpV);
			}

			// subtract offset inertia
			this.localInertia.subOffset(this.mass, tmpV);
		}

		this.inverseLocalInertia.invert(this.localInertia);

		//}

		if (this.type === BODY_STATIC) {
			this.inverseMass = 0;
			this.inverseLocalInertia.set(0, 0, 0, 0, 0, 0, 0, 0, 0);
		}

		this.syncShapes();
		this.awake();
	}

	/**
	 * Awake the rigid body.
	 */
	awake() {
		if (this.allowSleep && this.sleeping) {
			this.sleeping = false;
			this.sleepTime = 0;

			// awake connected constraints
			let cs = this.contactLink;
			while (cs !== null) {
				cs.body.sleepTime = 0;
				cs.body.sleeping = false;
				cs = cs.next;
			}

			let js = this.jointLink;
			while (js !== null) {
				js.body.sleepTime = 0;
				js.body.sleeping = false;
				js = js.next;
			}

			for (let shape = this.shapes; shape !== null; shape = shape.next) {
				shape.updateProxy();
			}
		}
	}

	/**
	 * Sleep the rigid body.
	 */
	sleep() {
		if (this.allowSleep && !this.sleeping) {
			this.linearVelocity.set(0, 0, 0);
			this.angularVelocity.set(0, 0, 0);
			this.sleepPosition.copy(this.position);
			this.sleepOrientation.copy(this.orientation);

			this.sleepTime = 0;
			this.sleeping = true;
			for (let shape = this.shapes; shape != null; shape = shape.next) {
				shape.updateProxy();
			}
		}
	}

	testWakeUp() {
		if (this.linearVelocity.testZero() || this.angularVelocity.testZero() || this.position.testDiff(this.sleepPosition) || this.orientation.testDiff(this.sleepOrientation)) {
			this.awake();
		} // awake the body
	}

	/**
	 * Get whether the rigid body has not any connection with others.
	 * @return {boolean}
	 */
	isLonely() {
		return this.numJoints === 0 && this.numContacts === 0;
	}

	/**
	 * The time integration of the motion of a rigid body, you can update the information such as the shape.
	 * This method is invoked automatically when calling the step of the World,
	 * There is no need to call from outside usually.
	 * @param  timeStep time
	 * @return {void}
	 */
	updatePosition(timeStep) {
		switch (this.type) {
			case BODY_STATIC:
				this.linearVelocity.set(0, 0, 0);
				this.angularVelocity.set(0, 0, 0);

				// ONLY FOR TEST
				if (this.controlPos) {
					this.position.copy(this.newPosition);
					this.controlPos = false;
				}
				if (this.controlRot) {
					this.orientation.copy(this.newOrientation);
					this.controlRot = false;
				}
				break;
			case BODY_DYNAMIC:
				if (this.isKinematic) {
					this.linearVelocity.set(0, 0, 0);
					this.angularVelocity.set(0, 0, 0);
				}

				if (this.controlPos) {
					this.linearVelocity.subVectors(this.newPosition, this.position).multiplyScalar(1 / timeStep);
					this.controlPos = false;
				}

				if (this.controlRot) {
					this.angularVelocity.copy(this.getAxis());
					this.orientation.copy(this.newOrientation);
					this.controlRot = false;
				}

				this.position.addScaledVector(this.linearVelocity, timeStep);
				this.orientation.addTime(this.angularVelocity, timeStep);

				this.updateMesh();
				break;
			default:
				printError('RigidBody', 'Invalid type.');
		}

		this.syncShapes();
		this.updateMesh();
	}

	getAxis() {
		return new Vec3(0, 1, 0).applyMatrix3(this.inverseLocalInertia, true).normalize();
	}

	rotateInertia(rot, inertia, out) {
		this.tmpInertia.multiplyMatrices(rot, inertia);
		out.multiplyMatrices(this.tmpInertia, rot, true);
	}

	syncShapes() {
		this.rotation.setQuat(this.orientation);
		this.rotateInertia(this.rotation, this.inverseLocalInertia, this.inverseInertia);

		for (let shape = this.shapes; shape != null; shape = shape.next) {
			shape.position.copy(shape.relativePosition).applyMatrix3(this.rotation, true).add(this.position);
			shape.rotation.multiplyMatrices(this.rotation, shape.relativeRotation);
			shape.updateProxy();
		}
	}


	//---------------------------------------------
	// APPLY IMPULSE FORCE
	//---------------------------------------------
	applyImpulse(position, force) {
		this.linearVelocity.addScaledVector(force, this.inverseMass);
		const rel = new Vec3().copy(position).sub(this.position).cross(force).applyMatrix3(this.inverseInertia, true);
		this.angularVelocity.add(rel);
	}


	//---------------------------------------------
	// SET DYNAMIC POSITION AND ROTATION
	//---------------------------------------------
	setPosition(pos) {
		this.newPosition.copy(pos).multiplyScalar(this.invScale);
		this.controlPos = true;
		if (!this.isKinematic) {
			this.isKinematic = true;
		}
	}

	setQuaternion(q) {
		this.newOrientation.set(q.x, q.y, q.z, q.w);
		this.controlRot = true;
		if (!this.isKinematic) {
			this.isKinematic = true;
		}
	}

	setRotation(rot) {
		this.newOrientation = new Quat().setFromEuler(rot.x * _Math.degtorad, rot.y * _Math.degtorad, rot.z * _Math.degtorad);//this.rotationVectToQuad( rot );
		this.controlRot = true;
	}

	//---------------------------------------------
	// RESET DYNAMIC POSITION AND ROTATION
	//---------------------------------------------
	resetPosition(x, y, z) {
		this.linearVelocity.set(0, 0, 0);
		this.angularVelocity.set(0, 0, 0);
		this.position.set(x, y, z).multiplyScalar(this.invScale);
		//this.position.set( x*OIMO.WorldScale.invScale, y*OIMO.WorldScale.invScale, z*OIMO.WorldScale.invScale );
		this.awake();
	}

	resetQuaternion(q) {
		this.angularVelocity.set(0, 0, 0);
		this.orientation = new Quat(q.x, q.y, q.z, q.w);
		this.awake();
	}

	resetRotation(x, y, z) {
		this.angularVelocity.set(0, 0, 0);
		this.orientation = new Quat().setFromEuler(x * _Math.degtorad, y * _Math.degtorad, z * _Math.degtorad);//this.rotationVectToQuad( new Vec3(x,y,z) );
		this.awake();
	}

	//---------------------------------------------
	// GET POSITION AND ROTATION
	//---------------------------------------------
	getPosition() {
		return this.pos;
	}

	getQuaternion() {
		return this.quaternion;
	}

	//---------------------------------------------
	// AUTO UPDATE THREE MESH
	//---------------------------------------------
	connectMesh(mesh) {
		this.mesh = mesh;
		this.updateMesh();
	}

	updateMesh() {
		this.pos.scale(this.position, this.scale);
		this.quaternion.copy(this.orientation);

		if (this.mesh) {
			this.mesh.position.copy(this.getPosition());
			this.mesh.quaternion.copy(this.getQuaternion());
		}
	}
}
