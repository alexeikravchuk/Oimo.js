import {
	SHAPE_BOX,
	SHAPE_SPHERE,
	SHAPE_CYLINDER,
	SHAPE_PLANE,
	BODY_DYNAMIC,
	BODY_STATIC
} from '../constants';
import {
	InfoDisplay,
	printError
} from './Utils';


// import { BruteForceBroadPhase } from '../collision/broadphase/BruteForceBroadPhase';
import { SAPBroadPhase } from '../collision/broadphase/sap/SAPBroadPhase';
// import { DBVTBroadPhase } from '../collision/broadphase/dbvt/DBVTBroadPhase';

import { BoxBoxCollisionDetector } from '../collision/narrowphase/BoxBoxCollisionDetector';
import { BoxCylinderCollisionDetector } from '../collision/narrowphase/BoxCylinderCollisionDetector';
import { CylinderCylinderCollisionDetector } from '../collision/narrowphase/CylinderCylinderCollisionDetector';
import { SphereBoxCollisionDetector } from '../collision/narrowphase/SphereBoxCollisionDetector';
import { SphereCylinderCollisionDetector } from '../collision/narrowphase/SphereCylinderCollisionDetector';
import { SphereSphereCollisionDetector } from '../collision/narrowphase/SphereSphereCollisionDetector';
import { SpherePlaneCollisionDetector } from '../collision/narrowphase/SpherePlaneCollisionDetector_X';
import { BoxPlaneCollisionDetector } from '../collision/narrowphase/BoxPlaneCollisionDetector_X';

import { _Math } from '../math/Math';
// import { Mat33 } from '../math/Mat33';
import { Quat } from '../math/Quat';
import { Vec3 } from '../math/Vec3';

import { ShapeConfig } from '../shape/ShapeConfig';
import { Box } from '../shape/Box';
import { Sphere } from '../shape/Sphere';
import { Cylinder } from '../shape/Cylinder';
import { Plane } from '../shape/Plane';
//import { TetraShape } from '../collision/shape/TetraShape';

import { Contact } from '../constraint/contact/Contact';

import { JointConfig } from '../constraint/joint/JointConfig';
import { HingeJoint } from '../constraint/joint/HingeJoint';
import { BallAndSocketJoint } from '../constraint/joint/BallAndSocketJoint';
import { DistanceJoint } from '../constraint/joint/DistanceJoint';
import { PrismaticJoint } from '../constraint/joint/PrismaticJoint';
import { SliderJoint } from '../constraint/joint/SliderJoint';
import { WheelJoint } from '../constraint/joint/WheelJoint';

import { RigidBody } from './RigidBody';

/**
 * The class of physical computing world.
 * You must be added to the world physical all computing objects
 *
 * @author saharan
 * @author lo-th
 */

// const Btypes = ['None', 'BruteForce', 'Sweep & Prune', 'Bounding Volume Tree'];

export class World {
	World = true;
	timer = null;

	/**
	 *  @param {Function} preLoop
	 *  @param {Function} postLoop
	 */
	preLoop = null;
	postLoop = null;

	/**
	 * This is the detailed information of the performance.
	 * @param {InfoDisplay} performance
	 * @param {Boolean} isStat
	 */
	performance = null;
	isStat;

	// The rigid body list
	rigidBodies = null;
	// number of rigid body
	numRigidBodies = 0;
	// The contact list
	contacts = null;
	unusedContacts = null;
	// The number of contact
	numContacts = 0;
	// The number of contact points
	numContactPoints = 0;
	// The joint list
	joints = null;
	// The number of joints.
	numJoints = 0;
	// The number of simulation islands.
	numIslands = 0;

	/**
	 * @param {object} options
	 * @param {Number} [options.worldscale] this world scale default is 0.1 to 10 meters max for dynamic body
	 * @param {Number} [options.timestep] The time between each step
	 * @param {Number} [options.iterations] The number of iterations for constraint solvers.
	 * @param {SAPBroadPhase|DBVTBroadPhase|BruteForceBroadPhase} [options.broatphase] It is a wide-area collision judgment that is used in order to reduce as much as possible a detailed collision judgment.
	 * @param {Boolean} [options.info] This is the detailed information of the performance.
	 * @param {Boolean} [options.random] Whether the constraints randomizer is enabled or not.
	 * @param {Array} [options.gravity] The gravity in the world.
	 */

	constructor(options = {}) {
		const {
			worldscale = 1,
			timeStep = 0.01666, // 1/60;
			iterations = 16,
			broadphase = new SAPBroadPhase(),
			info = false,
			random = true,
			gravity = [0, -9.81, 0],
		} = options;

		this.scale = worldscale;
		this.invScale = 1 / worldscale;

		this.timeStep = timeStep;
		this.timerate = timeStep * 1000;

		this.numIterations = iterations;

		this.broadPhase = broadphase;
		// this.broadPhaseType = Btypes[2];

		this.isStat = info;
		info && (this.performance = new InfoDisplay(this));

		this.enableRandomizer = random;

		this.gravity = new Vec3().fromArray(gravity);

		const numShapeTypes = 5; //4;//3;
		this.detectors = [];
		this.detectors.length = numShapeTypes;
		let i = numShapeTypes;

		while (i--) {
			this.detectors[i] = [];
			this.detectors[i].length = numShapeTypes;
		}

		this.detectors[SHAPE_SPHERE][SHAPE_SPHERE] = new SphereSphereCollisionDetector();
		this.detectors[SHAPE_SPHERE][SHAPE_BOX] = new SphereBoxCollisionDetector(false);
		this.detectors[SHAPE_BOX][SHAPE_SPHERE] = new SphereBoxCollisionDetector(true);
		this.detectors[SHAPE_BOX][SHAPE_BOX] = new BoxBoxCollisionDetector();

		// CYLINDER add
		this.detectors[SHAPE_CYLINDER][SHAPE_CYLINDER] = new CylinderCylinderCollisionDetector();

		this.detectors[SHAPE_CYLINDER][SHAPE_BOX] = new BoxCylinderCollisionDetector(true);
		this.detectors[SHAPE_BOX][SHAPE_CYLINDER] = new BoxCylinderCollisionDetector(false);

		this.detectors[SHAPE_CYLINDER][SHAPE_SPHERE] = new SphereCylinderCollisionDetector(true);
		this.detectors[SHAPE_SPHERE][SHAPE_CYLINDER] = new SphereCylinderCollisionDetector(false);

		// PLANE add

		this.detectors[SHAPE_PLANE][SHAPE_SPHERE] = new SpherePlaneCollisionDetector(true);
		this.detectors[SHAPE_SPHERE][SHAPE_PLANE] = new SpherePlaneCollisionDetector(false);

		this.detectors[SHAPE_PLANE][SHAPE_BOX] = new BoxPlaneCollisionDetector(true);
		this.detectors[SHAPE_BOX][SHAPE_PLANE] = new BoxPlaneCollisionDetector(false);

		// TETRA add
		//this.detectors[SHAPE_TETRA][SHAPE_TETRA] = new TetraTetraCollisionDetector();

		this.randX = 65535;
		this.randA = 98765;
		this.randB = 123456789;

		this.islandRigidBodies = [];
		this.islandStack = [];
		this.islandConstraints = [];
	}

	play() {
		if (this.timer !== null) {
			return;
		}

		const _this = this;
		this.timer = setInterval(function () {
			_this.step();
		}, this.timerate);
		//this.timer = setInterval( this.loop.bind(this) , this.timerate );
	}

	stop() {
		if (this.timer === null) {
			return;
		}

		clearInterval(this.timer);
		this.timer = null;
	}

	setGravity(ar) {
		this.gravity.fromArray(ar);
	}

	getInfo() {

		return this.isStat ?
			   this.performance.show() :
			   '';

	}

	// Reset the world and remove all rigid bodies, shapes, joints and any object from the world.
	clear() {

		this.stop();
		this.preLoop = null;
		this.postLoop = null;

		this.randX = 65535;

		while (this.joints !== null) {
			this.removeJoint(this.joints);
		}
		while (this.contacts !== null) {
			this.removeContact(this.contacts);
		}
		while (this.rigidBodies !== null) {
			this.removeRigidBody(this.rigidBodies);
		}

	}

	/**
	 * I'll add a rigid body to the world.
	 * Rigid body that has been added will be the operands of each step.
	 * @param  rigidBody  Rigid body that you want to add
	 */
	addRigidBody(rigidBody) {

		if (rigidBody.parent) {
			printError('World', 'It is not possible to be added to more than one world one of the rigid body');
		}

		rigidBody.setParent(this);
		//rigidBody.awake();

		for (var shape = rigidBody.shapes; shape !== null; shape = shape.next) {
			this.addShape(shape);
		}
		if (this.rigidBodies !== null) {
			(this.rigidBodies.prev = rigidBody).next = this.rigidBodies;
		}
		this.rigidBodies = rigidBody;
		this.numRigidBodies++;

	}

	/**
	 * I will remove the rigid body from the world.
	 * Rigid body that has been deleted is excluded from the calculation on a step-by-step basis.
	 * @param  rigidBody  Rigid body to be removed
	 */
	removeRigidBody(rigidBody) {
		const remove = rigidBody;
		if (remove.parent !== this) {
			return;
		}
		remove.awake();
		let js = remove.jointLink;
		while (js != null) {
			const joint = js.joint;
			js = js.next;
			this.removeJoint(joint);
		}
		for (let shape = rigidBody.shapes; shape !== null; shape = shape.next) {
			this.removeShape(shape);
		}
		const prev = remove.prev;
		const next = remove.next;
		if (prev !== null) {
			prev.next = next;
		}
		if (next !== null) {
			next.prev = prev;
		}
		if (this.rigidBodies === remove) {
			this.rigidBodies = next;
		}
		remove.prev = null;
		remove.next = null;
		remove.parent = null;
		this.numRigidBodies--;

	}

	getByName(name) {
		let body = this.rigidBodies;
		while (body !== null) {
			if (body.name === name) {
				return body;
			}
			body = body.next;
		}

		let joint = this.joints;
		while (joint !== null) {
			if (joint.name === name) {
				return joint;
			}
			joint = joint.next;
		}

		return null;
	}

	/**
	 * I'll add a shape to the world..
	 * Add to the rigid world, and if you add a shape to a rigid body that has been added to the world,
	 * Shape will be added to the world automatically, please do not call from outside this method.
	 * @param  shape  Shape you want to add
	 */
	addShape(shape) {
		if (!shape.parent || !shape.parent.parent) {
			printError('World', 'It is not possible to be added alone to shape world');
		}

		shape.proxy = this.broadPhase.createProxy(shape);
		shape.updateProxy();
		this.broadPhase.addProxy(shape.proxy);
	}


	/**
	 * I will remove the shape from the world.
	 * Add to the rigid world, and if you add a shape to a rigid body that has been added to the world,
	 * Shape will be added to the world automatically, please do not call from outside this method.
	 * @param  shape  Shape you want to delete
	 */
	removeShape(shape) {

		this.broadPhase.removeProxy(shape.proxy);
		shape.proxy = null;
	}


	/**
	 * I'll add a joint to the world.
	 * Joint that has been added will be the operands of each step.
	 * @param  joint Joint to be added
	 */
	addJoint(joint) {

		if (joint.parent) {
			printError('World', 'It is not possible to be added to more than one world one of the joint');
		}
		if (this.joints != null) {
			(this.joints.prev = joint).next = this.joints;
		}
		this.joints = joint;
		joint.setParent(this);
		this.numJoints++;
		joint.awake();
		joint.attach();

	}


	/**
	 * I will remove the joint from the world.
	 * Joint that has been added will be the operands of each step.
	 * @param  joint Joint to be deleted
	 */
	removeJoint(joint) {
		const remove = joint;
		const prev = remove.prev;
		const next = remove.next;
		if (prev !== null) {
			prev.next = next;
		}
		if (next !== null) {
			next.prev = prev;
		}
		if (this.joints === remove) {
			this.joints = next;
		}
		remove.prev = null;
		remove.next = null;
		this.numJoints--;
		remove.awake();
		remove.detach();
		remove.parent = null;
	}


	addContact(s1, s2) {
		let newContact;
		if (this.unusedContacts !== null) {
			newContact = this.unusedContacts;
			this.unusedContacts = this.unusedContacts.next;
		} else {
			newContact = new Contact();
		}
		newContact.attach(s1, s2);
		newContact.detector = this.detectors[s1.type][s2.type];
		if (this.contacts) {
			(this.contacts.prev = newContact).next = this.contacts;
		}
		this.contacts = newContact;
		this.numContacts++;
	}


	removeContact(contact) {
		const prev = contact.prev;
		const next = contact.next;
		if (next) {
			next.prev = prev;
		}
		if (prev) {
			prev.next = next;
		}
		if (this.contacts === contact) {
			this.contacts = next;
		}
		contact.prev = null;
		contact.next = null;
		contact.detach();
		contact.next = this.unusedContacts;
		this.unusedContacts = contact;
		this.numContacts--;
	}


	getContact(b1, b2) {

		b1 = b1.constructor === RigidBody ?
			 b1.name :
			 b1;
		b2 = b2.constructor === RigidBody ?
			 b2.name :
			 b2;

		let n1,
			n2;
		let contact = this.contacts;
		while (contact !== null) {
			n1 = contact.body1.name;
			n2 = contact.body2.name;
			if ((n1 === b1 && n2 === b2) || (n2 === b1 && n1 === b2)) {
				if (contact.touching) {
					return contact;
				} else {
					return null;
				}
			} else {
				contact = contact.next;
			}
		}
		return null;

	}


	checkContact(name1, name2) {
		let n1,	n2;
		let contact = this.contacts;
		while (contact !== null) {
			n1 = contact.body1.name || ' ';
			n2 = contact.body2.name || ' ';
			if ((n1 === name1 && n2 === name2) || (n2 === name1 && n1 === name2)) {
				return !!contact.touching;
			} else {
				contact = contact.next;
			}
		}
		//return false;
	}


	callSleep(body) {
		if (!body.allowSleep) {
			return false;
		}
		if (body.linearVelocity.lengthSq() > 0.04) {
			return false;
		}
		return body.angularVelocity.lengthSq() <= 0.25 ;
	}


	/**
	 * I will proceed only time step seconds time of World.
	 */
	step() {

		let next;
		const stat = this.isStat;
		stat && this.performance.setTime(0);

		let body = this.rigidBodies;

		while (body !== null) {
			body.addedToIsland = false;
			body.sleeping && body.testWakeUp();
			body = body.next;
		}

		//------------------------------------------------------
		//   UPDATE BROADPHASE CONTACT
		//------------------------------------------------------

		stat && this.performance.setTime(1);

		this.broadPhase.detectPairs();

		const {
			pairs,
			numPairs
		} = this.broadPhase;

		let contact;
		let i = numPairs;

		//do{
		while (i--) {
			//for(var i=0, l=numPairs; i<l; i++){
			const pair = pairs[i];
			let s1;
			let s2;
			if (pair.shape1.id < pair.shape2.id) {
				s1 = pair.shape1;
				s2 = pair.shape2;
			} else {
				s1 = pair.shape2;
				s2 = pair.shape1;
			}

			let link;
			if (s1.numContacts < s2.numContacts) {
				link = s1.contactLink;
			} else {
				link = s2.contactLink;
			}

			let exists = false;
			while (link) {
				contact = link.contact;
				if (contact.shape1 === s1 && contact.shape2 === s2) {
					contact.persisting = exists = true; // contact already exists
					break;
				}
				link = link.next;
			}

			!exists && this.addContact(s1, s2);
		}// while(i-- >0);

		stat && this.performance.calcBroadPhase();

		//------------------------------------------------------
		//   UPDATE NARROWPHASE CONTACT
		//------------------------------------------------------

		// update & narrow phase
		this.numContactPoints = 0;
		contact = this.contacts;
		while (contact !== null) {
			if (!contact.persisting) {
				if (contact.shape1.aabb.intersectTest(contact.shape2.aabb)) {
					/*var aabb1=contact.shape1.aabb;
					 var aabb2=contact.shape2.aabb;
					 if(
					 aabb1.minX>aabb2.maxX || aabb1.maxX<aabb2.minX ||
					 aabb1.minY>aabb2.maxY || aabb1.maxY<aabb2.minY ||
					 aabb1.minZ>aabb2.maxZ || aabb1.maxZ<aabb2.minZ
					 ){*/

					const next = contact.next;
					this.removeContact(contact);
					contact = next;
					continue;
				}
			}
			const b1 = contact.body1;
			const b2 = contact.body2;

			if (b1.isDynamic && !b1.sleeping || b2.isDynamic && !b2.sleeping) {
				contact.updateManifold();
			}

			this.numContactPoints += contact.manifold.numPoints;
			contact.persisting = false;
			contact.constraint.addedToIsland = false;
			contact = contact.next;

		}

		stat && this.performance.calcNarrowPhase();

		//------------------------------------------------------
		//   SOLVE ISLANDS
		//------------------------------------------------------
		const invTimeStep = 1 / this.timeStep;
		let joint;
		let constraint;

		for (joint = this.joints; joint !== null; joint = joint.next) {
			joint.addedToIsland = false;
		}

		// clear old island array
		this.islandRigidBodies = [];
		this.islandConstraints = [];
		this.islandStack = [];

		stat && this.performance.setTime(1);

		this.numIslands = 0;

		// build and solve simulation islands
		for (let base = this.rigidBodies; base !== null; base = base.next) {

			if (base.addedToIsland || base.isStatic || base.sleeping) {
				continue;
			}// ignore

			if (base.isLonely()) {// update single body
				if (base.isDynamic) {
					const gravity = base.gravity || this.gravity;
					base.linearVelocity.addScaledVector(gravity, this.timeStep);
					/*base.linearVelocity.x+=this.gravity.x*this.timeStep;
					 base.linearVelocity.y+=this.gravity.y*this.timeStep;
					 base.linearVelocity.z+=this.gravity.z*this.timeStep;*/
				}
				if (this.callSleep(base)) {
					base.sleepTime += this.timeStep;
					base.sleepTime > 0.5 ? base.sleep() : base.updatePosition(this.timeStep);
				} else {
					base.sleepTime = 0;
					base.updatePosition(this.timeStep);
				}
				this.numIslands++;
				continue;
			}

			let islandNumRigidBodies = 0;
			let islandNumConstraints = 0;
			let stackCount = 1;
			// add rigid body to stack
			this.islandStack[0] = base;
			base.addedToIsland = true;

			// build an island
			do {
				// get rigid body from stack
				body = this.islandStack[--stackCount];
				this.islandStack[stackCount] = null;
				body.sleeping = false;
				// add rigid body to the island
				this.islandRigidBodies[islandNumRigidBodies++] = body;
				if (body.isStatic) {
					continue;
				}

				// search connections
				for (let cs = body.contactLink; cs !== null; cs = cs.next) {
					contact = cs.contact;
					constraint = contact.constraint;
					if (constraint.addedToIsland || !contact.touching) {
						continue;
					}// ignore

					// add constraint to the island
					this.islandConstraints[islandNumConstraints++] = constraint;
					constraint.addedToIsland = true;
					next = cs.body;

					if (next.addedToIsland) {
						continue;
					}

					// add rigid body to stack
					this.islandStack[stackCount++] = next;
					next.addedToIsland = true;
				}
				for (let js = body.jointLink; js !== null; js = js.next) {
					constraint = js.joint;

					if (constraint.addedToIsland) {
						continue;
					}// ignore

					// add constraint to the island
					this.islandConstraints[islandNumConstraints++] = constraint;
					constraint.addedToIsland = true;
					next = js.body;
					if (next.addedToIsland || !next.isDynamic) {
						continue;
					}

					// add rigid body to stack
					this.islandStack[stackCount++] = next;
					next.addedToIsland = true;
				}
			} while (stackCount);

			// update velocities
			let gVel = new Vec3().addScaledVector(this.gravity, this.timeStep);
			/*var gx=this.gravity.x*this.timeStep;
			 var gy=this.gravity.y*this.timeStep;
			 var gz=this.gravity.z*this.timeStep;*/

			let j = islandNumRigidBodies;

			while (j--) {
				//or(var j=0, l=islandNumRigidBodies; j<l; j++){
				body = this.islandRigidBodies[j];

				if (body.useGravity && body.isDynamic) {
					body.linearVelocity.addEqual(gVel);
					/*body.linearVelocity.x+=gx;
					 body.linearVelocity.y+=gy;
					 body.linearVelocity.z+=gz;*/
				}
			}

			// randomizing order
			if (this.enableRandomizer) {
				//for(var j=1, l=islandNumConstraints; j<l; j++){
				j = islandNumConstraints;
				while (j--) {
					if (j !== 0) {
						const swap = (this.randX = (this.randX * this.randA + this.randB & 0x7fffffff)) / 2147483648.0 * j | 0;
						constraint = this.islandConstraints[j];
						this.islandConstraints[j] = this.islandConstraints[swap];
						this.islandConstraints[swap] = constraint;
					}
				}
			}

			// solve constraints

			j = islandNumConstraints;
			while (j--) {
				//for(j=0, l=islandNumConstraints; j<l; j++){
				this.islandConstraints[j].preSolve(this.timeStep, invTimeStep);// pre-solve
			}
			let k = this.numIterations;
			while (k--) {
				//for(var k=0, l=this.numIterations; k<l; k++){
				j = islandNumConstraints;
				while (j--) {
					//for(j=0, m=islandNumConstraints; j<m; j++){
					this.islandConstraints[j].solve();// main-solve
				}
			}
			j = islandNumConstraints;
			while (j--) {
				//for(j=0, l=islandNumConstraints; j<l; j++){
				this.islandConstraints[j].postSolve();// post-solve
				this.islandConstraints[j] = null;// gc
			}

			// sleeping check

			let sleepTime = 10;
			j = islandNumRigidBodies;
			while (j--) {
				//for(j=0, l=islandNumRigidBodies;j<l;j++){
				body = this.islandRigidBodies[j];
				if (this.callSleep(body)) {
					body.sleepTime += this.timeStep;
					if (body.sleepTime < sleepTime) {
						sleepTime = body.sleepTime;
					}
				} else {
					body.sleepTime = 0;
					sleepTime = 0;
				}
			}
			if (sleepTime > 0.5) {
				// sleep the island
				j = islandNumRigidBodies;
				while (j--) {
					//for(j=0, l=islandNumRigidBodies;j<l;j++){
					this.islandRigidBodies[j].sleep();
					this.islandRigidBodies[j] = null;// gc
				}
			} else {
				// update positions
				j = islandNumRigidBodies;
				while (j--) {
					//for(j=0, l=islandNumRigidBodies;j<l;j++){
					this.islandRigidBodies[j].updatePosition(this.timeStep);
					this.islandRigidBodies[j] = null;// gc
				}
			}
			this.numIslands++;
		}

		//------------------------------------------------------
		//   END SIMULATION
		//------------------------------------------------------
		stat && this.performance.calcEnd();
		this.postLoop && this.postLoop();
	}


	// remove something to world

	remove(obj) {

	}

	// add something to world
	add(options = {}) {

		let {
			type = 'box',
		} = options;

		if (typeof type === 'string') {
			type = [type];
		}

		const isJoint = type[0].substring(0, 5) === 'joint';

		return isJoint ? this.initJoint(type[0], options) : this.initBody(type, options);
	}


	initBody(type, options) {
		const {
			move = false,
			kinematic = false,
			pos = [0, 0, 0],
			posShape = [0, 0, 0],
			rot = [0, 0, 0],
			rotShape = [0, 0, 0],
			size = [1, 1, 1],
			density = 1,
			friction = 0.2,
			restitution = 0.2,
			belongsTo = 1,
			collidesWith = 0xffffffff,
			name = '',
			gravity,
			sleep,
			neverSleep,
			massPos,
			massRot,
		} = options;

		const invScale = this.invScale;

		// POSITION
		// body position
		const p = pos.map((x) => x * invScale);
		// shape position
		const p2 = posShape.map((x) => x * invScale);

		// ROTATION
		// body rotation in degree
		const r = rot.map((x) => x * _Math.degtorad);
		// shape rotation in degree
		const r2 = rotShape.map((x) => x * _Math.degtorad);

		// SIZE
		// shape size
		let s = Array.isArray(size) ? size : [size];
		s.length === 1 && (s[1] = s[0]);
		s.length === 2 && (s[2] = s[0]);
		s = s.map((x) => x * invScale);


		// body physics settings
		const sc = new ShapeConfig();

		// The density of the shape.
		sc.density = density;

		// The coefficient of friction of the shape.
		sc.friction = friction;

		// The coefficient of restitution of the shape.
		sc.restitution = restitution;

		// The bits of the collision groups to which the shape belongs.
		sc.belongsTo = belongsTo;

		// The bits of the collision groups with which the shape collides.
		sc.collidesWith = collidesWith;

		/* if(massPos){
		 massPos = massPos.map(function(x) { return x * invScale; });
		 sc.relativePosition.set( massPos[0], massPos[1], massPos[2] );
		 }
		 if(massRot){
		 massRot = massRot.map(function(x) { return x * _Math.degtorad; });
		 var q = new Quat().setFromEuler( massRot[0], massRot[1], massRot[2] );
		 sc.relativeRotation = new Mat33().setQuat( q );//_Math.EulerToMatrix( massRot[0], massRot[1], massRot[2] );
		 }*/

		const position = new Vec3(...p);
		const rotation = new Quat().setFromEuler(...r);

		// rigidbody
		const body = new RigidBody(position, rotation);
		body.gravity = gravity && new Vec3(...gravity);
		//var body = new RigidBody( p[0], p[1], p[2], r[0], r[1], r[2], r[3], this.scale, this.invScale );

		// SHAPES
		for (let i = 0; i < type.length; i++) {
			const n = i * 3;
			let shape;

			p2[n] !== undefined && sc.relativePosition.set(p2[n], p2[n + 1], p2[n + 2]);
			r2[n] !== undefined && sc.relativeRotation.setQuat(new Quat().setFromEuler(r2[n], r2[n + 1], r2[n + 2]));

			switch (type[i]) {
				case 'sphere':
					shape = new Sphere(sc, s[n]);
					break;
				case 'cylinder':
					shape = new Cylinder(sc, s[n], s[n + 1]);
					break;
				case 'box':
					shape = new Box(sc, s[n], s[n + 1], s[n + 2]);
					break;
				case 'plane':
					shape = new Plane(sc);
					break;
			}

			body.addShape(shape);
		}

		// body can sleep or not
		body.allowSleep = !(neverSleep || kinematic);

		body.isKinematic = kinematic;

		// body static or dynamic
		if (move) {
			body.setupMass(BODY_DYNAMIC, !!(massPos || massRot));
			// body can sleep or not
			// if(neverSleep) body.allowSleep = false;
			// else body.allowSleep = true;
		} else {
			body.setupMass(BODY_STATIC);
		}

		body.name = name;

		//else if( move ) body.name = this.numRigidBodies;

		// finally add to physics world
		this.addRigidBody(body);

		// force sleep on not
		move && (sleep ? body.sleep() : body.awake());

		return body;
	}


	initJoint(type, options) {
		let {
			axe1 = [1, 0, 0],
			axe2 = [1, 0, 0],
			pos1 = [0, 0, 0],
			pos2 = [0, 0, 0],
			min,
			max,
			limit = null,
			spring = null,
			motor = null,
			collision = false,
			body1,
			body2,
			name = '',
		} = options;

		const {
			invScale,
			scale
		} = this;

		pos1 = pos1.map((x) => x * invScale);
		pos2 = pos2.map((x) => x * invScale);

		if (type === 'jointDistance') {
			min = (min || 0) * invScale;
			max = (max || 10) * invScale;
		} else {
			min = (min || 57.29578) * _Math.degtorad;
			max = (max || 0) * _Math.degtorad;
		}

		// joint setting
		const jc = new JointConfig();
		jc.scale = scale;
		jc.invScale = invScale;
		jc.allowCollision = collision;
		jc.localAxis1.set(...axe1);
		jc.localAxis2.set(...axe2);
		jc.localAnchorPoint1.set(...pos1);
		jc.localAnchorPoint2.set(...pos2);

		if (!body1 || !body2) {
			return printError('World', 'Can\'t add joint if attach rigidbodys not define !');
		}

		const b1 = body1 instanceof RigidBody ? body1 : this.getByName(body1);
		const b2 = body2 instanceof RigidBody ? body2 : this.getByName(body2);

		if (!b1 || !b2) {
			return printError('World', 'Can\'t add joint attach rigidbodys not find !');
		}

		jc.body1 = b1;
		jc.body2 = b2;

		let joint;
		switch (type) {
			case 'jointDistance':
				joint = new DistanceJoint(jc, min, max);
				spring !== null && joint.limitMotor.setSpring(spring[0], spring[1]);
				motor !== null && joint.limitMotor.setMotor(motor[0], motor[1]);
				break;
			case 'jointHinge':
			case 'joint':
				joint = new HingeJoint(jc, min, max);
				spring !== null && joint.limitMotor.setSpring(spring[0], spring[1]);
				// soften the joint ex: 100, 0.2
				motor !== null && joint.limitMotor.setMotor(motor[0], motor[1]);
				break;
			case 'jointPrismatic':
				joint = new PrismaticJoint(jc, min, max);
				break;
			case 'jointSlide':
				joint = new SliderJoint(jc, min, max);
				break;
			case 'jointBall':
				joint = new BallAndSocketJoint(jc);
				break;
			case 'jointWheel':
				joint = new WheelJoint(jc);
				limit !== null && joint.rotationalLimitMotor1.setLimit(limit[0], limit[1]);
				spring !== null && joint.rotationalLimitMotor1.setSpring(spring[0], spring[1]);
				motor !== null && joint.rotationalLimitMotor1.setMotor(motor[0], motor[1]);
				break;
		}

		joint.name = name;

		// finally add to physics world
		this.addJoint(joint);

		return joint;
	}
}
