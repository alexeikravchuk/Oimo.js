import { _Math } from '../math/Math';
import { REVISION } from '../constants';


export function printError(clazz, msg) {
	console.error('[OIMO] ' + clazz + ': ' + msg);
}

// A performance evaluator
export class InfoDisplay {
	fps = 0;
	tt = 0;

	broadPhaseTime = 0;
	narrowPhaseTime = 0;
	solvingTime = 0;
	totalTime = 0;
	updateTime = 0;

	MaxBroadPhaseTime = 0;
	MaxNarrowPhaseTime = 0;
	MaxSolvingTime = 0;
	MaxTotalTime = 0;
	MaxUpdateTime = 0;

	infos = new Float32Array(13);
	f = [0, 0, 0];
	times = [0, 0, 0, 0];
	version = REVISION;

	constructor(world) {
		this.parent = world;
		this.broadPhase = this.parent.broadPhaseType;
	}

	setTime(n) {
		this.times[n || 0] = performance.now();
	}

	resetMax() {
		this.MaxBroadPhaseTime = 0;
		this.MaxNarrowPhaseTime = 0;
		this.MaxSolvingTime = 0;
		this.MaxTotalTime = 0;
		this.MaxUpdateTime = 0;
	}

	calcBroadPhase() {
		this.setTime(2);
		this.broadPhaseTime = this.times[2] - this.times[1];
	}

	calcNarrowPhase() {
		this.setTime(3);
		this.narrowPhaseTime = this.times[3] - this.times[2];
	}

	calcEnd() {
		const {
			times,
			broadPhaseTime,
			narrowPhaseTime,
			MaxBroadPhaseTime,
			MaxNarrowPhaseTime,
			MaxSolvingTime,
			MaxTotalTime,
			MaxUpdateTime
		} = this;

		const [t0, t1, t2] = times;

		this.setTime(2);
		const solvingTime = this.solvingTime = t2 - t1;
		const totalTime = this.totalTime = t2 - t0;
		const updateTime = this.updateTime = totalTime - (broadPhaseTime + narrowPhaseTime + solvingTime);

		if (this.tt === 100) {
			this.resetMax();
		}

		if (this.tt > 100) {
			if (broadPhaseTime > MaxBroadPhaseTime) {
				this.MaxBroadPhaseTime = broadPhaseTime;
			}
			if (narrowPhaseTime > MaxNarrowPhaseTime) {
				this.MaxNarrowPhaseTime = narrowPhaseTime;
			}
			if (solvingTime > MaxSolvingTime) {
				this.MaxSolvingTime = this.solvingTime;
			}
			if (totalTime > MaxTotalTime) {
				this.MaxTotalTime = totalTime;
			}
			if (updateTime > MaxUpdateTime) {
				this.MaxUpdateTime = updateTime;
			}
		}

		this.upfps();

		this.tt++;
		if (this.tt > 500) {
			this.tt = 0;
		}
	}

	upfps() {
		this.f[1] = Date.now();
		if (this.f[1] - 1000 > this.f[0]) {
			this.f[0] = this.f[1];
			this.fps = this.f[2];
			this.f[2] = 0;
		}
		this.f[2]++;
	}

	show() {
		return `
			Oimo.js ${ this.version } <br>
			${ this.broadPhase } <br><br>
			FPS: ${ this.fps } fps<br><br>
			rigidbody ${ this.parent.numRigidBodies } <br>
			contact &nbsp;&nbsp;${ this.parent.numContacts }<br>
			ct-point &nbsp;${ this.parent.numContactPoints }<br>
			paircheck ${ this.parent.broadPhase.numPairChecks }<br>
			island &nbsp;&nbsp;&nbsp;${ this.parent.numIslands }<br><br>
			Time in milliseconds<br><br>
			broadphase &nbsp;${ _Math.fix(this.broadPhaseTime) } | ${ _Math.fix(this.MaxBroadPhaseTime) } <br>
			narrowphase ${ _Math.fix(this.narrowPhaseTime) } | ${ _Math.fix(this.MaxNarrowPhaseTime) }<br>
			solving &nbsp;&nbsp;&nbsp;&nbsp;${ _Math.fix(this.solvingTime) } | ${ _Math.fix(this.MaxSolvingTime) }<br>
			total &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;${ _Math.fix(this.totalTime) } | ${ _Math.fix(this.MaxTotalTime) }<br>
			updating &nbsp;&nbsp;&nbsp;${ _Math.fix(this.updateTime) } | ${ _Math.fix(this.MaxUpdateTime) }<br>
		`;
	}

	toArray() {
		const {
			parent: {
				broadPhase: {
					types,
					numPairChecks
				},
				numRigidBodies,
				numContacts,
				numContactPoints,
				numIslands
			},
			broadPhaseTime,
			narrowPhaseTime,
			solvingTime,
			updateTime,
			totalTime,
			fps
		} = this;

		return this.infos = [
			types,
			numRigidBodies,
			numContacts,
			numPairChecks,
			numContactPoints,
			numIslands,
			broadPhaseTime,
			narrowPhaseTime,
			solvingTime,
			updateTime,
			totalTime,
			fps
		];
	}

}
