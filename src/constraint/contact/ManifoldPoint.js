import { Vec3 } from '../../math/Vec3';

/**
 * The class holds details of the contact point.
 * @author saharan
 */

export class ManifoldPoint {
	// Whether this manifold point is persisting or not.
	warmStarted = false;

	// The position of this manifold point.
	position = new Vec3();

	// The position in the first shape's coordinate.
	localPoint1 = new Vec3();

	// The position in the second shape's coordinate.
	localPoint2 = new Vec3();

	// The normal vector of this manifold point.
	normal = new Vec3();

	// The tangent vector of this manifold point.
	tangent = new Vec3();

	// The binormal vector of this manifold point.
	binormal = new Vec3();

	// The impulse in normal direction.
	normalImpulse = 0;

	// The impulse in tangent direction.
	tangentImpulse = 0;

	// The impulse in binormal direction.
	binormalImpulse = 0;

	// The denominator in normal direction.
	normalDenominator = 0;

	// The denominator in tangent direction.
	tangentDenominator = 0;

	// The denominator in binormal direction.
	binormalDenominator = 0;

	// The depth of penetration.
	penetration = 0;
}
