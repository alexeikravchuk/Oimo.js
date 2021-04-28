import { Mat33 } from '../math/Mat33';

/**
 * This class holds mass information of a shape.
 * @author lo-th
 * @author saharan
 */

export class MassInfo {
	// Mass of the shape.
	mass = 0;

	// The moment inertia of the shape.
	inertia = new Mat33();
}
