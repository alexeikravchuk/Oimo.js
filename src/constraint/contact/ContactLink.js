/**
 * A link list of contacts.
 * @author saharan
 */

export class ContactLink {
	// The previous contact link.
	prev = null;
	// The next contact link.
	next = null;
	// The shape of the contact.
	shape = null;
	// The other rigid body.
	body = null;

	constructor(contact) {
		// The contact of the link.
		this.contact = contact;
	}
}