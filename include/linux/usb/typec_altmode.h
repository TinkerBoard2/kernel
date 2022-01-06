/*
 * These are the connector states (USB, Safe and Alt Mode) defined in USB Type-C
 * Specification. SVID specific connector states are expected to follow and
 * start from the value TYPEC_STATE_MODAL.
 */
enum {
	TYPEC_STATE_SAFE,	/* USB Safe State */
	TYPEC_STATE_USB,	/* USB Operation */
	TYPEC_STATE_MODAL,	/* Alternate Modes */
};
