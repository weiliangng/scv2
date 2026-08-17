#ifndef SCAP_IO_OWNER_H
#define SCAP_IO_OWNER_H

/*
 * Fixed staging owner for the power-stage mode pins. Source mailboxes are
 * intentionally disconnected until the main mode-control rewrite.
 */
void ScapIo_Init(void);
void ScapIo_Tick1kHz(void);

#endif /* SCAP_IO_OWNER_H */
