// The front panel's network port, 1046 (panelsrv.c).
#ifndef PANELSRV_H
#define PANELSRV_H

#define PANELPORT 1046

// serveN() handler, threaded: serves one panel client; arg is the PDP1.
void handlepanelnet(int fd, void *arg);

#endif
