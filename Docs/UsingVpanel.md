## Using vpanel_pdp1

This document describes vpanel_pdp1, the on-screen PDP-1 front panel, and how to use it.

This is version 1.0\
Edit date 11-Nov-2026

## What is it?

Vpanel_pdp1 is a window showing the PDP-1 console: the lamps, the toggle switches, and the keys.
It can be used in place of the PiDP-1 hardware panel, it does the same job as panel_pidp1 (newpanel) does
for the hardware, and like that it is started before the emulator.
It can also run alongside the hardware panel as a second view of it, see below.

Above the console it also shows the rack top (I/O) indicator panel, laid out like the PiDP-1's top panel:

- TAPE READER, the reader buffer, bits 0-17
- TAPE READER status: BINARY, NAC, RC 1, RC 2, CLUTCH
- MEMORY: READ, STROBE, INHIBIT, WRITE
- TYPEWRITER: STATUS, BLACK, TYO, NAC, and the typewriter buffer, bits 12-17
- TAPE PUNCH, the punch buffer, channels 8-4 and 3-1, ON, and NAC
- DEFER 2 and OV 2

## With the hardware panel

Vpanel_pdp1 can run at the same time as the hardware panel, on a monitor or remotely for example.

It looks for the hardware panel's driver, panel_pidp1 or newpanel, when it starts and about once a second
after that.
While the driver runs, vpanel_pdp1 mirrors the hardware panel:

- the lamps show the same as the hardware's
- the switches and keys show where the physical ones are, clicking them does nothing
- the window title says *mirroring hardware panel*

When the driver isn't running, the on-screen switches and keys work the machine as usual.
If the driver stops, vpanel_pdp1 takes over the switches where the physical ones were left, so the
machine sees no change, only the keys go back to neutral.
If the driver starts, vpanel_pdp1 stops working the switches as soon as the driver changes one,
or within about a second if the physical switches match the screen.

Vpanel_pdp1 never overwrites a switch setting another program has changed, it takes the new setting instead.
So when the hardware panel starts with its switches set differently from the screen, the emulator only ever
sees the hardware's settings.
That matters for POWER: the emulator power clears, resetting the machine, if it sees POWER off even briefly.

The *-m* and *-a* switches, below, override this.

## Using the panel

Mouse buttons work the switches and keys:

| Button | Toggle switch | Key |
|--------|---------------|-----|
| left | flip it | press it down while held |
| middle | up (on) | nothing |
| right | down (off) | lift it up while held |

DEPOSIT works only by lifting it, the right button, as on the real machine.
START lifted is start in sequence break mode.

A switch that is up (on) has its dark lever above the pivot, down (off) below it.
The POWER, SINGLE STEP and SINGLE INST switches are on when the lever points left.
A key shows a dark lever while it is pressed down or lifted up, and only the end of it at rest.

Each lamp is shown on or off, a lamp lit for at least half the time since the last frame is on.

## Running from the command line

**vpanel_pdp1** [-s scale] [-i] [-n] [-m] [-a]

- s set the window size multiplier, 0.5 to 4.0, 1.5 for example
- i show the rack top (I/O) panel
- n don't show the rack top (I/O) panel
- m always mirror the switches, never work them, whether the hardware panel's driver runs or not
- a always work the switches, even with the hardware panel's driver running; any physical switch setting
  then replaces the on-screen one and undoes clicks on it, so this is only useful without the hardware panel

Command line switches override configuration file settings.

At scale 1 the window is 800 by 668, or 800 by 448 without the I/O panel.
At scale 2 that is 1600 by 1336, taller than a 1080 line screen, use 1.5 there or hide the I/O panel.

The window can also be resized with the mouse, the panel keeps its shape and fills as much of it as it can.

## Configuration parameters

These settings are read from */opt/pidp1-mods/pidp1.config* when vpanel_pdp1 starts:

| parameter | values | default | effect |
|-----------|--------|---------|--------|
| vpanelscale | 0.5 - 4.0 | 1.0 | the window size multiplier |
| vpaneliopanel | true, false | true | show or don't show the rack top (I/O) panel |

## The art

The panel art is built into the program, it needs no files at run time.
It is all made at twice the size it is drawn at scale 1, so it stays sharp up to scale 2.
It is made by *src/vpanel_pdp1/art/mkvpanelart.py*, from the original console art,
*pdp1_panel.png*, see that file if the art is to be changed.
