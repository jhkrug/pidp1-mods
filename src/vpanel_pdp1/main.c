/*
 * Browser/desktop-free "virtual" front panel for the pidp-1 emulator.
 *
 * Renders a bitmap image of the PDP-1 console, paneltex, baked in
 * from art/pdp1art.inc and overlays clickable lamp/switch/key sprites on
 * top of it at hand-tuned grid coordinates.
 * Mouse clicks toggle the on-screen switches and keys.
 * The resulting switch/key state is packed into the shared-memory Panel struct the emulator reads.
 * Lamp state is read but NOT via a live sample of panel->lightsN.
 * S single once-per-frame read of panel->lightsN would alias against any lamp activity
 * faster than this loop's ~30ms cadence, showing a lamp that's mostly on as off,
 * or vice versa, purely by bad luck of what the register held at the sampled
 * instant.
 * Instead this program reads panel->pwmcount[][]/panel->cyclecount added for the new hardware panel.
 * It already uses this derive a true duty-cycle-based brightness.
 * The brightess percentage is then collapsed to a boolean via a fixed threshold, PWM_ON_THRESHOLD,
 * once per frame, since this front end has no brightness ramp, only one on/off sprite pair per
 * lamp.
 * Neither this nor the hardware panel writes the pwm data, each keeps its own last read, so the two
 * may run at the same time.
 *
 * 5-Jul-2026 wje rework to fix the incorrectly labeled H.S CYCLE light name, add the new pwm logic,refomat
 * 1-Oct-2026 Claude: draw a frame only when an element changed or the window needs repainting.
 * 1-Oct-2026 Claude: Makefile rebuilds on a change to any included file.
 * 4-Oct-2026 wje panel lights were not turning off when the power switch was turned off
 * 8-Nov-2026 Claude: lights take each pwm count's difference from its last read instead of resetting the count.
 * 11-Nov-2026 Claude: add the rack top (I/O) panel lamps above the console, rows 7-9 of the Panel lights;
 *                    add a window scale factor, -s or vpanelscale; darker switch and key levers;
 *                    all art at 2x so it is sharp when scaled, see art/mkvpanelart.py
 * 11-Nov-2026 Claude: with the hardware panel's driver running, mirror its switches instead of
 *                    writing them;
 *                    never overwrite switch words someone else changed, take their values instead
 * 14-Nov-2026 Claude: the hardware panel is always the authority, -m and -a removed;
 *                    a change by another writer takes the keys too, so a key held in the browser stays held
 * 15-Nov-2026 Claude: each mouse button press looks for the hardware panel driver first,
 *                    thus a click never writes while the driver runs
 * 17-Nov-2026 Claude: -h host, network mode: the panel of a pdp1 on another computer, through its
 *                    panel port 1046 (pdp1/panelsrv.c); hwPanelRunning() moved to common.c
 * 18-Nov-2026 Claude: the mouse lets go only of a key it pressed, not one taken from another writer
 * 18-Nov-2026 Claude: network mode: IPv4, as pdp1 listens; the name looked up before the window opens;
 *                    no line for NETTIMEOUT_MS, or a connect that long, closes the connection;
 *                    clicks do nothing while not connected; vpanelhost removed, -h only
 */
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <errno.h>
#include <signal.h>
#include <fcntl.h>
#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include "common.h"
#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>

#include "panel_pidp1.h"
#include "configuration.h"

#ifndef CONFIG_FILE
#define CONFIG_FILE "/opt/pidp1-mods/pidp1.config"
#endif

#define nil NULL

// Duty-cycle fraction (0.0-1.0), computed per lamp per frame from
// panel->pwmcount[][] at or above which a lamp is drawn "on".
#define PWM_ON_THRESHOLD 0.5f

// A hack to be sure the H.S.CYCLE light shows up, the lack of intensity ramping means it is frequently missed.
// If the pwm count is >= HSC_STRETCH_THRSHOLD, it is set to on.
#define HSC_ROW  5
#define HSC_COL 14
#define HSC_STRETCH_THRESHOLD 30

// Baked-in PNG resource byte arrays for the panel background and lamp/switch/key
// sprite pairs generated offline from source art and saved as C byte
// arrays so this program has no runtime asset-file dependency.
#include "art/panelart.inc"
#include "art/pdp1art.inc"
#include "art/ioart.inc"

// Logical layout, in pixels at scale 1.
// The console art is CONSOLE_W x CONSOLE_H.
// The rack top (I/O) panel, when shown, is a strip IO_H high above it, so the console moves down by IO_H.
// The I/O panel's lamp grid, IO_*, MUST match art/mkvpanelart.py, which drew the labels for it.
#define CONSOLE_W   800
#define CONSOLE_H   448
#define IO_H        220
#define IO_XSCL     (12.95f * 800.0f / 500.0f)
#define IO_X0       (CONSOLE_W / 2.0f - 8.5f * IO_XSCL)
#define IO_Y0       33.0f
#define IO_YSCL     52.0f

// All the art is made at ART_SCALE times the logical size, each piece is drawn into a
// rectangle 1/ART_SCALE its pixel size, so it is sharp when the window is scaled up.
// MUST match ART_SCALE in art/mkvpanelart.py.
#define ART_SCALE   2

// Window scale factor limits.
#define MIN_SCALE   0.5f
#define MAX_SCALE   4.0f

// The window scale factor, vpanelscale or -s.
// The panel is laid out at scale 1 and SDL scales it, see main().
static float scale = 1.0f;

// Whether the rack top (I/O) panel is shown, vpaneliopanel or -i/-n.
static int showIo = 1;

// Who owns the switches.
// With the hardware panel's driver running, it writes the switch words in the Panel segment
// several hundred times a second, writing them here as well would make the two fight.
// So while the driver runs the physical panel is the authority: this mirrors its switches
// and works the switches itself only when the driver isn't running.
// The lamps are shared either way, each reader takes differences of the pwm counts.

// Set while the switches mirror the hardware panel's, see checkMode().
static int mirroring = 0;

// How often, in ms, to look for the hardware panel driver.
#define HWCHECK_MS  1000

// The switch words, sw0-sw3, as this last wrote them, and whether it has written them since
// it last became active.
// Before each write they are compared with the segment: a difference means someone else wrote
// them, the hardware panel's driver just starting for example. Its values are taken instead
// of being overwritten, and the driver is looked for at once rather than at the next HWCHECK_MS.
// Otherwise a stale POWER off written here could reach pdp1, which power clears on seeing it.
static int lastSw[4];
static int haveWritten = 0;

// Network mode, -h host[:port]: the panel of a pdp1 on another computer, through its
// panel port, see pdp1/panelsrv.c for the protocol.
// panel then points at netPanel, which holds the switch words as pdp1 last sent them, so
// mirrorSwitches() and the change checks in updatepanel() work as with the segment.
// The lamps come as a brightness, netLamps, already made from the pwm counts by pdp1.
// The hardware panel's driver runs on that computer, not here: pdp1 says whether it runs, netMirror.
#define NETPORT         1046
#define NETRETRY_MS     2000            // between two tries to connect
#define NETTIMEOUT_MS   3000            // a connect, or the wait for a line, this long closes the connection
#define NETRESOLVE_MS   30000           // a connect that fails looks the name up again at most this often
#define NETLINEMAX      512
static char *netHost = nil;             // nil: the segment, as always
static int netPort = NETPORT;
static int netFd = -1;
static int netConnecting = 0;           // a connect() is in progress on netFd
static int netHave = 0;                 // a P line came since the connection was made
static Uint32 netLastTry = 0;
static Uint32 netConnectStart = 0;      // when the connect in progress started
static Uint32 netLastLine = 0;          // when the last P line came
static struct sockaddr_in netAddr;      // netHost's address, IPv4 as pdp1 listens
static int netResolved = 0;             // netAddr is set
static Uint32 netResolveTime = 0;
static Panel netPanel;
static u8 netLamps[10 * 18];
static int netPresent = 0;
static int netMirror = 0;
static unsigned long netSeq = 0;        // the last S line's number
static char netIn[NETLINEMAX * 4];
static size_t netInLen = 0;

static SDL_Window *window;
static SDL_Renderer *renderer;

// A Grid maps abstract "panel units" (roughly millimeters on the physical
// console artwork, see init()'s `scl` conversion) to on-screen pixels via a
// simple per-axis offset+scale pair. Two grids exist (grid1, grid2) because
// the main panel and the narrower right-hand sense-switch/IR column use
// slightly different spacing constants in the source artwork.
typedef struct Grid Grid;
struct Grid
{
	float xoff, yoff;
	float xscl, yscl;
};

// One clickable/drawable panel element (a single lamp, toggle switch, or
// key). `tex` points at a 2-entry (lights/switches) or 3-entry (keys)
// texture array indexed by `state`, so `state` doubles as both the visible
// on/off (or up/down/neutral for keys) sprite index and the logical bit
// value read back in getnswitches()/updatepanel(). `grid`+`x`,`y` are the
// element's fixed design-time position (set once in elements.inc); putongrid()
// converts that into the actual pixel rectangle `r` used for hit-testing and
// drawing. `active` is a one-shot debounce latch used only for switches (see
// mouse()) so a single mouse-down doesn't toggle a switch on every frame the
// button stays held over it.
typedef struct Element Element;
struct Element
{
	SDL_Texture **tex;
	Grid *grid;
	float x, y;
	int state, active;
	SDL_Rect r;
    int drawnState;     // state as last drawn; after r, so elements.inc's initializers leave it 0
};

// One rack top (I/O) panel lamp, see iolamps[] in elements.inc.
// X, y are its grid3 position, row the Panel lights row, 7-9, and mask its bit in that row.
typedef struct IoLamp IoLamp;
struct IoLamp
{
	float x, y;
	int row;
	int mask;
};

// Shared-memory Panel struct definition and the sw0/sw1/sw2 bit-field enum
// (SW_*, KEY_*, L5_*), also used by the emulator (pdp1/panel1.c) and the
// hardware panel driver (panel_pidp1/newpanel.c). See that header's own
// comments for the full field layout; the bit constants referenced below
// (e.g. SW_EXTEND, KEY_START) come from it.
Panel *panel;

// The panel->cyclecount as of the end of the last computeLightWord() pass over
// all seven main-panel rows, used to compute expectedcycles.
u64 lastcyclecount;

// Each panel->pwmcount[][] counter as of the same pass; a lamp's on-cycles are the u16
// difference from it, since only pdp1 writes the counters.
u16 lastcount[10][18];

// Named sub-ranges into the `lights[]` array (elements.inc), one pointer per
// PDP-1 register/indicator group. Each pointer is set in init() to point at
// the first Element of its group; the group's element count is implied by
// the corresponding setnlights() call in updatepanel() (there is no stored
// length here, so the ranges must stay in lock-step with both elements.inc's
// ordering and init()'s carve-up below).
Element *pc_l, *ma_l;
Element *mb_l, *ac_l, *io_l;
Element *ff_l, *misc_l, *ss_l, *pf_l, *ir_l;
// Named sub-ranges into the `switches[]` array (elements.inc), analogous to
// the lights pointers above but for the input side (toggle switches).
Element *ta_sw, *tw_sw, *ext_sw;
Element *misc_sw, *ss_sw;

// Panel background and per-element sprite textures, populated by init()
// via loadtex(). lamptex/switchtex/hswitchtex are [off,on] (or
// [down,up]) pairs; keytex has a third entry for keys with a
// spring-loaded "up" position distinct from neutral and "down"
// (see the state==2 cases in mouse()/updatepanel()).
SDL_Texture *paneltex;
SDL_Texture *iotex;
SDL_Texture *lamptex[2];
SDL_Texture *switchtex[2];
SDL_Texture *hswitchtex[2];
SDL_Texture *keytex[3];

// grid1 covers the main panel body (PC/MA/MB/AC/IO/flip-flop lights, the
// EXTEND/TA/TW switch rows, and the key row); grid2 covers the narrower
// right-hand column (run/sstep/sinst, sense switches, program flags, IR).
// Both are derived from the same `scl` mm-to-pixel factor in init().
Grid grid1, grid2;

// grid3 covers the rack top (I/O) panel, one unit per lamp column and row.
Grid grid3;

// Set when the window needs a frame whatever the elements show: the first
// frame, and after SDL reports the window's contents may be gone.
int redrawNeeded = 1;

// Pulls in the static Element tables (lights[], switches[], keys[]) that
// enumerate every lamp/switch/key on the panel along with its design-time
// grid position. Kept in a separate file because it is a long, mostly
// declarative data table rather than logic; see elements.inc's own header
// comment for the coordinate convention it uses.
// Note that this is actual code.
#include "elements.inc"

// The I/O panel lamp Elements, made from iolamps[] by init().
Element iolights[nelem(iolamps)];

int computeLightWord(Panel *panelP, int row, u64 expectedcycles);
void loadConfig(void);
void usage(void);
void checkMode(void);
void setTitle(void);
void netPoll(void);
void mirrorSwitches(int withKeys);

// Decodes a baked-in PNG byte array (one of the pdp1art.inc/panelart.inc
// arrays) into a renderer-owned SDL_Texture. Wraps the raw bytes in an
// SDL_RWops "file" backed by the in-memory buffer so SDL_image can load it
// without ever touching the filesystem. Terminates the process via panic()
// (see common.c) if either the RWops wrapper or the image decode fails --
// there is no recoverable-failure path, since a missing panel sprite means
// the program cannot usefully continue.
// Returns: a valid, non-NULL SDL_Texture* owned by `renderer` (never
// returns on failure -- panic() calls exit()).
SDL_Texture*
loadtex(unsigned char *data, int sz)
{
	SDL_RWops *f;
	SDL_Texture *tex;

	f = SDL_RWFromConstMem(data, sz);
	if(f == nil)
		panic("Couldn't load resource");
	tex = IMG_LoadTexture_RW(renderer, f, 0);
	if(tex == nil)
		panic("Couldn't load image");
	SDL_RWclose(f);
	return tex;
}

// Debug helper: overlays a grid of red lines spaced g->xscl/g->yscl apart
// over `tex`, used during panel-art development to visually check element
// alignment against the background image. Not called anywhere; to use it,
// call it in draw() after the background is copied.
void
drawgrid(SDL_Texture *tex, Grid *g)
{
int w, h;
float x, y;

	SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
	SDL_QueryTexture(tex, nil, nil, &w, &h);
	for(x = g->xoff; x < w; x += g->xscl)
    {
		SDL_RenderDrawLine(renderer, x, 0, x, h);
    }

	for(y = g->yoff; y < h; y += g->yscl)
    {
		SDL_RenderDrawLine(renderer, 0, y, w, y);
    }
}

// Computes the on-screen pixel rectangle (e->r) for element `e` from its
// fixed grid-unit position (e->x, e->y) and its current texture's
// size divided by ART_SCALE, then stores it back into `e` for later use by ismouseover() (hit
// testing) and drawelement() (blit destination). Must be called once per
// element after every texture is loaded and before the first draw/mouse
// event; called from init() for every lamp/switch/key.
// Note: w/2 and h/2 are integer divisions of the texture's pixel
// dimensions, so odd-width/height sprites are rounded down by up to half a
// pixel relative to the float-based grid math used for the rest of the
// placement; visually negligible at this panel's resolution.
void
putongrid(Element *e)
{
	int w, h;
	SDL_QueryTexture(e->tex[0], nil, nil, &w, &h);
    w /= ART_SCALE;
    h /= ART_SCALE;
	e->r.x = e->grid->xoff - w/2 + e->grid->xscl*e->x;
	e->r.y = e->grid->yoff - h/2 + e->grid->yscl*e->y;
	e->r.w = w;
	e->r.h = h;
}

// Blits element `e`'s current-state sprite (e->tex[e->state]) into its
// precomputed screen rectangle (e->r, set by putongrid()), and notes the
// state drawn for changed().
void
drawelement(Element *e)
{
	SDL_RenderCopy(renderer, e->tex[e->state], nil, &e->r);
    e->drawnState = e->state;
}

// Returns 1 if any element of the n in elems has a state other than the one last drawn, else 0.
int
groupChanged(Element *elems, unsigned n)
{
    unsigned i;

    for(i = 0; i < n; i++)
    {
        if(elems[i].state != elems[i].drawnState)
        {
            return(1);
        }
    }

    return(0);
}

// Returns 1 if any lamp, switch or key differs from the last frame drawn, else 0.
// Comparing every element, rather than flagging each write to a state, cannot
// miss a writer: mouse(), initpanel() and updatepanel() all set states directly.
int
changed(void)
{
    return( groupChanged(lights, nelem(lights)) || groupChanged(switches, nelem(switches))
            || groupChanged(keys, nelem(keys)) || (showIo && groupChanged(iolights, nelem(iolights))) );
}

// Renders one complete frame: panel background, then every lamp, switch,
// and key sprite on top in that order (so switches/keys visually sit above
// the background art, and lights are drawn first so a switch/key sprite at
// the same grid position -- e.g. the sense-switch lamp+toggle pair -- can
// overlay it), then presents the frame. Called from the main loop only when
// changed() or redrawNeeded says the window is out of date. Every frame is
// drawn whole, since the back buffer's contents after a present are undefined.
void
draw(void)
{
unsigned i;
SDL_Rect consoleRect = { 0, 0, CONSOLE_W, CONSOLE_H };
SDL_Rect ioRect = { 0, 0, CONSOLE_W, IO_H };

    // A window resized to another shape has bars beside the panel, give them the panel's color.
	SDL_SetRenderDrawColor(renderer, 230, 230, 230, 255);
	SDL_RenderClear(renderer);

    if( showIo )
    {
        // The art is made at ART_SCALE times the logical size, the rectangles scale it to fit.
        SDL_RenderCopy(renderer, iotex, nil, &ioRect);
        consoleRect.y = IO_H;

        for(i = 0; i < nelem(iolights); i++)
        {
            drawelement(&iolights[i]);
        }
    }

	SDL_RenderCopy(renderer, paneltex, nil, &consoleRect);

	for(i = 0; i < nelem(lights); i++)
    {
		drawelement(&lights[i]);
    }

	for(i = 0; i < nelem(switches); i++)
    {
		drawelement(&switches[i]);
    }

	for(i = 0; i < nelem(keys); i++)
    {
		drawelement(&keys[i]);
    }

	SDL_RenderPresent(renderer);
}

// One-time SDL/panel setup, called once from main() before the event loop
// starts. Loads every sprite texture, derives grid1/grid2's pixel scale
// from a fixed mm-to-pixel conversion factor tuned against the panel art,
// positions every Element in lights[]/switches[]/keys[] onto its grid, and
// finally carves each of those flat arrays into the named per-register
// sub-ranges (pc_l, ma_l, ta_sw, ...) used by initpanel()/updatepanel().
// The carve-up below is purely pointer arithmetic over the arrays defined
// in elements.inc and encodes an implicit, unchecked dependency on that
// file's element ordering and group sizes -- reordering or resizing a
// group in elements.inc without updating the matching `e +=` line here (or
// vice versa) will silently misattribute lights/switches to the wrong
// register with no compile-time or run-time check.
void
init(void)
{
unsigned i;

	paneltex = loadtex(pdp1_panel_2x_png, pdp1_panel_2x_png_len);
	iotex = loadtex(io_panel_png, io_panel_png_len);
	lamptex[0] = loadtex(lamp_off_png, lamp_off_png_len);
	lamptex[1] = loadtex(lamp_on_png, lamp_on_png_len);
	switchtex[0] = loadtex(switch_d_png, switch_d_png_len);
	switchtex[1] = loadtex(switch_u_png, switch_u_png_len);
	hswitchtex[0] = loadtex(switch_r_png, switch_r_png_len);
	hswitchtex[1] = loadtex(switch_l_png, switch_l_png_len);
	keytex[0] = loadtex(key_n_png, key_n_png_len);
	keytex[1] = loadtex(key_d_png, key_d_png_len);
	keytex[2] = loadtex(key_u_png, key_u_png_len);


	float scl = 800.0/500.0;	// mm to px

	grid1.xscl = 12.95 * scl;
	grid1.yscl = 12.9  * scl;
	grid1.xoff =  4.5  * scl + grid1.xscl/2;
	grid1.yoff =  2.56 * scl;

	grid2 = grid1;
	grid2.xoff =  1.05 * scl + grid2.xscl/2;

    // With the I/O panel shown, the console is drawn below it.
    if( showIo )
    {
        grid1.yoff += IO_H;
        grid2.yoff += IO_H;
    }

    grid3.xscl = IO_XSCL;
    grid3.yscl = IO_YSCL;
    grid3.xoff = IO_X0;
    grid3.yoff = IO_Y0;

	for(i = 0; i < nelem(iolamps); i++)
    {
        iolights[i].tex = lamptex;
        iolights[i].grid = &grid3;
        iolights[i].x = iolamps[i].x;
        iolights[i].y = iolamps[i].y;
		putongrid(&iolights[i]);
    }

	for(i = 0; i < nelem(lights); i++)
    {
		putongrid(&lights[i]);
    }

	for(i = 0; i < nelem(switches); i++)
    {
		putongrid(&switches[i]);
    }

	for(i = 0; i < nelem(keys); i++)
    {
		putongrid(&keys[i]);
    }

	Element *e = lights;
	pc_l = e; e += 16;
	ma_l = e; e += 16;
	mb_l = e; e += 18;
	ac_l = e; e += 18;
	io_l = e; e += 18;
	ff_l = e; e += 13;
	misc_l = e; e += 3;
	ss_l = e; e += 6;
	pf_l = e; e += 6;
	ir_l = e; e += 5;

	e = switches;
	ext_sw = e; e++;
	ta_sw = e; e += 16;
	tw_sw = e; e += 18;
	misc_sw = e; e += 3;
	ss_sw = e; e += 6;
}

// Returns non-zero if screen point (x,y) falls within element `e`'s current
// hit rectangle (e->r, set by putongrid()), zero otherwise.
int
ismouseover(Element *e, int x, int y)
{
	return x >= e->r.x && x < e->r.x+e->r.w &&
		y >= e->r.y && y < e->r.y+e->r.h;
}

// Bitmask of currently-held mouse buttons, updated in main()'s event loop:
// bit0 (1) = left/SDL_BUTTON_LEFT, bit1 (2) = middle/SDL_BUTTON_MIDDLE,
// bit2 (4) = right/SDL_BUTTON_RIGHT (from `1 << (ev.button.button-1)`,
// since SDL numbers these buttons 1,2,3). Any higher SDL button number
// (e.g. the X1/X2 side buttons, button 4/5) would set bit3/bit4, which no
// code below ever tests, so those buttons are silently inert here rather
// than rejected -- harmless, but unremarked upon in the original source.
int buttonstate;

// Called on every mouse-motion and mouse-button SDL event with the
// pointer's current window coordinates; updates every switch's and key's
// `state` field to reflect the current mouse position/button combination.
// No return value.
//
// Switches (toggle-style) use `active` as a one-shot debounce latch: the
// state change (toggle/force-on/force-off, chosen by which button is held)
// is applied only on the frame the button first goes down over the
// element (the `!e->active` transition), not on every subsequent frame the
// button stays held there -- otherwise a switch would flicker rapidly
// while the mouse sits still with the button down (this function is
// called on every motion event, not just on press/release edges).
// `active` is cleared as soon as the button is released or the pointer
// leaves the element, re-arming the latch for the next click.
//
// Keys (momentary pushbuttons) intentionally have no such debounce: their
// `state` is simply the current button/hover condition every frame
// (0 = neutral, released or off the element), so releasing the mouse or
// moving off the key immediately returns it to neutral (state 0), matching
// a physical momentary key rather than a persistent toggle.
//
// Button-to-state mapping: for switches, left-click toggles the current
// state, middle-click forces "up"/on (state 1), right-click forces
// "down"/off (state 0). For keys, left-click sets state 1 ("down") and
// right-click sets state 2 ("up") -- middle-click has no effect on keys.
// If more than one button is held at once the if/else-if chain below gives
// left priority over middle over right; this precedence is arbitrary
// (chording is not an expected real input) and undocumented in the
// original source.
void
mouse(int x, int y)
{
unsigned i;
Element *e;

    // Mirroring, the switches and keys show the hardware panel's, clicks do nothing.
    // In network mode with no connection there is nowhere to send them, they do nothing either.
    if( mirroring || (netHost && !netHave) )
    {
        return;
    }

	for(unsigned i = 0; i < nelem(switches); i++)
    {
		e = &switches[i];
		if(buttonstate == 0 || !ismouseover(e, x, y))
        {
			e->active = 0;
		}
        else if(!e->active)
        {
			e->active = 1;
			if(buttonstate & 1)
            {
				e->state = !e->state;
            }
			else if(buttonstate & 2)
            {
				e->state = 1;
            }
			else if(buttonstate & 4)
            {
				e->state = 0;
            }
		}
	}

    // A key's active is set while the mouse holds it. Only then does the mouse let it go: a key taken
    // from another writer by mirrorSwitches(1), held in the browser for example, stays as that writer has it.
	for(i = 0; i < nelem(keys); i++)
    {
		e = &keys[i];
		if( (buttonstate & 5) && ismouseover(e, x, y) )
        {
			e->state = (buttonstate & 1)?1:2;
			e->active = 1;
        }
		else if( e->active )
        {
			e->state = 0;
			e->active = 0;
        }
	}
}

// Packs `n` consecutive bits of `b`, starting at `bit` and shifting right
// by one each iteration, into the `state` field of `l[0..n-1]`, one bit per
// Element, most-significant of the range first.
void
setnlights(int b, Element *l, int n, int bit)
{
int i;

	for(i = 0; i < n; i++, bit >>= 1)
    {
		l[i].state = !!(b & bit);
    }
}

// Inverse of setnlights(), scans `n` consecutive Elements in `sw[]` and for
// each one whose state equals 'state', sets the corresponding bit.
int
getnswitches(Element *sw, int bit, int n, int state)
{
int b, i;

	b = 0;
	for(i = 0; i < n; i++, bit >>= 1)
    {
		if(sw[i].state == state)
        {
			b |= bit;
        }
    }

	return( b );
}

// Attaches the /tmp/pdp1_panel shared-memory
// segment and seeds every switch Element's on-screen state from whatever
// is already in it so a freshly-started vpanel_pdp1 reflects switch
// positions left by a previous run/process rather than resetting them to
// "all down". Calls exit(1) if the segment cannot be mapped at all.
void
initpanel(void)
{
    // Network mode: nothing to map, the switches are set from the first line pdp1 sends.
    if( netHost )
    {
        panel = &netPanel;
        return;
    }

	panel = createseg("/tmp/pdp1_panel", sizeof(Panel));
	if(panel == nil)
		exit(1);

	// Start with the switches where the segment has them, the keys left neutral.
	mirrorSwitches(0);

	// Prime the PWM integration baselineto the segment's current cyclecount so the first
	// real pass measures only cycles that elapse after this program attaches,
    // not cycles counted since whatever process created or last reset the segment.
	lastcyclecount = panel->cyclecount;
    memcpy(lastcount, panel->pwmcount, sizeof(lastcount));
}

// Decide whether the switches mirror the hardware panel's or are worked here, see mirroring.
// Called at startup and every HWCHECK_MS from the main loop.
// Going from mirroring to active, the on-screen switches already show the last physical
// positions, so pdp1 sees no change; the keys go back to neutral.
void
checkMode(void)
{
int want;
unsigned i;

    want = netHost ? netMirror : hwPanelRunning();

    if( want == mirroring )
    {
        return;
    }

    mirroring = want;
    if( mirroring )
    {
        mirrorSwitches(1);
    }
    else
    {
        haveWritten = 0;        // nothing written yet to compare against
        for(i = 0; i < nelem(keys); i++)
        {
            keys[i].state = 0;
            keys[i].active = 0;
        }
        for(i = 0; i < nelem(switches); i++)
        {
            switches[i].active = 0;
        }
    }

    setTitle();
}

// The window title shows the mode, and in network mode the host and whether it is connected.
void
setTitle(void)
{
char title[256];

    if( netHost && !netHave )
    {
        snprintf(title, sizeof(title), "PDP-1 console (%s, not connected)", netHost);
    }
    else if( netHost )
    {
        snprintf(title, sizeof(title), "PDP-1 console (%s%s)", netHost,
                 mirroring?", mirroring hardware panel":(netPresent?"":", no panel yet"));
    }
    else
    {
        snprintf(title, sizeof(title), "%s", mirroring?"PDP-1 console (mirroring hardware panel)":"PDP-1 console");
    }
    SDL_SetWindowTitle(window, title);
}

// Network mode: closes the connection, darkens the lamps and starts the wait for the next try.
static void
netClose(void)
{
unsigned i;

    if( netFd >= 0 )
    {
        close(netFd);
    }
    netFd = -1;
    netConnecting = 0;
    netHave = 0;
    netPresent = 0;
    netMirror = 0;
    netInLen = 0;
    netLastTry = SDL_GetTicks();

    // pdp1 lets go of the keys this held when it sees the connection close; the screen does too,
    // so a key the mouse holds is not sent as pressed again after connecting.
    for(i = 0; i < nelem(keys); i++)
    {
        keys[i].state = 0;
        keys[i].active = 0;
    }
    for(i = 0; i < nelem(lights); i++)
    {
        lights[i].state = 0;
    }
    for(i = 0; i < nelem(iolights); i++)
    {
        iolights[i].state = 0;
    }
    setTitle();
}

// Network mode: looks netHost up into netAddr, IPv4 only: pdp1 listens on IPv4, and a name with an
// IPv6 address too, localhost for example, could otherwise give one that never answers.
// The lookup can take seconds when the name server or mDNS does not answer, so it is done before the
// window opens and again only after a connect fails, at most every NETRESOLVE_MS.
// Returns 1 when netAddr is set.
static int
netResolve(void)
{
struct addrinfo hints, *resP;
char portStr[16];

    netResolveTime = SDL_GetTicks();
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    snprintf(portStr, sizeof(portStr), "%d", netPort);
    if( getaddrinfo(netHost, portStr, &hints, &resP) != 0 )
    {
        return(0);
    }
    memcpy(&netAddr, resP->ai_addr, sizeof(netAddr));
    freeaddrinfo(resP);
    netResolved = 1;
    return(1);
}

// Network mode: a connect failed or the connection went quiet; closes it and, at most every
// NETRESOLVE_MS, has the next try look the name up again, its address may have changed.
static void
netFail(void)
{
    if( (SDL_GetTicks() - netResolveTime) >= NETRESOLVE_MS )
    {
        netResolved = 0;
    }
    netClose();
}

// Network mode: starts a connection without waiting for it, so the window stays live
// while the host does not answer. netPoll() sees it finish, or gives up after NETTIMEOUT_MS.
static void
netDial(void)
{
int fd, one;

    netLastTry = SDL_GetTicks();
    if( !netResolved )
    {
        // A name not found is looked up again only after NETRESOLVE_MS, each try can block.
        if( (SDL_GetTicks() - netResolveTime) < NETRESOLVE_MS )
        {
            return;
        }
        if( !netResolve() )
        {
            return;
        }
    }

    if( (fd = socket(AF_INET, SOCK_STREAM, 0)) < 0 )
    {
        return;
    }
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
    one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    if( (connect(fd, (struct sockaddr *)&netAddr, sizeof(netAddr)) == 0) || (errno == EINPROGRESS) )
    {
        netFd = fd;
        netConnecting = 1;
        netConnectStart = SDL_GetTicks();
        return;
    }
    close(fd);
    netFail();
}

// Network mode: takes one P line from pdp1, see pdp1/panelsrv.c.
static void
netLine(char *lineP)
{
int present, mirror, n, i, hi, lo;
unsigned long ack;
unsigned sw[4];
char *hexP;
const char *digits = "0123456789abcdef";
const char *hP, *lP;

    if( sscanf(lineP, "P %d %d %lu %o %o %o %o %n", &present, &mirror, &ack,
               &sw[0], &sw[1], &sw[2], &sw[3], &n) != 7 )
    {
        return;
    }
    hexP = lineP + n;
    if( strlen(hexP) < (2 * sizeof(netLamps)) )
    {
        return;
    }
    for(i = 0; i < (int)sizeof(netLamps); i++)
    {
        hP = strchr(digits, hexP[2 * i]);
        lP = strchr(digits, hexP[(2 * i) + 1]);
        hi = (hP && *hP) ? (int)(hP - digits) : 0;
        lo = (lP && *lP) ? (int)(lP - digits) : 0;
        netLamps[i] = (u8)((hi << 4) | lo);
    }

    // The switch words are taken only once pdp1 has answered the last request this sent:
    // before that they are older than what is on the screen, and taking them would undo a click.
    if( ack >= netSeq )
    {
        netPanel.sw0 = sw[0];
        netPanel.sw1 = sw[1];
        netPanel.sw2 = sw[2];
        netPanel.sw3 = sw[3];
    }
    netPresent = present;
    netMirror = mirror;
    netLastLine = SDL_GetTicks();

    // The first line after connecting: start with the switches where pdp1 has them, keys neutral,
    // as initpanel() does with the segment.
    if( !netHave )
    {
        netHave = 1;
        haveWritten = 0;
        mirrorSwitches(0);
        checkMode();
        setTitle();
    }
}

// Network mode, called every pass of the main loop: connects, and reads what pdp1 sent.
void
netPoll(void)
{
struct pollfd pfd;
int err;
socklen_t len;
ssize_t n;
char *startP, *nlP;
int oldPresent;

    if( netFd < 0 )
    {
        if( (SDL_GetTicks() - netLastTry) >= NETRETRY_MS )
        {
            netDial();
        }
        return;
    }

    if( netConnecting )
    {
        pfd.fd = netFd;
        pfd.events = POLLOUT;
        if( poll(&pfd, 1, 0) <= 0 )
        {
            // Not yet. A host that is off never answers, the kernel would try for minutes.
            if( (SDL_GetTicks() - netConnectStart) >= NETTIMEOUT_MS )
            {
                netFail();
            }
            return;
        }
        err = 0;
        len = sizeof(err);
        getsockopt(netFd, SOL_SOCKET, SO_ERROR, &err, &len);
        if( err )
        {
            netFail();
            return;
        }
        netConnecting = 0;
        netSeq = 0;
        netLastLine = SDL_GetTicks();
    }

    oldPresent = netPresent;
    for(;;)
    {
        n = recv(netFd, netIn + netInLen, sizeof(netIn) - 1 - netInLen, 0);
        if( n == 0 || ((n < 0) && (errno != EAGAIN) && (errno != EWOULDBLOCK) && (errno != EINTR)) )
        {
            netClose();
            return;
        }
        if( n < 0 )
        {
            break;
        }
        netInLen += n;
        netIn[netInLen] = '\0';

        startP = netIn;
        while( (nlP = strchr(startP, '\n')) )
        {
            *nlP = '\0';
            netLine(startP);
            startP = nlP + 1;
        }
        netInLen -= (startP - netIn);
        memmove(netIn, startP, netInLen);
        if( netInLen >= (sizeof(netIn) - 1) )
        {
            netInLen = 0;   // no newline in all of that, not a line from pdp1
        }
    }

    // pdp1 sends a line at least once a second. None for NETTIMEOUT_MS: the Pi is off or the network
    // has gone, without a close the kernel would report only after many minutes.
    if( (SDL_GetTicks() - netLastLine) >= NETTIMEOUT_MS )
    {
        netFail();
        return;
    }

    // The hardware panel's driver started or stopped on that computer.
    if( netHave && (netMirror != mirroring) )
    {
        checkMode();
    }
    else if( netPresent != oldPresent )
    {
        setTitle();
    }
}

// Network mode: asks pdp1 to change the switch words from what this showed, netPanel's, to want.
// Returns 1 when sent.
static int
netSend(const int want[3])
{
char line[128];
int len;

    netSeq++;
    len = snprintf(line, sizeof(line), "S %lu %o %o %o %o %o %o\n", netSeq, (unsigned)want[0],
                   (unsigned)want[1], (unsigned)want[2], (unsigned)netPanel.sw0,
                   (unsigned)netPanel.sw1, (unsigned)netPanel.sw2);
    if( send(netFd, line, len, 0) != len )     // SIGPIPE is ignored, see main()
    {
        netFail();
        return(0);
    }
    return(1);
}

// Set the on-screen switches, and with withKeys the keys, from the switch words in the segment,
// the inverse of the packing in updatepanel().
// At startup the keys are left alone, withKeys 0: a key left pressed in the segment by a program
// that has gone would otherwise be written back as pressed until the mouse moved.
// Taken from another writer that is still running, withKeys 1, it is released when that writer lets go.
void
mirrorSwitches(int withKeys)
{
int sw2;

    setnlights(panel->sw0, ta_sw, 16, 0100000);
    setnlights(panel->sw1, tw_sw, 18, 0400000);
    sw2 = panel->sw2;
    setnlights(sw2, ss_sw, 6, 0100000);
    ext_sw->state = !!(panel->sw0 & SW_EXTEND);
    misc_sw[0].state = !!(panel->sw0 & SW_POWER);
    misc_sw[1].state = !!(sw2 & SW_SSTEP);
    misc_sw[2].state = !!(sw2 & SW_SINST);

    if( !withKeys )
    {
        return;
    }

    // Key states: 0 neutral, 1 pressed down, 2 lifted up.
    keys[0].state = (sw2 & KEY_START)?1:((sw2 & KEY_START_UP)?2:0);
    keys[1].state = (sw2 & KEY_STOP)?1:0;
    keys[2].state = (sw2 & KEY_CONT)?1:0;
    keys[3].state = (sw2 & KEY_EXAM)?1:0;
    keys[4].state = (sw2 & KEY_DEP)?2:0;
    keys[5].state = (sw2 & KEY_READIN)?1:0;
    keys[6].state = (sw2 & KEY_READER)?1:((sw2 & KEY_READER_UP)?2:0);
    keys[7].state = (sw2 & KEY_FEED)?1:0;
}

// Reads one main-panel light row's worth (18 columns) of panel->pwmcount[][], takes each
// column's difference from lastcount[][] and moves lastcount[][] on, and packs a
// boolean-per-column "on" word from the differences, suitable for passing to setnlights().
//
// Each column's fractional on-time over the last window (count/expectedcycles) is compared against PWM_ON_THRESHOLD.
// Columns at or above it set their bit.
//
// Eexpectedcycles is the number of emulated cycles elapsed since this row was last read.
int
computeLightWord(Panel *panelP, int row, u64 expectedcycles)
{
int col, word;
u16 *rowP;
u16 count;
u16 now;

	word = 0;
    rowP = panelP->pwmcount[row];

	for(col = 0; col < 18; col++)
    {
        // The u16 difference is right across the count's wrap.
        now = rowP[col];
        count = (u16)(now - lastcount[row][col]);
        lastcount[row][col] = now;

        // HSC hack.
        if( (row == HSC_ROW) && (col == HSC_COL) && (count >= HSC_STRETCH_THRESHOLD) )
        {
			word |= 1 << col;
        }
        else
        {
            if(count > expectedcycles)
            {
                count = (u16)expectedcycles;	// expectedcycles already <= 65535, see caller
            }

            if( (float)count >= (PWM_ON_THRESHOLD * (float)expectedcycles) )
            {
                word |= 1 << col;
            }
        }
	}

	return(word);
}

// Fans the lamp words, one per Panel lights row, out into the lamp Elements.
// Rows 7-9, the I/O panel, only with withIo.
//
// Each row is read into a word exactly once, then reused for
// every setnlights() call that reads that row (row 5 feeds both
// ff_l and misc_l; row 6 feeds ir_l, ss_l, and pf_l) --
// computeLightWord() moves lastcount[][] on as it reads, so
// calling it twice for the same row would see a zero
// difference the second time.
static void
setLampWords(const int w[10], int withIo)
{
unsigned i;

    setnlights(w[0], pc_l, 16, 0100000);
    setnlights(w[1], ma_l, 16, 0100000);
    setnlights(w[2], mb_l, 18, 0400000);
    setnlights(w[3], ac_l, 18, 0400000);
    setnlights(w[4], io_l, 18, 0400000);
    setnlights(w[5], ff_l, 13, L5_RUN);
    setnlights(w[5], misc_l, 3, L5_PWR);
    setnlights(w[6], ir_l, 5, 0400000);
    setnlights(w[6], ss_l, 6, 0004000);
    setnlights(w[6], pf_l, 6, 0000040);

    if( withIo )
    {
        for(i = 0; i < nelem(iolights); i++)
        {
            iolights[i].state = !!(w[iolamps[i].row] & iolamps[i].mask);
        }
    }
}

// Network mode: the lamp words from pdp1's brightness, on at PWM_ON_THRESHOLD or more as with the
// segment: pdp1 rounds, so half is 128. pdp1 sends them dark with POWER off and has already stretched H.S. CYCLE.
static void
netLamps2Words(void)
{
int w[10];
int r, c;

    for(r = 0; r < 10; r++)
    {
        w[r] = 0;
        for(c = 0; c < 18; c++)
        {
            if( netPresent && (netLamps[(r * 18) + c] >= (u8)((PWM_ON_THRESHOLD * 255.0f) + 0.5f)) )
            {
                w[r] |= 1 << c;
            }
        }
    }
    setLampWords(w, showIo);
}

// Called once per main-loop iteration after mouse() has updated every
// switch/key Element's `state` from the current input and before draw().
// Two jobs: (1) collapse the on-screen switch/key state into panel->sw0-2
// so mouse clicks here actually affect the running machine, and (2) turn rows 0-6 of
// panel->pwmcount[][], the per-lamp on-cycle tally, into lamp words with
// computeLightWord() and fan those out into the individual lamp Elements' `state`, so
// draw() shows how long each lamp was lit since the last pass.
//
// The sw0/sw1/sw2 bit layout and the lamp-row packing below must
// stay in lock-step with pdp1/panel1.c's updateswitches()/updatelights()
// and with the SW_*/KEY_*/L5_* bit constants in panel_pidp1.h!
//
// DEP is intentionally wired ONLY to state==2 ("up"/right-click) below,
// with no state==1 ("down"/left-click) case.
// This mimics the pdp-1 deposit switch operation by lifting it up.
void
updatepanel(void)
{
	int sw, r;
	int w[10];
	u64 currentcyclecount, expectedcycles;

    // Network mode with no connection: nothing to read, and nowhere to write.
    if( netHost && !netHave )
    {
        return;
    }

    // Someone else changed the switch words since this wrote them, take theirs and look for the driver now.
    // If it is there, checkMode() starts mirroring and nothing is written below.
    // The keys are taken too: a key held in the web front panel stays held until it lets go,
    // which is another change taken here.
    if( !mirroring && haveWritten &&
        ((panel->sw0 != lastSw[0]) || (panel->sw1 != lastSw[1]) || (panel->sw2 != lastSw[2]) ||
         (panel->sw3 != lastSw[3])) )
    {
        mirrorSwitches(1);
        checkMode();
    }

    // Mirroring, show the hardware panel's switches and write nothing.
    // Otherwise pack the on-screen switches and keys into sw0-sw3 for pdp1.
    if( mirroring )
    {
        mirrorSwitches(1);
    }
    else
    {
        sw = getnswitches(ta_sw, 0100000, 16, 1);
        if(ext_sw->state) sw |= SW_EXTEND;
        if(misc_sw[0].state) sw |= SW_POWER;
        lastSw[0] = sw;

        lastSw[1] = getnswitches(tw_sw, 0400000, 18, 1);

        sw = getnswitches(ss_sw, 0100000, 6, 1);
        if(misc_sw[1].state) sw |= SW_SSTEP;
        if(misc_sw[2].state) sw |= SW_SINST;
        if(keys[0].state == 1) sw |= KEY_START;
        if(keys[0].state == 2) sw |= KEY_START_UP;
        if(keys[1].state == 1) sw |= KEY_STOP;
        if(keys[2].state == 1) sw |= KEY_CONT;
        if(keys[3].state == 1) sw |= KEY_EXAM;
        if(keys[4].state == 2) sw |= KEY_DEP;
        if(keys[5].state == 1) sw |= KEY_READIN;
        if(keys[6].state == 1) sw |= KEY_READER;
        if(keys[6].state == 2) sw |= KEY_READER_UP;
        if(keys[7].state == 1) sw |= KEY_FEED;
        lastSw[2] = sw;

        lastSw[3] = 0;	// no spacewar controllers for now

        if( netHost )
        {
            // Network mode: only a change is sent, it changes the bits that differ from netPanel's,
            // what pdp1 last sent; netPanel then shows the change until pdp1 has answered it.
            if( (lastSw[0] != panel->sw0) || (lastSw[1] != panel->sw1) || (lastSw[2] != panel->sw2) )
            {
                if( !netSend(lastSw) )
                {
                    return;
                }
                panel->sw0 = lastSw[0];
                panel->sw1 = lastSw[1];
                panel->sw2 = lastSw[2];
            }
            lastSw[3] = panel->sw3;     // not this program's, see mirrorSwitches()
            haveWritten = 1;
        }
        else
        {
            // Recorded from what is written, not read back: a read back could pick up another writer's value.
            panel->sw0 = lastSw[0];
            panel->sw1 = lastSw[1];
            panel->sw2 = lastSw[2];
            panel->sw3 = lastSw[3];
            haveWritten = 1;
        }
    }

    if( netHost )
    {
        netLamps2Words();
        return;
    }

    // With the power off, pdp1 stops advancing cyclecount, the lights need to be forced off.
    if( !(panel->sw0 & SW_POWER) )
    {
        unsigned i;

        for(i = 0; i < nelem(lights); i++)
        {
            lights[i].state = 0;
        }

        for(i = 0; i < nelem(iolights); i++)
        {
            iolights[i].state = 0;
        }

        // Counts accumulated before pdp1 saw the switch go off are dropped.
        memcpy(lastcount, panel->pwmcount, sizeof(lastcount));
        lastcyclecount = panel->cyclecount;
        return;
    }

	currentcyclecount = panel->cyclecount;
	expectedcycles = currentcyclecount - lastcyclecount;
	if(expectedcycles != 0)
    {
        lastcyclecount = currentcyclecount;

        // The u16 counts may have wrapped more than once in a window this long (a stalled loop),
        // so their differences mean nothing: take the current counts as the new base and keep
        // the lights as they are for this pass.
        if( expectedcycles > 65535 )
        {
            memcpy(lastcount, panel->pwmcount, sizeof(lastcount));
            return;
        }

        // Each row is read exactly once, see setLampWords().
        // Rows 7-9, the I/O panel, only when it is shown.
        // Otherwise they are not read, lastcount[][] for them is never used.
        for(r = 0; r < (showIo ? 10 : 7); r++)
        {
            w[r] = computeLightWord(panel, r, expectedcycles);
        }
        setLampWords(w, showIo);
	}
}



// Get our settings from the common config file, /opt/pidp1-mods/pidp1.config
void
loadConfig(void)
{
ConfigurationP confP;
ConfigurationSettingP settingP;

    if( !(confP = loadConfigFile(CONFIG_FILE)) )
    {
        return;         // no config file found
    }

    if( (settingP = findConfigurationSetting(confP, "vpanelscale")) )
    {
        scale = (isnan(settingP->fvalue))?(float)settingP->ivalue:settingP->fvalue;
    }

    if( (settingP = findConfigurationSetting(confP, "vpaneliopanel")) )
    {
        showIo = settingP->onOff;
    }
}

void
usage(void)
{
    printf("Usage: vpanel_pdp1 [-s scale] [-i] [-n] [-h host[:port]]\n");
    printf("-s scale, window size multiplier, %.1f to %.1f, for example 1.5\n", MIN_SCALE, MAX_SCALE);
    printf("-i, show the rack top (I/O) panel\n");
    printf("-n, don't show the rack top (I/O) panel\n");
    printf("-h host[:port], the panel of the pdp1 on host, IPv4, through its panel port, default %d\n", NETPORT);
    printf("While the hardware panel's driver runs, the switches mirror the hardware panel's.\n");
    exit(1);
}

// Entry point. Initializes the SDL/SDL_image and the window,
// loads and positions every panel Element, attaches the shared
// Panel segment, then runs the display loop forever.
//
// Everything is laid out and drawn in logical pixels, the console art's 800 wide.
// SDL_RenderSetLogicalSize() scales that to the window, and scales mouse event
// coordinates back, so hit-testing needs no change for the scale factor.
// The window is resizable, the panel keeps its shape inside it.
int
main(int argc, char *argv[])
{
	SDL_Event ev;
    int opt;
    Uint32 lastCheck = 0;
    int logicalH;

    loadConfig();           // command line switches override the config file

    while( (opt = getopt(argc, argv, "s:inh:")) != -1 )
    {
        switch( opt )
        {
        case 's':
            scale = atof(optarg);
            break;

        case 'i':
            showIo = 1;
            break;

        case 'n':
            showIo = 0;
            break;

        case 'h':
            netHost = strdup(optarg);
            break;

        default:
            usage();
            break;
        }
    }

    if( optind < argc )
    {
        usage();
    }

    // host:port, IPv4 only, as pdp1 listens.
    if( netHost )
    {
        char *colonP = strchr(netHost, ':');

        if( colonP )
        {
            *colonP = '\0';
            netPort = atoi(colonP + 1);
        }
        if( (netPort <= 0) || (netPort > 65535) || !netHost[0] )
        {
            usage();
        }

        // A send() to a connection pdp1 has closed would end the program, as on any system.
        signal(SIGPIPE, SIG_IGN);

        // Looked up now, before the window opens, see netResolve(). Not found: tried again later.
        if( !netResolve() )
        {
            fprintf(stderr, "vpanel_pdp1: can't find %s now, trying again every %d s\n", netHost,
                    NETRESOLVE_MS / 1000);
        }
    }

    if( !(scale >= MIN_SCALE) )     // also catches nan
    {
        scale = MIN_SCALE;
    }
    if( scale > MAX_SCALE )
    {
        scale = MAX_SCALE;
    }

    logicalH = CONSOLE_H + (showIo?IO_H:0);

	SDL_Init(SDL_INIT_EVERYTHING);
	IMG_Init(IMG_INIT_PNG);

    // Smooth scaling for any scale other than 1, this must be set before the textures are made.
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "linear");

	if(SDL_CreateWindowAndRenderer((int)lroundf(CONSOLE_W * scale), (int)lroundf(logicalH * scale),
                                   SDL_WINDOW_RESIZABLE, &window, &renderer) < 0)
		panic("error: SDL_CreateWindowAndRenderer() failed: %s", SDL_GetError());
	SDL_SetWindowTitle(window, "PDP-1 console");
    SDL_RenderSetLogicalSize(renderer, CONSOLE_W, logicalH);

	init();

	initpanel();
    checkMode();
    setTitle();

	for(;;)
    {
		while(SDL_PollEvent(&ev))
        {
			switch(ev.type)
            {
			case SDL_MOUSEMOTION:
				mouse(ev.motion.x, ev.motion.y);
				break;
			case SDL_MOUSEBUTTONDOWN:
				buttonstate |= 1<<(ev.button.button-1);
                // Look for the hardware panel driver before each press, not only every HWCHECK_MS.
                // When the driver runs, checkMode() starts mirroring and mouse() ignores the click.
                checkMode();
				mouse(ev.button.x, ev.button.y);
				break;
			case SDL_MOUSEBUTTONUP:
				buttonstate &= ~(1<<(ev.button.button-1));
				mouse(ev.button.x, ev.button.y);
				break;
            case SDL_WINDOWEVENT:
                // Events after which a backend may have lost or resized the window's contents.
                if( (ev.window.event == SDL_WINDOWEVENT_EXPOSED) || (ev.window.event == SDL_WINDOWEVENT_SIZE_CHANGED)
                    || (ev.window.event == SDL_WINDOWEVENT_SHOWN) || (ev.window.event == SDL_WINDOWEVENT_RESTORED) )
                {
                    redrawNeeded = 1;
                }
                break;
			case SDL_QUIT:
				exit(0);
			}
        }

        // updatepanel() runs every pass even when nothing is drawn: it reads
        // pwmcount[][], and a skipped read would put the skipped time into the
        // next window's duty.
        // Look for the hardware panel's driver now and then, it can start or stop at any time.
        if( !netHost && ((SDL_GetTicks() - lastCheck) >= HWCHECK_MS) )
        {
            checkMode();
            lastCheck = SDL_GetTicks();
        }

        if( netHost )
        {
            netPoll();
        }
		updatepanel();

        // An unchanged frame is not drawn: when exposed, every presented frame
        // costs the compositor too, and most frames repeat the last one.
        if( redrawNeeded || changed() )
        {
            draw();
            redrawNeeded = 0;
        }

		SDL_Delay(30);	// ~33fps fixed refresh; not synced to the emulator's cycle rate
	}

	return 0;
}
