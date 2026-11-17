// The front panel's network port, 1046: the lamps and switches of the Panel segment for a panel
// program on another computer, vpanel_pdp1 -h for example.
//
// Each connection gets lines of text, at most every PANELNETMS while something changed and at
// least once a second:
//
//   P <present> <mirror> <ack> <sw0> <sw1> <sw2> <sw3> <lamps>
//
// present is 1 once pdp1 has its Panel, mirror is 1 while the hardware panel's driver runs,
// ack is the last switch request's number, the sw words are octal and lamps is 360 hex digits,
// the brightness 00-ff of the 10 rows of 18 lamps, row 0 column 0 first.
// The brightness is the lamp's part of the emulated cycles since the last line, from the
// pwmcount[][] differences, as web_pdp1/panel.go makes it for the browser. It is rounded, so a lamp
// lit at least half the time, PWM_ON_THRESHOLD in vpanel_pdp1, is 80 (128) and over.
//
// A client changes switches and keys with
//
//   S <seq> <want0> <want1> <want2> <base0> <base1> <base2>
//
// in octal, seq a number that goes up. base is what the client showed, want what it wants.
// Only the bits that differ between them are changed, on top of the segment's current words,
// so a setting another program changed meanwhile is not overwritten. That matters for POWER:
// pdp1 power clears, resetting the machine, if it sees POWER off even briefly.
// Nothing is written while the hardware panel's driver runs, the physical panel is the authority.
// The driver is looked for before every write.
// The keys a client holds down are let go when its connection closes.
// A client that goes without closing, a laptop that sleeps for example, counts as gone once what
// was sent to it has had no answer for USERTIMEOUTMS: lines go to it at least once a second.
//
// The pwmcount[][] counts wrap and every reader takes differences, so each connection keeps its
// own copies and reads alongside newpanel, vpanel_pdp1 and web_pdp1.
//
// 17-Nov-2026 Claude: initial version
// 18-Nov-2026 Claude: a client that goes without closing is gone after USERTIMEOUTMS; brightness rounded

#include <errno.h>
#include <poll.h>
#include <pthread.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>

#include "common.h"
#include "panel_pidp1.h"
#include "pdp1.h"
#include "panelsrv.h"

#define PANELNETMS      33          // the shortest time between two lines
#define RESENDMS        1000        // an unchanged state is sent again after this time
#define HWCHECKMS       1000        // how often to look for the hardware panel's driver
#define USERTIMEOUTMS   3000        // data sent with no answer for this long closes the connection
#define ROWS            10
#define COLS            18
#define KEYBITS         0001777     // KEY_START ... KEY_FEED in sw2, the momentary keys

// H.S. CYCLE lights for only a few cycles at a time and would barely show, so as vpanel_pdp1 does,
// it is sent fully on when it lit at least this many cycles since the last line.
#define HSC_ROW         5
#define HSC_COL         14
#define HSC_STRETCH     30

// A P line: the fields, 360 hex digits and the newline.
#define LINEMAX         512

// The switch words' read-modify-write, between the connections.
static pthread_mutex_t swLock = PTHREAD_MUTEX_INITIALIZER;

// One connection's state.
typedef struct
{
    int fd;
    PDP1 *pdp;
    u16 lastCount[ROWS][COLS];
    u64 lastCycle;
    int based;                  // lastCount/lastCycle hold a read
    u8 lamps[ROWS * COLS];      // the last brightness made
    int mirror;
    int held;                   // the key bits this client holds down in sw2
    unsigned long ack;          // the last request's seq
    char in[LINEMAX];
    size_t have;
} Conn;

static u64
nowms(void)
{
struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return( ((u64)ts.tv_sec * 1000) + (ts.tv_nsec / 1000000) );
}

// Single aligned loads and stores, as the other panel programs make them.
static int
getSw(Panel *panelP, int i)
{
int *swP = &panelP->sw0;

    return( __atomic_load_n(&swP[i], __ATOMIC_RELAXED) );
}

static void
setSw(Panel *panelP, int i, int v)
{
int *swP = &panelP->sw0;

    __atomic_store_n(&swP[i], v, __ATOMIC_RELAXED);
}

// Takes the current counts as the base for the next differences.
static void
rebase(Conn *cP, Panel *panelP)
{
    memcpy(cP->lastCount, panelP->pwmcount, sizeof(cP->lastCount));
    cP->lastCycle = __atomic_load_n(&panelP->cyclecount, __ATOMIC_RELAXED);
    cP->based = 1;
}

// Turns the count differences since the last call into a brightness, 0-255, in cP->lamps.
// They are kept when no emulated time has passed, and all dark with POWER off, when pdp1 stops counting.
static void
readLamps(Conn *cP, Panel *panelP)
{
u64 now, elapsed, on;
u16 cnt;
int r, c;

    if( !cP->based || !(getSw(panelP, 0) & SW_POWER) )
    {
        rebase(cP, panelP);
        memset(cP->lamps, 0, sizeof(cP->lamps));
        return;
    }

    now = __atomic_load_n(&panelP->cyclecount, __ATOMIC_RELAXED);
    elapsed = now - cP->lastCycle;
    if( elapsed == 0 )
    {
        return;
    }

    if( elapsed > 65535 )
    {
        // The u16 counts may have wrapped more than once, their differences mean nothing.
        rebase(cP, panelP);
        return;
    }

    cP->lastCycle = now;
    for(r = 0; r < ROWS; r++)
    {
        for(c = 0; c < COLS; c++)
        {
            cnt = panelP->pwmcount[r][c];
            on = (u16)(cnt - cP->lastCount[r][c]);
            cP->lastCount[r][c] = cnt;
            if( on > elapsed )
            {
                on = elapsed;
            }
            if( (r == HSC_ROW) && (c == HSC_COL) && (on >= HSC_STRETCH) )
            {
                on = elapsed;
            }
            cP->lamps[(r * COLS) + c] = (u8)(((on * 510) + elapsed) / (2 * elapsed));
        }
    }
}

// Applies one S line, see the top of this file.
static void
request(Conn *cP, char *lineP)
{
unsigned long seq;
unsigned want[3], base[3];
int changed, cur, i, mirror;
Panel *panelP;

    if( sscanf(lineP, "S %lu %o %o %o %o %o %o", &seq, &want[0], &want[1], &want[2],
               &base[0], &base[1], &base[2]) != 7 )
    {
        return;
    }
    cP->ack = seq;      // answered whether or not it is applied

    mirror = hwPanelRunning();
    if( mirror != cP->mirror )
    {
        cP->held = 0;   // whichever had the keys, the other writes sw2 now
    }
    cP->mirror = mirror;

    panelP = cP->pdp->panel;
    if( !panelP || mirror )
    {
        return;
    }

    pthread_mutex_lock(&swLock);
    for(i = 0; i < 3; i++)
    {
        changed = (int)(want[i] ^ base[i]);
        if( !changed )
        {
            continue;
        }
        cur = getSw(panelP, i);
        setSw(panelP, i, (cur & ~changed) | ((int)want[i] & changed));
        if( i == 2 )
        {
            cP->held = (cP->held & ~(changed & KEYBITS)) | ((int)want[2] & changed & KEYBITS);
        }
    }
    pthread_mutex_unlock(&swLock);
}

// Reads what the client sent and applies each whole line.
// Returns 0, or -1 when the client has gone.
static int
readRequests(Conn *cP)
{
ssize_t n;
char *startP, *nlP;

    n = read(cP->fd, cP->in + cP->have, sizeof(cP->in) - 1 - cP->have);
    if( n <= 0 )
    {
        return( ((n < 0) && ((errno == EINTR) || (errno == EAGAIN))) ? 0 : -1 );
    }
    cP->have += n;
    cP->in[cP->have] = '\0';

    startP = cP->in;
    while( (nlP = strchr(startP, '\n')) )
    {
        *nlP = '\0';
        request(cP, startP);
        startP = nlP + 1;
    }

    cP->have -= (startP - cP->in);
    memmove(cP->in, startP, cP->have);
    if( cP->have >= (sizeof(cP->in) - 1) )
    {
        cP->have = 0;   // a line this long is not a request
    }
    return( 0 );
}

// Makes the P line for the current state in lineP.
static void
makeLine(Conn *cP, char *lineP)
{
static const char hex[] = "0123456789abcdef";
Panel *panelP;
int sw[4] = { 0, 0, 0, 0 };
int i, n;

    panelP = cP->pdp->panel;
    if( panelP )
    {
        readLamps(cP, panelP);
        for(i = 0; i < 4; i++)
        {
            sw[i] = getSw(panelP, i);
        }
    }
    else
    {
        memset(cP->lamps, 0, sizeof(cP->lamps));
    }

    n = snprintf(lineP, LINEMAX, "P %d %d %lu %o %o %o %o ", (panelP != nil), cP->mirror, cP->ack,
                 sw[0], sw[1], sw[2], sw[3]);
    for(i = 0; i < (ROWS * COLS); i++)
    {
        lineP[n++] = hex[cP->lamps[i] >> 4];
        lineP[n++] = hex[cP->lamps[i] & 0xf];
    }
    lineP[n++] = '\n';
    lineP[n] = '\0';
}

// Writes all of the line. Returns 0, or -1 when the client has gone.
static int
sendLine(int fd, const char *lineP)
{
size_t len, done;
ssize_t n;

    len = strlen(lineP);
    for(done = 0; done < len; done += n)
    {
        n = send(fd, lineP + done, len - done, MSG_NOSIGNAL);
        if( n < 0 )
        {
            if( errno == EINTR )
            {
                n = 0;
                continue;
            }
            return( -1 );
        }
    }
    return( 0 );
}

// Serves one connection until the client goes, on a thread of its own (serveN()).
// arg is the PDP1, whose panel is set when the emulator starts to run.
void
handlepanelnet(int fd, void *arg)
{
Conn *cP;
struct pollfd pfd;
char line[LINEMAX], last[LINEMAX];
u64 now, nextTick, lastSent, lastCheck;
int wait, n, mirror;
unsigned int userTimeout = USERTIMEOUTMS;

    if( !(cP = calloc(1, sizeof(*cP))) )
    {
        close(fd);
        return;
    }
    cP->fd = fd;
    cP->pdp = (PDP1*)arg;
    cP->mirror = hwPanelRunning();
    nodelay(fd);

    // Without it, a send() to a client that has gone blocks for the TCP retransmission timeout,
    // about 15 minutes, and the keys it held stay held all that time.
    setsockopt(fd, IPPROTO_TCP, TCP_USER_TIMEOUT, &userTimeout, sizeof(userTimeout));

    pfd.fd = fd;
    pfd.events = POLLIN;
    last[0] = '\0';
    lastSent = 0;
    lastCheck = nowms();
    nextTick = lastCheck;

    for(;;)
    {
        now = nowms();
        wait = (nextTick > now) ? (int)(nextTick - now) : 0;
        n = poll(&pfd, 1, wait);
        if( (n < 0) && (errno != EINTR) )
        {
            break;
        }
        if( (n > 0) && (pfd.revents & (POLLIN | POLLHUP | POLLERR)) && (readRequests(cP) < 0) )
        {
            break;
        }

        now = nowms();
        if( now < nextTick )
        {
            continue;
        }
        nextTick = now + PANELNETMS;

        if( (now - lastCheck) >= HWCHECKMS )
        {
            mirror = hwPanelRunning();
            if( mirror != cP->mirror )
            {
                cP->held = 0;
            }
            cP->mirror = mirror;
            lastCheck = now;
        }

        makeLine(cP, line);
        if( strcmp(line, last) || ((now - lastSent) >= RESENDMS) )
        {
            if( sendLine(fd, line) < 0 )
            {
                break;
            }
            strcpy(last, line);
            lastSent = now;
        }
    }

    // Let go of the keys the client held.
    if( cP->held && !cP->mirror && cP->pdp->panel )
    {
        pthread_mutex_lock(&swLock);
        setSw(cP->pdp->panel, 2, getSw(cP->pdp->panel, 2) & ~cP->held);
        pthread_mutex_unlock(&swLock);
    }

    close(fd);
    free(cP);
}
