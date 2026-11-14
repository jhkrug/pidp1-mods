// The front panel in the browser.
//
// The emulator, pdp1, shares its panel state in /tmp/pdp1_panel, the Panel struct in
// src/common/panel_pidp1.h, with the panel drivers: newpanel for the PiDP-1 hardware, vpanel_pdp1 on screen.
// This reads it like they do and sends the browser, about 30 times a second while one is connected:
//
//	{"type":"panel", "lamps":<180 brightness bytes, 10 rows of 18>, "sw":[sw0,sw1,sw2,sw3],
//	 "mirror":<true while the hardware panel's driver runs>, "present":<true once the segment exists>}
//
// The browser sends {"type":"panel_sw", "sw":[sw0,sw1,sw2], "base":[sw0,sw1,sw2]} when a switch or key changes:
// base is what it last showed, sw what it wants. Only the bits that differ are changed in the segment,
// so a setting another program changed meanwhile is never overwritten. That matters for POWER: pdp1 power
// clears, resetting the machine, if it sees POWER off even briefly.
// Nothing is written while mirroring: the hardware panel's driver owns the switches then.
//
// The lamp counts, pwmcount[][], wrap and every reader takes differences, so this can read them alongside
// newpanel and vpanel_pdp1.
//
// 13-Nov-2026 Claude: initial version
// 14-Nov-2026 Claude: H.S. CYCLE stretched as in vpanel_pdp1

package main

import (
	"bytes"
	"io/ioutil"
	"log"
	"os"
	"path/filepath"
	"strings"
	"sync"
	"sync/atomic"
	"syscall"
	"time"
	"unsafe"
)

// The Panel struct's layout, src/common/panel_pidp1.h: 15 ints, u16 pwmcount[10][18], u64 cyclecount.
const (
	panelPath     = "/tmp/pdp1_panel"
	panelSize     = 432
	offSw         = 0   // sw0-sw3, 4 ints
	offPwm        = 60  // pwmcount[10][18], u16
	offCycle      = 424 // cyclecount, u64
	panelRows     = 10
	panelCols     = 18
	swPower       = 0200000 // SW_POWER in sw0
	keyBits       = 0001777 // KEY_START ... KEY_FEED in sw2, the momentary keys
	panelInterval = 33 * time.Millisecond
	hwCheckEvery  = time.Second

	// H.S. CYCLE lights for only a few cycles at a time and would barely show, so as vpanel_pdp1 does,
	// it is sent fully on when it lit at least this many cycles since the last read.
	hscRow, hscCol, hscStretch = 5, 14, 30
)

// The hardware panel driver's process names, as vpanel_pdp1 looks for them.
var hwPanelNames = []string{"panel_pidp1", "newpanel"}

type Panel struct {
	mu        sync.Mutex
	mem       []byte // the mapped segment, nil until it exists
	ino       uint64 // its inode, to notice it being made again
	lastCount [panelRows][panelCols]uint16
	lastCycle uint64
	lamps     []byte // last brightness sent
	mirror    bool
	held      int32 // key bits this browser is holding down in sw2
}

// open maps the segment if it exists and isn't mapped yet, or was made again since.
// It never makes it: pdp1 and the panel drivers do that.
func (p *Panel) open() {
	fi, err := os.Stat(panelPath)
	if err != nil || fi.Size() < panelSize {
		p.close()
		return
	}
	ino := fi.Sys().(*syscall.Stat_t).Ino
	if p.mem != nil && ino == p.ino {
		return
	}
	p.close()
	f, err := os.OpenFile(panelPath, os.O_RDWR, 0)
	if err != nil {
		log.Printf("panel: %v", err)
		return
	}
	defer f.Close()
	mem, err := syscall.Mmap(int(f.Fd()), 0, panelSize, syscall.PROT_READ|syscall.PROT_WRITE, syscall.MAP_SHARED)
	if err != nil {
		log.Printf("panel: mmap: %v", err)
		return
	}
	p.mem, p.ino = mem, ino
	p.rebase()
	log.Printf("panel: attached %s", panelPath)
}

func (p *Panel) close() {
	if p.mem != nil {
		syscall.Munmap(p.mem)
		p.mem = nil
	}
}

// Single aligned loads and stores, as the C programs make them, so no reader sees a half-written word.
func (p *Panel) sw(i int) int32 {
	return atomic.LoadInt32((*int32)(unsafe.Pointer(&p.mem[offSw+4*i])))
}

func (p *Panel) setSw(i int, v int32) {
	atomic.StoreInt32((*int32)(unsafe.Pointer(&p.mem[offSw+4*i])), v)
}

func (p *Panel) count(r, c int) uint16 {
	return *(*uint16)(unsafe.Pointer(&p.mem[offPwm+2*(r*panelCols+c)]))
}

func (p *Panel) cycles() uint64 {
	return atomic.LoadUint64((*uint64)(unsafe.Pointer(&p.mem[offCycle])))
}

// rebase takes the current counts as the base for the next differences.
func (p *Panel) rebase() {
	for r := 0; r < panelRows; r++ {
		for c := 0; c < panelCols; c++ {
			p.lastCount[r][c] = p.count(r, c)
		}
	}
	p.lastCycle = p.cycles()
}

// readLamps turns the count differences since the last call into a brightness, 0-255, per lamp.
// It returns the last brightness when no emulated time has passed, and all dark with POWER off,
// when pdp1 stops counting.
func (p *Panel) readLamps() []byte {
	lamps := make([]byte, panelRows*panelCols)
	if p.sw(0)&swPower == 0 {
		p.rebase()
		return lamps
	}
	now := p.cycles()
	elapsed := now - p.lastCycle
	if elapsed == 0 {
		if p.lamps != nil {
			return p.lamps
		}
		return lamps
	}
	if elapsed > 65535 {
		// The u16 counts may have wrapped more than once, their differences mean nothing.
		p.rebase()
		if p.lamps != nil {
			return p.lamps
		}
		return lamps
	}
	p.lastCycle = now
	for r := 0; r < panelRows; r++ {
		for c := 0; c < panelCols; c++ {
			cnt := p.count(r, c)
			on := uint64(cnt - p.lastCount[r][c])
			p.lastCount[r][c] = cnt
			if on > elapsed {
				on = elapsed
			}
			if r == hscRow && c == hscCol && on >= hscStretch {
				on = elapsed
			}
			lamps[r*panelCols+c] = byte(on * 255 / elapsed)
		}
	}
	return lamps
}

// hwPanelRunning reports whether the hardware panel's driver is running, matching /proc/<pid>/comm exactly.
func hwPanelRunning() bool {
	comms, _ := filepath.Glob("/proc/[0-9]*/comm")
	for _, f := range comms {
		b, err := ioutil.ReadFile(f)
		if err != nil {
			continue // it ended while we looked
		}
		name := strings.TrimSpace(string(b))
		for _, n := range hwPanelNames {
			if name == n {
				return true
			}
		}
	}
	return false
}

// request applies a browser's switch change: only the bits that differ between base and want,
// on top of the segment's current words, so nothing another program changed is overwritten.
func (p *Panel) request(want, base []int32) {
	p.mu.Lock()
	defer p.mu.Unlock()
	if p.mem == nil || p.mirror || len(want) < 3 || len(base) < 3 {
		return
	}
	for i := 0; i < 3; i++ {
		changed := want[i] ^ base[i]
		if changed == 0 {
			continue
		}
		cur := p.sw(i)
		p.setSw(i, (cur&^changed)|(want[i]&changed))
		if i == 2 {
			p.held = (p.held &^ (changed & keyBits)) | (want[2] & changed & keyBits)
		}
	}
}

// releaseKeys lets go of any keys the browser held when it went away.
func (p *Panel) releaseKeys() {
	p.mu.Lock()
	defer p.mu.Unlock()
	if p.mem != nil && p.held != 0 && !p.mirror {
		p.setSw(2, p.sw(2)&^p.held)
	}
	p.held = 0
}

// handlePanel sends the panel state to the browser while one is connected.
func (s *PeriphServer) handlePanel() {
	p := s.panel
	tick := time.NewTicker(panelInterval)
	defer tick.Stop()
	var lastCheck time.Time
	var lastSent time.Time
	var lastSw [4]int32
	var lastLamps []byte
	var lastMirror, lastPresent bool

	for range tick.C {
		if !s.webConnected() {
			continue
		}

		p.mu.Lock()
		if time.Since(lastCheck) >= hwCheckEvery {
			p.open()
			mirror := hwPanelRunning()
			if mirror != p.mirror {
				p.held = 0 // whichever had the keys, the other writes sw2 now
			}
			p.mirror = mirror
			lastCheck = time.Now()
		}

		msg := Message{Type: "panel", Present: p.mem != nil, Mirror: p.mirror}
		var sw [4]int32
		if p.mem != nil {
			p.lamps = p.readLamps()
			for i := 0; i < 4; i++ {
				sw[i] = p.sw(i)
			}
			msg.Lamps = p.lamps
			msg.Sw = sw[:]
		}
		p.mu.Unlock()

		// Send changes at once, and the unchanged state once a second so a new page fills in.
		changed := msg.Present != lastPresent || msg.Mirror != lastMirror || sw != lastSw ||
			!bytes.Equal(msg.Lamps, lastLamps)
		if changed || time.Since(lastSent) >= time.Second {
			s.sendToWeb(msg)
			lastSent = time.Now()
			lastSw, lastLamps, lastMirror, lastPresent = sw, msg.Lamps, msg.Mirror, msg.Present
		}
	}
}
