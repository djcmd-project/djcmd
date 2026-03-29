/*
 * djcmd_help.c -- Help view for djcmd
 * Copyright (C) 2025  djcmd contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "djcmd_help.h"

void draw_help_view(void)
{
	static const char *lines[] = {
		" djcmd \u2014 Key Bindings  (j/k or PgDn/PgUp to scroll,  ? or ESC to close)",
		"",
		" DECK SELECTION & CUE",
		"   1 2 3 4       Select active deck (A B C D)",
		"   5 6 7 8       Toggle headphone CUE (PFL) for deck A B C D",
		"   T             Toggle 2 / 4 deck mode",
		"",
		" PLAYBACK  (gang-aware when Gang mode is on)",
		"   SPACE         Play / pause",
		"   s             Stop",
		"   Ctrl+T        TAP BPM (active deck \u2014 hit on the beat)",
		"   S             Snap Grid (set first beat to current playhead)",
		"   r             Restart from beginning",
		"   l             Toggle loop between cue 1 and cue 2",
		"   b             Re-analyse BPM via autocorrelation (background, saves cache)",
		"   B             Enter BPM manually  \u2014 type value, Enter=confirm, Esc=cancel",
		"   H             Cycle BPM display: normal \u2192 \u00d72 \u2192 \u00bd \u2192 normal  (beat ruler follows; playback unchanged)",
		"   K             Toggle key lock (master tempo / WSOLA time-stretch)",
		"",
		" NAVIGATIONAL BEAT JUMPS",
		"   LEFT / RIGHT  Seek \u00b11 beat",
		"   S-L / S-R     Seek \u00b18 beats  (2 bars)",
		"   UP / DOWN     Seek \u00b116 beats (4 bars)  [Note: may require Shift+Arrow]",
		"                 Jumping is quantized and resets time-stretcher for clarity.",
		"",
		" PERFORMANCE FEATURES",
		"   Instant Doubles  Loading a track already playing elsewhere clones its position,",
		"                    pitch, and loop state exactly.",
		"   Musical Key      Key info (8A, Cm, etc.) imported from Mixxx and shown in header.",
		"   Per-Deck VU      Lightweight LED in header: - (low), # (good), ! (clipping).",
		"",
		" VISUAL AIDS",
		"   Phase Meter   Shows drift between master and active deck in deck header:",
		"                 [ <<<<:     ] lagging,  [     :>>>> ] rushing.",
		"   XFader Bar    Horizontal fader position shown in the status line.",
		"",
		" LOOPS  (waveform shows [ and ] markers + length while active)",
		"   MIDI loop_in_a/b    Set loop start point",
		"   MIDI loop_out_a/b   Set loop end + engage loop (press again to exit)",
		"   MIDI loop_toggle_a/b  Toggle loop on/off",
		"   MIDI loop_half_a/b  Halve loop length",
		"   MIDI loop_double_a/b  Double loop length",
		"   Autoloop pads (AUTOROLL mode): beat-quantised 1/2/4/8 bar loops",
		"   Roll pads: hold=temporary loop, release=resume from saved position",
		"",
		" SYNC & BEAT GRID",
		"   M             Set active deck as sync MASTER",
		"   y             Toggle sync SLAVE  (BPM + phase lock to master)",
		"   ] / [         Pitch bend fwd / back  (accumulates, auto-decays)",
		"                 MIDI: use pitch_bend_a/b/c/d (relative encoder) for per-deck bend",
		"",
		" BEAT GRID ADJUSTMENT",
		"   } / {         Shift grid \u00b11 beat  (coarse: moves every bar marker)",
		"   ) / (         Shift grid \u00b1\u00bc beat  (fine: 16th-note precision)",
		"   Z             Reset grid to frame 0  (clears offset, keeps BPM, saves cache)",
		"",
		"   Workflow: use } / { to get within one beat, then ) / ( to fine-tune.",
		"   If autocorrelation is wrong, use B to type the correct BPM first,",
		"   then adjust the grid phase with } { ) (.",
		"",
		" SYNC OPTIONS  (ESC \u2192 SYNC tab)",
		"   Quantize play    : SPACE on a sync-locked deck waits for beat 1 of",
		"                      master's next bar  (\u23f3 WAIT shown in deck header).",
		"                      Cancel with SPACE again.",
		"   Smart BPM range  : folds BPM by octaves to prevent e.g. 90\u219190 not 180.",
		"   Auto master handoff: loading onto the master deck passes master to",
		"                      whichever deck is currently playing.",
		"   Vinyl mode        : motorised platters (NS7III) only.",
		"     ON  \u2014 playhead runs independently at pitch speed until you",
		"           touch the vinyl surface or platter rim.  Touch engages",
		"           1:1 scratch; release snaps back to normal speed instantly.",
		"     OFF \u2014 platter velocity always controls the audio (DVS mode).",
		"           Motor wow/flutter affects playback; stopping the platter",
		"           pauses audio even without a touch event.",
		"",
		" GANG MODE",
		"   G             Toggle gang mode on / off",
		"   F1\u2013F4         Toggle deck A\u2013D in / out of gang",
		"",
		" CUE POINTS  (imported from Mixxx if available)",
		"   F5\u2013F8         Set cue 1\u20134  (pads 1\u20138 set cues 1\u20138 in HOTCUE mode)",
		"   F9\u2013F12        Jump to cue 1\u20134  (pads 1\u20138 jump to cues 1\u20138)",
		"   MIDI cue_default_a/b  CUE transport button (standard CDJ behaviour):",
		"     While playing   : jump to master cue point and pause",
		"     While paused    : set master cue here, begin held playback",
		"     Release while held : stop and snap back to cue point",
		"",
		" PERFORMANCE PADS  (MIDI only \u2014 8 pads per deck)",
		"   Three pad modes selected by the PAD MODE button:",
		"   HOTCUE mode  : pads 1\u20138 set/jump to hot cue points",
		"                  SHIFT+pad = delete that cue point",
		"                  Cue LEDs light in unique colours per pad",
		"   AUTOROLL mode: pads 1\u20134 = 1/2/4/8 bar beat-aligned autoloops",
		"                  Toggle off by pressing the same pad again",
		"                  PARAM L/R buttons halve/double loop size",
		"   ROLL mode    : hold pad = temporary loop, release = resume",
		"",
		" PITCH / SPEED",
		"   e / d         Pitch +0.5% / \u22120.5%  (fine step, keyboard \u2014 ignores range setting)",
		"   E / D         Pitch +5%   / \u22125%    (coarse step, keyboard \u2014 ignores range setting)",
		"   0             Reset pitch to 0%  (also resets MIDI fader to center)",
		"   V             Cycle pitch range for active deck: \u00b18% \u2192 \u00b125% \u2192 \u00b150% \u2192 \u00b18%",
		"                 Range affects MIDI pitch fader only. Displayed in PITCH bar.",
		"   ] / [         Pitch bend fwd / back  (accumulates, auto-decays)",
		"",
		" VOLUME, EQ, CROSSFADER",
		"   + / -         Deck volume up / down",
		"   A             Toggle auto-gain (active deck)",
		"   m / n         MASTER volume up / down",
		"   q / a         EQ LOW  kill / boost",
		"   w / x         EQ MID  kill / boost",
		"   t / g         EQ HIGH kill / boost",
		"   < / >         Crossfade Left / Right",
		"",
		" BROWSER / PLAYLIST",
		"   j / k         Navigate up / down",
		"   ENTER         Enter directory / Load selected to active deck",
		"   BACKSPACE     Go up one directory",
		"   ! @ # $       Load highlighted track to Deck A / B / C / D",
		"   p             Add highlighted track to Playlist",
		"   i             MusicBrainz tag lookup (metadata fetch)",
		"   DEL           Remove highlighted entry from playlist",
		"   Ctrl+X        Clear entire playlist",
		"   ~             Jump to $HOME",
		"   \\             Jump to filesystem root /",
		"",
		" OPTIONS OVERLAY",
		"   ESC           Open / Close Options menu (or Close Help)",
		"   LEFT / RIGHT  Switch between tabs (INFO, AUDIO, MIDI, FX...)",
		"   j / k         Navigate rows within a tab",
		"   - / +         Adjust settings or toggle options",
		"   MIDI Tab      Learn and bind your controller (L=learn, S=save)",
		"",
		" SYSTEM",
		"   TAB           Cycle bottom panel (Browser \u2192 Playlist \u2192 Library)",
		"                 In 4-Deck mode: TAB toggles split view on / off",
		"   ?             Toggle this help view",
		"   Q             Quit djcmd (confirm with y)",
	};
	int n_lines = sizeof(lines) / sizeof(lines[0]);

	werase(g_win_main);
	wattron(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);
	box(g_win_main, 0, 0);
	mvwprintw(g_win_main, 0, 2, " djcmd HELP ");
	wattroff(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);

	int start = g_help_scroll;
	if (start < 0) start = 0;
	if (start > n_lines - (g_rows - 4)) start = n_lines - (g_rows - 4);
	if (start < 0) start = 0;
	g_help_scroll = start;

	for (int i = 0; i < (g_rows - 4) && (start + i) < n_lines; i++) {
		mvwprintw(g_win_main, i + 2, 2, "%s", lines[start + i]);
	}

	wrefresh(g_win_main);
}
