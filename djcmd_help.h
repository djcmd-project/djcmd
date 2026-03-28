/*
 * djcmd_help.h — Help view interface for djcmd
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

#ifndef DJCMD_HELP_H
#define DJCMD_HELP_H

#include <ncurses.h>

/* ── ncurses colour pair IDs ──────────────────────────────────────────────── */
/* These must match the init_pair() calls in djcmd.c's theme setup.          */
#define COLOR_HEADER 1
#define COLOR_ACTIVE 2
#define COLOR_VU     3
#define COLOR_WFM    4
#define COLOR_STATUS 5
#define COLOR_HOT    6

/* ── Globals owned by djcmd.c ─────────────────────────────────────────────── */
extern WINDOW *g_win_main;
extern int g_rows;
extern int g_help_scroll;

/* ── Function prototype ───────────────────────────────────────────────────── */
void draw_help_view(void);

#endif /* DJCMD_HELP_H */
