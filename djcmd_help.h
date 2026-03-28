/*
 * djcmd_help.h — Help view interface for djcmd
 * Copyright (C) 2025  djcmd contributors
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
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
