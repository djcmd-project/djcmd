# Windows Makefile for djcmd — MinGW-w64 cross-compile or native Windows
#
# Prerequisites (install via MSYS2/pacman or download manually):
#   pacman -S mingw-w64-x86_64-portaudio
#   pacman -S mingw-w64-x86_64-pdcurses
#   pacman -S mingw-w64-x86_64-sqlite3
#   pacman -S mingw-w64-x86_64-pthreads
#
# Then from this directory:
#   make
#
# Or cross-compile from Linux:
#   make CC=x86_64-w64-mingw32-gcc

CC      = gcc
TARGET  = djcmd.exe

# ── Compiler flags ────────────────────────────────────────────────────
OPT_FLAGS  = -O2 -g
WARN_FLAGS = -Wall -Wextra -Wno-unused-parameter -Wno-sign-compare \
             -Wno-unused-function -Wno-unused-variable

# Tell djcmd to use the Windows compatibility layer
WIN_FLAGS  = -D_WIN32_WINNT=0x0600 -DDJCMD_WINDOWS

CFLAGS  = $(OPT_FLAGS) $(WARN_FLAGS) $(WIN_FLAGS) -I. -I..

# ── Libraries ────────────────────────────────────────────────────────
# PortAudio: -lportaudio (replaces ALSA -lasound)
# WinMM:     -lwinmm    (replaces ALSA rawmidi)
# PDCurses:  -lpdcurses  (replaces -lncurses)
# pthreads:  -lpthread   (pthreads-win32, same API)
# SQLite3:   -lsqlite3
LIBS = -lportaudio -lwinmm -lpdcurses -lpthread -lsqlite3 -lm -lws2_32

LDFLAGS = $(LIBS)

# ── Source files ─────────────────────────────────────────────────────
# Copy djcmd.c and audiofile.c from parent directory; Windows-patched
# versions live here.
SRCS = djcmd_win32.c audiofile.c
OBJS = $(SRCS:.c=.o)

.PHONY: all clean deps check-deps setup-sources

all: setup-sources $(TARGET)

# Copy source files from parent if Windows-patched copies don't exist yet
setup-sources:
	@test -f audiofile.c  || cp ../audiofile.c  .
	@test -f audiofile.h  || cp ../audiofile.h  .
	@test -f minimp3.h    || cp ../minimp3.h    . 2>/dev/null || true
	@test -f dr_flac.h    || cp ../dr_flac.h    . 2>/dev/null || true
	@test -f djcmd_config.h || cp ../djcmd_config.h . 2>/dev/null || true
	@test -f ns7iii_map.h   || cp ../ns7iii_map.h   . 2>/dev/null || true

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)
	@echo "Build OK → $(TARGET)"

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

djcmd_win32.o: djcmd_win32.c audiofile.h platform_win32.h audio_win32.h midi_win32.h
audiofile.o:   audiofile.c audiofile.h minimp3.h dr_flac.h

clean:
	rm -f $(TARGET) $(OBJS) audiofile.c audiofile.h minimp3.h dr_flac.h \
	      djcmd_config.h ns7iii_map.h

# ── Dependency check ─────────────────────────────────────────────────
check-deps:
	@pkg-config --exists portaudio-2.0 && echo "portaudio  OK" || echo "portaudio  MISSING"
	@pkg-config --exists pdcurses      && echo "pdcurses   OK" || echo "pdcurses   MISSING (try 'pacman -S mingw-w64-x86_64-pdcurses')"
	@pkg-config --exists sqlite3       && echo "sqlite3    OK" || echo "sqlite3    MISSING"
	@echo "winmm: built-in to Windows"
	@echo "pthreads: check for pthread.h in MinGW include path"

# ── Download single-header deps ──────────────────────────────────────
deps: minimp3.h dr_flac.h

minimp3.h:
	curl -fsSL https://raw.githubusercontent.com/lieff/minimp3/master/minimp3.h -o minimp3.h

dr_flac.h:
	curl -fsSL https://raw.githubusercontent.com/mackron/dr_libs/master/dr_flac.h -o dr_flac.h
