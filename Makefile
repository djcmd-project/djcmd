# djcmd Makefile 
# Supports: make powerpc, make x86_64

CC      = cc
TARGET  = djcmd

# ── Architecture-specific flags ──────────────────────────────────────
PPC_TUNE = -mcpu=7450 -mtune=7450 -ffast-math -funroll-loops -fomit-frame-pointer
X86_TUNE = -march=native -mtune=native -ffast-math -funroll-loops -fomit-frame-pointer

OPT_FLAGS  = -O2 -g
WARN_FLAGS = -Wall -Wextra -Wno-unused-parameter -Wno-sign-compare \
            -Wno-unused-function -Wno-unused-variable

# ── Libraries ────────────────────────────────────────────────────────
LIBS = -lasound -lpthread -lm -lncurses -lsqlite3

CFLAGS  += $(OPT_FLAGS) $(WARN_FLAGS)
LDFLAGS = $(LIBS)

SRCS = djcmd.c djcmd_audio.c djcmd_fx.c djcmd_help.c audiofile.c
HDRS = audiofile.h djcmd_audio.h djcmd_config.h djcmd_fx.h djcmd_help.h \
       ns7iii_map.h ns7iii_displaysub.h dr_flac.h minimp3.h

.PHONY: all clean install deps check-deps check-headers powerpc x86_64

# Default target (PowerPC)
all: powerpc

powerpc: CFLAGS += $(PPC_TUNE)
powerpc: $(TARGET)

x86_64: CFLAGS += $(X86_TUNE)
x86_64: $(TARGET)

$(TARGET): check-headers $(SRCS) $(HDRS)
	$(CC) $(CFLAGS) -o $@ $(SRCS) $(LDFLAGS)
	@echo "Build OK → $(TARGET)"
	@size $(TARGET)

# ── Download single-header libs (run once) ───────────────────────────
deps: minimp3.h dr_flac.h

minimp3.h:
	curl -fsSL \
	  https://raw.githubusercontent.com/lieff/minimp3/master/minimp3.h \
	  -o minimp3.h
	@echo "minimp3.h downloaded"

dr_flac.h:
	curl -fsSL \
	  https://raw.githubusercontent.com/mackron/dr_libs/master/dr_flac.h \
	  -o dr_flac.h
	@echo "dr_flac.h downloaded"

# ── Pre-build sanity checks ──────────────────────────────────────────
check-headers:
	@test -f minimp3.h || (echo "ERROR: minimp3.h missing — run: make deps" && exit 1)
	@test -f dr_flac.h || (echo "ERROR: dr_flac.h missing — run: make deps" && exit 1)

install: $(TARGET)
	install -m 755 $(TARGET) /usr/local/bin/

run: $(TARGET)
	./$(TARGET)

# AddressSanitizer build
asan: $(SRCS) $(HDRS)
	$(CC) -O1 -g -fsanitize=address -fno-omit-frame-pointer \
	    $(WARN_FLAGS) -o djcmd_asan $(SRCS) $(LIBS)
	@echo "ASAN build → djcmd_asan"

clean:
	rm -f $(TARGET) djcmd_asan *.o

# ── Dependency status ────────────────────────────────────────────────
check-deps:
	@pkg-config --exists alsa    && echo "alsa      OK" || echo "alsa      MISSING (pacman -S alsa-lib)"
	@pkg-config --exists ncurses && echo "ncurses   OK" || echo "ncurses   MISSING (pacman -S ncurses)"
	@pkg-config --exists sqlite3 && echo "sqlite3   OK" || echo "sqlite3   MISSING (pacman -S sqlite)"
	@which curl > /dev/null      && echo "curl      OK" || echo "curl      MISSING (pacman -S curl)"
	@echo "pthread:  always available in glibc"
	@echo ""
	@echo "Beat detection: sourced from ~/.mixxx/mixxxdb.sqlite"
	@echo "  Analyse your tracks in Mixxx once, then djcmd reads BPM + beatgrid directly."
	@echo "  Tracks not in Mixxx fall back to onset detection (120 BPM default)."
