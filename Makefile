# djcmd Makefile
# Supports: make powerpc, make x86_64, make i686, make rpi4, make aarch64

CC      = cc
TARGET  = djcmd

# ── Architecture-specific flags ──────────────────────────────────────
PPC_TUNE  = -mcpu=7450 -mtune=7450 -ffast-math -funroll-loops -fomit-frame-pointer
G3_TUNE   = -mcpu=750 -mtune=750 -ffast-math -fomit-frame-pointer
X86_TUNE  = -march=native -mtune=native -ffast-math -funroll-loops -fomit-frame-pointer
P3_TUNE   = -march=pentium3 -mtune=pentium3 -msse -ffast-math -fomit-frame-pointer
I686_TUNE = -march=pentium4 -mtune=pentium4 -ffast-math -mfpmath=sse -funroll-loops
LEGACY_TUNE = -march=i686 -Os -fomit-frame-pointer
# Raspberry Pi 4 (cortex-a72); armhf = 32-bit Raspbian, aarch64 = 64-bit Raspberry Pi OS
RPI4_TUNE   = -march=armv8-a -mtune=cortex-a72 -mfpu=neon-fp-armv8 \
              -mfloat-abi=hard -ffast-math -funroll-loops -fomit-frame-pointer
AARCH64_TUNE = -march=armv8-a -mtune=cortex-a72 \
               -ffast-math -funroll-loops -fomit-frame-pointer
# Raspbian splits ncurses wide-char support into libncursesw (unlike Arch)
RPI_LIBS = -lasound -lpthread -lm -lncursesw -lsqlite3

OPT_FLAGS  = -O2 -g
WARN_FLAGS = -Wall -Wextra -Wno-unused-parameter -Wno-sign-compare \
            -Wno-unused-function -Wno-unused-variable

# ── Libraries ────────────────────────────────────────────────────────
LIBS = -lasound -lpthread -lm -lncurses -lsqlite3

CFLAGS  += $(OPT_FLAGS) $(WARN_FLAGS)
LDFLAGS = $(LIBS)

SRCS = djcmd.c djcmd_audio.c djcmd_fx.c djcmd_help.c audiofile.c
HDRS = audiofile.h djcmd_audio.h djcmd_config.h djcmd_fx.h djcmd_help.h \
       ns7iii_map.h dr_flac.h minimp3.h

.PHONY: all clean install deps check-deps check-headers powerpc x86_64 i686 g3 p3 legacy rpi4 aarch64

# Default target (Arch Linux POWER)
all: powerpc

powerpc: CFLAGS += $(PPC_TUNE)
powerpc: $(TARGET)

g3: CFLAGS += $(G3_TUNE)
g3: $(TARGET)

x86_64: CFLAGS += $(X86_TUNE)
x86_64: $(TARGET)

i686: CFLAGS += $(I686_TUNE)
i686: $(TARGET)

p3: CFLAGS += $(P3_TUNE)
p3: $(TARGET)

legacy: CFLAGS += $(LEGACY_TUNE)
legacy: $(TARGET)

rpi4: LIBS = $(RPI_LIBS)
rpi4: CFLAGS += $(RPI4_TUNE)
rpi4: $(TARGET)

aarch64: LIBS = $(RPI_LIBS)
aarch64: CFLAGS += $(AARCH64_TUNE)
aarch64: $(TARGET)

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

# Debug build (no optimizations)
debug: $(SRCS) $(HDRS)
	$(CC) -O0 -g3 $(WARN_FLAGS) -o djcmd_debug $(SRCS) $(LIBS)
	@echo "Debug build → djcmd_debug"

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
