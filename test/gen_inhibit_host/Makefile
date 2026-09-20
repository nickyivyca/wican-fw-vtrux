# Host build of gen_inhibit_core. No ESP-IDF, no hardware.
#
# -Wconversion is on deliberately: the core mixes uint8/uint32/int32/int64 and
# an implicit narrowing is exactly the kind of bug a replay trace would show
# as a plausible-looking wrong number rather than as a crash.

CC      ?= gcc
CFLAGS  ?= -std=c11 -O2 -g -Wall -Wextra -Wpedantic -Wconversion -Wshadow \
           -Wstrict-prototypes -Wmissing-prototypes
COREDIR ?= ../../main
INCS    := -I$(COREDIR)

BIN := host_runner extract_probe

all: $(BIN)

host_runner: host_runner.c $(COREDIR)/gen_inhibit_core.c $(COREDIR)/gen_inhibit_core.h
	$(CC) $(CFLAGS) $(INCS) -o $@ host_runner.c $(COREDIR)/gen_inhibit_core.c

extract_probe: extract_probe.c $(COREDIR)/gen_inhibit_core.c $(COREDIR)/gen_inhibit_core.h
	$(CC) $(CFLAGS) $(INCS) -o $@ extract_probe.c $(COREDIR)/gen_inhibit_core.c

# A sanitiser build catches what -O2 hides. Same sources, different binary.
asan: CFLAGS += -fsanitize=address,undefined -fno-omit-frame-pointer -O1
asan: clean $(BIN)

clean:
	rm -f $(BIN) *.o

.PHONY: all asan clean
