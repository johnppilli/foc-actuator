# Desktop build: FOC library + unit tests + closed-loop simulator.
# The firmware has its own Makefile in firmware/ (arm-none-eabi-gcc).
#
#   make            build everything
#   make test       build and run all unit tests
#   make sim        build the simulator
#   make plots      run the simulator scenarios and render PNGs into build/
#   make clean

CC       ?= cc
PYTHON   ?= python3
CFLAGS   ?= -std=c11 -O2 -g -Wall -Wextra -Wshadow -Wdouble-promotion -Werror
CPPFLAGS := -Ifoc -Isim
LDLIBS   := -lm
BUILD    := build

LIB_SRC  := $(wildcard foc/*.c)
SIM_SRC  := sim/motor_sim.c
LIB_OBJ  := $(patsubst %.c,$(BUILD)/%.o,$(LIB_SRC) $(SIM_SRC))

TEST_SRC := $(wildcard tests/test_*.c)
TEST_BIN := $(patsubst tests/%.c,$(BUILD)/%,$(TEST_SRC))

.PHONY: all test sim plots sim-data clean

all: $(TEST_BIN) $(BUILD)/focsim

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

$(BUILD)/test_%: tests/test_%.c $(LIB_OBJ)
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(CPPFLAGS) $^ -o $@ $(LDLIBS)

$(BUILD)/focsim: sim/sim_main.c $(LIB_OBJ)
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(CPPFLAGS) $^ -o $@ $(LDLIBS)

sim: $(BUILD)/focsim

test: $(TEST_BIN)
	@fail=0; for t in $(TEST_BIN); do ./$$t || fail=1; done; \
	if [ $$fail -ne 0 ]; then echo "TESTS FAILED"; exit 1; fi; \
	echo "ALL TESTS PASSED"

SCENARIOS := step step-free load calib haptic-spring haptic-detents haptic-endstops openloop haptic-curves
SIM_CSV   := $(patsubst %,$(BUILD)/%.csv,$(SCENARIOS))

$(BUILD)/%.csv: $(BUILD)/focsim
	./$(BUILD)/focsim $* > $@

sim-data: $(SIM_CSV)

plots: sim-data
	$(PYTHON) tools/plot_sim.py $(BUILD)/step.csv $(BUILD)/step-free.csv $(BUILD)/load.csv \
	    $(BUILD)/calib.csv $(BUILD)/haptic-spring.csv $(BUILD)/haptic-detents.csv \
	    $(BUILD)/haptic-endstops.csv $(BUILD)/openloop.csv --outdir $(BUILD)
	$(PYTHON) tools/plot_haptic.py $(BUILD)/haptic-curves.csv --out $(BUILD)/haptic-curves.png

clean:
	rm -rf $(BUILD)
