SDCC ?= sdcc
SDAS ?= sdas8051
PYTHON ?= python3

BUILD := build
MCU := $(BUILD)/mcu
HOST := $(BUILD)/host
FW := firmware
FW_INC := $(addprefix -I$(FW)/,include common app boot)
FW_HEADERS := $(filter-out $(FW)/tests/%,$(wildcard $(FW)/*/*.h))
ESP_HEADERS := $(wildcard esphome/components/*/*.h) $(FW_HEADERS)
MKESPHOME := $(FW)/tools/mkesphome.py

all: test

.DELETE_ON_ERROR:

# SDCC reports a failed static assert, and code overlapping an absolute area, only as a warning,
# and still succeeds.
fail_on_output = out=$$($(1) 2>&1); status=$$?; [ -z "$$out" ] || { echo "$$out"; status=1; }; exit $$status

SDCC_FLAGS := -mmcs51 --model-small --opt-code-size --std-c11 $(FW_INC)
# Files with functions called from interrupts: their locals must not share RAM with main-loop
# functions.
ISR_SRC := app/hal.c app/sched.c app/phase.c app/window.c

FW_CONSTANTS := $(BUILD)/fw_constants.mk
ifeq ($(filter clean,$(MAKECMDGOALS)),)
include $(FW_CONSTANTS)
endif
hex = $(shell printf '0x%04X' $$(( $(1) )))
SDCC_LINK = --iram-size 256 --xram-loc 0 --xram-size $(call hex,$(FWC_XRAM_SIZE))
# The linker's --code-size limits the number of code bytes, including absolute areas, not the end
# address. The application must end below its trailer. The bootloader starts after its vector
# table, whose bytes count too, as does its version byte at the end of its region, so its code can
# never reach that byte.
APP_CODE_LOC = $(call hex,$(FWC_APP_BASE))
APP_CODE_SIZE = $(call hex,$(FWC_APP_END) - $(FWC_TRAILER_SIZE) - $(FWC_APP_BASE))
BOOT_CODE_LOC = $(call hex,$(FWC_BOOT_CODE_BASE))
BOOT_CODE_SIZE = $(call hex,$(FWC_APP_BASE) - $(FWC_BOOT_CODE_BASE))

APP_SRC := app/main.c app/app.c app/hal.c app/sched.c app/phase.c app/gate.c app/safety.c app/window.c \
	app/status.c common/link.c common/crc16.c
BOOT_SRC := boot/boot_main.c boot/boot_hal.c boot/boot.c boot/update.c common/link.c common/crc16.c
APP_REL := $(patsubst %.c,$(MCU)/app/%.rel,$(APP_SRC))
BOOT_REL := $(patsubst %.c,$(MCU)/boot/%.rel,$(BOOT_SRC)) $(MCU)/boot/boot/vectors.rel

$(MCU)/app/%.rel: $(FW)/%.c $(FW_HEADERS) Makefile
	@mkdir -p $(@D)
	$(call fail_on_output,$(SDCC) $(SDCC_FLAGS) $(if $(filter $*.c,$(ISR_SRC)),--nooverlay) -c $< -o $@)

$(MCU)/boot/%.rel: $(FW)/%.c $(FW_HEADERS) Makefile
	@mkdir -p $(@D)
	$(call fail_on_output,$(SDCC) $(SDCC_FLAGS) -c $< -o $@)

$(FW_CONSTANTS): $(FW_HEADERS) $(MKESPHOME)
	@mkdir -p $(@D)
	$(PYTHON) $(MKESPHOME) --make > $@

$(MCU)/constants_check.c: $(FW_HEADERS) $(MKESPHOME)
	@mkdir -p $(@D)
	$(PYTHON) $(MKESPHOME) --sdcc-check > $@

$(MCU)/constants_check.rel: $(MCU)/constants_check.c Makefile
	$(call fail_on_output,$(SDCC) $(SDCC_FLAGS) -c $< -o $@)

$(MCU)/boot/vectors.inc: $(FW_CONSTANTS) $(FW)/include/stc15.h Makefile
	@mkdir -p $(@D)
	printf 'APP_BASE = %s\nBOOT_CODE_BASE = %s\n' $(APP_CODE_LOC) $(BOOT_CODE_LOC) > $@
	sed -n 's/^__s[a-z]* __at(\(0x[0-9A-F]*\)) \([A-Z0-9_]*\);$$/\2 = \1/p' $(FW)/include/stc15.h >> $@

$(MCU)/boot/boot/vectors.rel: $(FW)/boot/vectors.asm $(MCU)/boot/vectors.inc Makefile
	@mkdir -p $(@D)
	$(SDAS) -plosgff -I$(MCU)/boot -o $@ $<

$(MCU)/app.ihx: $(APP_REL) Makefile
	$(PYTHON) $(FW)/tools/isr_overlay.py $(APP_REL:.rel=.asm) --nooverlay $(patsubst %.c,$(MCU)/app/%.asm,$(ISR_SRC))
	$(call fail_on_output,$(SDCC) -mmcs51 $(SDCC_LINK) --code-loc $(APP_CODE_LOC) --code-size $(APP_CODE_SIZE) \
		$(filter %.rel,$^) -o $@)

$(MCU)/boot.ihx: $(BOOT_REL) Makefile
	$(call fail_on_output,$(SDCC) -mmcs51 $(SDCC_LINK) --code-loc $(BOOT_CODE_LOC) --code-size $(BOOT_CODE_SIZE) \
		$(filter %.rel,$^) -o $@)

firmware: $(MCU)/app.ihx $(MCU)/boot.ihx $(MCU)/constants_check.rel
	$(PYTHON) $(MKESPHOME) --build $(MCU)

check: $(MCU)/app.ihx $(MCU)/boot.ihx $(MCU)/constants_check.rel
	$(PYTHON) $(MKESPHOME) --build $(MCU) --check

# -iquote: firmware/app/sched.h must not hide the system's <sched.h>.
HOST_INC := $(addprefix -iquote $(FW)/,include common app boot tests)
HOST_CFLAGS := -std=c99 -Wall -Wextra -Werror -O1 -g $(HOST_INC)
HOST_CXXFLAGS := -std=c++17 -Wall -Wextra -Werror -O1 -g -iquote . $(HOST_INC)
SANITIZE := -fsanitize=address,undefined
# Each program takes from the library only what it uses.
HOST_FW_SRC := app/app.c app/gate.c app/phase.c app/safety.c app/sched.c app/status.c app/window.c \
	boot/boot.c boot/update.c common/crc16.c common/link.c
HOST_FW_LIB := $(HOST)/firmware.a

$(HOST)/fw/%.o: $(FW)/%.c $(FW_HEADERS) Makefile
	@mkdir -p $(@D)
	$(CC) $(HOST_CFLAGS) -c $< -o $@

# link.c as the MCU programs build it.
$(HOST)/fw/link_single.o: $(FW)/common/link.c $(FW_HEADERS) Makefile
	@mkdir -p $(@D)
	$(CC) $(HOST_CFLAGS) -DLINK_SINGLE -c $< -o $@

$(HOST_FW_LIB): $(patsubst %.c,$(HOST)/fw/%.o,$(HOST_FW_SRC))
	rm -f $@
	$(AR) rcs $@ $^

MCU_SIM := $(HOST)/mcu_sim.o $(HOST_FW_LIB)

$(HOST)/mcu_sim.o: esphome/tests/mcu_sim.c esphome/tests/mcu_sim.h $(FW_HEADERS) Makefile
	@mkdir -p $(@D)
	$(CC) $(HOST_CFLAGS) -c $< -o $@

FW_TESTS := $(patsubst $(FW)/tests/%.c,$(HOST)/fw-tests/%,$(wildcard $(FW)/tests/test_*.c)) \
	$(HOST)/fw-tests/test_link_single

$(HOST)/fw-tests/test_%: $(FW)/tests/test_%.c $(FW)/tests/test.h $(HOST_FW_LIB) $(FW_HEADERS) Makefile
	@mkdir -p $(@D)
	$(CC) $(HOST_CFLAGS) $< $(HOST_FW_LIB) -o $@

$(HOST)/fw-tests/test_link_single: $(FW)/tests/test_link.c $(FW)/tests/test.h $(HOST)/fw/link_single.o \
		$(HOST_FW_LIB) $(FW_HEADERS) Makefile
	@mkdir -p $(@D)
	$(CC) $(HOST_CFLAGS) -DLINK_SINGLE $< $(HOST)/fw/link_single.o $(HOST_FW_LIB) -o $@

$(HOST)/test_rpstd01: esphome/tests/test_rpstd01.cpp esphome/tests/sim_wrapper.h $(ESP_HEADERS) $(MCU_SIM) Makefile
	@mkdir -p $(@D)
	$(CXX) $(HOST_CXXFLAGS) $(SANITIZE) $< $(MCU_SIM) -o $@

$(HOST)/test_tool: esphome/tests/tool/test_tool.cpp $(ESP_HEADERS) $(wildcard esphome/tests/tool/fake_idf/*.h) Makefile
	@mkdir -p $(@D)
	$(CXX) $(HOST_CXXFLAGS) $(SANITIZE) -iquote esphome/tests/tool/fake_idf $< -o $@

$(HOST)/test_tool_update: esphome/tests/tool/test_update.cpp esphome/tests/sim_wrapper.h $(ESP_HEADERS) \
		$(wildcard esphome/tests/tool/fake_idf/*.h) $(MCU_SIM) Makefile
	@mkdir -p $(@D)
	$(CXX) $(HOST_CXXFLAGS) $(SANITIZE) -iquote esphome/tests/tool/fake_idf $< $(MCU_SIM) -o $@

$(HOST)/test_mcuflash: esphome/tests/stc/test_mcuflash.cpp $(ESP_HEADERS) Makefile
	@mkdir -p $(@D)
	$(CXX) $(HOST_CXXFLAGS) $(SANITIZE) $< -o $@

$(HOST)/test_stc_isp: esphome/tests/stc/test_stc_isp.cpp $(ESP_HEADERS) Makefile
	@mkdir -p $(@D)
	$(CXX) $(HOST_CXXFLAGS) $(SANITIZE) $< -o $@

$(HOST)/fake_mcu: esphome/tests/fake_mcu.c $(MCU_SIM) $(FW_HEADERS) Makefile
	@mkdir -p $(@D)
	$(CC) $(HOST_CFLAGS) $< $(MCU_SIM) -o $@

STC := esphome/tests/stc
ESP_TESTS := $(addprefix $(HOST)/,test_rpstd01 test_tool test_tool_update test_mcuflash)

test: $(FW_TESTS) $(ESP_TESTS) $(HOST)/test_stc_isp $(HOST)/fake_mcu
	@set -e; for t in $(FW_TESTS) $(ESP_TESTS); do echo "== $$t"; $$t; done
	@echo "== $(HOST)/test_stc_isp"
	@$(HOST)/test_stc_isp $(STC)/vectors.txt $(STC)/test_image.bin
	@echo "== $(FW)/tools"
	@$(PYTHON) -m unittest discover -s $(FW)/tools -p 'test_*.py'
	@if command -v $(SDCC) >/dev/null; then $(MAKE) --no-print-directory check; \
	else echo "SDCC not on PATH: make check skipped"; fi

VENV_PYTHON = $(if $(ESPHOME_VENV),$(ESPHOME_VENV)/bin/python)
ESPHOME_PATH_PYTHON = $(wildcard $(dir $(shell command -v esphome))python)
ESPHOME_PYTHON = $(or $(VENV_PYTHON),$(ESPHOME_PATH_PYTHON),python3)

test-host: $(HOST)/fake_mcu
	@$(ESPHOME_PYTHON) -c 'import esphome, pytest, pytest_asyncio, aioesphomeapi' 2>/dev/null || { \
		echo "make test-host needs ESPHome, pytest, pytest-asyncio and aioesphomeapi in one Python:"; \
		echo "set ESPHOME_VENV to a virtual environment that has them, or put its bin folder on PATH"; \
		exit 1; }
	$(ESPHOME_PYTHON) -m pytest -p no:cacheprovider -q $(wildcard esphome/tests/test_*.py) $(PYTEST_ARGS)

STCGAL_COMMIT := fdf5fdd60515a260bb303dcf7b251c2b2671f91c
STCGAL := $(BUILD)/stcgal

stc-vectors:
	@[ -d $(STCGAL) ] || git clone -q https://github.com/grigorig/stcgal.git $(STCGAL)
	git -C $(STCGAL) checkout -q $(STCGAL_COMMIT)
	$(PYTHON) $(STC)/gen_stc_vectors.py $(STCGAL) $(STC)/test_image.bin $(STC)/vectors.txt

clean:
	rm -rf $(BUILD)

.PHONY: all firmware check test test-host stc-vectors clean
