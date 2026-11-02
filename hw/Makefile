# Copyright 2026 ETH Zurich and University of Bologna.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

PYTHON ?= python
IMPORT_SCRIPT = import_cfg.py
APPLY_SCRIPT = apply_cfg.py

CFG ?=

.PHONY: config
config:
ifeq ($(CFG),)
	$(PYTHON) $(APPLY_SCRIPT)
else
	$(PYTHON) $(IMPORT_SCRIPT) $(CFG)
	$(PYTHON) $(APPLY_SCRIPT) $(basename $(notdir $(CFG)))
endif