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