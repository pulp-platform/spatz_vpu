import sys
try:
    import json5 as hjson  # prefer json5 for Snitch cluster JSON5 configs
except ImportError:
    import hjson            # fall back to hjson for native Spatz configs
from pathlib import Path

if len(sys.argv) < 2:
    print("Usage: python script.py <config.hjson>")
    sys.exit(1)

# hw/ holds the cfg outputs; this script itself lives in util/ alongside the
# other generation/tooling scripts.
hw_dir = Path(__file__).parent.parent / "hw"

cfg_source_path = Path(sys.argv[1])
cfg_name = cfg_source_path.stem   # strip extension; apply_cfg.py appends .hjson

SPATZ_KEYS = ['mempool', 'vlen', 'n_fpu', 'n_ipu', 'spatz_fpu', 'spatz_nports',
              'double_bw', 'buf_fpu', 'isa']

DEFAULT_SPATZ_CFG = {
    'mempool': False,
    'vlen': 512,
    'n_fpu': 4,
    'n_ipu': 1,
    'spatz_fpu': True,
    'spatz_nports': 4,
    'double_bw': False,
    'buf_fpu': 1,
    'isa': 'rv32imafd',
}

with open(cfg_source_path, "r") as f:
    data = hjson.load(f)

# Two source shapes are supported:
# - a Snitch/Spatz cluster cfg (e.g. spatz_cluster.*.hjson): Spatz params
#   sit directly under a top-level 'cluster' key, and the ISA string is
#   per-core under 'cluster.cores[0].isa'.
# - a native spatz_vpu cfg (e.g. spatz.default.dram.hjson): params already
#   sit directly under a top-level 'spatz' key, including a flat 'isa'.
cfg = data.get('cluster') or data.get('spatz')
if not cfg:
    print(f"[import_cfg] Warning: '{cfg_source_path}' carries no (or an "
          f"empty) 'cluster'/'spatz' configuration; falling back to "
          f"defaults.", file=sys.stderr)
    cfg = DEFAULT_SPATZ_CFG

spatz_cfg = {'spatz': {k: cfg[k] for k in SPATZ_KEYS if k in cfg}}

if 'isa' not in spatz_cfg['spatz']:
    cores = cfg.get('cores')
    if cores and 'isa' in cores[0]:
        spatz_cfg['spatz']['isa'] = cores[0]['isa']

cfg_dest_path = hw_dir / 'cfg' / (cfg_name + '.hjson')
cfg_dest_path.parent.mkdir(parents=True, exist_ok=True)
with open(cfg_dest_path, "w") as f:
    hjson.dump(spatz_cfg, f)
