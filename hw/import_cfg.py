import sys
try:
    import json5 as hjson  # prefer json5 for Snitch cluster JSON5 configs
except ImportError:
    import hjson            # fall back to hjson for native Spatz configs
from pathlib import Path

if len(sys.argv) < 2:
    print("Usage: python script.py <config.hjson>")
    sys.exit(1)

script_dir = Path(__file__).parent

cfg_source_path = Path(sys.argv[1])
cfg_name = cfg_source_path.stem   # strip extension; apply_cfg.py appends .hjson

SPATZ_KEYS = ['mempool', 'vlen', 'n_fpu', 'n_ipu', 'spatz_fpu', 'spatz_nports',
              'double_bw', 'buf_fpu', 'pace', 'rvf', 'rvd']

DEFAULT_SPATZ_CFG = {
    'mempool': False,
    'vlen': 512,
    'n_fpu': 4,
    'n_ipu': 1,
    'spatz_fpu': True,
    'spatz_nports': 4,
    'double_bw': False,
    'buf_fpu': 1,
    'pace': False,
    'rvf': True,
    'rvd': True,
}

with open(cfg_source_path, "r") as f:
    data = hjson.load(f)

cfg = data.get('spatz')
if not cfg:
    print(f"[import_cfg] Warning: '{cfg_source_path}' carries no (or an "
          f"empty) 'spatz' configuration; falling back to defaults.",
          file=sys.stderr)
    cfg = DEFAULT_SPATZ_CFG

spatz_cfg = {'spatz': {k: cfg[k] for k in SPATZ_KEYS if k in cfg}}

cfg_dest_path = script_dir / 'cfg' / (cfg_name + '.hjson')
cfg_dest_path.parent.mkdir(parents=True, exist_ok=True)
with open(cfg_dest_path, "w") as f:
    hjson.dump(spatz_cfg, f)
