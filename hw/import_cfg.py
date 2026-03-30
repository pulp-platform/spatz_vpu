import sys
import hjson
from pathlib import Path

if len(sys.argv) < 2:
    print("Usage: python script.py <config.hjson>")
    sys.exit(1)

# get dir to which the output is exported
script_dir = Path(__file__).parent

# get path of config file
cfg_source_path = Path(sys.argv[1])
cfg_name = cfg_source_path.name

with open(cfg_source_path, "r") as f:
    data = hjson.load(f)
    
# copy only relevant information
spatz_cfg = {'spatz': {}}
keys = ['mempool', 'vlen', 'n_fpu', 'n_ipu', 'spatz_fpu', 'spatz_nports', 'double_bw', 'buf_fpu']

for k in keys:
    if k in data['cluster']:
        spatz_cfg['spatz'][k] = data['cluster'][k]


# dump output into cfg folder
cfg_dest_path = script_dir / 'cfg' / cfg_name
cfg_dest_path.parent.mkdir(parents=True, exist_ok=True)
with open(cfg_dest_path, "w") as f:
    hjson.dump(spatz_cfg, f)






