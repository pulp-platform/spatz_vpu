import sys
import os
import hjson
from pathlib import Path
from mako.template import Template

# get dir to which the output is exported
script_dir = Path(__file__).parent

# ---- Default config name ----
DEFAULT_CONFIG = "spatz.default.dram"

# ---- Get optional CLI argument ----
if len(sys.argv) > 1:
    cfg_name = sys.argv[1]
else:
    cfg_name = DEFAULT_CONFIG

# ---- Input files ----
cfg_path = script_dir / "cfg" / (cfg_name + ".hjson")
template_path = script_dir / "src" / "spatz_pkg.sv.tpl"
output_path = script_dir / "src" / "generated" / "spatz_pkg.sv"

if not os.path.exists(cfg_path):
    raise FileNotFoundError(f"Config not found: {cfg_path}")

with open(cfg_path, "r") as f:
    data = hjson.load(f)

cfg = data["spatz"]

# ---- Load template ----
with open(template_path, "r") as f:
    template = Template(f.read())

# ---- Render ----
rendered = template.render(cfg=cfg)

# ---- Write output ----
with open(output_path, "w") as f:
    f.write(rendered)

