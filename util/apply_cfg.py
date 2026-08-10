import sys
import os
import hjson
from pathlib import Path
from mako.template import Template

from isa import parse_isa_string

# hw/ holds the cfg inputs, template, and generated output; this script
# itself lives in util/ alongside the other generation/tooling scripts.
hw_dir = Path(__file__).parent.parent / "hw"

# ---- Default config name ----
DEFAULT_CONFIG = "spatz.default.dram"

# ---- Get optional CLI argument ----
if len(sys.argv) > 1:
    cfg_name = sys.argv[1]
else:
    cfg_name = DEFAULT_CONFIG

# ---- Input files ----
cfg_path = hw_dir / "cfg" / (cfg_name + ".hjson")
template_path = hw_dir / "src" / "spatz_pkg.sv.tpl"
output_path = hw_dir / "src" / "generated" / "spatz_pkg.sv"

if not os.path.exists(cfg_path):
    raise FileNotFoundError(f"Config not found: {cfg_path}")

with open(cfg_path, "r") as f:
    data = hjson.load(f)

cfg = data["spatz"]

# The template derives RVF/RVD from cfg['cores'][0]['isa_parsed'], matching
# the shape the cluster's own clustergen.py passes in. Build the same shape
# here from a plain 'isa' string so standalone generation stays on the same
# code path (rather than sourcing RVF/RVD from separate config flags).
cfg["cores"] = [{"isa_parsed": parse_isa_string(cfg["isa"])}]

# ---- Load template ----
with open(template_path, "r") as f:
    template = Template(f.read())

# ---- Render ----
rendered = template.render(cfg=cfg)

# ---- Write output ----
with open(output_path, "w") as f:
    f.write(rendered)

