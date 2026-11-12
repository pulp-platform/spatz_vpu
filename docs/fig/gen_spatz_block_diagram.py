#!/usr/bin/env python3
# Generates the Spatz top-level block diagram as Excalidraw JSON and SVG.
import json, random, sys, html
from pathlib import Path

out_dir = Path(sys.argv[1])
random.seed(7)

FONT = 14
CHAR_W = 0.6 * FONT  # monospace estimate
IN_COL = "#1971c2"   # input to DUT
OUT_COL = "#2f9e44"  # output from DUT
BLK_BG = "#f1f3f5"
DUT_BG = "#fff3bf"
INT_BG = "#ffffff"

rects, texts, arrows = [], [], []

def rect(x, y, w, h, bg=BLK_BG, dashed=False, stroke="#1e1e1e", sw=2):
    rects.append(dict(x=x, y=y, w=w, h=h, bg=bg, dashed=dashed, stroke=stroke, sw=sw))

def text(x, y, s, size=FONT, color="#1e1e1e", align="left", bold=False):
    texts.append(dict(x=x, y=y, s=s, size=size, color=color, align=align, bold=bold))

def arrow(x1, y1, x2, y2, color, label=None, lx=None, ly=None, size=FONT):
    arrows.append(dict(x1=x1, y1=y1, x2=x2, y2=y2, color=color))
    if label:
        text(lx if lx is not None else min(x1, x2) + 8,
             ly if ly is not None else y1 - size - 6, label, color=color, size=size)

def block(x, y, w, h, title, sub=None, bg=BLK_BG, size=16):
    rect(x, y, w, h, bg)
    lines = title.split("\n")
    ty = y + 10
    for l in lines:
        text(x + w / 2, ty, l, size=size, align="center", bold=True)
        ty += size * 1.3
    if sub:
        for l in sub.split("\n"):
            text(x + w / 2, ty + 2, l, size=12, align="center", color="#495057")
            ty += 12 * 1.35

# ---------------------------------------------------------------- layout
DX, DY, DW, DH = 850, 220, 460, 700          # DUT
SNX, SNW = 40, 170                            # Snitch
MUXX, MUXW = 370, 200                         # demux / arbiter
LA0, LA1 = MUXX + MUXW, DX                    # left arrow span
RA0, RA1 = DX + DW, DX + DW + 330             # right arrow span
ICX, ICW = RA1, 230                           # TCDM interconnect
BKX, BKW = ICX + ICW + 60, 180                # TCDM banks

# Enclosing cluster frame
rect(20, 20, BKX + BKW + 30 - 20, 1330 - 20, bg="transparent", dashed=True, stroke="#868e96", sw=1)
text(30, 28, "spatz_cluster  (hw/system/spatz_cluster/src/spatz_cluster.sv)  —  core complex: spatz_cc (hw/ip/spatz_cc/src/spatz_cc.sv)",
     size=14, color="#868e96")

# DUT
rect(DX, DY, DW, DH, DUT_BG, sw=3)
text(DX + DW / 2, DY + 14, "spatz", size=24, align="center", bold=True)
text(DX + DW / 2, DY + 46, "hw/src/spatz.sv  (DUT)", size=14, align="center")
text(DX + DW / 2, DY + 66, "default cfg: N_FPU=4, N_IPU=1, VLEN=512, ELEN=64, NrMemPorts=4",
     size=12, align="center", color="#495057")
# Internal blocks (context only)
ib = [("spatz_fpu_sequencer", "FP regfile, FP LSU", DX + 20, DY + 100),
      ("spatz_controller", "decoder, CSRs, scoreboard", DX + 240, DY + 100),
      ("spatz_vfu", "4x FPnew + 1x IPU", DX + 20, DY + 250),
      ("spatz_vlsu", "4 memory ports", DX + 240, DY + 250),
      ("spatz_vsldu", "slides, vcompress", DX + 20, DY + 400),
      ("spatz_vrf", "4 banks, latch-based", DX + 240, DY + 400)]
for t, s, x, y in ib:
    block(x, y, 200, 90, t, s, bg=INT_BG, size=14)
text(DX + DW / 2, DY + 520, "[ ventaglio ]  only with `define VENTAGLIO", size=12, align="center", color="#868e96")

# ---------------------------------------------------------------- top: clock / reset
block(DX, 70, DW, 60, "cluster clock / reset / hart id", "spatz_cluster → spatz_cc", size=14)
for i, (sig, col) in enumerate([("clk_i", IN_COL), ("rst_ni", IN_COL), ("testmode_i", IN_COL), ("hart_id_i[31:0]", IN_COL)]):
    x = DX + 40 + i * 115
    arrow(x, 130, x, DY, col)
    text(x + 6, 160, sig, color=col)

# ---------------------------------------------------------------- left: Snitch
block(SNX, DY, SNW, 760, "Snitch core", "hw/ip/snitch/src/\nsnitch.sv\n\ninteger core,\nfcsr, acc offload\nmaster")

# issue path through stream_demux
block(MUXX, DY + 90, MUXW, 170, "stream_demux", "i_stream_demux_offload\nsel = acc_qreq.addr\n(SPATZ / DMA)", size=14)
arrow(SNX + SNW, DY + 175, MUXX, DY + 175, IN_COL, "acc_qvalid / acc_qreq", lx=SNX + SNW + 6, ly=DY + 157, size=12)
arrow(MUXX, DY + 205, SNX + SNW, DY + 205, OUT_COL, "acc_qready", lx=SNX + SNW + 6, ly=DY + 210, size=12)
y = DY + 120
for sig, dirn in [("issue_valid_i", "in"), ("issue_req_i : spatz_issue_req_t", "in"), ("issue_ready_o", "out")]:
    if dirn == "in":
        arrow(LA0, y, LA1, y, IN_COL, sig)
    else:
        arrow(LA1, y, LA0, y, OUT_COL, sig)
    y += 50
# issue_rsp goes straight to Snitch (acc_qrsp_i)
y = DY + 300
arrow(LA1, y, SNX + SNW, y, OUT_COL, "issue_rsp_o : spatz_issue_rsp_t   (→ Snitch acc_qrsp_i, not demuxed)",
      lx=SNX + SNW + 8)

# response path through stream_arbiter
block(MUXX, DY + 350, MUXW, 170, "stream_arbiter", "i_stream_arbiter_offload\nmerges Spatz + DMA\nresponses", size=14)
arrow(MUXX, DY + 435, SNX + SNW, DY + 435, OUT_COL, "acc_pvalid / acc_prsp", lx=SNX + SNW + 6, ly=DY + 417, size=12)
arrow(SNX + SNW, DY + 465, MUXX, DY + 465, IN_COL, "acc_pready", lx=SNX + SNW + 6, ly=DY + 470, size=12)
y = DY + 385
for sig, dirn in [("rsp_valid_o", "out"), ("rsp_o : spatz_rsp_t", "out"), ("rsp_ready_i", "in")]:
    if dirn == "in":
        arrow(LA0, y, LA1, y, IN_COL, sig)
    else:
        arrow(LA1, y, LA0, y, OUT_COL, sig)
    y += 50

# direct side channels to Snitch
y = DY + 580
for sig, dirn in [("spatz_mem_finished_o[1:0]", "out"), ("spatz_mem_str_finished_o[1:0]", "out"),
                  ("fpu_rnd_mode_i : roundmode_e", "in"), ("fpu_fmt_mode_i : fmt_mode_t", "in"),
                  ("fpu_status_o : status_t", "out")]:
    if dirn == "in":
        arrow(SNX + SNW, y, LA1, y, IN_COL, sig, lx=SNX + SNW + 8)
    else:
        arrow(LA1, y, SNX + SNW, y, OUT_COL, sig, lx=SNX + SNW + 8)
    y += 36
text(SNX + SNW + 8, DY + 770, "[1] = VLSU (vector), [0] = FP LSU (scalar FP)", size=12, color="#495057")

# ---------------------------------------------------------------- right: TCDM
block(ICX, DY + 60, ICW, 330, "spatz_tcdm_\ninterconnect", "i_tcdm_interconnect\n(spatz_cluster.sv)\n\nports [0..3]: Spatz VLSU\nport  [4]: Snitch + FP LSU\n(via reqrsp_to_tcdm)", size=15)
block(BKX, DY + 60, BKW, 330, "TCDM / L1", "16 SRAM banks\n128 KiB\ntc_sram_impl +\nspatz_amo_shim\n\nfixed latency,\nin-order responses", size=15)
arrow(ICX + ICW, DY + 200, BKX, DY + 200, "#1e1e1e")
arrow(BKX, DY + 250, ICX + ICW, DY + 250, "#1e1e1e")
y = DY + 110
for sig, dirn in [("spatz_mem_req_o[3:0] : spatz_mem_req_t", "out"), ("spatz_mem_req_valid_o[3:0]", "out"),
                  ("spatz_mem_req_ready_i[3:0]", "in"), ("spatz_mem_rsp_i[3:0] : spatz_mem_rsp_t", "in"),
                  ("spatz_mem_rsp_valid_i[3:0]", "in")]:
    if dirn == "in":
        arrow(RA1, y, RA0, y, IN_COL, sig, lx=RA0 + 8)
    else:
        arrow(RA0, y, RA1, y, OUT_COL, sig, lx=RA0 + 8)
    y += 55
text(RA0 + 8, y - 20, "no rsp ready: responses always accepted", size=12, color="#495057")
text(RA0 + 8, y - 4, "one response per request, stores included", size=12, color="#495057")

# ---------------------------------------------------------------- bottom: FP LSU path
BY = 1060
block(DX + 20, BY, 200, 90, "reqrsp_mux", "i_reqrsp_mux\nSnitch data + FP LSU", size=14)
block(DX + 280, BY, 200, 90, "reqrsp_demux", "addr_decode_napot\nTCDM range / SoC", size=14)
block(ICX, BY, ICW, 90, "reqrsp_to_tcdm", "i_reqrsp_to_tcdm\n→ interconnect port [4]", size=14)
block(DX + 280, BY + 150, 200, 70, "SoC / AXI", "data_req_o of spatz_cc", size=14)
arrow(DX + 80, DY + DH, DX + 80, BY, OUT_COL, "fp_lsu_mem_req_o : dreq_t", lx=DX - 150, ly=DY + DH + 50)
arrow(DX + 160, BY, DX + 160, DY + DH, IN_COL, "fp_lsu_mem_rsp_i : drsp_t", lx=DX + 168, ly=DY + DH + 50)
arrow(SNX + SNW / 2, DY + 760, SNX + SNW / 2, BY + 45, "#1e1e1e")
arrow(SNX + SNW / 2, BY + 45, DX + 20, BY + 45, "#1e1e1e", "Snitch data_req (reqrsp_iso)", lx=SNX + SNW / 2 + 10, ly=BY + 22)
arrow(DX + 220, BY + 45, DX + 280, BY + 45, "#1e1e1e")
arrow(DX + 480, BY + 30, ICX, BY + 30, "#1e1e1e", "TCDM range", lx=DX + 500, ly=BY + 8)
arrow(DX + 380, BY + 90, DX + 380, BY + 150, "#1e1e1e")
arrow(ICX + ICW / 2, BY, ICX + ICW / 2, DY + 390, "#1e1e1e")
text(DX + 20, BY + 100, "MemPool-only ports (fp_lsu_mem_req_valid_o, ..._ready_i,", size=12, color="#868e96")
text(DX + 20, BY + 116, "..._rsp_valid_i, ..._rsp_ready_o) not shown", size=12, color="#868e96")

# legend
LX, LY = 40, BY + 110
rect(LX, LY, 360, 110, bg="#ffffff", stroke="#868e96", sw=1)
text(LX + 12, LY + 10, "Legend", size=14, bold=True)
arrow(LX + 12, LY + 45, LX + 72, LY + 45, IN_COL)
text(LX + 82, LY + 37, "input of spatz (DUT)", color=IN_COL)
arrow(LX + 12, LY + 75, LX + 72, LY + 75, OUT_COL)
text(LX + 82, LY + 67, "output of spatz (DUT)", color=OUT_COL)
text(LX + 12, LY + 88, "black: cluster-internal connection", size=12)

# ---------------------------------------------------------------- Excalidraw
def base(t, x, y, w, h, **kw):
    e = dict(id=f"{t}-{random.getrandbits(40):x}", type=t, x=x, y=y, width=w, height=h, angle=0,
             strokeColor="#1e1e1e", backgroundColor="transparent", fillStyle="solid", strokeWidth=2,
             strokeStyle="solid", roughness=0, opacity=100, groupIds=[], frameId=None, roundness=None,
             seed=random.getrandbits(31), version=1, versionNonce=random.getrandbits(31), isDeleted=False,
             boundElements=None, updated=1, link=None, locked=False)
    e.update(kw)
    return e

els = []
for r in rects:
    els.append(base("rectangle", r["x"], r["y"], r["w"], r["h"], strokeColor=r["stroke"],
                    backgroundColor=r["bg"], strokeWidth=r["sw"],
                    strokeStyle="dashed" if r["dashed"] else "solid", roundness={"type": 3}))
for a in arrows:
    dx, dy = a["x2"] - a["x1"], a["y2"] - a["y1"]
    els.append(base("arrow", a["x1"], a["y1"], abs(dx), abs(dy), strokeColor=a["color"],
                    points=[[0, 0], [dx, dy]], lastCommittedPoint=None, startBinding=None,
                    endBinding=None, startArrowhead=None, endArrowhead="arrow", elbowed=False))
for t in texts:
    w = len(t["s"]) * 0.6 * t["size"]
    h = t["size"] * 1.25
    x = t["x"] - w / 2 if t["align"] == "center" else t["x"]
    els.append(base("text", x, t["y"], w, h, strokeColor=t["color"], text=t["s"], originalText=t["s"],
                    fontSize=t["size"], fontFamily=3, textAlign=t["align"], verticalAlign="top",
                    containerId=None, lineHeight=1.25, autoResize=True))
doc = dict(type="excalidraw", version=2, source="https://excalidraw.com", elements=els,
           appState=dict(gridSize=None, viewBackgroundColor="#ffffff"), files={})
(out_dir / "spatz_block_diagram.excalidraw").write_text(json.dumps(doc, indent=1))

# ---------------------------------------------------------------- SVG
W, H = BKX + BKW + 50, 1350
svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" '
       f'font-family="Consolas, \'DejaVu Sans Mono\', monospace">',
       '<rect width="100%" height="100%" fill="#ffffff"/>', '<defs>']
for c in {a["color"] for a in arrows}:
    cid = c.strip("#")
    svg.append(f'<marker id="ah{cid}" markerWidth="10" markerHeight="8" refX="9" refY="4" orient="auto">'
               f'<path d="M0,0 L10,4 L0,8 z" fill="{c}"/></marker>')
svg.append('</defs>')
for r in rects:
    dash = ' stroke-dasharray="8 6"' if r["dashed"] else ""
    fill = "none" if r["bg"] == "transparent" else r["bg"]
    svg.append(f'<rect x="{r["x"]}" y="{r["y"]}" width="{r["w"]}" height="{r["h"]}" rx="8" fill="{fill}" '
               f'stroke="{r["stroke"]}" stroke-width="{r["sw"]}"{dash}/>')
for a in arrows:
    svg.append(f'<line x1="{a["x1"]}" y1="{a["y1"]}" x2="{a["x2"]}" y2="{a["y2"]}" stroke="{a["color"]}" '
               f'stroke-width="2" marker-end="url(#ah{a["color"].strip("#")})"/>')
for t in texts:
    anchor = "middle" if t["align"] == "center" else "start"
    fw = ' font-weight="bold"' if t["bold"] else ""
    svg.append(f'<text x="{t["x"]}" y="{t["y"] + t["size"]}" font-size="{t["size"]}" fill="{t["color"]}" '
               f'text-anchor="{anchor}"{fw}>{html.escape(t["s"])}</text>')
svg.append('</svg>')
(out_dir / "fig" / "spatz_block_diagram.svg").write_text("\n".join(svg))
print("ok", len(els), "elements")
