// Copyright 2023 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Author: Matheus Cavalcante, ETH Zurich
//
// This is the toplevel module of Spatz. It contains all other Spatz modules.
// This includes the Controller, which interfaces with the main core and handles
// instruction decoding, operation issuing to the other units, and result write
// back to the core. The Vector Functional Unit (VFU) is the high throughput
// unit that executes all arithmetic and logical operations. The Load/Store Unit
// (LSU) is used to load vectors from memory to the register file and store them
// back again. Finally, the Vector Register File (VRF) is the main register file
// that stores all of the currently used vectors close to the execution units.

`define X_INTERFACE

module spatz import spatz_pkg::*; import rvv_pkg::*; import fpnew_pkg::*; import cc_pkg::*; #(
    parameter int                  unsigned NrMemPorts          = 1,
    parameter bit                           RegisterRsp         = 0,
    // Memory request (VLSU)
    parameter type                          spatz_mem_req_t     = logic,
    parameter type                          spatz_mem_rsp_t     = logic,
    // Memory request (FP Sequencer)
    parameter type                          dreq_t              = logic,
    parameter type                          drsp_t              = logic,
`ifndef X_INTERFACE
    // Snitch interface
    parameter type                          spatz_issue_req_t   = logic,
    parameter type                          spatz_issue_rsp_t   = logic,
    parameter type                          spatz_rsp_t         = logic,
`endif
    // X-Interface (used when X_INTERFACE is defined; harmless otherwise)
    parameter type                          x_issue_req_t       = logic,
    parameter type                          x_issue_resp_t      = logic,
    parameter type                          x_register_t        = logic,
    parameter type                          x_commit_t          = logic,
    parameter type                          x_result_t          = logic,
    /// FPU configuration.
    parameter fpu_implementation_t          FPUImplementation   = fpu_implementation_t'(0),
    /// Address width of the scalar FPU LSU request interface.
    parameter int                  unsigned AddrWidth           = 32,
    // Derived parameters. DO NOT CHANGE!
    parameter int                  unsigned NumOutstandingLoads = 8
  ) (
    input  logic                              clk_i,
    input  logic                              rst_ni,
    input  logic                              testmode_i,
    input  logic             [31:0]           hart_id_i,
`ifdef X_INTERFACE
    // X-Interface
    input  logic                              x_issue_valid_i,
    output logic                              x_issue_ready_o,
    input  x_issue_req_t                      x_issue_req_i,
    output x_issue_resp_t                     x_issue_resp_o,
    input  logic                              x_register_valid_i,
    output logic                              x_register_ready_o,
    input  x_register_t                       x_register_i,
    input  logic                              x_commit_valid_i,
    input  x_commit_t                         x_commit_i,
    output logic                              x_result_valid_o,
    input  logic                              x_result_ready_i,
    output x_result_t                         x_result_o,
`else
    // Snitch Interface
    input  logic                              issue_valid_i,
    output logic                              issue_ready_o,
    input  spatz_issue_req_t                  issue_req_i,
    output spatz_issue_rsp_t                  issue_rsp_o,
    output logic                              rsp_valid_o,
    input  logic                              rsp_ready_i,
    output spatz_rsp_t                        rsp_o,
`endif
    // Memory Request
    output spatz_mem_req_t   [NrMemPorts-1:0] spatz_mem_req_o,
    output logic             [NrMemPorts-1:0] spatz_mem_req_valid_o,
    input  logic             [NrMemPorts-1:0] spatz_mem_req_ready_i,
    input  spatz_mem_rsp_t   [NrMemPorts-1:0] spatz_mem_rsp_i,
    input  logic             [NrMemPorts-1:0] spatz_mem_rsp_valid_i,
    // Memory Finished
    output logic             [1:0]            spatz_mem_finished_o,
    output logic             [1:0]            spatz_mem_str_finished_o,
    // FPU memory interface interface
`ifdef MEMPOOL_SPATZ
    output logic                              fp_lsu_mem_req_valid_o,
    input  logic                              fp_lsu_mem_req_ready_i,
    input  logic                              fp_lsu_mem_rsp_valid_i,
    output logic                              fp_lsu_mem_rsp_ready_o,
`endif
    output dreq_t                             fp_lsu_mem_req_o,
    input  drsp_t                             fp_lsu_mem_rsp_i,
    // FPU side channel
    input  roundmode_e                        fpu_rnd_mode_i,
    input  fmt_mode_t                         fpu_fmt_mode_i,
    input  pace_mode_t                        fpu_pace_mode_i,
    output status_t                           fpu_status_o
  );

  ////////////////
  // Parameters //
  ////////////////

  // Number of ports of the vector register file
  localparam int unsigned NrWritePorts = 2 + NumVLSUInterfaces; // 1 for VFU and SLDU each and 1 for each VLSU
  localparam int unsigned NrReadPorts  = 4 + 2*NumVLSUInterfaces; // 3 for VFU, 1 for SLDU and 2 for each VLSU interface

  // FPU buffer size (need atleast depth of 2 to hide conflicts)
  localparam int unsigned FpuBufDepth = 4;
  localparam int unsigned VlsuBufDepth = 2;

`ifdef X_INTERFACE
  // These types become locally used in Spatz when using XIF
  // Use FLEN (64-bit for RVD) so that double-precision scalar operands
  // forwarded by the FPU sequencer (fpr_rdata) are not truncated.
  typedef logic [FLEN-1:0] data_t;

  typedef enum logic [31:0] {
    SPATZ = 0
  } acc_addr_e;

  typedef struct packed {
    acc_addr_e addr;
    logic [5:0] id;
    logic [31:0] data_op;
    data_t data_arga;
    data_t data_argb;
    data_t data_argc;
  } spatz_issue_req_t;

  typedef struct packed {
    logic accept;
    logic writeback;
    logic loadstore;
    logic exception;
    logic isfloat;
  } spatz_issue_rsp_t;

  typedef struct packed {
    logic [5:0] id;
    logic error;
    data_t data;
  } spatz_rsp_t;
`endif
  /////////////
  // Signals //
  /////////////

  // Spatz request
  spatz_req_t spatz_req;
  logic       spatz_req_valid;

  logic     vfu_req_ready;
  logic     vfu_rsp_ready;
  logic     vfu_rsp_valid, vfu_rsp_buf_valid;
  vfu_rsp_t vfu_rsp, vfu_rsp_buf;

  logic      vlsu_req_ready;
  logic      vlsu_rsp_valid, vlsu_rsp_buf_valid;
  vlsu_rsp_t vlsu_rsp, vlsu_rsp_buf;

  logic       vsldu_req_ready;
  logic       vsldu_rsp_valid;
  vsldu_rsp_t vsldu_rsp;

`ifdef VENTAGLIO
  // VTL gets its own control-path handshakes to the controller. Sharing
  // the physical VRF port with VSLDU does not extend to admit/retire —
  // the controller has to know which unit took an op and which unit is
  // retiring it.
  logic       vtl_req_ready;
  logic       vtl_rsp_valid;
  vsldu_rsp_t vtl_rsp;
`endif

  // Buffer structure to track data information for writes from FPU and VLSU to VRF
  // When the responses from EX units are not committed to the VRF,
  // buffers store the metadata to commit to the VRF in later cycles

  logic [cnt_width(FpuBufDepth)-1:0] vfu_buf_usage;
  logic [cnt_width(VlsuBufDepth)-1:0] vlsu_buf_usage;

  typedef struct packed {
    vrf_data_t wdata;
    vrf_addr_t waddr;
    vrf_be_t   wbe;
    spatz_id_t wid;
    vfu_rsp_t rsp;
    logic rsp_valid;
  } vfu_buf_t;

  vfu_buf_t vfu_buf_data;

  typedef struct packed {
    vrf_data_t wdata;
    vrf_addr_t waddr;
    vrf_be_t   wbe;
    spatz_id_t wid;
    vlsu_rsp_t rsp;
    logic rsp_valid;
  } vlsu_buf_t;

  vlsu_buf_t vlsu_buf_data;

  /////////////////////
  //  FPU sequencer  //
  /////////////////////

  // X Interface
  spatz_issue_req_t issue_req;
  logic             issue_valid;
  logic             issue_ready;
  spatz_issue_rsp_t issue_rsp;
  spatz_rsp_t       resp;
  logic             resp_valid;
  logic             resp_ready;

  // Did we finish a memory request?
  logic fp_lsu_mem_finished;
  logic fp_lsu_mem_str_finished;
  logic spatz_mem_finished;
  logic spatz_mem_str_finished;
  assign spatz_mem_finished_o     = {spatz_mem_finished, fp_lsu_mem_finished};
  assign spatz_mem_str_finished_o = {spatz_mem_str_finished, fp_lsu_mem_str_finished};

  /////////////////////////////
  //  Top-boundary acc bus   //
  /////////////////////////////
  // Acc-shaped signals at the top boundary. The FPU sequencer (and the
  // FPU-less bypass) consume these instead of the top-level ports directly.
  //   - In acc mode: aliased to the top-level acc ports.
  //   - In X mode:   produced/consumed by the X-to-acc converter below.

  spatz_issue_req_t acc_issue_req_top;
  logic             acc_issue_valid_top;
  logic             acc_issue_ready_top;
  spatz_issue_rsp_t acc_issue_rsp_top;
  spatz_rsp_t       acc_rsp_top;
  logic             acc_rsp_valid_top;
  logic             acc_rsp_ready_top;

`ifdef X_INTERFACE
  //////////////////////////
  //  X-to-acc converter  //
  //////////////////////////
  //
  // The cluster's X-Interface variant sends the rs values (x_register channel)
  // concurrently with the issue request (x_issue channel) instead of waiting
  // for the accept response. The converter below combines x_issue + x_register
  // into a single acc-shaped issue request.
  //
  // Result handling:
  //   * accept=1, writeback=0 -> the X result is synthesized immediately at
  //     issue handshake time (rd taken from instr[11:7], we=0). Spatz will
  //     never produce an acc rsp for these instructions, so this is the only
  //     retirement signal the host receives.
  //   * accept=1, writeback=1 -> the synthesized result is suppressed. The X
  //     result is forwarded from Spatz's acc rsp when it arrives.
  //   * accept=0 -> no X result is produced; the host learns of the rejection
  //     from x_issue_resp.accept.
  //
  // Arbitration: the synthesized path has priority over the acc rsp path.
  // When the synthesized path is firing, acc_rsp_ready_top is held low so
  // Spatz stalls its rsp; this avoids the need for a result FIFO.
  //
  // Commit channel: per cluster spec, the commit channel is assumed always-
  // ready (no ready signal exists). commit_kill is not supported and the
  // x_commit_* inputs are intentionally unused.

  logic synth_result_valid;
  logic combined_issue_valid;
  logic issue_proceed;

  always_comb begin
    // --- Issue request (X -> acc) ------------------------------------------
    acc_issue_req_top           = '0;
    // Use the destination register (instr[11:7]) as the acc id, following the
    // acc convention (id = rd).  The FPU sequencer overrides id[5] with use_fd
    // for FPR/GPR routing and echoes the id back, so x_result_o.rd correctly
    // reflects the destination register at result time.  The original XIF
    // counter is not used because Snitch's retire path uses x_result_i.rd, not
    // x_result_i.id.
    acc_issue_req_top.id        = {1'b0, x_issue_req_i.instr[11:7]};
    acc_issue_req_top.data_op   = x_issue_req_i.instr;
    acc_issue_req_top.data_arga = x_register_i.rs[0];
    acc_issue_req_top.data_argb = x_register_i.rs[1];
    acc_issue_req_top.data_argc = x_register_i.rs[2];
    // acc_issue_req_top.addr is unused: Spatz is the only unit on the iface.

    // --- Synthesized X-result for non-writeback instructions ---------------
    combined_issue_valid = x_issue_valid_i & x_register_valid_i;
    // Gate on acc_issue_ready_top so the synthesized result only fires in the
    // same cycle the issue handshake completes (atomic). Without this gate,
    // synth_result_valid stays high the entire time Snitch is stalled on the
    // issue, which forces acc_rsp_ready_top=0 and deadlocks the FP LSU
    // response path.
    synth_result_valid   = combined_issue_valid
                         & acc_issue_rsp_top.accept
                         & ~acc_issue_rsp_top.writeback
                         & acc_issue_ready_top;

    // --- Issue handshake ---------------------------------------------------
    // Writeback instructions: proceed as soon as Spatz is ready.
    // Non-writeback instructions: also require x_result to be ready, since
    // the synthesized result must be consumed atomically with the issue.
    issue_proceed       = acc_issue_rsp_top.writeback | x_result_ready_i;
    acc_issue_valid_top = combined_issue_valid;
    x_issue_ready_o     = acc_issue_ready_top & x_register_valid_i & issue_proceed;
    x_register_ready_o  = acc_issue_ready_top & x_issue_valid_i    & issue_proceed;

    // --- Issue response ----------------------------------------------------
    x_issue_resp_o               = '0;
    x_issue_resp_o.accept        = acc_issue_rsp_top.accept;
    x_issue_resp_o.writeback     = acc_issue_rsp_top.writeback;
    x_issue_resp_o.register_read = '1; // rs sent concurrently in this variant

    // --- Result mux (synthesized path has priority) ------------------------
    x_result_o        = '0;
    if (synth_result_valid) begin
      x_result_o.id   = x_issue_req_i.id;
      x_result_o.rd   = x_issue_req_i.instr[11:7];
      x_result_o.we   = 1'b0;
      x_result_o.data = '0;
    end else begin
      x_result_o.id   = acc_rsp_top.id;
      x_result_o.rd   = acc_rsp_top.id[4:0];
      x_result_o.we   = 1'b1;
      x_result_o.data = acc_rsp_top.data;
    end
    x_result_valid_o  = synth_result_valid | acc_rsp_valid_top;

    // Hold back Spatz's rsp when the synthesized path is firing.
    acc_rsp_ready_top = x_result_ready_i & ~synth_result_valid;
  end

  // Commit channel intentionally unused (no kill, no ready in this variant).

`else
  // acc mode: alias top-level acc ports onto the internal acc-shaped bus.
  assign acc_issue_req_top   = issue_req_i;
  assign acc_issue_valid_top = issue_valid_i;
  assign issue_ready_o       = acc_issue_ready_top;
  assign issue_rsp_o         = acc_issue_rsp_top;
  assign rsp_o               = acc_rsp_top;
  assign rsp_valid_o         = acc_rsp_valid_top;
  assign acc_rsp_ready_top   = rsp_ready_i;
`endif

  if (!FPU) begin: gen_no_fpu_sequencer
    // Spatz configured without an FPU. Just forward the requests to Spatz.
    assign issue_req           = acc_issue_req_top;
    assign issue_valid         = acc_issue_valid_top;
    assign acc_issue_ready_top = issue_ready;
    assign acc_issue_rsp_top   = issue_rsp;

    assign acc_rsp_top         = resp;
    assign acc_rsp_valid_top   = resp_valid;
    assign resp_ready          = acc_rsp_ready_top;

    // Tie the memory interface to zero
    assign fp_lsu_mem_req_o        = '0;
`ifdef MEMPOOL_SPATZ
    assign fp_lsu_mem_req_valid_o  = 1'b0;
    assign fp_lsu_mem_rsp_ready_o  = 1'b0;
`endif
    assign fp_lsu_mem_finished     = 1'b0;
    assign fp_lsu_mem_str_finished = 1'b0;
  end: gen_no_fpu_sequencer else begin: gen_fpu_sequencer
    spatz_fpu_sequencer #(
      .dreq_t             (dreq_t              ),
      .drsp_t             (drsp_t              ),
      .spatz_issue_req_t  (spatz_issue_req_t   ),
      .spatz_issue_rsp_t  (spatz_issue_rsp_t   ),
      .spatz_rsp_t        (spatz_rsp_t         ),
      .AddrWidth          (AddrWidth           ),
      .NumOutstandingLoads(NumOutstandingLoads )
    ) i_fpu_sequencer (
      .clk_i                    ( clk_i                  ),
      .rst_ni                   ( rst_ni                 ),
      // Snitch interface (acc-shaped, sourced from top boundary bus)
      .issue_req_i              ( acc_issue_req_top      ),
      .issue_valid_i            ( acc_issue_valid_top    ),
      .issue_ready_o            ( acc_issue_ready_top    ),
      .issue_rsp_o              ( acc_issue_rsp_top      ),
      .resp_o                   ( acc_rsp_top            ),
      .resp_valid_o             ( acc_rsp_valid_top      ),
      .resp_ready_i             ( acc_rsp_ready_top      ),
      // Spatz interface
      .issue_req_o              ( issue_req              ),
      .issue_valid_o            ( issue_valid            ),
      .issue_ready_i            ( issue_ready            ),
      .issue_rsp_i              ( issue_rsp              ),
      .resp_i                   ( resp                   ),
      .resp_valid_i             ( resp_valid             ),
      .resp_ready_o             ( resp_ready             ),
      // Memory interface
`ifdef MEMPOOL_SPATZ
      .fp_lsu_mem_req_valid_o   ( fp_lsu_mem_req_valid_o ),
      .fp_lsu_mem_req_ready_i   ( fp_lsu_mem_req_ready_i ),
      .fp_lsu_mem_rsp_valid_i   ( fp_lsu_mem_rsp_valid_i ),
      .fp_lsu_mem_rsp_ready_o   ( fp_lsu_mem_rsp_ready_o ),
`endif
      .fp_lsu_mem_req_o         ( fp_lsu_mem_req_o       ),
      .fp_lsu_mem_rsp_i         ( fp_lsu_mem_rsp_i       ),
      .fp_lsu_mem_finished_o    ( fp_lsu_mem_finished    ),
      .fp_lsu_mem_str_finished_o( fp_lsu_mem_str_finished),
      // Spatz VLSU side channel
      .spatz_mem_finished_i     ( spatz_mem_finished     ),
      .spatz_mem_str_finished_i ( spatz_mem_str_finished )
    );
  end: gen_fpu_sequencer

  /////////
  // VRF //
  /////////

  // Write ports
  vrf_addr_t [NrWritePorts-1:0] vrf_waddr, vrf_waddr_buf;
  vrf_data_t [NrWritePorts-1:0] vrf_wdata, vrf_wdata_buf;
  logic      [NrWritePorts-1:0] vrf_we;
  logic      [NrWritePorts-1:0] vrf_we_mask;
  vrf_be_t   [NrWritePorts-1:0] vrf_wbe, vrf_wbe_buf;
  logic      [NrWritePorts-1:0] vrf_wvalid;
  logic      [NrWritePorts-1:0] vrf_wvalid_mask;
`ifdef VENTAGLIO
  logic      [NrWritePorts-1:0] vrf_vtl_redirect_write;
`endif
  // Read ports
  vrf_addr_t [NrReadPorts-1:0]  vrf_raddr;
  logic      [NrReadPorts-1:0]  vrf_re;
  vrf_data_t [NrReadPorts-1:0]  vrf_rdata;
  logic      [NrReadPorts-1:0]  vrf_rvalid;
`ifdef VENTAGLIO
  logic      [NrReadPorts-1:0]  vrf_vtl_redirect_read;

  // Per-vreg "writer in flight" — sourced from controller, consumed by Ventaglio
  // for prefetch gating. Combinational from controller's write_table_q.valid.
  logic      [NRVREG-1:0]       vreg_write_pending;

  // VTL-VRF forwarding path
  vrf_addr_t                    vrf_vtl_waddr;
  vrf_data_t                    vrf_vtl_wdata;
  logic                         vrf_vtl_we;
  vrf_be_t                      vrf_vtl_wbe;
  logic                         vrf_vtl_wvalid;
  logic                         vrf_vtl_wscatter_en;
  // Read ports
  vrf_addr_t                    vrf_vtl_raddr;
  logic                         vrf_vtl_re;
  vrf_data_t                    vrf_vtl_rdata;
  logic                         vrf_vtl_rvalid;
  logic                         vrf_vtl_rgather_en;
`endif

  // PACE parameter memory
  // With DOUBLE_BW both VLSU write ports carry consecutive 256-bit chunks each cycle,
  // so we must capture both to reconstruct the contiguous parameter stream.
`ifdef DOUBLE_BW
  // Use WD0-only for pace_mem: each beat captures 256 bits from WD0 (lower addresses).
  // This avoids requiring simultaneous WD0+WD1 grants, which is unreliable in DOUBLE_BW.
  localparam int unsigned PaceBufWidth = N_FU * ELEN;
`else
  localparam int unsigned PaceBufWidth = N_FU * ELEN;
`endif
`ifdef DOUBLE_BW
  localparam int unsigned PaceLdIdx    = VLSU_VD_WD0;
`else
  localparam int unsigned PaceLdIdx    = VLSU_VD_WD;
`endif
  localparam int unsigned PaceBufDepth = (PaceParamWidth + PaceBufWidth - 1) / PaceBufWidth;
  logic                   pace_mem_we;
  logic [PaceParamWidth-1:0] pace_params;
  logic                   pace_mem_init_done;

`ifdef DOUBLE_BW
  assign pace_mem_we = fpu_pace_mode_i.enable & vrf_we[VLSU_VD_WD0] & (~pace_mem_init_done);
`else
  assign pace_mem_we = fpu_pace_mode_i.enable & vrf_we[PaceLdIdx] & (~pace_mem_init_done);
`endif

  always_comb begin
    vrf_we_mask = vrf_we;
    vrf_wvalid  = vrf_wvalid_mask;
    vrf_we_mask[PaceLdIdx] = fpu_pace_mode_i.enable & (~pace_mem_init_done) ? 1'b0 : vrf_we[PaceLdIdx];
    vrf_wvalid[PaceLdIdx]  = fpu_pace_mode_i.enable & (~pace_mem_init_done) ? vrf_we[PaceLdIdx] : vrf_wvalid_mask[PaceLdIdx];
`ifdef DOUBLE_BW
    vrf_we_mask[VLSU_VD_WD1] = fpu_pace_mode_i.enable & (~pace_mem_init_done) ? 1'b0 : vrf_we[VLSU_VD_WD1];
    vrf_wvalid[VLSU_VD_WD1]  = fpu_pace_mode_i.enable & (~pace_mem_init_done) ? vrf_we[VLSU_VD_WD1] : vrf_wvalid_mask[VLSU_VD_WD1];
`endif
  end

  spatz_vrf #(
    .NrReadPorts (NrReadPorts ),
    .NrWritePorts(NrWritePorts),
    .FpuBufDepth (FpuBufDepth )
  ) i_vrf (
    .clk_i           (clk_i         ),
    .rst_ni          (rst_ni        ),
    .testmode_i      (testmode_i    ),
    // Write Ports
    .waddr_i         (vrf_waddr_buf ),
    .wdata_i         (vrf_wdata_buf ),
    .we_i            (vrf_we_mask    ),
    .wbe_i           (vrf_wbe_buf    ),
    .wvalid_o        (vrf_wvalid_mask),
  `ifdef BUF_FPU
    .fpu_buf_usage_i (vfu_buf_usage ),
  `endif
    // Read Ports
    .raddr_i         (vrf_raddr     ),
    .re_i            (vrf_re        ),
    .rdata_o         (vrf_rdata     ),
    .rvalid_o        (vrf_rvalid    )
`ifdef VENTAGLIO
    ,
    .vtl_redirect_write_i (vrf_vtl_redirect_write),
    .vtl_redirect_read_i  (vrf_vtl_redirect_read ),
    // master ports to VTL (write side)
    .waddr_o        (vrf_vtl_waddr),
    .wdata_o        (vrf_vtl_wdata),
    .we_o           (vrf_vtl_we),
    .wbe_o          (vrf_vtl_wbe),
    .wvalid_i       (vrf_vtl_wvalid),
    .wscatter_en_o  (vrf_vtl_wscatter_en),
    // master ports to VTL (read side)
    .raddr_o        (vrf_vtl_raddr),
    .re_o           (vrf_vtl_re),
    .rdata_i        (vrf_vtl_rdata),
    .rvalid_i       (vrf_vtl_rvalid),
    .rgather_en_o   (vrf_vtl_rgather_en)
`endif
  );

  pace_mem #(
    .BufDepth  (PaceBufDepth ),
    .BufWidth  (PaceBufWidth ),
    .ParamWidth(PaceParamWidth)
  ) i_pace_mem (
    .clk_i  (clk_i                  ),
    .rst_ni (rst_ni                 ),
    .we_i   (pace_mem_we            ),
    .done_o (pace_mem_init_done     ),
    .init_i (fpu_pace_mode_i.enable ),
`ifdef DOUBLE_BW
    .data_i (vrf_wdata_buf[VLSU_VD_WD0]),
`else
    .data_i (vrf_wdata_buf[PaceLdIdx]),
`endif
    .data_o (pace_params            )
  );

  ////////////////
  // Controller //
  ////////////////

  // Scoreboard read enable and write enable input signals
  logic      [NrReadPorts-1:0]              sb_re;
  logic      [NrWritePorts-1:0]             sb_we, sb_we_buf;
  spatz_id_t [NrReadPorts+NrWritePorts-1:0] sb_id, sb_buf_id;

  spatz_controller #(
    .NrVregfilePorts  (NrReadPorts+NrWritePorts),
    .NrWritePorts     (NrWritePorts            ),
    .RegisterRsp      (RegisterRsp             ),
    .spatz_issue_req_t(spatz_issue_req_t       ),
    .spatz_issue_rsp_t(spatz_issue_rsp_t       ),
    .spatz_rsp_t      (spatz_rsp_t             )
  ) i_controller (
    .clk_i            (clk_i           ),
    .rst_ni           (rst_ni          ),
    // X-intf
    .issue_valid_i    (issue_valid     ),
    .issue_ready_o    (issue_ready     ),
    .issue_req_i      (issue_req       ),
    .issue_rsp_o      (issue_rsp       ),
    .rsp_valid_o      (resp_valid      ),
    .rsp_ready_i      (resp_ready      ),
    .rsp_o            (resp            ),
    // FPU side channel
    .fpu_rnd_mode_i   (fpu_rnd_mode_i  ),
    .fpu_fmt_mode_i   (fpu_fmt_mode_i  ),
    // Spatz request
    .spatz_req_valid_o(spatz_req_valid ),
    .spatz_req_o      (spatz_req       ),
    // VFU
    .vfu_req_ready_i  (vfu_req_ready       ),
    .vfu_rsp_valid_i  (vfu_rsp_buf_valid   ),
    .vfu_rsp_ready_o  (vfu_rsp_ready       ),
    .vfu_rsp_i        (vfu_rsp_buf         ),
    // VLSU
    .vlsu_req_ready_i (vlsu_req_ready      ),
    .vlsu_rsp_valid_i (vlsu_rsp_buf_valid  ),
    .vlsu_rsp_i       (vlsu_rsp_buf        ),
    // VLSD
    .vsldu_req_ready_i(vsldu_req_ready ),
    .vsldu_rsp_valid_i(vsldu_rsp_valid ),
    .vsldu_rsp_i      (vsldu_rsp       ),
`ifdef VENTAGLIO
    // VTL (Ventaglio): separate admit/retire path
    .vtl_req_ready_i  (vtl_req_ready   ),
    .vtl_rsp_valid_i  (vtl_rsp_valid   ),
    .vtl_rsp_i        (vtl_rsp         ),
`endif
    // Scoreboard check
    .sb_id_i          (sb_buf_id         ),
    .sb_wrote_result_i(vrf_wvalid        ),
    .sb_enable_i      ({sb_we_buf, sb_re}),
    .sb_enable_o      ({vrf_we, vrf_re}  )
`ifdef VENTAGLIO
    ,
    .sb_vtl_redirect_read_o  (vrf_vtl_redirect_read ),
    .sb_vtl_redirect_write_o (vrf_vtl_redirect_write),
    // Per-vreg "writer in flight" feed to Ventaglio's prefetch trigger.
    .vreg_write_pending_o    (vreg_write_pending    )
`endif
  );

  /////////
  // BUF //
  /////////

`ifdef BUF_FPU
  // Buffering of FPU writes to VRF to hide the conflicts
  // This feature allows to not stall the FPU and achieve high FPU utilizations
  logic vfu_buf_en, vfu_buf_push, vfu_buf_pop, vrf_vfu_wvalid, vfu_buf_full, vfu_buf_empty;

  // If cannot write to VRF for a valid VFU result, enable the buffer
  assign vfu_buf_en =  sb_we[VFU_VD_WD] && (!vrf_wvalid[VFU_VD_WD] || (vrf_wvalid[VFU_VD_WD] && !vfu_buf_empty));
  assign vfu_buf_push = vfu_buf_en && !vfu_buf_full;
  assign vfu_buf_pop = vrf_wvalid[VFU_VD_WD] && !vfu_buf_empty;

  // Ack 1'b1 to the VFU as long as the buffer is not full
  assign vrf_vfu_wvalid = sb_we[VFU_VD_WD] && !vfu_buf_full;

  cc_fifo #(
    .FallThrough (1'b0        ),
    .data_t        (vfu_buf_t   ),
    .Depth        (FpuBufDepth )
  ) i_vfu_buf (
    .clk_i      (clk_i                   ),
    .rst_ni     (rst_ni                  ),
    .clr_i     (1'b0),
    .flush_i    (1'b0                    ),
    .full_o     (vfu_buf_full            ),
    .empty_o    (vfu_buf_empty           ),
    .usage_o    (vfu_buf_usage           ),
    .data_i     ({vrf_wdata[VFU_VD_WD],
                  vrf_waddr[VFU_VD_WD],
                  vrf_wbe  [VFU_VD_WD],
                  sb_id [SB_VFU_VD_WD],
                  vfu_rsp,
                  vfu_rsp_valid}         ),
    .push_i     (vfu_buf_push            ),
    .data_o     (vfu_buf_data            ),
    .pop_i      (vfu_buf_pop             )
  );

`ifdef DOUBLE_BW
  // Buffering of VLSU1 when conflicting with VLSU0
  logic vlsu_buf_en, vlsu_buf_push, vlsu_buf_pop, vrf_vlsu_wvalid, vlsu_buf_full, vlsu_buf_empty;

  assign vlsu_buf_en =  sb_we[VLSU_VD_WD1] && (!vrf_wvalid[VLSU_VD_WD1] || (vrf_wvalid[VLSU_VD_WD1] && !vlsu_buf_empty));
  assign vlsu_buf_push = vlsu_buf_en && !vlsu_buf_full;
  assign vlsu_buf_pop = vrf_wvalid[VLSU_VD_WD1] && !vlsu_buf_empty;
  assign vrf_vlsu_wvalid = sb_we[VLSU_VD_WD1] && !vlsu_buf_full;

  cc_fifo #(
    .FallThrough (1'b0         ),
    .data_t        (vlsu_buf_t   ),
    .Depth        (VlsuBufDepth )
  ) i_vlsu_buf (
    .clk_i      (clk_i                    ),
    .rst_ni     (rst_ni                   ),
    .clr_i     (1'b0),
    .flush_i    (1'b0                     ),
    .full_o     (vlsu_buf_full            ),
    .empty_o    (vlsu_buf_empty           ),
    .usage_o    (vlsu_buf_usage           ),
    .data_i     ({vrf_wdata[VLSU_VD_WD1],
                  vrf_waddr[VLSU_VD_WD1],
                  vrf_wbe  [VLSU_VD_WD1],
                  sb_id [SB_VLSU_VD_WD1],
                  vlsu_rsp,
                  vlsu_rsp_valid}         ),
    .push_i     (vlsu_buf_push            ),
    .data_o     (vlsu_buf_data            ),
    .pop_i      (vlsu_buf_pop             )
  );

`endif
`endif

  always_comb begin
    // Default assignments
    sb_we_buf = sb_we;
    vrf_wdata_buf = vrf_wdata;
    vrf_waddr_buf = vrf_waddr;
    vrf_wbe_buf = vrf_wbe;
    sb_buf_id = sb_id;
    // Responses
    vfu_rsp_buf = vfu_rsp;
    // vfu_rsp_valid (result_tag.last && ...) is computed inside spatz_vfu.sv
    // purely from its own internal pipeline state, with no confirmation that
    // the corresponding VRF write request was actually accepted this cycle.
    // The i_vfu_buf FIFO below only re-adds that vrf_wvalid confirmation
    // while it is actively buffering (write-port contention); the common
    // buffer-empty passthrough case had none, letting the controller clear
    // this instruction's write_table/scoreboard entry (unblocking dependent
    // reads) before the VRF write for the last element has actually landed.
    // A "wb" (scalar move, e.g. vmv.x.s) response never issues a VRF write
    // at all, so it must not be gated on vrf_wvalid.
    vfu_rsp_buf_valid = vfu_rsp_valid & (vfu_rsp.wb | vrf_wvalid[VFU_VD_WD]);
    vlsu_rsp_buf = vlsu_rsp;
    vlsu_rsp_buf_valid = vlsu_rsp_valid;

    // If the buffering feature is used for the FPU or VLSU,
    // Use the metadata to commit the data to the VRF
`ifdef BUF_FPU
    if (!vfu_buf_empty) begin
      sb_we_buf    [VFU_VD_WD] = 1'b1;
      vrf_wdata_buf[VFU_VD_WD] = vfu_buf_data.wdata;
      vrf_waddr_buf[VFU_VD_WD] = vfu_buf_data.waddr;
      vrf_wbe_buf  [VFU_VD_WD] = vfu_buf_data.wbe;
      sb_buf_id    [SB_VFU_VD_WD] = vfu_buf_data.wid;
      vfu_rsp_buf = vfu_buf_data.rsp;
      vfu_rsp_buf_valid = vfu_buf_data.rsp_valid & vrf_wvalid[VFU_VD_WD];
    end else begin
      // If the buffer is being enabled in this cycle, don't send the response now
      if (vfu_buf_en) begin
        vfu_rsp_buf_valid = 1'b0;
      end
    end

`ifdef DOUBLE_BW
    // VLSU1 buffering
    // Do not retire a load response while interface 1's final VRF write is
    // only being accepted into the conflict buffer. The dependent consumer
    // would otherwise observe the load as finished before the buffered word
    // has become visible in the VRF.
    if (vlsu_rsp_valid && vlsu_buf_push)
      vlsu_rsp_buf_valid = 1'b0;

    if (!vlsu_buf_empty) begin
      sb_we_buf    [VLSU_VD_WD1] = 1'b1;
      vrf_wdata_buf[VLSU_VD_WD1] = vlsu_buf_data.wdata;
      vrf_waddr_buf[VLSU_VD_WD1] = vlsu_buf_data.waddr;
      vrf_wbe_buf  [VLSU_VD_WD1] = vlsu_buf_data.wbe;
      sb_buf_id    [SB_VLSU_VD_WD1] = vlsu_buf_data.wid;
      if (vlsu_buf_data.rsp_valid) begin
        vlsu_rsp_buf = vlsu_buf_data.rsp;
        vlsu_rsp_buf_valid = vrf_wvalid[VLSU_VD_WD1];
      end
    end else begin
      // If the buffer is being enabled in this cycle, don't send the response now
      if (vlsu_buf_en) begin
        vlsu_rsp_buf_valid = 1'b0;
      end
    end
`endif
`endif
  end // always_comb

  /////////
  // VFU //
  /////////

`ifdef VENTAGLIO
  logic vfu_vtl_req_ready;
`endif

  spatz_vfu #(
    .FPUImplementation(FPUImplementation)
  ) i_vfu (
    .clk_i            (clk_i                                                   ),
    .rst_ni           (rst_ni                                                  ),
    .hart_id_i        (hart_id_i                                               ),
    // Request
    .spatz_req_i         (spatz_req                                            ),
    .spatz_req_valid_i   (spatz_req_valid                                      ),
    .spatz_req_ready_o   (vfu_req_ready                                        ),
`ifdef VENTAGLIO
    .vfu_vtl_req_ready_o (vfu_vtl_req_ready                                    ),
`endif
    // Response
    .vfu_rsp_valid_o  (vfu_rsp_valid                                           ),
    .vfu_rsp_ready_i  (vfu_rsp_ready                                           ),
    .vfu_rsp_o        (vfu_rsp                                                 ),
    // VRF
    .vrf_waddr_o      (vrf_waddr[VFU_VD_WD]                                    ),
    .vrf_wdata_o      (vrf_wdata[VFU_VD_WD]                                    ),
    .vrf_we_o         (sb_we[VFU_VD_WD]                                        ),
    .vrf_wbe_o        (vrf_wbe[VFU_VD_WD]                                      ),
`ifdef BUF_FPU
    .vrf_wvalid_i     (vrf_vfu_wvalid                                          ),
`else
    .vrf_wvalid_i     (vrf_wvalid[VFU_VD_WD]                                   ),
`endif
    .vrf_raddr_o      (vrf_raddr[VFU_VD_RD:VFU_VS2_RD]                         ),
    .vrf_re_o         (sb_re[VFU_VD_RD:VFU_VS2_RD]                             ),
    .vrf_rdata_i      (vrf_rdata[VFU_VD_RD:VFU_VS2_RD]                         ),
    .vrf_rvalid_i     (vrf_rvalid[VFU_VD_RD:VFU_VS2_RD]                        ),
    .vrf_id_o         ({sb_id[SB_VFU_VD_WD], sb_id[SB_VFU_VD_RD:SB_VFU_VS2_RD]}),
    // FPU side-channel
    .pace_mode_i      (fpu_pace_mode_i                                         ),
    .pace_param_i     (pace_params                                             ),
    .fpu_status_o     (fpu_status_o                                            )
  );

  //////////
  // VLSU //
  //////////

`ifdef DOUBLE_BW
  spatz_doublebw_vlsu #(
    .NrMemPorts      (NrMemPorts      ),
    .spatz_mem_req_t (spatz_mem_req_t ),
    .spatz_mem_rsp_t (spatz_mem_rsp_t )
  ) i_vlsu (
    .clk_i                   (clk_i                                                ),
    .rst_ni                  (rst_ni                                               ),
    .hart_id_i               (hart_id_i                                            ),
    // Request
    .spatz_req_i             (spatz_req                                            ),
    .spatz_req_valid_i       (spatz_req_valid                                      ),
    .spatz_req_ready_o       (vlsu_req_ready                                       ),
    // Response
    .vlsu_rsp_valid_o        (vlsu_rsp_valid                                       ),
    .vlsu_rsp_o              (vlsu_rsp                                             ),
    .vlsu_buf_full_i         (vlsu_buf_full                                        ),
    .vlsu_buf_empty_i        (vlsu_buf_empty                                       ),
    // VRF
    .vrf_wvalid_i            ({vrf_vlsu_wvalid, vrf_wvalid[VLSU_VD_WD0]}           ),
    .vrf_waddr_o             (vrf_waddr[VLSU_VD_WD1:VLSU_VD_WD0]                   ),
    .vrf_wdata_o             (vrf_wdata[VLSU_VD_WD1:VLSU_VD_WD0]                   ),
    .vrf_we_o                (sb_we[VLSU_VD_WD1:VLSU_VD_WD0]                       ),
    .vrf_wbe_o               (vrf_wbe[VLSU_VD_WD1:VLSU_VD_WD0]                     ),
    // Read from VRF
    .vrf_raddr_o             (vrf_raddr[VLSU_VS2_RD1:VLSU_VD_RD0]                  ),
    .vrf_re_o                (sb_re[VLSU_VS2_RD1:VLSU_VD_RD0]                      ),
    .vrf_rdata_i             (vrf_rdata[VLSU_VS2_RD1:VLSU_VD_RD0]                  ),
    .vrf_rvalid_i            (vrf_rvalid[VLSU_VS2_RD1:VLSU_VD_RD0]                 ),
    .vrf_id_o                ({sb_id[SB_VLSU_VD_WD1], sb_id[SB_VLSU_VS2_RD1], sb_id[SB_VLSU_VD_RD1],   // VLSU Interface-1
                               sb_id[SB_VLSU_VD_WD0], sb_id[SB_VLSU_VS2_RD0], sb_id[SB_VLSU_VD_RD0]}), // VLSU Interface-0
    // Interface Memory
    .spatz_mem_req_o         (spatz_mem_req_o                                      ),
    .spatz_mem_req_valid_o   (spatz_mem_req_valid_o                                ),
    .spatz_mem_req_ready_i   (spatz_mem_req_ready_i                                ),
    .spatz_mem_rsp_i         (spatz_mem_rsp_i                                      ),
    .spatz_mem_rsp_valid_i   (spatz_mem_rsp_valid_i                                ),
    .spatz_mem_finished_o    (spatz_mem_finished                                   ),
    .spatz_mem_str_finished_o(spatz_mem_str_finished                               )
  );
`else
  spatz_vlsu #(
    .NrMemPorts      (NrMemPorts      ),
    .spatz_mem_req_t (spatz_mem_req_t ),
    .spatz_mem_rsp_t (spatz_mem_rsp_t )
  ) i_vlsu (
    .clk_i                   (clk_i                                                ),
    .rst_ni                  (rst_ni                                               ),
    .hart_id_i               (hart_id_i                                            ),
    // Request
    .spatz_req_i             (spatz_req                                            ),
    .spatz_req_valid_i       (spatz_req_valid                                      ),
    .spatz_req_ready_o       (vlsu_req_ready                                       ),
    // Response
    .vlsu_rsp_valid_o        (vlsu_rsp_valid                                       ),
    .vlsu_rsp_o              (vlsu_rsp                                             ),
    // VRF
    .vrf_waddr_o             (vrf_waddr[VLSU_VD_WD]                                ),
    .vrf_wdata_o             (vrf_wdata[VLSU_VD_WD]                                ),
    .vrf_we_o                (sb_we[VLSU_VD_WD]                                    ),
    .vrf_wbe_o               (vrf_wbe[VLSU_VD_WD]                                  ),
    .vrf_wvalid_i            (vrf_wvalid[VLSU_VD_WD]                               ),
    .vrf_raddr_o             ({vrf_raddr[VLSU_VS2_RD],  vrf_raddr[VLSU_VD_RD] }    ),
    .vrf_re_o                ({sb_re[VLSU_VS2_RD],      sb_re[VLSU_VD_RD]     }    ),
    .vrf_rdata_i             ({vrf_rdata[VLSU_VS2_RD],  vrf_rdata[VLSU_VD_RD] }    ),
    .vrf_rvalid_i            ({vrf_rvalid[VLSU_VS2_RD], vrf_rvalid[VLSU_VD_RD]}    ),
    .vrf_id_o                ({sb_id[SB_VLSU_VD_WD],    sb_id[VLSU_VS2_RD],    sb_id[VLSU_VD_RD]}),
    // Interface Memory
    .spatz_mem_req_o         (spatz_mem_req_o                                      ),
    .spatz_mem_req_valid_o   (spatz_mem_req_valid_o                                ),
    .spatz_mem_req_ready_i   (spatz_mem_req_ready_i                                ),
    .spatz_mem_rsp_i         (spatz_mem_rsp_i                                      ),
    .spatz_mem_rsp_valid_i   (spatz_mem_rsp_valid_i                                ),
    .spatz_mem_finished_o    (spatz_mem_finished                                   ),
    .spatz_mem_str_finished_o(spatz_mem_str_finished                               )
  );
`endif

  /////////////////
  // VSLDU + VTL //
  /////////////////
  //
  // With Ventaglio compiled in (`ifdef VENTAGLIO), VSLDU and VTL share the
  // physical VRF write/read slot (VSLDU_VD_WD / VSLDU_VS2_RD) through a
  // priority mux (VTL wins). The two units have separate admit/retire
  // handshakes to the controller (vsldu_* and vtl_*).
  //
  // Without Ventaglio, only VSLDU exists; it wires directly to the slot
  // and the cluster is vanilla Spatz.

`ifdef VENTAGLIO
  // Per-unit master signals that feed the priority arbiter below.
  vrf_addr_t mst_vtl_waddr,  mst_vsldu_waddr;
  vrf_data_t mst_vtl_wdata,  mst_vsldu_wdata;
  logic      mst_vtl_we,     mst_vsldu_we;
  vrf_be_t   mst_vtl_wbe,    mst_vsldu_wbe;
  logic      mst_vtl_wvalid, mst_vsldu_wvalid;
  spatz_id_t mst_vtl_wid,    mst_vsldu_wid;

  vrf_addr_t mst_vtl_raddr,  mst_vsldu_raddr;
  logic      mst_vtl_re,     mst_vsldu_re;
  vrf_data_t mst_vtl_rdata,  mst_vsldu_rdata;
  logic      mst_vtl_rvalid, mst_vsldu_rvalid;
  spatz_id_t mst_vtl_rid,    mst_vsldu_rid;

  always_comb begin : proc_arbitrate_vtl_vsldu_write
    sb_we[VSLDU_VD_WD]     = 1'b0; // default disable
    vrf_waddr[VSLDU_VD_WD] = '0;
    vrf_wdata[VSLDU_VD_WD] = '0;
    vrf_wbe[VSLDU_VD_WD]   = '0;
    mst_vtl_wvalid         = '0;
    mst_vsldu_wvalid       = '0;
    sb_id[SB_VSLDU_VD_WD]  = '0;
    // VTL issues write requests with priority
    if (mst_vtl_we) begin
      vrf_waddr[VSLDU_VD_WD] = mst_vtl_waddr;
      vrf_wdata[VSLDU_VD_WD] = mst_vtl_wdata;
      sb_we[VSLDU_VD_WD]     = 1'b1;
      vrf_wbe[VSLDU_VD_WD]   = mst_vtl_wbe;
      mst_vtl_wvalid         = vrf_wvalid[VSLDU_VD_WD];
      sb_id[SB_VSLDU_VD_WD]  = mst_vtl_wid;
    end else if (mst_vsldu_we) begin
      vrf_waddr[VSLDU_VD_WD] = mst_vsldu_waddr;
      vrf_wdata[VSLDU_VD_WD] = mst_vsldu_wdata;
      sb_we[VSLDU_VD_WD]     = 1'b1;
      vrf_wbe[VSLDU_VD_WD]   = mst_vsldu_wbe;
      mst_vsldu_wvalid       = vrf_wvalid[VSLDU_VD_WD];
      sb_id[SB_VSLDU_VD_WD]  = mst_vsldu_wid;
    end
  end

  always_comb begin : proc_arbitrate_vtl_vsldu_read
    vrf_raddr[VSLDU_VS2_RD] = '0;
    sb_re[VSLDU_VS2_RD]     = 1'b0;
    mst_vtl_rdata           = '0;
    mst_vtl_rvalid          = '0;
    mst_vsldu_rdata         = '0;
    mst_vsldu_rvalid        = '0;
    sb_id[SB_VSLDU_VS2_RD]  = '0;
    if (mst_vtl_re) begin
      vrf_raddr[VSLDU_VS2_RD] = mst_vtl_raddr;
      sb_re[VSLDU_VS2_RD]     = 1'b1;
      mst_vtl_rdata           = vrf_rdata[VSLDU_VS2_RD];
      mst_vtl_rvalid          = vrf_rvalid[VSLDU_VS2_RD];
      sb_id[SB_VSLDU_VS2_RD]  = mst_vtl_rid;
    end else if (mst_vsldu_re) begin
      vrf_raddr[VSLDU_VS2_RD] = mst_vsldu_raddr;
      sb_re[VSLDU_VS2_RD]     = 1'b1;
      mst_vsldu_rdata         = vrf_rdata[VSLDU_VS2_RD];
      mst_vsldu_rvalid        = vrf_rvalid[VSLDU_VS2_RD];
      sb_id[SB_VSLDU_VS2_RD]  = mst_vsldu_rid;
    end
  end

  ventaglio #(
    .NarrowDataWidth  (VRFWordWidth ),
    .WideDataWidth    (VRFWordWidth * VENTAGLIO_WFACTOR)
  ) i_vtl (
    .clk_i            (clk_i             ),
    .rst_ni           (rst_ni            ),
    .testmode_i       (testmode_i        ),

    // Request
    .spatz_req_i          (spatz_req                                      ),
    .spatz_req_valid_i    (spatz_req_valid                                ),
    // VTL has its own admit handshake to the controller. Ventaglio drives
    // vtl_req_ready high when its spill is free to accept the next
    // use_vtl op (vventclr or vfx); the controller gates vtl_stall on it.
    .vtl_req_ready_o      (vtl_req_ready                                  ),
    // req_ready signal from VFU
    .spatz_vfu_req_ready_i(vfu_vtl_req_ready                              ),
    // Response — VTL retirement path, distinct from VSLDU.
    // Only fires for vventclr; vfxmacc/vfxmul retire via vfu_rsp.
    .vtl_rsp_valid_o  (vtl_rsp_valid                                  ),
    .vtl_rsp_o        (vtl_rsp                                        ),
    // from VFU
    .vfu_rsp_valid_i  (vfu_rsp_valid    ),
    .vfu_rsp_i        (vfu_rsp          ),

    // slave ports from VRF
    .waddr_i          (vrf_vtl_waddr     ),
    .wdata_i          (vrf_vtl_wdata     ),
    .we_i             (vrf_vtl_we        ),
    .wbe_i            (vrf_vtl_wbe       ),
    .wvalid_o         (vrf_vtl_wvalid    ),
    .wscatter_en_i    (vrf_vtl_wscatter_en),

    .raddr_i          (vrf_vtl_raddr     ),
    .re_i             (vrf_vtl_re        ),
    .rdata_o          (vrf_vtl_rdata     ),
    .rvalid_o         (vrf_vtl_rvalid    ),
    .rgather_en_i     (vrf_vtl_rgather_en),
    // master ports to VRF (routed through the arbiter above)
    .vrf_waddr_o      (mst_vtl_waddr       ),
    .vrf_wdata_o      (mst_vtl_wdata       ),
    .vrf_we_o         (mst_vtl_we          ),
    .vrf_wbe_o        (mst_vtl_wbe         ),
    .vrf_wvalid_i     (mst_vtl_wvalid      ),
    .vrf_raddr_o      (mst_vtl_raddr       ),
    .vrf_re_o         (mst_vtl_re          ),
    .vrf_rdata_i      (mst_vtl_rdata       ),
    .vrf_rvalid_i     (mst_vtl_rvalid      ),
    .vrf_id_o         ({mst_vtl_wid, mst_vtl_rid})
  );

  spatz_vsldu i_vsldu (
    .clk_i            (clk_i                                          ),
    .rst_ni           (rst_ni                                         ),
    // Request
    .spatz_req_i      (spatz_req                                      ),
    .spatz_req_valid_i(spatz_req_valid                                ),
    .spatz_req_ready_o(vsldu_req_ready                                ),
    // Response
    .vsldu_rsp_valid_o(vsldu_rsp_valid                                ),
    .vsldu_rsp_o      (vsldu_rsp                                      ),
    // VRF (master ports; routed through proc_arbitrate_vtl_vsldu_* muxes)
    .vrf_waddr_o      (mst_vsldu_waddr                                ),
    .vrf_wdata_o      (mst_vsldu_wdata                                ),
    .vrf_we_o         (mst_vsldu_we                                   ),
    .vrf_wbe_o        (mst_vsldu_wbe                                  ),
    .vrf_wvalid_i     (mst_vsldu_wvalid                               ),
    .vrf_raddr_o      (mst_vsldu_raddr                                ),
    .vrf_re_o         (mst_vsldu_re                                   ),
    .vrf_rdata_i      (mst_vsldu_rdata                                ),
    .vrf_rvalid_i     (mst_vsldu_rvalid                               ),
    .vrf_id_o         ({mst_vsldu_wid, mst_vsldu_rid}                 )
  );

`else // !VENTAGLIO — vanilla Spatz: VSLDU wires directly to its slot.

  spatz_vsldu i_vsldu (
    .clk_i            (clk_i                                          ),
    .rst_ni           (rst_ni                                         ),
    // Request
    .spatz_req_i      (spatz_req                                      ),
    .spatz_req_valid_i(spatz_req_valid                                ),
    .spatz_req_ready_o(vsldu_req_ready                                ),
    // Response
    .vsldu_rsp_valid_o(vsldu_rsp_valid                                ),
    .vsldu_rsp_o      (vsldu_rsp                                      ),
    // VRF (direct connection to the VSLDU slot)
    .vrf_waddr_o      (vrf_waddr[VSLDU_VD_WD]                         ),
    .vrf_wdata_o      (vrf_wdata[VSLDU_VD_WD]                         ),
    .vrf_we_o         (sb_we[VSLDU_VD_WD]                             ),
    .vrf_wbe_o        (vrf_wbe[VSLDU_VD_WD]                           ),
    .vrf_wvalid_i     (vrf_wvalid[VSLDU_VD_WD]                        ),
    .vrf_raddr_o      (vrf_raddr[VSLDU_VS2_RD]                        ),
    .vrf_re_o         (sb_re[VSLDU_VS2_RD]                            ),
    .vrf_rdata_i      (vrf_rdata[VSLDU_VS2_RD]                        ),
    .vrf_rvalid_i     (vrf_rvalid[VSLDU_VS2_RD]                       ),
    .vrf_id_o         ({sb_id[SB_VSLDU_VD_WD], sb_id[SB_VSLDU_VS2_RD]})
  );
`endif // VENTAGLIO

  ////////////////
  // Assertions //
  ////////////////

  if (spatz_pkg::N_IPU == 0)
    $error("[spatz] Each Spatz needs at least one IPU");

  if (spatz_pkg::N_FU != 2**$clog2(spatz_pkg::N_FU))
    $error("[spatz] The number of FUs needs to be a power of two");

  if (spatz_pkg::VLEN != 2**$clog2(spatz_pkg::VLEN))
    $error("[spatz] The vector length needs to be a power of two");

  if (spatz_pkg::ELEN*spatz_pkg::N_FU > spatz_pkg::VLEN)
    $error("[spatz] VLEN needs a min size of N_FU*%d.", spatz_pkg::ELEN);

  if (NrMemPorts == 0)
    $error("[spatz] Spatz requires at least one memory port.");

endmodule : spatz
