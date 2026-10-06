// Copyright 2023 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Author: Matheus Cavalcante, ETH Zurich
//
// The Vector Functional Unit (VFU) executes all arithmetic and logical
// vector instructions. It can be configured with a parameterizable amount
// of IPUs that work in parallel.

module spatz_vfu
  import spatz_pkg::*;
  import rvv_pkg::*;
  import cf_math_pkg::idx_width;
  import fpnew_pkg::*; #(
    /// FPU configuration.
    parameter fpu_implementation_t FPUImplementation = fpu_implementation_t'(0)
  ) (
    input  logic             clk_i,
    input  logic             rst_ni,
    input  logic [31:0]      hart_id_i,
    // Spatz req
    input  spatz_req_t       spatz_req_i,
    input  logic             spatz_req_valid_i,
    output logic             spatz_req_ready_o,
`ifdef VENTAGLIO
    output logic             vfu_vtl_req_ready_o,
`endif
    // VFU response
    output logic             vfu_rsp_valid_o,
    input  logic             vfu_rsp_ready_i,
    output vfu_rsp_t         vfu_rsp_o,
    // VRF
    output vrf_addr_t        vrf_waddr_o,
    output vrf_data_t        vrf_wdata_o,
    output logic             vrf_we_o,
    output vrf_be_t          vrf_wbe_o,
    input  logic             vrf_wvalid_i,
    output spatz_id_t  [3:0] vrf_id_o,
    output vrf_addr_t  [2:0] vrf_raddr_o,
    output logic       [2:0] vrf_re_o,
    input  vrf_data_t  [2:0] vrf_rdata_i,
    input  logic       [2:0] vrf_rvalid_i,
    // FPU side channel
    output status_t          fpu_status_o,
    output logic             vxsat_o
  );

// Include FF
`include "common_cells/registers.svh"

  // Instruction tag (propagated together with the operands through the pipelines)
  typedef struct packed {
    spatz_id_t id;

    vew_e vsew;
    vlen_t vstart;

    // Encodes both the scalar RD and the VD address in the VRF
    vfu_rsp_addr_t vd_addr;
    logic wb;
    logic last;

    // Is this a narrowing instruction?
    logic narrowing;
    logic narrowing_upper;

    // Is this a reduction?
    logic reduction;
    // Is this a comparison?
    logic is_cmp;
    vlen_t vl;
    // Is this instruction unmasked?
    logic vm;
    // Is this instruction mask agnostic?
    logic vma;
    // Is this merge instruction?
    logic is_merge;
    // valid bytes in this VRF word
    logic [$clog2(VRFWordBWidth+1)-1:0] valid_bytes;
  } vfu_tag_t;

  logic [N_FPU-1:0] fpu_load_ready;
  logic [N_FPU-1:0] fpu_stage_valid;
  logic fpu_gather_busy;


  ///////////////////////
  //  Operation queue  //
  ///////////////////////

  spatz_req_t spatz_req;
  logic       spatz_req_valid;
  logic       spatz_req_ready;

`ifdef VENTAGLIO
  assign      vfu_vtl_req_ready_o = spatz_req_ready;
`endif

  logic operation_queue_full, operation_queue_empty;
  spatz_req_t dimc_req;
  logic dimc_req_valid;
  logic dimc_queue_full, dimc_queue_empty;
  logic exclusive_request;
  logic [NrParallelInstructions-1:0] exclusive_inflight_q, exclusive_inflight_d;
  `FF(exclusive_inflight_q, exclusive_inflight_d, '0)
  assign exclusive_request = (FPU && spatz_req_i.op inside {[VFADD:VSDOTP], VFXMADD}) ||
                              spatz_req_i.op_arith.is_reduction;
  always_comb begin
    exclusive_inflight_d = exclusive_inflight_q;
    if (vfu_rsp_valid_o) exclusive_inflight_d[vfu_rsp_o.id] = 1'b0;
    if (spatz_req_valid_i && spatz_req_ready_o && spatz_req_i.ex_unit == VFU && exclusive_request)
      exclusive_inflight_d[spatz_req_i.id] = 1'b1;
  end

  // Keep upstream registered dispatch for ordinary arithmetic, so the
  // controller installs scoreboard dependencies before operand reads.
  logic operation_queue_ready;
  assign operation_queue_full = !operation_queue_ready;
  assign operation_queue_empty = !spatz_req_valid;
  spill_register #(
    .T(spatz_req_t)
  ) i_operation_queue (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .data_i(spatz_req_i),
    .valid_i(spatz_req_valid_i && spatz_req_i.ex_unit == VFU &&
             spatz_req_i.op != DIMC_OP && spatz_req_ready_o),
    .ready_o(operation_queue_ready),
    .data_o(spatz_req), .valid_o(spatz_req_valid), .ready_i(spatz_req_ready)
  );

  // A computing DIMC instruction must not block an independent IPU partial
  // sum. Dependencies remain enforced by the controller's VRF scoreboard.
  fifo_v3 #(
    .FALL_THROUGH(1'b0),
    .DEPTH       (2),
    .dtype       (spatz_req_t)
  ) i_dimc_operation_queue (
    .clk_i(clk_i), .rst_ni(rst_ni), .flush_i(1'b0), .testmode_i(1'b0),
    .full_o(dimc_queue_full), .empty_o(dimc_queue_empty), .usage_o(),
    .data_i(spatz_req_i),
    .push_i(spatz_req_valid_i && spatz_req_i.ex_unit == VFU &&
            spatz_req_i.op == DIMC_OP && spatz_req_ready_o),
    .data_o(dimc_req),
    .pop_i(dimc_instr_done && !dimc_queue_empty)
  );

  // Preserve ordering around FPU/reduction operations at dispatch. Blocking
  // them after queueing could deadlock a DIMC operand dependent on such work.
  assign spatz_req_ready_o = spatz_req_i.op == DIMC_OP ?
                              (!dimc_queue_full && !(|exclusive_inflight_q)) :
                              (!operation_queue_full &&
                               (!exclusive_request || (!dimc_req_valid && !dimc_inflight)));
  assign dimc_req_valid    = !dimc_queue_empty;

  ///////////////
  //  Control  //
  ///////////////

  // Vector length counter
  vlen_t vl_q, vl_d;
  `FF(vl_q, vl_d, '0)

  // Are we busy?
  logic busy_q, busy_d;
  `FF(busy_q, busy_d, 1'b0)

  // Number of elements in one VRF word
  logic [$clog2(N_FU*(ELEN/8)):0] nr_elem_word;
  logic [$clog2((ELEN/8)):0] nr_elem_word_divsqrt;
  assign nr_elem_word = (N_FU * (1 << (MAXEW - spatz_req.vtype.vsew))) >> spatz_req.op_arith.is_narrowing;

  logic is_divsqrt_insn;
  assign is_divsqrt_insn = spatz_req.op inside {VFDIV, VFSQRT};

  logic divsqrt_shared_active;
  assign divsqrt_shared_active = is_divsqrt_insn && divsqrt_is_shared;

  assign nr_elem_word_divsqrt = (1 << (MAXEW - spatz_req.vtype.vsew));

  // Are we running integer or floating-point instructions?
  typedef enum logic [1:0] {
    VFU_RunningIPU, VFU_RunningFPU
   } state_t;
   state_t state_d, state_q;
  `FF(state_q, state_d, VFU_RunningFPU)

  // Propagate the tags through the functional units
  vfu_tag_t ipu_result_tag, fpu_result_tag, result_tag, result_buf_tag_d, result_buf_tag_q;
  vfu_tag_t input_tag;
  logic result_buf_valid_d, result_buf_valid_q;

  assign result_tag = result_buf_valid_q ? result_buf_tag_q : (state_q == VFU_RunningIPU ? ipu_result_tag : fpu_result_tag);

  // Number of words advanced by vstart
  vlen_t vstart;
  assign vstart = ((spatz_req.vstart / N_FU) >> (MAXEW - spatz_req.vtype.vsew)) << (MAXEW - spatz_req.vtype.vsew);

  // Should we stall?
  logic stall;

  // Do we have the reduction operand?
  logic reduction_operand_ready_d, reduction_operand_ready_q;

  typedef enum logic [1:0]{
    READ_OPERANDS, READ_V0_t, READ_VD_t
  } operand_state_t;
   operand_state_t operand_state_d, operand_state_q;
  `FF(operand_state_q, operand_state_d, READ_OPERANDS)

  // Are the VFU operands ready?
  logic op1_is_ready, op2_is_ready, op3_is_ready, operands_ready;
  assign op1_is_ready   = spatz_req_valid && (operand_state_q == READ_OPERANDS) && ((!spatz_req.op_arith.is_reduction && (!spatz_req.use_vs1 || vrf_rvalid_i[1])) || (spatz_req.op_arith.is_reduction && reduction_operand_ready_q));
  assign op2_is_ready   = spatz_req_valid && (operand_state_q == READ_OPERANDS) && ((!spatz_req.use_vs2 || vrf_rvalid_i[0]) || spatz_req.op_arith.is_reduction);
  assign op3_is_ready   = spatz_req_valid && (operand_state_q == READ_OPERANDS) && (!spatz_req.vd_is_src || vrf_rvalid_i[2]);
  assign operands_ready = op1_is_ready && op2_is_ready && op3_is_ready && (!spatz_req.op_arith.is_scalar || vfu_rsp_ready_i) && !stall;

  // Valid operations
  logic [N_FU*ELENB-1:0] valid_operations;
  assign valid_operations = (spatz_req.op_arith.is_scalar || spatz_req.op_arith.is_reduction) ? (ELEN == 32 ? 4'hf : 8'hff) : '1;

  // Pending results
  logic [N_FU*ELENB-1:0] pending_results;
  assign pending_results = result_tag.wb ? (ELEN == 32 ? 4'hf : 8'hff) : '1;

  // Did we issue a microoperation?
  logic word_issued;

  // Currently running instructions
  logic [NrParallelInstructions-1:0] running_d, running_q;
  `FF(running_q, running_d, '0)

  // Is this a FPU instruction
  logic is_fpu_insn;
  assign is_fpu_insn = FPU && spatz_req.op inside {[VFADD:VSDOTP]};

  // Is the FPU busy?
  logic is_fpu_busy;

  // Is the IPU busy?
  logic is_ipu_busy;

  // DIMC control
  localparam int unsigned DimcSectionWidth    = 512;
  localparam int unsigned DimcWordsPerSection = DimcSectionWidth / VRFWordWidth;
  localparam int unsigned DimcSections        =
      (NrWordsPerVector + DimcWordsPerSection - 1) / DimcWordsPerSection;
  localparam int unsigned DimcRowWidth        = DimcSectionWidth * DimcSections;
  localparam int unsigned DimcSectionIdxWidth = DimcSections > 1 ? $clog2(DimcSections) : 1;
  localparam int unsigned DimcValidBitsWidth  = $clog2(DimcRowWidth + 1);
  localparam int unsigned DimcResultsPerWord  = VRFWordWidth / 32;
  localparam int unsigned DimcVqmmaccRows     = 8;
  localparam int unsigned DimcInitCycles      = 3;
  localparam int unsigned DimcInitCountWidth  =
      DimcInitCycles > 1 ? $clog2(DimcInitCycles) : 1;
  localparam int unsigned DimcVqmmaccWords    =
      (DimcVqmmaccRows * 32 + VRFWordWidth - 1) / VRFWordWidth;
  localparam int unsigned DimcResultWords     = 2 * DimcVqmmaccWords;
  localparam int unsigned DimcWriteIdxWidth   =
      DimcResultWords > 1 ? $clog2(DimcResultWords) : 1;
  typedef logic [DimcSectionWidth-1:0] dimc_data_t;

  typedef enum logic [2:0] {
    DimcIdle,
    DimcLoadFeature,
    DimcLoadKernel,
    DimcComputeInit,
    DimcComputeIssue,
    DimcWriteResult
  } dimc_state_t;

  dimc_state_t dimc_state_d, dimc_state_q;
  // The queue entry can retire before the last row because its operands and
  // configuration are already latched. Architectural completion is still the
  // final accepted VRF write, through dimc_rsp_done.
  logic dimc_request_released_d, dimc_request_released_q;
  logic [DimcSectionIdxWidth-1:0] dimc_section_d, dimc_section_q;
  logic [4:0]                     dimc_row_d, dimc_row_q;
  logic [DimcInitCountWidth-1:0]  dimc_init_count_d, dimc_init_count_q;
  logic [4:0]                     dimc_capture_count_d, dimc_capture_count_q;
  vrf_data_t                      dimc_result_d [DimcResultWords-1:0];
  vrf_data_t                      dimc_result_q [DimcResultWords-1:0];
  logic [4:0]                     dimc_tail_capture_count_d, dimc_tail_capture_count_q;
  vrf_data_t                      dimc_tail_result_d [DimcResultWords-1:0];
  vrf_data_t                      dimc_tail_result_q [DimcResultWords-1:0];
  logic                           dimc_wb_pending_d, dimc_wb_pending_q;
  spatz_id_t                      dimc_wb_id_d, dimc_wb_id_q;
  vrf_addr_t                      dimc_wb_base_addr_d, dimc_wb_base_addr_q;
  logic [DimcWriteIdxWidth-1:0]   dimc_wb_word_d, dimc_wb_word_q;
  logic [DimcWriteIdxWidth-1:0]   dimc_wb_last_word_d, dimc_wb_last_word_q;
  vrf_data_t                      dimc_wb_result_d [DimcResultWords-1:0];
  vrf_data_t                      dimc_wb_result_q [DimcResultWords-1:0];
  logic                           dimc_tail_pending_d, dimc_tail_pending_q;
  logic                           dimc_tail_done_pending_d, dimc_tail_done_pending_q;
  spatz_id_t                      dimc_tail_id_d, dimc_tail_id_q;
  vrf_addr_t                      dimc_tail_base_addr_d, dimc_tail_base_addr_q;
  logic [DimcWriteIdxWidth-1:0]   dimc_tail_first_word_d, dimc_tail_first_word_q;
  logic [DimcWriteIdxWidth-1:0]   dimc_tail_last_word_d, dimc_tail_last_word_q;
  logic [4:0]                     dimc_tail_row_limit_d, dimc_tail_row_limit_q;
  logic [4:0]                     dimc_tail_row_offset_d, dimc_tail_row_offset_q;
  spatz_id_t                      dimc_active_id_d, dimc_active_id_q;
  vreg_t                          dimc_active_vs1_d, dimc_active_vs1_q;
  vreg_t                          dimc_active_vs2_d, dimc_active_vs2_q;
  vreg_t                          dimc_active_vd_d, dimc_active_vd_q;
  vlen_t                          dimc_active_vl_d, dimc_active_vl_q;
  dimc_cfg_t                      dimc_active_cfg_d, dimc_active_cfg_q;

  `FF(dimc_state_q, dimc_state_d, DimcIdle)
  `FF(dimc_request_released_q, dimc_request_released_d, 1'b0)
  `FF(dimc_section_q, dimc_section_d, '0)
  `FF(dimc_row_q, dimc_row_d, '0)
  `FF(dimc_init_count_q, dimc_init_count_d, '0)
  `FF(dimc_capture_count_q, dimc_capture_count_d, '0)
  `FF(dimc_tail_capture_count_q, dimc_tail_capture_count_d, '0)
  `FF(dimc_wb_pending_q, dimc_wb_pending_d, 1'b0)
  `FF(dimc_wb_id_q, dimc_wb_id_d, '0)
  `FF(dimc_wb_base_addr_q, dimc_wb_base_addr_d, '0)
  `FF(dimc_wb_word_q, dimc_wb_word_d, '0)
  `FF(dimc_wb_last_word_q, dimc_wb_last_word_d, '0)
  `FF(dimc_tail_pending_q, dimc_tail_pending_d, 1'b0)
  `FF(dimc_tail_done_pending_q, dimc_tail_done_pending_d, 1'b0)
  `FF(dimc_tail_id_q, dimc_tail_id_d, '0)
  `FF(dimc_tail_base_addr_q, dimc_tail_base_addr_d, '0)
  `FF(dimc_tail_first_word_q, dimc_tail_first_word_d, '0)
  `FF(dimc_tail_last_word_q, dimc_tail_last_word_d, '0)
  `FF(dimc_tail_row_limit_q, dimc_tail_row_limit_d, '0)
  `FF(dimc_tail_row_offset_q, dimc_tail_row_offset_d, '0)
  `FF(dimc_active_id_q, dimc_active_id_d, '0)
  `FF(dimc_active_vs1_q, dimc_active_vs1_d, '0)
  `FF(dimc_active_vs2_q, dimc_active_vs2_d, '0)
  `FF(dimc_active_vd_q, dimc_active_vd_d, '0)
  `FF(dimc_active_vl_q, dimc_active_vl_d, '0)
  `FF(dimc_active_cfg_q, dimc_active_cfg_d, '0)
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      dimc_result_q <= '{default: '0};
      dimc_tail_result_q <= '{default: '0};
      dimc_wb_result_q <= '{default: '0};
    end else begin
      dimc_result_q <= dimc_result_d;
      dimc_tail_result_q <= dimc_tail_result_d;
      dimc_wb_result_q <= dimc_wb_result_d;
    end
  end

  logic       dimc_busy;
  logic       dimc_start;
  logic       dimc_turnover;
  logic       dimc_instr_done;
  logic       dimc_rsp_done;
  logic       dimc_load_feature;
  logic       dimc_vrf_read_feature;
  logic       dimc_vrf_read_kernel;
  logic       dimc_vrf_read_upper;
  logic       dimc_read_grant;
  logic       dimc_read_turn_q, dimc_read_turn_d;
  logic       dimc_read_request, normal_read_request;
  logic       dimc_inflight;
  `FF(dimc_read_turn_q, dimc_read_turn_d, 1'b1)
  logic       dimc_write_grant, normal_write_request;
  logic       dimc_write_turn_q, dimc_write_turn_d;
  `FF(dimc_write_turn_q, dimc_write_turn_d, 1'b1)
  logic       dimc_write_valid;
  logic       dimc_wb_accept;
  logic       dimc_wb_done;
  logic       dimc_wb_can_enqueue;
  logic       dimc_capture_complete;
  logic       dimc_wb_write_through;
  logic       dimc_wb_write_through_accept;
  logic       dimc_compute_fire;
  logic       dimc_capture_valid;
  logic       dimc_handoff_tail;
  logic [4:0] dimc_row_limit;
  logic [4:0] dimc_capture_row;
  logic [4:0] dimc_capture_row_limit;
  logic [4:0] dimc_capture_row_offset;
  logic [4:0] dimc_result_row_offset;
  logic [4:0] dimc_result_index;
  spatz_id_t  dimc_capture_id;
  vrf_addr_t  dimc_capture_base_addr;
  logic [DimcWriteIdxWidth-1:0] dimc_capture_first_write_word;
  logic [DimcWriteIdxWidth-1:0] dimc_capture_last_write_word;
  vrf_addr_t  dimc_result_base_addr;
  logic [DimcWriteIdxWidth-1:0] dimc_first_write_word;
  logic [DimcWriteIdxWidth-1:0] dimc_last_write_word;
  logic [DimcWriteIdxWidth-1:0] dimc_write_word;
  logic [DimcWriteIdxWidth-1:0] dimc_write_last_word;
  spatz_id_t  dimc_write_id;
  vreg_t      dimc_kernel_vreg;
  logic [31:0] dimc_result_word;
  vrf_addr_t  dimc_feature_addr;
  vrf_addr_t  dimc_kernel_addr;
  vrf_addr_t  dimc_upper_addr;
  vrf_addr_t  dimc_write_addr;
  vrf_data_t  dimc_write_data;

  logic       dimc_readyn;
  logic       dimc_compe;
  logic       dimc_fcsn;
  logic [1:0] dimc_mode;
  logic [1:0] dimc_fa;
  dimc_data_t dimc_fd;
  logic [23:0] dimc_addin;
  logic       dimc_sout;
  logic [2:0] dimc_res_out;
  logic [23:0] dimc_psout;
  dimc_data_t dimc_q;
  dimc_data_t dimc_d;
  logic [6:0] dimc_ra;
  logic [6:0] dimc_wa;
  logic       dimc_rcsn;
  logic       dimc_rcsn0;
  logic       dimc_rcsn1;
  logic       dimc_rcsn2;
  logic       dimc_rcsn3;
  logic       dimc_wcsn;
  logic       dimc_wen;
  dimc_data_t dimc_mask;
  logic [7:0] dimc_mct;
  logic [7:0] dimc_active_mct;
  logic [8:0] dimc_tail_quads;
  logic [DimcValidBitsWidth-1:0] dimc_active_bits;
  logic [DimcValidBitsWidth-1:0] dimc_tail_bits;

  assign dimc_busy  = dimc_state_q != DimcIdle;
  // Latch the next queued instruction at the edge that finishes the current
  // last row. If capture/writeback is blocked, retain the ordinary idle path.
  assign dimc_turnover = dimc_state_q == DimcComputeIssue &&
                         dimc_row_q == DimcVqmmaccRows - 1 &&
                         dimc_request_released_q && !dimc_tail_pending_q &&
                         dimc_capture_complete && dimc_wb_can_enqueue;
  assign dimc_inflight = dimc_busy || dimc_tail_pending_q ||
                         dimc_tail_done_pending_q || dimc_wb_pending_q;
  assign dimc_start = dimc_req_valid && !is_fpu_busy &&
                      reduction_state_q == Reduction_NormalExecution &&
                      (!dimc_busy || dimc_turnover) && !dimc_tail_done_pending_q;

  // Alternate ownership when both queues need the shared read ports. In
  // particular, a DIMC operand waiting on an older IPU result must not starve
  // the IPU reads that produce that result. A stalled grant also yields.
  assign dimc_read_request = dimc_state_q inside {DimcLoadFeature, DimcLoadKernel};
  assign normal_read_request = spatz_req_valid &&
      (operand_state_q != READ_OPERANDS || reduction_state_q == Reduction_Read_V0_t ||
       (vl_q < spatz_req.vl && (spatz_req.use_vs1 || spatz_req.use_vs2 || spatz_req.vd_is_src)));
  assign dimc_read_grant = dimc_read_request && (!normal_read_request || dimc_read_turn_q);
  assign dimc_read_turn_d = dimc_read_request && normal_read_request ?
                            !dimc_read_turn_q : 1'b1;
  assign dimc_capture_valid = !dimc_readyn;
  assign dimc_capture_row   = dimc_tail_pending_q ? dimc_tail_capture_count_q :
                                                    dimc_capture_count_q;
  assign dimc_capture_complete = dimc_capture_valid &&
                                 (dimc_state_q == DimcComputeIssue || dimc_tail_pending_q) &&
                                 (dimc_capture_row == dimc_capture_row_limit - 1'b1);
  assign dimc_wb_write_through = dimc_capture_complete && !dimc_wb_pending_q;
  assign dimc_write_valid   = dimc_wb_pending_q || dimc_wb_write_through;
  assign dimc_write_id      = dimc_wb_write_through ? dimc_capture_id : dimc_wb_id_q;
  assign dimc_write_word    = dimc_wb_write_through ? dimc_capture_first_write_word :
                                                        dimc_wb_word_q;
  assign dimc_write_last_word = dimc_wb_write_through ? dimc_capture_last_write_word :
                                                          dimc_wb_last_word_q;
  assign dimc_write_addr    = dimc_wb_write_through ?
                              vrf_addr_t'(int'(dimc_capture_base_addr) +
                                          int'(dimc_capture_first_write_word)) :
                              vrf_addr_t'(int'(dimc_wb_base_addr_q) + int'(dimc_wb_word_q));
  assign dimc_write_data    = dimc_wb_write_through ?
                              (dimc_tail_pending_q ?
                               dimc_tail_result_d[dimc_capture_first_write_word] :
                               dimc_result_d[dimc_capture_first_write_word]) :
                              dimc_wb_result_q[dimc_wb_word_q];
  // A later DIMC destination can depend on an older IPU write. Yield even
  // when the selected write is blocked by the scoreboard, to avoid deadlock.
  assign normal_write_request = &(result_valid | ~pending_results) && !result_tag.reduction;
  assign dimc_write_grant = dimc_write_valid && (!normal_write_request || dimc_write_turn_q);
  assign dimc_write_turn_d = dimc_write_valid && normal_write_request ?
                             !dimc_write_turn_q : 1'b1;
  assign dimc_wb_accept     = dimc_write_grant && vrf_wvalid_i;
  assign dimc_wb_write_through_accept = dimc_wb_write_through && dimc_wb_accept;
  assign dimc_wb_done       = dimc_wb_accept && (dimc_write_word == dimc_write_last_word);
  assign dimc_wb_can_enqueue = !dimc_wb_pending_q || dimc_wb_done;
  assign dimc_rsp_done      = dimc_wb_done;

  always_comb begin : dimc_mct_proc
    dimc_active_bits = DimcValidBitsWidth'(dimc_active_vl_q) << dimc_active_cfg_q.ci[1:0];
    dimc_tail_bits   = '0;
    dimc_tail_quads  = '0;
    dimc_active_mct  = '0;

    if (dimc_active_bits < DimcValidBitsWidth'(DimcRowWidth)) begin
      dimc_tail_bits  = DimcValidBitsWidth'(DimcRowWidth) - dimc_active_bits;
      dimc_tail_quads = 9'(dimc_tail_bits[DimcValidBitsWidth-1:2]);
      dimc_active_mct = dimc_tail_quads[8] ? 8'hff : dimc_tail_quads[7:0];
    end
  end : dimc_mct_proc

  DIMC #(
    .SECTION_WIDTH(DimcSectionWidth),
    .NUM_SECTIONS (DimcSections)
  ) i_dimc (
    .RCK   (clk_i        ),
    .RESETn(rst_ni       ),
    .READYN(dimc_readyn  ),
    .COMPE (dimc_compe   ),
    .FCSN  (dimc_fcsn    ),
    .MODE  (dimc_mode    ),
    .FA    (dimc_fa      ),
    .FD    (dimc_fd      ),
    .ADDIN (dimc_addin   ),
    .SOUT  (dimc_sout    ),
    .RES_OUT(dimc_res_out),
    .PSOUT (dimc_psout   ),
    .Q     (dimc_q       ),
    .D     (dimc_d       ),
    .RA    (dimc_ra      ),
    .WA    (dimc_wa      ),
    .RCSN  (dimc_rcsn    ),
    .RCSN0 (dimc_rcsn0   ),
    .RCSN1 (dimc_rcsn1   ),
    .RCSN2 (dimc_rcsn2   ),
    .RCSN3 (dimc_rcsn3   ),
    .WCK   (clk_i        ),
    .WCSN  (dimc_wcsn    ),
    .WEN   (dimc_wen     ),
    .M     (dimc_mask    ),
    .MCT   (dimc_mct     )
  );

  // Scalar results (sent back to Snitch)
  elen_t scalar_result;

  // Is this the last request?
  logic last_request;

  // Reduction state
  typedef enum logic [3:0] {
    Reduction_NormalExecution,
    Reduction_Wait,
    Reduction_Read_V0_t,
    Reduction_Init,
    Reduction_Reduce,
    Reduction_IntraLane,
    Reduction_InterLane,
    Reduction_SIMD,
    Reduction_WriteBack
   } reduction_state_t;
   reduction_state_t reduction_state_d, reduction_state_q;
  `FF(reduction_state_q, reduction_state_d, Reduction_NormalExecution)

  // Reduction intralane
  vlen_t reduction_pointer_d, reduction_pointer_q;
  logic [idx_width(ELEN*N_FU)-1 : 0] shift_amnt_d, shift_amnt_q;
  vrf_data_t result_buf_d, result_buf_q;

  // Reduction mask index
  vlen_t v0_word_idx;
  int unsigned v0_shift;

  `FF(result_buf_valid_q, result_buf_valid_d, 1'b0)
  `FF(reduction_pointer_q, reduction_pointer_d, '0)
  `FF(shift_amnt_q, shift_amnt_d, ELEN)
  `FF(result_buf_q, result_buf_d, '0)
  `FF(result_buf_tag_q, result_buf_tag_d, '0)

  // Is the reduction done?
  logic reduction_done;

  // Are we producing the upper or lower part of the results of a narrowing instruction?
  logic narrowing_upper_d, narrowing_upper_q;
  `FF(narrowing_upper_q, narrowing_upper_d, 1'b0)

  // Are we reading the upper or lower part of the operands of a widening instruction?
  logic widening_upper_d, widening_upper_q;
  `FF(widening_upper_q, widening_upper_d, 1'b0)

  // Are any results valid?
  logic [N_FU*ELEN-1:0]  result;
  logic [N_FU*ELENB-1:0] result_valid;
  logic [N_FU*ELENB-1:0]  saturated;
  logic                  result_ready;

  // it represents the VRF word index. Multiplication by 8 to account for LMUL
  logic [$clog2(NrWordsPerVector*8):0] word_idx_d, word_idx_q;
  logic [$clog2(N_FU)-1:0] divsqrt_slot_q, divsqrt_slot_d;
  logic last_divsqrt;

  assign last_divsqrt = divsqrt_shared_active ? ((vl_q + (divsqrt_slot_q+1)*nr_elem_word_divsqrt) >= spatz_req.vl) : 1'b0;

  `FF(word_idx_q, word_idx_d, '0)

  // All the results of the current VRF word in the FUs are ready
  logic fu_word_complete;
  assign fu_word_complete = &(result_valid | ~pending_results);

  always_comb begin: control_proc
    // Maintain state
    vl_d              = vl_q;
    busy_d            = busy_q;
    running_d         = running_q;
    state_d           = state_q;
    narrowing_upper_d = narrowing_upper_q;
    widening_upper_d  = widening_upper_q;

    // We are not stalling
    stall = 1'b0;

    // This is not the last request
    last_request = 1'b0;

    // We are handling an instruction
    spatz_req_ready = 1'b0;

    // Do not ack anything
    vfu_rsp_valid_o = 1'b0;
    vfu_rsp_o       = '0;

    // Change number of remaining elements
    if (word_issued) begin
      vl_d              = vl_q + nr_elem_word;
      // Update narrowing information
      narrowing_upper_d = narrowing_upper_q ^ spatz_req.op_arith.is_narrowing;
      widening_upper_d  = widening_upper_q ^ (spatz_req.op_arith.widen_vs1 || spatz_req.op_arith.widen_vs2); // toggle the signal if requires widening
    end

    // Current state of the VFU
    if (spatz_req_valid)
      unique case (state_q)
        VFU_RunningIPU: begin
          // Only go to the FPU state once the IPUs are no longer busy
          if (is_fpu_insn) begin
            if (is_ipu_busy || dimc_inflight || dimc_start)
              stall = 1'b1;
            else begin
              state_d = VFU_RunningFPU;
              stall   = 1'b1;
            end
          end
        end
        VFU_RunningFPU: begin
          // Only go back to the IPU state once the FPUs are no longer busy
          if (!is_fpu_insn)
            if (is_fpu_busy)
              stall = 1'b1;
            else begin
              state_d = VFU_RunningIPU;
              stall   = 1'b1;
            end
        end
        default:;
      endcase

    // Only ordinary IPU arithmetic overlaps DIMC. Reductions and FPU work
    // retain their existing exclusive execution protocol.
    if (dimc_read_grant ||
        ((is_fpu_insn || spatz_req.op_arith.is_reduction) && (dimc_inflight || dimc_start)))
      stall = 1'b1;

    // Finished the execution!
    if (spatz_req_valid && ((vl_d >= spatz_req.vl && !spatz_req.op_arith.is_reduction) || reduction_done || last_divsqrt)) begin
      if(divsqrt_shared_active) begin
          last_request            = 1'b1;
        if (result_tag.last && result_ready && fu_word_complete) begin
          spatz_req_ready         = spatz_req_valid;
          busy_d                  = 1'b0;
          vl_d                    = '0;
          running_d[spatz_req.id] = 1'b0;
          widening_upper_d        = 1'b0;
          narrowing_upper_d       = 1'b0;
        end
      end else begin
        spatz_req_ready         = spatz_req_valid;
        busy_d                  = 1'b0;
        vl_d                    = '0;
        last_request            = 1'b1;
        running_d[spatz_req.id] = 1'b0;
        widening_upper_d        = 1'b0;
        narrowing_upper_d       = 1'b0;
      end
    end
    // Do we have a new instruction?
    else if (spatz_req_valid && !running_d[spatz_req.id]) begin
      // Start at vstart
      vl_d                    = vstart;
      busy_d                  = 1'b1;
      running_d[spatz_req.id] = 1'b1;

      // Change number of remaining elements
      if (word_issued)
        vl_d = vl_q + nr_elem_word;
    end

    // An instruction finished execution
    if (dimc_rsp_done) begin
      vfu_rsp_o.id      = dimc_write_id;
      vfu_rsp_o.rd      = '0;
      vfu_rsp_o.wb      = 1'b0;
      vfu_rsp_o.result  = '0;
      vfu_rsp_valid_o   = 1'b1;
    end else if ((result_tag.last && result_ready && fu_word_complete && (reduction_state_q inside {Reduction_NormalExecution, Reduction_Wait} || ! result_tag.reduction)) || reduction_done) begin
      vfu_rsp_o.id      = result_tag.id;
      vfu_rsp_o.rd      = result_tag.vd_addr[GPRWidth-1:0];
      vfu_rsp_o.wb      = result_tag.wb;
      vfu_rsp_o.result  = result_tag.wb ? scalar_result : '0;
      vfu_rsp_valid_o   = 1'b1;
    end
  end: control_proc

  //////////////
  // Operands //
  //////////////

  operation_e fpu_op;
  fp_format_e fpu_src_fmt, fpu_dst_fmt;
  int_format_e fpu_int_fmt;
  logic fpu_op_mode;
  logic fpu_vectorial_op;

  logic [N_FPU-1:0] fpu_busy_d, fpu_busy_q;
  `FF(fpu_busy_q, fpu_busy_d, '0)

  status_t [N_FPU-1:0] fpu_status_d, fpu_status_q;
  `FF(fpu_status_q, fpu_status_d, '0)

  always_comb begin: gen_decoder
      fpu_op           = fpnew_pkg::FMADD;
      fpu_op_mode      = 1'b0;
      fpu_vectorial_op = 1'b0;
      is_fpu_busy      = |fpu_busy_q || |fpu_stage_valid || fpu_gather_busy;
      fpu_src_fmt      = fpnew_pkg::FP32;
      fpu_dst_fmt      = fpnew_pkg::FP32;
      fpu_int_fmt      = fpnew_pkg::INT32;

      fpu_status_o = '0;
      for (int fpu = 0; fpu < N_FPU; fpu++)
        fpu_status_o |= fpu_status_q[fpu];

      if (FPU) begin
        unique case (spatz_req.vtype.vsew)
          EW_64: begin
            if (RVD) begin
              fpu_src_fmt = fpnew_pkg::FP64;
              fpu_dst_fmt = fpnew_pkg::FP64;
              // A scalar fcvt.w.d/wu.d (and fcvt.d.w/d.wu) is decoded as EW_64 to
              // select FP64, but its integer operand is a 32-bit GPR, so INT64
              // saturates an out-of-range or NaN source to INT64_MAX and returns the
              // wrong low 32 bits. Vector EW_64 fcvt keeps INT64.
              fpu_int_fmt = spatz_req.op_arith.is_scalar ? fpnew_pkg::INT32 : fpnew_pkg::INT64;
            end
          end
          EW_32: begin
            fpu_src_fmt      = spatz_req.op_arith.is_narrowing || spatz_req.op_arith.widen_vs1 || spatz_req.op_arith.widen_vs2 ? fpnew_pkg::FP64 : fpnew_pkg::FP32;
            fpu_dst_fmt      = spatz_req.op_arith.widen_vs1 || spatz_req.op_arith.widen_vs2 || spatz_req.op == VSDOTP ? fpnew_pkg::FP64          : fpnew_pkg::FP32;
            fpu_int_fmt      = spatz_req.op_arith.is_narrowing && spatz_req.op inside {VI2F, VU2F} ? fpnew_pkg::INT64                            : fpnew_pkg::INT32;
            fpu_vectorial_op = FLEN > 32;
          end
          EW_16: begin
            fpu_src_fmt      = spatz_req.op_arith.is_narrowing || spatz_req.op_arith.widen_vs1 || spatz_req.op_arith.widen_vs2 ? fpnew_pkg::FP32 : (spatz_req.fm.src ? fpnew_pkg::FP16ALT : fpnew_pkg::FP16);
            fpu_dst_fmt      = spatz_req.op_arith.widen_vs1 || spatz_req.op_arith.widen_vs2 || spatz_req.op == VSDOTP          ? fpnew_pkg::FP32 : (spatz_req.fm.dst ? fpnew_pkg::FP16ALT : fpnew_pkg::FP16);
            fpu_int_fmt      = spatz_req.op_arith.is_narrowing && spatz_req.op inside {VI2F, VU2F}                             ? fpnew_pkg::INT32 : fpnew_pkg::INT16;
            fpu_vectorial_op = 1'b1;
          end
          EW_8: begin
            fpu_src_fmt      = spatz_req.op_arith.is_narrowing || spatz_req.op_arith.widen_vs1 || spatz_req.op_arith.widen_vs2 ? (spatz_req.fm.src ? fpnew_pkg::FP16ALT : fpnew_pkg::FP16) : (spatz_req.fm.src ? fpnew_pkg::FP8ALT : fpnew_pkg::FP8);
            fpu_dst_fmt      = spatz_req.op_arith.widen_vs1 || spatz_req.op_arith.widen_vs2 || spatz_req.op == VSDOTP          ? (spatz_req.fm.dst ? fpnew_pkg::FP16ALT : fpnew_pkg::FP16) : (spatz_req.fm.dst ? fpnew_pkg::FP8ALT : fpnew_pkg::FP8);
            fpu_int_fmt      = spatz_req.op_arith.is_narrowing && spatz_req.op inside {VI2F, VU2F}                             ? fpnew_pkg::INT16 : fpnew_pkg::INT8;
            fpu_vectorial_op = 1'b1;
          end
          default:;
        endcase

        unique case (spatz_req.op)
          VFADD: fpu_op = fpnew_pkg::ADD;
          VFSUB: begin
            fpu_op      = fpnew_pkg::ADD;
            fpu_op_mode = 1'b1;
          end
          VFMUL  : fpu_op = fpnew_pkg::MUL;
          VFMADD : fpu_op = fpnew_pkg::FMADD;
          VFMSUB : begin
            fpu_op      = fpnew_pkg::FMADD;
            fpu_op_mode = 1'b1;
          end
          VFNMSUB: fpu_op = fpnew_pkg::FNMSUB;
          VFNMADD: begin
            fpu_op      = fpnew_pkg::FNMSUB;
            fpu_op_mode = 1'b1;
          end

          VFMINMAX: begin
            fpu_op = fpnew_pkg::MINMAX;
            fpu_dst_fmt = fpu_src_fmt;
          end


          VFSGNJ : begin
            fpu_op = fpnew_pkg::SGNJ;
            fpu_dst_fmt = fpu_src_fmt;
          end
          VFCLASS: begin
            fpu_op = fpnew_pkg::CLASSIFY;
            fpu_dst_fmt = fpu_src_fmt;
          end
          VFCMP  : begin
            fpu_op = fpnew_pkg::CMP;
            fpu_dst_fmt = fpu_src_fmt;
            if (spatz_req.rm == fpnew_pkg::RUP)
                // Boolean result inverted
                fpu_op_mode = 1'b1;
          end

          VF2F: fpu_op = fpnew_pkg::F2F;
          VF2I: fpu_op = fpnew_pkg::F2I;
          VF2U: begin
            fpu_op      = fpnew_pkg::F2I;
            fpu_op_mode = 1'b1;
          end
          VI2F: fpu_op = fpnew_pkg::I2F;
          VU2F: begin
            fpu_op      = fpnew_pkg::I2F;
            fpu_op_mode = 1'b1;
          end

          VSDOTP: fpu_op = fpnew_pkg::SDOTP;
          VFDIV:  fpu_op = fpnew_pkg::DIV;
          VFSQRT: fpu_op = fpnew_pkg::SQRT;

          default:;
        endcase
      end
    end: gen_decoder

  // Reduction registers
  vrf_data_t [$clog2(N_FU)-1:0] reduction_q, reduction_d;
  vrf_data_t reduction_vector_data, reduction_scalar_data;
  `FF(reduction_q, reduction_d, '0)
  elen_t reduction_neutral_value;

  // IPU results
  logic [N_FU*ELEN-1:0]  ipu_result;
  logic [N_FU*ELENB-1:0] ipu_result_valid;
  logic [N_FU*ELENB-1:0] ipu_saturated;
  logic [N_FU*ELENB-1:0] ipu_in_ready;

  // FPU results
  logic [N_FU*ELEN-1:0]  fpu_result;
  logic [N_FU*ELENB-1:0] fpu_result_valid;
  logic [N_FU*ELENB-1:0] fpu_in_ready;

  // Valid bytes in current VRF word
  logic [$clog2(MAXVL*8+1)-1:0] remaining_bytes;
  logic [$clog2(VRFWordBWidth+1)-1:0] valid_bytes_rd, valid_bytes_wr;
  logic [VRFWordWidth-1:0] tail_mask;
  vew_e src_vsew;
  logic [$clog2(MAXVL*8+1)-1:0] rem_rd, rem_wr;

  always_comb begin: proc_tail_mask
    src_vsew = vew_e'(spatz_req.vtype.vsew + spatz_req.op_arith.is_narrowing);

    // Read side: source-width bytes
    rem_rd = (vl_q < spatz_req.vl) ? ((spatz_req.vl - vl_q) << src_vsew) : '0;
    valid_bytes_rd = (rem_rd >= VRFWordBWidth) ? ($bits(valid_bytes_rd))'(VRFWordBWidth) : rem_rd[$bits(valid_bytes_rd)-1:0];

    // Write side: dest-width bytes
    rem_wr = (vl_q < spatz_req.vl) ? ((spatz_req.vl - vl_q) << spatz_req.vtype.vsew) : '0;
    valid_bytes_wr = (rem_wr >= VRFWordBWidth) ? ($bits(valid_bytes_wr))'(VRFWordBWidth) : rem_wr[$bits(valid_bytes_wr)-1:0];

    for (int b = 0; b < VRFWordBWidth; b++)
      tail_mask[b*8 +: 8] = (b < valid_bytes_rd) ? 8'hFF : 8'h00;
  end: proc_tail_mask

  // apply mask to read data
  vrf_data_t [2:0] vrf_rdata_masked;
  for (genvar i = 0; i < 3; i++) begin: gen_data_mask
    assign vrf_rdata_masked[i] = vrf_rdata_i[i] & tail_mask;
  end: gen_data_mask

  // Operands and result signals
  logic [N_FU*ELEN-1:0]  operand1, operand2, operand3;
  logic [N_FU*ELEN-1:0]  operand_v0_t_lo,operand_v0_t_lo_q;
  logic [N_FU*ELEN-1:0]  operand_v0_t_hi,operand_v0_t_hi_q;
  logic [N_FU*ELENB-1:0] in_ready;

  logic reduction_v0_t_is_ready;
  assign reduction_v0_t_is_ready = (reduction_state_q == Reduction_Read_V0_t) && vrf_rvalid_i[0] && vrf_rvalid_i[1];
  logic reduction_v0_t_read_done;
  `FFLARNC(reduction_v0_t_read_done,1'b1,reduction_v0_t_is_ready,vfu_rsp_valid_o,1'b0,clk_i,rst_ni);

  // Back up v0.t for reduction instructions
  logic [N_FU*ELEN-1:0]  reduction_operand_v0_t_lo,reduction_operand_v0_t_lo_q;
  logic [N_FU*ELEN-1:0]  reduction_operand_v0_t_hi,reduction_operand_v0_t_hi_q;
  `FFL(reduction_operand_v0_t_lo_q, reduction_operand_v0_t_lo, reduction_v0_t_is_ready, '0)
  `FFL(reduction_operand_v0_t_hi_q, reduction_operand_v0_t_hi, reduction_v0_t_is_ready, '0)
  logic [VLEN-1:0] reduction_operand_v0_t_q;
  assign reduction_operand_v0_t_q = {reduction_operand_v0_t_hi_q, reduction_operand_v0_t_lo_q};

  // FF to back up the mask destination register for comparison instructions
  logic [N_FU*ELEN-1:0] cmp_mask_dst_lo, cmp_mask_dst_lo_q;
  logic [N_FU*ELEN-1:0] cmp_mask_dst_hi, cmp_mask_dst_hi_q;

    // The signal to choose comparison instructions
  logic is_cmp_req;
  assign is_cmp_req = (spatz_req.op == VFCMP) || spatz_req.op inside {VMSEQ, VMSNE, VMSLT, VMSLTU, VMSLE, VMSLEU, VMSGT, VMSGTU};

  // FSM to manage operands between normal calculation and v0.t fetching
  logic v0_t_is_ready;
  assign v0_t_is_ready   = !dimc_read_grant && (operand_state_q == READ_V0_t) && vrf_rvalid_i[0] && vrf_rvalid_i[1];
  logic vd_t_is_ready;
  assign vd_t_is_ready   = !dimc_read_grant && (operand_state_q == READ_VD_t) && vrf_rvalid_i[0] && vrf_rvalid_i[1];
  logic v0_t_read_done;
  logic v0_t_read_done_d;
  always_comb begin
    v0_t_read_done_d = v0_t_read_done;
    if (last_request) begin
      v0_t_read_done_d = 1'b0;
    end else begin
      if (v0_t_is_ready) begin
        v0_t_read_done_d = 1'b1;
      end
    end
  end
  `FF(v0_t_read_done,v0_t_read_done_d,1'b0,clk_i,rst_ni);

  logic switch_to_read_v0t;
  assign switch_to_read_v0t = (operand_state_q == READ_OPERANDS) && spatz_req_valid
                          && !spatz_req.op_arith.is_scalar && !spatz_req.op_arith.vm
                          && !v0_t_read_done && !spatz_req.op_arith.is_reduction;

  always_comb begin: operand_selection
    operand_state_d = operand_state_q;
      unique case(operand_state_q)
        READ_V0_t:
          if(v0_t_is_ready) operand_state_d = (is_cmp_req && spatz_req.vtype.vma == 0) ? READ_VD_t : READ_OPERANDS;
          else operand_state_d = operand_state_q;
        READ_VD_t:
          operand_state_d = vd_t_is_ready ? READ_OPERANDS : READ_VD_t;
        READ_OPERANDS:
          operand_state_d = switch_to_read_v0t ? READ_V0_t : READ_OPERANDS;
        default: operand_state_d = operand_state_q;
      endcase
  end:operand_selection

  vlen_t vl_q_plus_nr_elem_word;
  assign vl_q_plus_nr_elem_word = vl_q + nr_elem_word;

  always_comb begin: operand_proc
    reduction_operand_v0_t_lo = '0;
    reduction_operand_v0_t_hi = '0;
    operand_v0_t_lo = '0;
    operand_v0_t_hi = '0;
    cmp_mask_dst_lo = '0;
    cmp_mask_dst_hi = '0;
    operand1 = '0;
    operand2 = '0;
    case (operand_state_q)
      READ_OPERANDS: begin
          if(reduction_state_q == Reduction_Read_V0_t) begin
            reduction_operand_v0_t_lo = vrf_rdata_i[0];
            reduction_operand_v0_t_hi = vrf_rdata_i[1];
          end
          else begin
            if (spatz_req.op_arith.is_scalar)
              operand1 = {1*N_FU{spatz_req.rs1}};
            else if (spatz_req.use_vs1)
              operand1 = spatz_req.op_arith.is_reduction ? $unsigned(reduction_q[1]) : vrf_rdata_masked[1];
            else begin
              // Replicate scalar operands
              unique case (spatz_req.op == VSDOTP ? vew_e'(spatz_req.vtype.vsew + 1) : spatz_req.vtype.vsew)
                EW_8 : operand1   = MAXEW == EW_32 ? {4*N_FU{spatz_req.rs1[7:0]}}  : {8*N_FU{spatz_req.rs1[7:0]}};
                EW_16: operand1   = MAXEW == EW_32 ? {2*N_FU{spatz_req.rs1[15:0]}} : {4*N_FU{spatz_req.rs1[15:0]}};
                EW_32: operand1   = MAXEW == EW_32 ? {1*N_FU{spatz_req.rs1[31:0]}} : {2*N_FU{spatz_req.rs1[31:0]}};
                default: operand1 = {1*N_FU{spatz_req.rs1}};
              endcase
            end

            //VFMV_F_S
            if (spatz_req.use_rd && spatz_req.use_vs2 && spatz_req.op_arith.is_scalar)
              operand2 = vrf_rdata_i[0];
            else if ((!spatz_req.op_arith.is_scalar || spatz_req.op == VADD) && spatz_req.use_vs2)
              operand2 = spatz_req.op_arith.is_reduction ? $unsigned(reduction_q[0]) : vrf_rdata_masked[0];
            else
              // Replicate scalar operands
              unique case (spatz_req.op == VSDOTP ? vew_e'(spatz_req.vtype.vsew + 1) : spatz_req.vtype.vsew)
                EW_8 : operand2   = MAXEW == EW_32 ? {4*N_FU{spatz_req.rs2[7:0]}}  : {8*N_FU{spatz_req.rs2[7:0]}};
                EW_16: operand2   = MAXEW == EW_32 ? {2*N_FU{spatz_req.rs2[15:0]}} : {4*N_FU{spatz_req.rs2[15:0]}};
                EW_32: operand2   = MAXEW == EW_32 ? {1*N_FU{spatz_req.rs2[31:0]}} : {2*N_FU{spatz_req.rs2[31:0]}};
                default: operand2 = {1*N_FU{spatz_req.rs2}};
              endcase
          end
      end
      READ_V0_t: begin
        operand_v0_t_lo = vrf_rdata_i[0];
        operand_v0_t_hi = vrf_rdata_i[1];
      end
      READ_VD_t: begin
        cmp_mask_dst_lo = vrf_rdata_i[0];
        cmp_mask_dst_hi = vrf_rdata_i[1];
      end
      default:;
    endcase
    operand3 = spatz_req.op_arith.is_scalar ? {1*N_FU{spatz_req.rsd}} : vrf_rdata_masked[2]; // VFU_VD_RD // operand3 is used in MAC computation, like VMADD
  end: operand_proc

  logic [N_FU*ELEN-1:0]  fpu_result_temp;
  logic [N_FU*ELENB-1:0] fpu_result_valid_temp;

  assign in_ready     = state_q == VFU_RunningIPU ? ipu_in_ready     : fpu_in_ready;
  assign result       = state_q == VFU_RunningIPU ? ipu_result       : fpu_result_temp;
  assign result_valid = state_q == VFU_RunningIPU ? ipu_result_valid : fpu_result_valid_temp;
  assign saturated    = (state_q == VFU_RunningIPU) ? ipu_saturated : '0;

  ///////////////////////
  //      DIV/SQRT     //
  ///////////////////////

  // Accumulate the slots already completed for the current VRF word
  logic [N_FU*ELEN-1:0]  divsqrt_acc_d, divsqrt_acc_q;
  logic [N_FU*ELENB-1:0] divsqrt_acc_valid_d, divsqrt_acc_valid_q;
  `FF(divsqrt_acc_q, divsqrt_acc_d, '0)
  `FF(divsqrt_acc_valid_q, divsqrt_acc_valid_d, '0)

  logic divsqrt_shared_ready;
  logic divsqrt_closing_slot;

  // Handshake on FPU0 output
  logic divsqrt_pop;

  logic divsqrt_done_q, divsqrt_done_d;
  `FF(divsqrt_done_q, divsqrt_done_d, 1'b0)

  // Set once the last element of the instruction has been committed to the VRF
  always_comb begin : proc_divsqrt_done
    divsqrt_done_d = divsqrt_done_q;
    if (divsqrt_pop && result_tag.last)
      divsqrt_done_d = 1'b1;
    else if (fpu_load_ready[0])
      divsqrt_done_d = 1'b0;
  end : proc_divsqrt_done

  // This result completes the VRF word: last slot or last element of the instruction
  assign divsqrt_closing_slot = (divsqrt_slot_q == N_FU - 1) || result_tag.last;

  // Intermediate slots are always absorbed by the accumulator --> the closing slot is handed over only when the VRF commits the word
  assign divsqrt_shared_ready = !divsqrt_done_q && (!divsqrt_closing_slot || (!dimc_write_grant && vrf_wvalid_i));

  assign divsqrt_pop = divsqrt_shared_active && (fpu_result_valid[ELENB-1:0] == '1) && divsqrt_shared_ready;

  always_comb begin : proc_divsqrt_acc
    divsqrt_acc_d       = divsqrt_acc_q;
    divsqrt_acc_valid_d = divsqrt_acc_valid_q;

    // Capture the slot handed over by FPU0
    if (divsqrt_pop) begin
      divsqrt_acc_d[divsqrt_slot_q*ELEN +: ELEN] = fpu_result[ELEN-1:0];
      divsqrt_acc_valid_d[divsqrt_slot_q*ELENB +: ELENB] = '1;
    end

    // Word committed to the VRF, or not a shared divsqrt: restart clean
    if (result_ready || !divsqrt_shared_active) begin
      divsqrt_acc_d       = '0;
      divsqrt_acc_valid_d = '0;
    end
  end : proc_divsqrt_acc

  always_comb begin : proc_fpu_result_mux
    fpu_result_temp       = '0;
    fpu_result_valid_temp = '0;

    if (divsqrt_shared_active) begin
      fpu_result_temp       = divsqrt_acc_q;
      fpu_result_valid_temp = divsqrt_acc_valid_q;

      if (fpu_result_valid[ELENB-1:0] == '1) begin
        fpu_result_temp[divsqrt_slot_q*ELEN +: ELEN ] = fpu_result[ELEN-1:0];
        fpu_result_valid_temp[divsqrt_slot_q*ELENB +: ELENB] = '1;

        if (result_tag.last)
          fpu_result_valid_temp = '1;
      end
    end else begin
      fpu_result_temp       = fpu_result;
      fpu_result_valid_temp = fpu_result_valid;
    end
  end : proc_fpu_result_mux

  assign scalar_result = (spatz_req.op_arith.is_scalar || result_tag.last) ? result[ELEN-1:0] : '0;

  `FF(divsqrt_slot_q, divsqrt_slot_d, '0)

  always_comb begin : proc_divsqrt_slot
    divsqrt_slot_d = divsqrt_slot_q;
    if (spatz_req_valid && !is_divsqrt_insn)
      divsqrt_slot_d = '0;
    else if (divsqrt_pop)
      divsqrt_slot_d = (divsqrt_slot_q == N_FU - 1 || result_tag.last) ? '0 : divsqrt_slot_q + 1;
  end : proc_divsqrt_slot

  logic divsqrt_inflight_q, divsqrt_inflight_d;
  `FF(divsqrt_inflight_q, divsqrt_inflight_d, 1'b0)

  always_comb begin : proc_divsqrt_inflight
    divsqrt_inflight_d = divsqrt_inflight_q;
    if (divsqrt_shared_active && fpu_load_ready[0])
      divsqrt_inflight_d = 1'b1;
    else if (divsqrt_pop)
      divsqrt_inflight_d = 1'b0;
  end : proc_divsqrt_inflight


  `FFL(operand_v0_t_lo_q, operand_v0_t_lo, v0_t_is_ready, '0)
  `FFL(operand_v0_t_hi_q, operand_v0_t_hi, v0_t_is_ready, '0)
  `FFL(cmp_mask_dst_lo_q, cmp_mask_dst_lo, vd_t_is_ready, '0)
  `FFL(cmp_mask_dst_hi_q, cmp_mask_dst_hi, vd_t_is_ready, '0)

  logic [VLEN-1:0] operand_v0_t_q;
  assign operand_v0_t_q = {operand_v0_t_hi_q, operand_v0_t_lo_q};
  logic [VLEN-1:0] cmp_mask_dst_q;
  assign cmp_mask_dst_q = {cmp_mask_dst_hi_q, cmp_mask_dst_lo_q};

  // Inactive elements are set to 1 under ma and left undisturbed under mu
  logic [VLEN-1:0] cmp_dst_inactive;
  assign cmp_dst_inactive = result_tag.vma ? '1 : cmp_mask_dst_q;

  always_comb begin : dimc_proc
	    dimc_state_d      = dimc_state_q;
    dimc_request_released_d = dimc_request_released_q;
	    dimc_section_d    = dimc_section_q;
	    dimc_row_d        = dimc_row_q;
	    dimc_init_count_d = dimc_init_count_q;
	    dimc_capture_count_d = dimc_capture_count_q;
	    dimc_result_d     = dimc_result_q;
	    dimc_tail_capture_count_d = dimc_tail_capture_count_q;
	    dimc_tail_result_d = dimc_tail_result_q;
	    dimc_wb_pending_d = dimc_wb_pending_q;
	    dimc_wb_id_d      = dimc_wb_id_q;
	    dimc_wb_base_addr_d = dimc_wb_base_addr_q;
	    dimc_wb_word_d    = dimc_wb_word_q;
	    dimc_wb_last_word_d = dimc_wb_last_word_q;
	    dimc_wb_result_d  = dimc_wb_result_q;
	    dimc_tail_pending_d = dimc_tail_pending_q;
	    dimc_tail_done_pending_d = dimc_tail_done_pending_q;
	    dimc_tail_id_d = dimc_tail_id_q;
	    dimc_tail_base_addr_d = dimc_tail_base_addr_q;
	    dimc_tail_first_word_d = dimc_tail_first_word_q;
	    dimc_tail_last_word_d = dimc_tail_last_word_q;
	    dimc_tail_row_limit_d = dimc_tail_row_limit_q;
	    dimc_tail_row_offset_d = dimc_tail_row_offset_q;
    dimc_active_id_d = dimc_active_id_q;
    dimc_active_vs1_d = dimc_active_vs1_q;
    dimc_active_vs2_d = dimc_active_vs2_q;
    dimc_active_vd_d = dimc_active_vd_q;
    dimc_active_vl_d = dimc_active_vl_q;
    dimc_active_cfg_d = dimc_active_cfg_q;

    dimc_instr_done      = 1'b0;
    dimc_load_feature    = !dimc_active_cfg_q.feature_reuse;
    dimc_vrf_read_feature = 1'b0;
    dimc_vrf_read_kernel  = 1'b0;
    dimc_vrf_read_upper   = 1'b0;
    dimc_compute_fire     = 1'b0;
    dimc_handoff_tail     = 1'b0;

    if (dimc_wb_pending_q && dimc_wb_accept) begin
      if (dimc_wb_word_q == dimc_wb_last_word_q) begin
        dimc_wb_pending_d = 1'b0;
      end else begin
        dimc_wb_word_d = dimc_wb_word_q + 1'b1;
      end
    end

    if (dimc_tail_done_pending_q && dimc_wb_can_enqueue) begin
      dimc_wb_pending_d        = 1'b1;
      dimc_wb_id_d             = dimc_tail_id_q;
      dimc_wb_base_addr_d      = dimc_tail_base_addr_q;
      dimc_wb_word_d           = dimc_tail_first_word_q;
      dimc_wb_last_word_d      = dimc_tail_last_word_q;
      dimc_wb_result_d         = dimc_tail_result_q;
      dimc_tail_done_pending_d = 1'b0;
    end

    dimc_row_limit   = DimcVqmmaccRows;
    dimc_kernel_vreg = vreg_t'(dimc_active_vs2_q + dimc_row_q);
    dimc_result_word = {8'b0, dimc_psout};

    dimc_feature_addr = vrf_addr_t'((int'(dimc_active_vs1_q) * NrWordsPerVector) +
	                                    (int'(dimc_section_q) * DimcWordsPerSection));
    dimc_kernel_addr  = vrf_addr_t'((int'(dimc_kernel_vreg) * NrWordsPerVector) +
	                                    (int'(dimc_section_q) * DimcWordsPerSection));
    dimc_upper_addr   = '0;
    dimc_result_base_addr = vrf_addr_t'(int'(dimc_active_vd_q) * NrWordsPerVector);
    dimc_result_row_offset = dimc_active_cfg_q.ci[2] ? 5'd8 : 5'd0;
    dimc_first_write_word  = dimc_active_cfg_q.ci[2] ?
                             DimcWriteIdxWidth'(DimcVqmmaccWords) : '0;
    dimc_last_write_word   = dimc_first_write_word;
    dimc_capture_row_limit = dimc_tail_pending_q ? dimc_tail_row_limit_q : dimc_row_limit;
    dimc_capture_row_offset = dimc_tail_pending_q ? dimc_tail_row_offset_q : dimc_result_row_offset;
    dimc_capture_id = dimc_tail_pending_q ? dimc_tail_id_q : dimc_active_id_q;
    dimc_capture_base_addr = dimc_tail_pending_q ? dimc_tail_base_addr_q : dimc_result_base_addr;
    dimc_capture_first_write_word =
        dimc_tail_pending_q ? dimc_tail_first_word_q : dimc_first_write_word;
    dimc_capture_last_write_word =
        dimc_tail_pending_q ? dimc_tail_last_word_q : dimc_last_write_word;
    dimc_result_index = dimc_capture_row_offset + dimc_capture_row;

    dimc_compe = 1'b0;
    dimc_fcsn  = 1'b1;
    dimc_mode  = dimc_active_cfg_q.ci[1:0];
    dimc_fa    = 2'(dimc_section_q);
    dimc_fd    = '0;
    dimc_addin = '0;
    dimc_d     = '0;
    dimc_ra    = {dimc_kernel_vreg, 2'b00};
    dimc_wa    = {dimc_kernel_vreg, 2'(dimc_section_q)};
    dimc_rcsn  = 1'b1;
    dimc_rcsn0 = 1'b1;
    dimc_rcsn1 = 1'b1;
    dimc_rcsn2 = 1'b1;
    dimc_rcsn3 = 1'b1;
    dimc_wcsn  = 1'b1;
    dimc_wen   = 1'b1;
    dimc_mask  = '1;
    dimc_mct   = dimc_active_mct;

    unique case (dimc_state_q)
      DimcIdle: ; // Initial issue and final-row turnover share the launch below.

      DimcLoadFeature: begin
        dimc_fa = 2'(dimc_section_q);
        if (int'(dimc_section_q) < DimcSections) begin
          dimc_vrf_read_feature = dimc_read_grant;
          dimc_vrf_read_upper   = dimc_read_grant && DimcWordsPerSection > 1;
          dimc_upper_addr       = dimc_feature_addr + 1'b1;
          dimc_fd               = {vrf_rdata_i[2], vrf_rdata_i[1]};
          dimc_fcsn             = ~(dimc_read_grant && vrf_rvalid_i[1] &&
                                    (!dimc_vrf_read_upper || vrf_rvalid_i[2]));
          if (dimc_read_grant && vrf_rvalid_i[1] && (!dimc_vrf_read_upper || vrf_rvalid_i[2])) begin
            if (dimc_section_q == DimcSectionIdxWidth'(DimcSections - 1)) begin
              dimc_section_d = '0;
              dimc_state_d   = dimc_active_cfg_q.kernel_load ? DimcLoadKernel : DimcComputeInit;
            end else begin
              dimc_section_d = dimc_section_q + 1'b1;
            end
          end
        end else begin
          dimc_fd   = '0;
          dimc_fcsn = 1'b0;
          if (dimc_section_q == DimcSectionIdxWidth'(DimcSections - 1)) begin
            dimc_section_d = '0;
            dimc_state_d   = dimc_active_cfg_q.kernel_load ? DimcLoadKernel : DimcComputeInit;
          end else begin
            dimc_section_d = dimc_section_q + 1'b1;
          end
        end
      end

      DimcLoadKernel: begin
        dimc_wa = {dimc_kernel_vreg, 2'(dimc_section_q)};
        if (int'(dimc_section_q) < DimcSections) begin
          dimc_vrf_read_kernel = dimc_read_grant;
          dimc_vrf_read_upper  = dimc_read_grant && DimcWordsPerSection > 1;
          dimc_upper_addr      = dimc_kernel_addr + 1'b1;
          dimc_d               = {vrf_rdata_i[2], vrf_rdata_i[0]};
          dimc_wcsn            = ~(dimc_read_grant && vrf_rvalid_i[0] &&
                                   (!dimc_vrf_read_upper || vrf_rvalid_i[2]));
          dimc_wen             = ~(dimc_read_grant && vrf_rvalid_i[0] &&
                                   (!dimc_vrf_read_upper || vrf_rvalid_i[2]));
          if (dimc_read_grant && vrf_rvalid_i[0] && (!dimc_vrf_read_upper || vrf_rvalid_i[2])) begin
            if (dimc_section_q == DimcSectionIdxWidth'(DimcSections - 1)) begin
              dimc_section_d = '0;
              if (dimc_row_q == dimc_row_limit - 1'b1) begin
                dimc_row_d   = '0;
	                dimc_state_d = dimc_active_cfg_q.feature_reuse
	                                   ? DimcComputeIssue
	                                   : DimcComputeInit;
              end else begin
                dimc_row_d = dimc_row_q + 1'b1;
              end
            end else begin
              dimc_section_d = dimc_section_q + 1'b1;
            end
          end
        end else begin
          dimc_d    = '0;
          dimc_wcsn = 1'b0;
          dimc_wen  = 1'b0;
          if (dimc_section_q == DimcSectionIdxWidth'(DimcSections - 1)) begin
            dimc_section_d = '0;
            if (dimc_row_q == dimc_row_limit - 1'b1) begin
              dimc_row_d   = '0;
	              dimc_state_d = dimc_active_cfg_q.feature_reuse
	                                 ? DimcComputeIssue
	                                 : DimcComputeInit;
            end else begin
              dimc_row_d = dimc_row_q + 1'b1;
            end
          end else begin
            dimc_section_d = dimc_section_q + 1'b1;
          end
        end
      end

      DimcComputeInit: begin
        if (dimc_init_count_q == DimcInitCountWidth'(DimcInitCycles - 1)) begin
          dimc_init_count_d = '0;
          dimc_state_d      = DimcComputeIssue;
        end else begin
          dimc_init_count_d = dimc_init_count_q + 1'b1;
        end
      end

      DimcComputeIssue: begin
        if (dimc_row_q == '0) begin
          dimc_capture_count_d = '0;
          dimc_result_d        = '{default: '0};
        end
        dimc_compute_fire = 1'b1;
        dimc_compe = 1'b1;
        dimc_ra    = {dimc_kernel_vreg, 2'b00};
        dimc_rcsn  = 1'b0;
        dimc_rcsn0 = 1'b0;
        dimc_rcsn1 = 1'b0;
        dimc_rcsn2 = 1'b0;
        dimc_rcsn3 = 1'b0;
        if (dimc_row_q == dimc_row_limit - 2 && !dimc_request_released_q) begin
          dimc_instr_done = 1'b1;
          dimc_request_released_d = 1'b1;
        end
        if (dimc_row_q == dimc_row_limit - 1'b1) begin
          dimc_row_d                = '0;
          dimc_state_d              = DimcIdle;
          if (!dimc_capture_complete) begin
            dimc_tail_pending_d       = 1'b1;
            dimc_tail_id_d            = dimc_active_id_q;
            dimc_tail_base_addr_d     = dimc_result_base_addr;
            dimc_tail_first_word_d    = dimc_first_write_word;
            dimc_tail_last_word_d     = dimc_last_write_word;
            dimc_tail_row_limit_d     = dimc_row_limit;
            dimc_tail_row_offset_d    = dimc_result_row_offset;
            dimc_handoff_tail         = 1'b1;
            dimc_instr_done           = !dimc_request_released_q;
          end
        end else begin
          dimc_row_d = dimc_row_q + 1'b1;
        end
		      end

      DimcWriteResult: begin
	        if (dimc_wb_can_enqueue) begin
	          dimc_wb_pending_d   = 1'b1;
	          dimc_wb_id_d        = dimc_active_id_q;
	          dimc_wb_base_addr_d = dimc_result_base_addr;
	          dimc_wb_word_d      = dimc_first_write_word;
	          dimc_wb_last_word_d = dimc_last_write_word;
	          dimc_wb_result_d    = dimc_result_q;
	          dimc_state_d        = DimcIdle;
            dimc_instr_done     = !dimc_request_released_q;
	        end
	      end

      default: dimc_state_d = DimcIdle;
    endcase

    if (dimc_capture_valid && (dimc_state_q == DimcComputeIssue || dimc_tail_pending_q)) begin
      for (int unsigned word = 0; word < DimcResultWords; word++) begin
        for (int unsigned slot = 0; slot < DimcResultsPerWord; slot++) begin
          if (dimc_result_index == 5'(word * DimcResultsPerWord + slot)) begin
            if (dimc_tail_pending_q) begin
              dimc_tail_result_d[word][32*slot +: 32] = dimc_result_word;
            end else begin
              dimc_result_d[word][32*slot +: 32] = dimc_result_word;
            end
          end
        end
      end

      if (dimc_capture_row == dimc_capture_row_limit - 1'b1) begin
        if (dimc_tail_pending_q) begin
          dimc_tail_capture_count_d = '0;
        end else begin
          dimc_capture_count_d = '0;
        end
        if (dimc_wb_can_enqueue) begin
          dimc_wb_pending_d   = !dimc_wb_write_through_accept ||
                                (dimc_capture_first_write_word !=
                                 dimc_capture_last_write_word);
          dimc_wb_id_d        = dimc_capture_id;
          dimc_wb_base_addr_d = dimc_capture_base_addr;
          dimc_wb_word_d      = dimc_wb_write_through_accept &&
                                (dimc_capture_first_write_word !=
                                 dimc_capture_last_write_word)
                                  ? dimc_capture_first_write_word + 1'b1
                                  : dimc_capture_first_write_word;
          dimc_wb_last_word_d = dimc_capture_last_write_word;
          dimc_wb_result_d    = dimc_tail_pending_q ? dimc_tail_result_d : dimc_result_d;
          if (dimc_tail_pending_q) begin
            dimc_tail_pending_d = 1'b0;
          end else begin
            dimc_state_d        = DimcIdle;
            dimc_instr_done     = !dimc_request_released_q;
          end
        end else begin
          if (dimc_tail_pending_q) begin
            dimc_tail_pending_d      = 1'b0;
            dimc_tail_done_pending_d = 1'b1;
          end else begin
            dimc_state_d             = DimcWriteResult;
          end
        end
      end else begin
        if (dimc_tail_pending_q) begin
          dimc_tail_capture_count_d = dimc_tail_capture_count_q + 1'b1;
        end else begin
          dimc_capture_count_d = dimc_capture_count_q + 1'b1;
        end
      end
    end

    if (dimc_handoff_tail) begin
      dimc_tail_capture_count_d = dimc_capture_count_d;
      dimc_tail_result_d        = dimc_result_d;
    end

    if (dimc_start) begin
      dimc_section_d = '0;
      dimc_row_d = '0;
      dimc_init_count_d = '0;
      dimc_capture_count_d = '0;
      dimc_request_released_d = 1'b0;
      dimc_active_id_d  = dimc_req.id;
      dimc_active_vs1_d = dimc_req.vs1;
      dimc_active_vs2_d = dimc_req.vs2;
      dimc_active_vd_d  = dimc_req.vd;
      dimc_active_vl_d  = dimc_req.vl;
      dimc_active_cfg_d = dimc_req.op_cfg.dimc;
      // Do not clear dimc_result_d here: a turnover may be writing the old
      // instruction's final word through to VRF in this same cycle. Row zero
      // clears the accumulator when the next computation actually starts.
      if (!dimc_req.op_cfg.dimc.feature_reuse)
        dimc_state_d = DimcLoadFeature;
      else if (dimc_req.op_cfg.dimc.kernel_load)
        dimc_state_d = DimcLoadKernel;
      else
        dimc_state_d = DimcComputeIssue;
    end
  end : dimc_proc

  ///////////////////////
  //  Reduction logic  //
  ///////////////////////

  // Reduction pointer
  logic [$clog2(N_FU)-1:0] pnt;
  assign pnt = reduction_pointer_d[$clog2(N_FU)-1:0];

  // To identify the VLEN (Vector Length) /DLEN (Datapath Length)
  vlen_t fill_cnt;
  assign fill_cnt = ((spatz_req.vl - 1) >> (MAXEW - spatz_req.vtype.vsew)) >> (is_fpu_insn ? $clog2(N_FPU) : $clog2(N_IPU));

  // Are the reduction operands ready?
  `FF(reduction_operand_ready_q, reduction_operand_ready_d, 1'b0)

  // Do we need to request reduction operands?
  logic [1:0] reduction_operand_request;

  // Handle FPU latencies for reduction operations, Used during the intra lane phase to empty the internal pipeline registers
  fp_format_e el_type;
  assign el_type = (spatz_req.vtype.vsew) == EW_64 ? fpnew_pkg::FP64 :
                   (spatz_req.vtype.vsew) == EW_32 ? fpnew_pkg::FP32 :
                   (spatz_req.vtype.vsew) == EW_16 ? ( spatz_req.fm.src ? fpnew_pkg::FP16ALT : fpnew_pkg::FP16) :
                   (spatz_req.vtype.vsew) == EW_8  ? ( spatz_req.fm.src ? fpnew_pkg::FP8ALT : fpnew_pkg::FP8) : fpnew_pkg::FP64;
  logic [5:0] FPUlatency, lat_count_d, lat_count_q;
  assign FPUlatency = FPUImplementation.PipeRegs[ADDMUL][el_type];
  `FF(lat_count_q, lat_count_d, '0)

  logic [$clog2(N_FU)-1:0] num_inter_lane_iterations_d, num_inter_lane_iterations_q;
  `FF(num_inter_lane_iterations_q, num_inter_lane_iterations_d, '0)

  logic [N_FU*ELEN-1:0] v0_mask; // bit mask to select the valid elements in v0.t for reduction instructions
  logic [N_FU*ELEN-1:0] mask; // bit mask

  always_comb begin: reduction_neutral_value_selection
    reduction_neutral_value = '0;
    if(spatz_req.op_arith.is_reduction == 1'b1) begin
      case(spatz_req.op)
        VADD: // VREDSUM_VS, VFREDUSUM_VS, VFREDOSUM_VS
          reduction_neutral_value = '0;
        VAND: // VREDAND_VS:
          reduction_neutral_value = '1;
        VOR, // VREDOR_VS,
        VXOR: // VREDXOR_VS:
          reduction_neutral_value = '0;
        VMINU: // VREDMINU_VS:
          reduction_neutral_value = '1;
        VMIN: // VREDMIN_VS:
          unique case(spatz_req.vtype.vsew)
            EW_8:reduction_neutral_value = {1'b0,7'h7f};
            EW_16:reduction_neutral_value = {1'b0,15'h7fff};
            EW_32:reduction_neutral_value = {1'b0,31'h7fffffff};
            default:
              if(MAXEW == EW_64) reduction_neutral_value = {1'b0,63'h7fffffffffffffff};
          endcase
        VMAXU: // VREDMAXU_VS
          reduction_neutral_value = '0;
        VMAX: // VREDMAX_VS
          unique case(spatz_req.vtype.vsew)
            EW_8:reduction_neutral_value = {1'b1,7'h0};
            EW_16:reduction_neutral_value = {1'b1,15'h0};
            EW_32:reduction_neutral_value = {1'b1,31'h0};
            default:
              if(MAXEW == EW_64) reduction_neutral_value = {1'b1,63'h0};
          endcase
        VFMINMAX: begin
         if(spatz_req.rm == fpnew_pkg::RNE) begin // VFREDMIN_VS
          unique case(fpu_src_fmt)
          // + infinity
            fpnew_pkg::FP64:reduction_neutral_value = {1'b0,11'h7ff,52'h0};
            fpnew_pkg::FP32:reduction_neutral_value = {1'b0,8'hff,23'h0};
            fpnew_pkg::FP16:reduction_neutral_value = {1'b0,5'h1f,10'h0};
            fpnew_pkg::FP16ALT:reduction_neutral_value = {1'b0,8'hff,7'h0};
            fpnew_pkg::FP8:reduction_neutral_value = {1'b0,5'h1f,2'h0};
            fpnew_pkg::FP8ALT:reduction_neutral_value = {1'b0,4'hf,3'h0};
          endcase
         end
         if (spatz_req.rm == fpnew_pkg::RTZ) begin // VFREDMAX_VS
          unique case(fpu_src_fmt)
          // - infinity
            fpnew_pkg::FP64:reduction_neutral_value = {1'b1,11'h7ff,52'h0};
            fpnew_pkg::FP32:reduction_neutral_value = {1'b1,8'hff,23'h0};
            fpnew_pkg::FP16:reduction_neutral_value = {1'b1,5'h1f,10'h0};
            fpnew_pkg::FP16ALT:reduction_neutral_value = {1'b1,8'hff,7'h0};
            fpnew_pkg::FP8:reduction_neutral_value = {1'b1,5'h1f,2'h0};
            fpnew_pkg::FP8ALT:reduction_neutral_value = {1'b1,4'hf,3'h0};
          endcase
         end
        end
        default: reduction_neutral_value='0;
      endcase
    end
  end

  // FUs can accept a new word of operands
  logic fu_can_accept;
  assign fu_can_accept = spatz_req_valid && &(in_ready | ~valid_operations) && operands_ready && !stall;

  // the word can only advance when all slots have been consumed
  logic fu_word_can_advance;
  assign fu_word_can_advance = !divsqrt_shared_active || (fu_word_complete && vrf_wvalid_i);

  always_comb begin: proc_reduction
    // Maintain state
    reduction_state_d   = reduction_state_q;
    reduction_pointer_d = reduction_pointer_q;
    lat_count_d = lat_count_q;
    num_inter_lane_iterations_d = num_inter_lane_iterations_q;
    v0_mask = '1;
    mask = '1;
    reduction_vector_data = '0;
    reduction_scalar_data = '0;
    result_buf_tag_d = result_buf_tag_q;

    // No operands
    reduction_d               = reduction_q;
    reduction_operand_ready_d = 1'b0;

    // Did we issue a word to the FUs?
    word_issued = 1'b0;

    // Are we ready to accept a result?
    result_ready = 1'b0;

    // Reduction did not finish
    reduction_done = 1'b0;

    // Only request when initializing the reduction register
    reduction_operand_request[0] = (reduction_state_q == Reduction_Init) || !spatz_req.op_arith.is_reduction;
    reduction_operand_request[1] = (reduction_state_q inside {Reduction_Init, Reduction_Reduce}) || !spatz_req.op_arith.is_reduction;

    result_buf_d = result_buf_q;
    result_buf_valid_d = result_buf_valid_q;
    shift_amnt_d = shift_amnt_q;

    // For vl not aligned with 256 bits DLEN masking is done
    // This masking is currently done only for reduction operations
    // TODO: make it more general later (not just for reductions) and add more data types

    if (reduction_pointer_q == fill_cnt) begin
      // If this is the last VRF word, then mask the unused data
      automatic logic [$clog2(VRFWordWidth)-1:0] width;
      width = (spatz_req.vl << spatz_req.vtype.vsew << 3); // in number of bits required for mask
      mask =  (width == 0) ? '1 : (1 << width)-1;
    end

    if (is_fpu_insn)
      v0_shift = $clog2(VRFWordWidth/(N_FPU*ELEN));
    else
      v0_shift = $clog2(VRFWordWidth/(N_IPU*ELEN));

    v0_word_idx = reduction_pointer_q >> v0_shift;

    // Creating v0_mask for reduction instructions
    if (!spatz_req.op_arith.is_reduction || spatz_req.op_arith.vm) begin
      v0_mask = '1; // unmasked
    end else begin
      unique case (spatz_req.vtype.vsew)
        EW_8: begin
          for (int i = 0; i < VRFWordWidth/8; i++)
            v0_mask[8*i +: 8] = {8{reduction_operand_v0_t_q[v0_word_idx * (VRFWordWidth/8) + i]}};
        end
        EW_16: begin
          for (int i = 0; i < VRFWordWidth/16; i++)
            v0_mask[16*i +: 16] = {16{reduction_operand_v0_t_q[v0_word_idx * (VRFWordWidth/16) + i]}};
        end
        EW_32: begin
          for (int i = 0; i < VRFWordWidth/32; i++)
            v0_mask[32*i +: 32] = {32{reduction_operand_v0_t_q[v0_word_idx * (VRFWordWidth/32) + i]}};
        end
        default: begin
          if (MAXEW == EW_64)
            for (int i = 0; i < VRFWordWidth/64; i++)
              v0_mask[64*i +: 64] = {64{reduction_operand_v0_t_q[v0_word_idx * (VRFWordWidth/64) + i]}};
        end
      endcase
    end

    // Preprocess vector data for masking before reduction
    // tail masking + v0.t masking
    // use of reduction useless value
    unique case (spatz_req.vtype.vsew)
      EW_8:
        reduction_vector_data = (vrf_rdata_i[1] & mask & v0_mask) | ({N_FU*(ELEN/8){reduction_neutral_value[7:0]}} & ~(mask & v0_mask));
      EW_16:
        reduction_vector_data = (vrf_rdata_i[1] & mask & v0_mask) | ({N_FU*(ELEN/16){reduction_neutral_value[15:0]}} & ~(mask & v0_mask));
      EW_32:
        reduction_vector_data = (vrf_rdata_i[1] & mask & v0_mask) | ({N_FU*(ELEN/32){reduction_neutral_value[31:0]}} & ~(mask & v0_mask));
      default:
        if (MAXEW == EW_64)
          reduction_vector_data = (vrf_rdata_i[1] & mask & v0_mask) | ({N_FU{reduction_neutral_value[63:0]}} & ~(mask & v0_mask));
    endcase

    unique case (reduction_state_q)
      Reduction_NormalExecution: begin
        // Did we issue a word to the FUs?
        word_issued = fu_can_accept && fu_word_can_advance;

        // Are we ready to accept a result?
        result_ready = !dimc_write_grant && fu_word_complete && ((result_tag.wb && vfu_rsp_ready_i) || vrf_wvalid_i || (result_tag.is_cmp && !result_tag.last));

        // Initialize the pointers
        reduction_pointer_d = '0;

        // Do we have a new reduction instruction?
        if (spatz_req_valid && !stall && !running_q[spatz_req.id] && spatz_req.op_arith.is_reduction)
           reduction_state_d = (!spatz_req.op_arith.vm) ? Reduction_Read_V0_t : (is_fpu_busy || divsqrt_inflight_q) ? Reduction_Wait : Reduction_Init;
      end

      Reduction_Wait: begin
        // Are we ready to accept a result?
        result_ready = !dimc_write_grant && fu_word_complete && ((result_tag.wb && vfu_rsp_ready_i) || vrf_wvalid_i);

        if (!is_fpu_busy)
          reduction_state_d = Reduction_Init;
      end

      Reduction_Read_V0_t:begin
        if(reduction_v0_t_is_ready)
          if (!is_fpu_busy)
            reduction_state_d = Reduction_Init;
          else reduction_state_d = Reduction_Wait;
        else reduction_state_d = Reduction_Read_V0_t;
      end

      Reduction_Init: begin
        // Initialize the reduction with scalar value

        if (spatz_req.op inside {VFADD, VADD, VXOR}) begin
          reduction_scalar_data = spatz_req.vtype.vsew == EW_8  ? vrf_rdata_i[0][ 7:0] :
                                  spatz_req.vtype.vsew == EW_16 ? vrf_rdata_i[0][15:0] :
                                  spatz_req.vtype.vsew == EW_32 ? vrf_rdata_i[0][31:0] : vrf_rdata_i[0][63:0];
        end else if (spatz_req.op inside {VFMINMAX, VAND, VOR, VMAX, VMAXU, VMIN, VMINU}) begin
          reduction_scalar_data = spatz_req.vtype.vsew == EW_8  ? {32{vrf_rdata_i[0][ 7:0]}} :
                                  spatz_req.vtype.vsew == EW_16 ? {16{vrf_rdata_i[0][15:0]}} :
                                  spatz_req.vtype.vsew == EW_32 ? { 8{vrf_rdata_i[0][31:0]}} : {4{vrf_rdata_i[0][63:0]}};
        end

        // verilator lint_off SELRANGE
        unique case (spatz_req.vtype.vsew)
          EW_8 : begin
            reduction_d[0] = reduction_scalar_data;
            reduction_d[1] = reduction_vector_data;
          end
          EW_16: begin
            reduction_d[0] = reduction_scalar_data;
            reduction_d[1] = reduction_vector_data;
          end
          EW_32: begin
            reduction_d[0] = reduction_scalar_data;
            reduction_d[1] = reduction_vector_data;
          end
          default: begin
          `ifdef MEMPOOL_SPATZ
            reduction_d = '0;
          `else
            if (MAXEW == EW_64) begin
              reduction_d[0] = reduction_scalar_data;
              reduction_d[1] = reduction_vector_data;
            end
          `endif
          end
        endcase
        // verilator lint_on SELRANGE
        if (vrf_rvalid_i[0] && vrf_rvalid_i[1]) begin
          reduction_operand_ready_d = 1'b1;
          reduction_pointer_d = reduction_pointer_q + 1;
          reduction_state_d = Reduction_Reduce;

          // Request next word
          word_issued = is_fpu_insn ? 1'b1 : (VRFWordWidth==ELEN*N_IPU) ? 1'b1 : !(|pnt) ? 1'b1 : 1'b0;
          if (reduction_pointer_q == fill_cnt) begin
            reduction_state_d = Reduction_IntraLane;
            reduction_pointer_d = '0;
          end
        end
      end

      Reduction_Reduce: begin
        // Feed rest of the vector in consecutive cycles to the FU operand 1
        // operand 0 uses the result from the FU or maintains an identity value
        // IPU uses reduction pointer to access the sub word
        // Switch to intra-lane once we are done reading the whole vector from the VRF

        // verilator lint_off SELRANGE
        `ifdef MEMPOOL_SPATZ
          reduction_d = '0;
        `else
          // Maintain the input operand or set to '0 for the inputs until valid results arrive
          reduction_d[0] = result_valid ? result : spatz_req.op inside {VFMINMAX, VAND, VOR, VMAX, VMAXU, VMIN, VMINU} ? reduction_q[0] : '0;
          if ((N_IPU>0) && ~is_fpu_insn) begin
            // If IPU is used and if the DLEN < VRF word size, select the chunk
            reduction_d[1] = (VRFWordWidth==N_IPU*ELEN) ? $unsigned(reduction_vector_data) :
                                                          $unsigned(reduction_vector_data[ELEN*N_IPU*reduction_pointer_q[idx_width(VRFWordWidth/(N_IPU*ELEN))-1:0]+:ELEN*N_IPU]);
          end else begin
            reduction_d[1] = $unsigned(reduction_vector_data);
          end
        `endif

        // verilator lint_on SELRANGE
        if (vrf_rvalid_i[1]) begin
          reduction_operand_ready_d = 1'b1;
          reduction_pointer_d = reduction_pointer_q + 1;
          word_issued = is_fpu_insn ? 1'b1 : (VRFWordWidth==ELEN*N_IPU) ? 1'b1 : !(|pnt) ? 1'b1 : 1'b0;
          if (result_valid[0]) begin
            result_ready = 1'b1;
          end
          if (reduction_pointer_q == fill_cnt) begin
            reduction_state_d = Reduction_IntraLane;
            reduction_pointer_d = '0;
          end
        end
      end

      Reduction_IntraLane: begin
        // This stage drains the pipeline stages of the FU
        // Every 2 consecutive results is collected and fed back to FU
        // Output is a single ELEN result from FU

        // verilator lint_off SELRANGE
        `ifdef MEMPOOL_SPATZ
          reduction_d = '0;
        `else
          reduction_d[0] = result_valid ? $unsigned(result) : reduction_q[0];
          reduction_d[1] = result_buf_valid_q ? $unsigned(result_buf_q) :  reduction_q[1];
        `endif
        lat_count_d = result_valid[0] ? '0 : lat_count_q + 1;

        // verilator lint_on SELRANGE
        if (~result_buf_valid_q & result_valid[0]) begin
          // First result is written into a buffer and its tag
          result_buf_valid_d = 1'b1;
          result_buf_d = result;
          result_buf_tag_d = result_tag;
          result_ready = 1'b1;
        end

        // verilator lint_on SELRANGE
        else if (result_buf_valid_q & result_valid[0]) begin
          // If there is an existing result in buffer and one from FU, initiate a reduction

          // Trigger a request
          reduction_operand_ready_d = 1'b1;
          result_buf_valid_d = 1'b0;

          // Bump pointer
          reduction_pointer_d = reduction_pointer_q + 1;

          // Acknowledge result
          result_ready = 1'b1;
        end

        // Are we done?
        // Wait until FU latency to ensure the pipelines are drained
        if (!result_valid[0] && (lat_count_q > (FPUlatency + 1))) begin
          reduction_state_d = Reduction_InterLane;

          // Written for max 8 FPU / 8 IPUs
          num_inter_lane_iterations_d = (spatz_req.vl << spatz_req.vtype.vsew) <= (ELENB) ? 0:
                                        (spatz_req.vl << spatz_req.vtype.vsew) <= (2*ELENB) ? 1 :
                                          (spatz_req.vl << spatz_req.vtype.vsew) <= (4*ELENB) ? 2 : 3;
          num_inter_lane_iterations_d = is_fpu_insn ? (num_inter_lane_iterations_d > $clog2(N_FPU) ? $clog2(N_FPU) : num_inter_lane_iterations_d) :
                                                      (num_inter_lane_iterations_d > $clog2(N_IPU) ? $clog2(N_IPU) : num_inter_lane_iterations_d);

          // Skip interlane
          if (num_inter_lane_iterations_d == 0) begin
            reduction_state_d = (spatz_req.vtype.vsew == MAXEW) ? Reduction_WriteBack : spatz_req.vl == 1 ? Reduction_WriteBack : Reduction_SIMD;
            shift_amnt_d = ELEN >> (MAXEW - spatz_req.vtype.vsew);
          end else begin
            shift_amnt_d = ELEN;
          end
          reduction_pointer_d = '0;
          lat_count_d = '0;
        end
      end

      Reduction_InterLane: begin
        // The single ELEN result from each FU is reduced across all FUs in log tree
        // The final result goes into the SIMD reduction stage for smaller element widths or the writeback stage

        // verilator lint_off SELRANGE
        `ifdef MEMPOOL_SPATZ
          reduction_d = '0;
        `else
            reduction_d[0] = (result_buf_valid_q ? $unsigned(result_buf_q) : $unsigned(result)) >> shift_amnt_q;
            reduction_d[1] = (result_buf_valid_q ? $unsigned(result_buf_q) : $unsigned(result));
        `endif

        // verilator lint_on SELRANGE
        if (result_valid[0] || result_buf_valid_q) begin
            // Trigger a request
            reduction_operand_ready_d = 1'b1;

            // Bump pointer
            reduction_pointer_d = reduction_pointer_q + 1;

            // Acknowledge result
            result_ready = result_valid[0];
            result_buf_valid_d = 1'b0;

            // Update shift amnt
            shift_amnt_d = shift_amnt_q << 1;
        end

        // Are we done?
        if ((reduction_pointer_q == (num_inter_lane_iterations_q-1)) && (reduction_operand_ready_d == 1'b1)) begin
          if (spatz_req.vtype.vsew != MAXEW) begin
            reduction_state_d = Reduction_SIMD;
          end else begin
            reduction_state_d = Reduction_WriteBack;
          end
          shift_amnt_d = ELEN >> (MAXEW - spatz_req.vtype.vsew);
          num_inter_lane_iterations_d = '0;
          reduction_pointer_d = '0;
          result_buf_valid_d = '0;
        end
      end

      Reduction_SIMD: begin
        // In the SIMD stage, log tree reduction for lesser element widths is done

        // verilator lint_off SELRANGE
        `ifdef MEMPOOL_SPATZ
          reduction_d = '0;
        `else
            reduction_d[0] = (result_buf_valid_q ? $unsigned(result_buf_q) : $unsigned(result)) >> shift_amnt_q;
            reduction_d[1] = (result_buf_valid_q ? $unsigned(result_buf_q) : $unsigned(result));
        `endif

        // verilator lint_on SELRANGE
        if (result_valid[0] || result_buf_valid_q) begin

            result_buf_valid_d = 1'b0;

            // Trigger a request
            reduction_operand_ready_d = 1'b1;

            // Bump pointer
            reduction_pointer_d = reduction_pointer_q + 1;

            // Acknowledge result
            result_ready = result_valid[0];

            // Update shift amnt
            shift_amnt_d = shift_amnt_q << 1;
        end

        // Are we done?
        if ((reduction_pointer_q == (MAXEW - spatz_req.vtype.vsew - 1)) && (reduction_operand_ready_d == 1'b1)) begin
          reduction_state_d = Reduction_WriteBack;
          reduction_pointer_d = '0;
          shift_amnt_d = ELEN;
        end
      end

      Reduction_WriteBack: begin
        // Acknowledge result
        if (vrf_wvalid_i) begin
          result_ready = result_valid[0];
          result_buf_valid_d = 1'b0;
          result_buf_d = '0;
          result_buf_tag_d = '0;

          // We are done with the reduction
          reduction_state_d = Reduction_NormalExecution;

          // Finish the reduction
          reduction_done = 1'b1;
        end
      end

      default;
    endcase
  end: proc_reduction

  ///////////////////////
  // Operand Requester //
  ///////////////////////

  vrf_be_t       vreg_wbe;
  logic          vreg_we;
  logic    [2:0] vreg_r_req;

  // Address register
  vrf_addr_t [2:0] vreg_addr_q, vreg_addr_d;
  `FF(vreg_addr_q, vreg_addr_d, '0)

  // Calculate new vector register address
  always_comb begin : vreg_addr_proc
    vreg_addr_d = vreg_addr_q;

    vrf_raddr_o = vreg_addr_d;
    vrf_waddr_o = dimc_write_grant ? dimc_write_addr : vrf_addr_t'(result_tag.vd_addr);

    // Tag (propagated with the operations)
    input_tag = '{
      id             : spatz_req.id,
      vsew           : spatz_req.vtype.vsew,
      vstart         : spatz_req.vstart,
      vd_addr        : spatz_req.op_arith.is_scalar ? vfu_rsp_addr_t'(spatz_req.rd) : vfu_rsp_addr_t'(vreg_addr_q[2]),
      wb             : spatz_req.op_arith.is_scalar,
      last           : last_request,
      narrowing      : spatz_req.op_arith.is_narrowing,
      narrowing_upper: narrowing_upper_q,
      reduction      : spatz_req.op_arith.is_reduction,
      is_cmp         : is_cmp_req,
      vl             : spatz_req.vl,
      vm             : spatz_req.op_arith.vm,
      vma            : spatz_req.vtype.vma,
      is_merge       : (spatz_req.op == VMERGE),
      valid_bytes    : valid_bytes_wr // count of the number of valid bytes in the VRF word (write side)
    };

    case(operand_state_q)
       READ_OPERANDS:begin
        if(switch_to_read_v0t) begin
          vrf_raddr_o = vreg_addr_d;

        end else if(reduction_state_q == Reduction_Read_V0_t) begin
          vreg_addr_d[0] =  0 << $clog2(NrWordsPerVector);
          vreg_addr_d[1] =  1 << $clog2(NrWordsPerVector);
          vrf_raddr_o = vreg_addr_d;
        end
        else begin

          if (spatz_req_valid && vl_q == '0) begin
            vreg_addr_d[0] = (spatz_req.vs2 + vstart) << $clog2(NrWordsPerVector);
            vreg_addr_d[1] = (spatz_req.vs1 + vstart) << $clog2(NrWordsPerVector);
            vreg_addr_d[2] = (spatz_req.vd + vstart) << $clog2(NrWordsPerVector);

          // Direct feedthrough
          vrf_raddr_o = vreg_addr_d;
          if (!spatz_req.op_arith.is_scalar)
            input_tag.vd_addr = vfu_rsp_addr_t'(vreg_addr_d[2]);

          // Did we commit a word already?
          if (word_issued) begin
            vreg_addr_d[0] = vreg_addr_d[0] + (!spatz_req.op_arith.widen_vs2 || widening_upper_q);
            vreg_addr_d[1] = vreg_addr_d[1] + (!spatz_req.op_arith.widen_vs1 || widening_upper_q);
            vreg_addr_d[2] = vreg_addr_d[2] + (!spatz_req.op_arith.is_reduction && (!spatz_req.op_arith.is_narrowing || narrowing_upper_q) && !is_cmp_req);
          end
          end else if (spatz_req_valid && vl_q < spatz_req.vl && word_issued) begin
            vreg_addr_d[0] = vreg_addr_q[0] + (!spatz_req.op_arith.widen_vs2 || widening_upper_q);
            vreg_addr_d[1] = vreg_addr_q[1] + (!spatz_req.op_arith.widen_vs1 || widening_upper_q);
            vreg_addr_d[2] = vreg_addr_q[2] + (!spatz_req.op_arith.is_reduction && (!spatz_req.op_arith.is_narrowing || narrowing_upper_q) && !is_cmp_req);
          end
        end
       end
       READ_V0_t: begin
         vreg_addr_d[0] = ( 0 + vstart) << $clog2(NrWordsPerVector);
         vreg_addr_d[1] = ( 1 + vstart) << $clog2(NrWordsPerVector);
         vrf_raddr_o = vreg_addr_d;
       end
       READ_VD_t: begin
          vreg_addr_d[0] = spatz_req.vd << $clog2(NrWordsPerVector);
          vreg_addr_d[1] = (spatz_req.vd << $clog2(NrWordsPerVector)) + 1;
          vrf_raddr_o = vreg_addr_d;
        end
       default:;
   endcase
    if (dimc_vrf_read_feature)
      vrf_raddr_o[1] = dimc_feature_addr;
    if (dimc_vrf_read_kernel)
      vrf_raddr_o[0] = dimc_kernel_addr;
    if (dimc_vrf_read_upper)
      vrf_raddr_o[2] = dimc_upper_addr;
  end: vreg_addr_proc

  logic [VRFWordBWidth-1:0] tail_wbe;
  always_comb begin : operand_req_proc
    vreg_r_req = '0;
    vreg_we    = '0;

    unique case(operand_state_q)
      READ_V0_t: vreg_r_req = 3'b011;
      READ_VD_t: vreg_r_req = 3'b011;
      READ_OPERANDS: begin
        if (switch_to_read_v0t) begin
          vreg_r_req = '0;  // avoid unuseful read
        end
        else if(reduction_state_q == Reduction_Read_V0_t) vreg_r_req = 3'b011;
        else
          if (spatz_req_valid && vl_q < spatz_req.vl)
            vreg_r_req = {spatz_req.vd_is_src, spatz_req.use_vs1 && reduction_operand_request[1], spatz_req.use_vs2 && reduction_operand_request[0]};
      end
      default:;
    endcase
    // Got a new result
    if (fu_word_complete && !result_tag.reduction) begin
      vreg_we  = !result_tag.wb;
      if (result_tag.is_cmp) begin
        vreg_we    = result_tag.last;
      end
    end

    // Reduction finished execution
    if (reduction_state_q == Reduction_WriteBack && (result_valid[0] || result_buf_valid_q)) begin
      vreg_we = 1'b1;
    end
    if (dimc_read_grant) vreg_r_req = '0;
    if (dimc_vrf_read_feature) vreg_r_req[1] = 1'b1;
    if (dimc_vrf_read_kernel) vreg_r_req[0] = 1'b1;
    if (dimc_vrf_read_upper) vreg_r_req[2] = 1'b1;
    if (dimc_write_grant) vreg_we = 1'b1;
  end : operand_req_proc

 // vreg_wbe logic
 vlen_t vreg_wb_word_cnt_q, vreg_wb_word_cnt_d;
 `FF(vreg_wb_word_cnt_q, vreg_wb_word_cnt_d, '0)
 vew_e sew_wb;
 logic widening_wb;
 assign widening_wb = spatz_req.op_arith.widen_vs1 || spatz_req.op_arith.widen_vs2;
 assign sew_wb = vew_e'(int'(spatz_req.vtype.vsew) + widening_wb);

 vrf_be_t       vreg_wbe_pre;
 logic [VRFWordBWidth-1:0] tail_wbe_eff;

always_comb begin : vreg_wbe_proc
    vreg_wbe   = '0;
    vreg_wbe_pre = '0;
    vreg_wb_word_cnt_d = vreg_wb_word_cnt_q;
    vreg_wbe_pre = '0;

    //write just the significant bytes (tail unisturbed)
    for (int b = 0; b < N_FU*ELENB; b++)
      tail_wbe[b] = (b < result_tag.valid_bytes);

    if (result_tag.narrowing) begin
      if (result_tag.narrowing_upper)
        tail_wbe_eff = (tail_wbe << N_FU*ELENB/2) & {{N_FU*ELENB/2{1'b1}}, {N_FU*ELENB/2{1'b0}}};
      else
        tail_wbe_eff = tail_wbe & {{N_FU*ELENB/2{1'b0}}, {N_FU*ELENB/2{1'b1}}};
    end else
      tail_wbe_eff = tail_wbe;

    if ((result_tag.last && result_ready && fu_word_complete && (reduction_state_q inside {Reduction_NormalExecution, Reduction_Wait})) || reduction_done)
      vreg_wb_word_cnt_d = 0;
    else if (result_ready && fu_word_complete && (!result_tag.narrowing || result_tag.narrowing_upper))
      vreg_wb_word_cnt_d = vreg_wb_word_cnt_q + 1;
    // Got a new result
    if (fu_word_complete && !result_tag.reduction) begin
      vreg_wbe = '1;
      if (result_tag.is_cmp) begin
        // every vector element requires 1 bit of wbe --> ceil(vl/8)
        automatic logic [$clog2((MAXVL+7)/8+1)-1:0] mask_bytes;
        mask_bytes = (result_tag.vl + 7) >> 3;
        vreg_wbe   = (mask_bytes >= N_FU*ELENB) ? '1 : vrf_be_t'((vrf_be_t'(1) << mask_bytes) - 1);
      end else if(!result_tag.vm && !result_tag.is_merge && !spatz_req.op_arith.is_scalar && !result_tag.narrowing) begin //masking the wb results
        unique case (sew_wb) // add widening support
          EW_8:for(int i=0;i<VRFWordBWidth;i=i+1)begin
            vreg_wbe[i*1+:1] = {1{operand_v0_t_q[vreg_wb_word_cnt_q * VRFWordBWidth + i]}};
          end
          EW_16:for(int i=0;i<VRFWordBWidth/2;i=i+1)begin
            vreg_wbe[i*2+:2] = {2{operand_v0_t_q[vreg_wb_word_cnt_q * (VRFWordBWidth/2) + i]}};
          end
          EW_32: for(int i=0;i<VRFWordBWidth/4;i=i+1)begin
            vreg_wbe[i*4+:4] = {4{operand_v0_t_q[vreg_wb_word_cnt_q * (VRFWordBWidth/4) + i]}};
          end
          default: if (MAXEW == EW_64) for(int i=0;i<VRFWordBWidth/8;i=i+1)begin
            vreg_wbe[i*8+:8] = {8{operand_v0_t_q[vreg_wb_word_cnt_q * (VRFWordBWidth/8) + i]}};
          end
        endcase
        vreg_wbe &= tail_wbe_eff; // tail-undisturbed + masking (v0.t)
      end else if(result_tag.narrowing) begin
        if(!result_tag.vm && !spatz_req.op_arith.is_scalar) begin
          unique case (sew_wb)
            EW_16:for(int i=0;i<VRFWordBWidth/2;i=i+1)begin
              vreg_wbe_pre[i*2+:2] = {2{operand_v0_t_q[vreg_wb_word_cnt_q * (VRFWordBWidth/2) + i]}};
              vreg_wbe = result_tag.narrowing_upper ? {vreg_wbe_pre[N_FU*ELENB-1:(N_FU*ELENB/2)],{(N_FU*ELENB/2){1'b0}}} : {{(N_FU*ELENB/2){1'b0}}, vreg_wbe_pre[(N_FU*ELENB/2)-1:0]};
            end
            EW_32: for(int i=0;i<VRFWordBWidth/4;i=i+1)begin
              vreg_wbe_pre[i*4+:4] = {4{operand_v0_t_q[vreg_wb_word_cnt_q * (VRFWordBWidth/4) + i]}};
              vreg_wbe = result_tag.narrowing_upper ? {vreg_wbe_pre[N_FU*ELENB-1:(N_FU*ELENB/2)],{(N_FU*ELENB/2){1'b0}}} : {{(N_FU*ELENB/2){1'b0}}, vreg_wbe_pre[(N_FU*ELENB/2)-1:0]};
            end
            EW_64: for(int i=0;i<VRFWordBWidth/8;i=i+1)begin
              vreg_wbe_pre[i*8+:8] = {8{operand_v0_t_q[vreg_wb_word_cnt_q * (VRFWordBWidth/8) + i]}};
              vreg_wbe = result_tag.narrowing_upper ? {vreg_wbe_pre[N_FU*ELENB-1:(N_FU*ELENB/2)],{(N_FU*ELENB/2){1'b0}}} : {{(N_FU*ELENB/2){1'b0}}, vreg_wbe_pre[(N_FU*ELENB/2)-1:0]};
            end
            default:;
          endcase
          vreg_wbe &= tail_wbe_eff;
        end else
          vreg_wbe = tail_wbe_eff;
      end else begin
        vreg_wbe = tail_wbe_eff;
      end
    end

    // Reduction finished execution
    if (reduction_state_q == Reduction_WriteBack && (result_valid[0] || result_buf_valid_q)) begin
      unique case (spatz_req.vtype.vsew)
        EW_8 : vreg_wbe = 1'h1;
        EW_16: vreg_wbe = 2'h3;
        EW_32: vreg_wbe = 4'hf;
        default: if (MAXEW == EW_64) vreg_wbe = 8'hff;
      endcase
    end
    if (dimc_write_grant) vreg_wbe = '1;
end:vreg_wbe_proc

logic vfcmp_result_accepted;
assign vfcmp_result_accepted = result_tag.is_cmp && fu_word_complete && result_ready;

  always_comb begin : VRF_cnt_proc
    word_idx_d = word_idx_q;
    if (vfcmp_result_accepted) begin
      if (result_tag.last)
        word_idx_d = '0;
      else
        word_idx_d = word_idx_q + 1;
    end
  end

  logic [N_FU*ELEN-1:0] vreg_wdata, wdata_d, wdata_q;
  always_comb begin : align_result
    // Data from the FU to be written to the VRF
    // For reductions, if the result is present in the buffer used for intra-lane reductions
    vreg_wdata = result_buf_valid_q ? result_buf_q : result;

    if (result_tag.narrowing) begin
      unique case (MAXEW)
        EW_64: begin
          if (RVD)
            for (int element = 0; element < N_FU; element++)
              vreg_wdata[32*element + (N_FU * ELEN * result_tag.narrowing_upper / 2) +: 32] = result[64*element +: 32];
        end
        EW_32: begin
          for (int element = 0; element < (MAXEW == EW_64 ? N_FU*2 : N_FU); element++)
            vreg_wdata[16*element + (N_FU * ELEN * result_tag.narrowing_upper / 2) +: 16] = result[32*element +: 16];
        end
        default:;
      endcase

    end else if (result_tag.is_cmp) begin
      automatic logic v0_bit;
      vreg_wdata = '0;

      unique case (result_tag.vsew)
        EW_8: begin
          for (int i = 0; i < VRFWordWidth/8; i++) begin
            v0_bit = (result_tag.vm) ? 1'b1 : operand_v0_t_q[i + (VRFWordWidth/8)*word_idx_q];
            vreg_wdata[i + (VRFWordWidth/8)*word_idx_q] = result[i*8] & v0_bit;
          end
        end
        EW_16: begin
          for (int i = 0; i < VRFWordWidth/16; i++) begin
            v0_bit = (result_tag.vm) ? 1'b1 : operand_v0_t_q[i + (VRFWordWidth/16)*word_idx_q];
            vreg_wdata[i + (VRFWordWidth/16)*word_idx_q] = result[i*16] & v0_bit;
          end
        end
        EW_32: begin
          for (int i = 0; i < VRFWordWidth/32; i++) begin
            v0_bit = (result_tag.vm) ? 1'b1 : operand_v0_t_q[i + (VRFWordWidth/32)*word_idx_q];
            vreg_wdata[i + (VRFWordWidth/32)*word_idx_q] = result[i*32] & v0_bit;
          end
        end
        EW_64: begin
          for (int i = 0; i < VRFWordWidth/64; i++) begin
            v0_bit = (result_tag.vm) ? 1'b1 : operand_v0_t_q[i + (VRFWordWidth/64)*word_idx_q];
            vreg_wdata[i + (VRFWordWidth/64)*word_idx_q] = result[i*64] & v0_bit;
          end
        end
        default:;
      endcase
    end
  end

  always_comb begin : wdata_proc
    wdata_d = wdata_q;
    if (vfcmp_result_accepted) begin
      if (result_tag.last)
        wdata_d = '0;
      else
        wdata_d = wdata_q | vreg_wdata;
    end
  end

  `FF(wdata_q, wdata_d, '0)

  // Register file signals
  assign vrf_re_o    = vreg_r_req;
  assign vrf_we_o    = vreg_we;
  assign vrf_wbe_o   = vreg_wbe;
  always_comb begin : vrf_wdata_proc
    if (dimc_write_grant) vrf_wdata_o = dimc_write_data;
    else if (result_tag.is_cmp) begin
      if(result_tag.vm)
        vrf_wdata_o = wdata_q | vreg_wdata;
      else
        vrf_wdata_o = ((wdata_q | vreg_wdata) & operand_v0_t_q) | (cmp_dst_inactive & ~operand_v0_t_q);
    end else begin
      vrf_wdata_o = vreg_wdata;
    end
  end
  always_comb begin
    vrf_id_o = {dimc_write_grant ? dimc_write_id : result_tag.id, {3{spatz_req.id}}};
    if (dimc_vrf_read_kernel)  vrf_id_o[0] = dimc_active_id_q;
    if (dimc_vrf_read_feature) vrf_id_o[1] = dimc_active_id_q;
    if (dimc_vrf_read_upper)   vrf_id_o[2] = dimc_active_id_q;
  end
  assign vxsat_o = |(saturated & vreg_wbe) && vreg_we && vrf_wvalid_i &&
                   !dimc_write_grant && !result_tag.reduction;

  //////////
  // IPUs //
  //////////

  // If there are fewer IPUs than FPUs, pipeline the execution of the integer instructions
  logic     [N_IPU*ELENB-1:0] int_ipu_in_ready;
  logic     [N_IPU*ELEN-1:0]  int_ipu_operand1;
  logic     [N_IPU*ELEN-1:0]  int_ipu_operand2;
  logic     [N_IPU*ELEN-1:0]  int_ipu_operand3;
  logic     [N_IPU*ELEN-1:0]  int_ipu_result;
  vfu_tag_t [N_IPU-1:0]       int_ipu_result_tag;
  logic     [N_IPU*ELENB-1:0] int_ipu_result_valid;
  logic     [N_IPU*ELENB-1:0] int_ipu_saturated;
  logic                       int_ipu_result_ready;
  logic     [N_IPU-1:0]       int_ipu_busy;

  // A serialized IPU can be idle while its assembled VRF word is still pending.
  assign is_ipu_busy = |int_ipu_busy || |ipu_result_valid;

  logic [N_FU*ELEN-1:0] ipu_wide_operand1, ipu_wide_operand2, ipu_wide_operand3;
  always_comb begin: gen_ipu_widening
    automatic logic [N_FU*ELEN/2-1:0] shift_operand1 = !widening_upper_q ? operand1[N_FU*ELEN/2-1:0] : operand1[N_FU*ELEN-1:N_FU*ELEN/2];
    automatic logic [N_FU*ELEN/2-1:0] shift_operand2 = !widening_upper_q ? operand2[N_FU*ELEN/2-1:0] : operand2[N_FU*ELEN-1:N_FU*ELEN/2];

    ipu_wide_operand1 = operand1;
    ipu_wide_operand2 = operand2;
    ipu_wide_operand3 = operand3;

    case (spatz_req.vtype.vsew)
      EW_32: begin
        for (int el = 0; el < N_FU; el++) begin
          if (spatz_req.op_arith.widen_vs1 && MAXEW == EW_64)
            ipu_wide_operand1[64*el +: 64] = spatz_req.op_arith.signed_vs1 ? {{32{shift_operand1[32*el+31]}}, shift_operand1[32*el +: 32]} : {32'b0, shift_operand1[32*el +: 32]};

          if (spatz_req.op_arith.widen_vs2 && MAXEW == EW_64)
            ipu_wide_operand2[64*el +: 64] = spatz_req.op_arith.signed_vs2 ? {{32{shift_operand2[32*el+31]}}, shift_operand2[32*el +: 32]} : {32'b0, shift_operand2[32*el +: 32]};
        end
      end
      EW_16: begin
        for (int el = 0; el < (MAXEW == EW_64 ? 2*N_FU : N_FU); el++) begin
          if (spatz_req.op_arith.widen_vs1)
            ipu_wide_operand1[32*el +: 32] = spatz_req.op_arith.signed_vs1 ? {{16{shift_operand1[16*el+15]}}, shift_operand1[16*el +: 16]} : {16'b0, shift_operand1[16*el +: 16]};

          if (spatz_req.op_arith.widen_vs2)
            ipu_wide_operand2[32*el +: 32] = spatz_req.op_arith.signed_vs2 ? {{16{shift_operand2[16*el+15]}}, shift_operand2[16*el +: 16]} : {16'b0, shift_operand2[16*el +: 16]};
        end
      end
      EW_8: begin
        for (int el = 0; el < (MAXEW == EW_64 ? 4*N_FU : 2*N_FU); el++) begin
          if (spatz_req.op_arith.widen_vs1)
            ipu_wide_operand1[16*el +: 16] = spatz_req.op_arith.signed_vs1 ? {{8{shift_operand1[8*el+7]}}, shift_operand1[8*el +: 8]} : {8'b0, shift_operand1[8*el +: 8]};

          if (spatz_req.op_arith.widen_vs2)
            ipu_wide_operand2[16*el +: 16] = spatz_req.op_arith.signed_vs2 ? {{8{shift_operand2[8*el+7]}}, shift_operand2[8*el +: 8]} : {8'b0, shift_operand2[8*el +: 8]};
        end
      end
      default:;
    endcase
  end: gen_ipu_widening

  if (N_IPU < N_FU) begin: gen_pipeline_ipu
    logic [N_FU*ELEN-1:0] ipu_result_d, ipu_result_q;
    logic [N_FU*ELENB-1:0] ipu_result_valid_q, ipu_result_valid_d;
    logic [N_FU*ELENB-1:0] ipu_saturated_q, ipu_saturated_d;
    logic [idx_width(N_FU/N_IPU)-1:0] ipu_result_pnt_d, ipu_result_pnt_q;
    vfu_tag_t ipu_result_tag_d, ipu_result_tag_q;
    logic [idx_width(N_FU/N_IPU)-1:0] ipu_operand_pnt_d, ipu_operand_pnt_q;

    `FF(ipu_result_q, ipu_result_d, '0)
    `FF(ipu_result_valid_q, ipu_result_valid_d, '0)
    `FF(ipu_saturated_q, ipu_saturated_d, '0)
    `FF(ipu_result_pnt_q, ipu_result_pnt_d, '0)
    `FF(ipu_result_tag_q, ipu_result_tag_d, '0)
    `FF(ipu_operand_pnt_q, ipu_operand_pnt_d, '0)

    always_comb begin
      // Maintain state
      ipu_result_d       = ipu_result_q;
      ipu_result_valid_d = ipu_result_valid_q;
      ipu_saturated_d    = ipu_saturated_q;
      ipu_result_pnt_d   = ipu_result_pnt_q;
      ipu_operand_pnt_d  = ipu_operand_pnt_q;
      ipu_result_tag_d   = ipu_result_tag_q;

      // Send operands
      ipu_in_ready     = 1'b0;
      int_ipu_operand1 = ipu_wide_operand1[ipu_operand_pnt_q*ELEN*N_IPU +: ELEN*N_IPU];
      int_ipu_operand2 = ipu_wide_operand2[ipu_operand_pnt_q*ELEN*N_IPU +: ELEN*N_IPU];
      int_ipu_operand3 = ipu_wide_operand3[ipu_operand_pnt_q*ELEN*N_IPU +: ELEN*N_IPU];
      if (spatz_req_valid && operands_ready && &int_ipu_in_ready && !is_fpu_insn) begin
        ipu_operand_pnt_d = ipu_operand_pnt_q + 1;
        if (ipu_operand_pnt_d == '0 || !(&valid_operations[ipu_operand_pnt_d*ELENB*N_IPU +: ELENB*N_IPU]))
          ipu_operand_pnt_d = '0;

        // Issued all elements
        if (ipu_operand_pnt_d == 0)
          ipu_in_ready = '1;
      end

      // Clean-up results
      if (result_ready) begin
        ipu_result_d       = '0;
        ipu_result_valid_d = '0;
        ipu_saturated_d    = '0;
        ipu_result_tag_d   = '0;
      end

      // Store results
      int_ipu_result_ready = '0;
      // Hold a completed word when DIMC or VRF backpressure occupies the
      // write port. Do not overwrite its first slice with the next word.
      if (&int_ipu_result_valid &&
          (!(|ipu_result_valid_q[ipu_result_pnt_q*ELENB*N_IPU +: ELENB*N_IPU]) || result_ready)) begin
        ipu_result_d[ipu_result_pnt_q*ELEN*N_IPU +: ELEN*N_IPU]         = int_ipu_result;
        ipu_result_valid_d[ipu_result_pnt_q*ELENB*N_IPU +: ELENB*N_IPU] = int_ipu_result_valid;
        ipu_saturated_d[ipu_result_pnt_q*ELENB*N_IPU +: ELENB*N_IPU]    = int_ipu_saturated;
        ipu_result_tag_d                                                = int_ipu_result_tag[0];
        ipu_result_pnt_d                                                = ipu_result_pnt_q + 1;
        int_ipu_result_ready                                            = 1'b1;

        // Scalar operation
        if (ipu_result_tag_d.wb || ipu_result_tag_d.reduction)
          ipu_result_pnt_d = '0;
      end
    end

    // Forward results
    assign ipu_result       = ipu_result_q;
    assign ipu_result_valid = ipu_result_valid_q;
    assign ipu_saturated    = ipu_saturated_q;
    assign ipu_result_tag   = ipu_result_tag_q;
  end: gen_pipeline_ipu else begin: gen_no_pipeline_ipu
    assign ipu_in_ready         = int_ipu_in_ready;
    assign int_ipu_operand1     = ipu_wide_operand1;
    assign int_ipu_operand2     = ipu_wide_operand2;
    assign int_ipu_operand3     = ipu_wide_operand3;
    assign ipu_result           = int_ipu_result;
    assign ipu_result_valid     = int_ipu_result_valid;
    assign ipu_saturated        = int_ipu_saturated;
    assign int_ipu_result_ready = result_ready;
    assign ipu_result_tag       = int_ipu_result_tag[0];
  end

  for (genvar ipu = 0; unsigned'(ipu) < N_IPU; ipu++) begin : gen_ipus
    logic ipu_ready;
    assign int_ipu_in_ready[ipu*ELENB +: ELENB] = {ELENB{ipu_ready}};

    logic is_widening;
    assign is_widening = spatz_req.op_arith.widen_vs1 || spatz_req.op_arith.widen_vs2;

    vew_e sew;
    assign sew = vew_e'(int'(spatz_req.vtype.vsew) + is_widening);

    spatz_ipu #(
      .tag_t(vfu_tag_t)
    ) i_ipu (
      .clk_i            (clk_i                                                                                           ),
      .rst_ni           (rst_ni                                                                                          ),
      .operation_i      (spatz_req.op                                                                                    ),
      // Only the IPU0 executes scalar instructions
      .operation_valid_i(spatz_req_valid && operands_ready && (!spatz_req.op_arith.is_scalar || ipu == 0) && !is_fpu_insn),
      .operation_ready_o(ipu_ready                                                                                       ),
      .op_s1_i          (int_ipu_operand1[ipu*ELEN +: ELEN]                                                              ),
      .op_s2_i          (int_ipu_operand2[ipu*ELEN +: ELEN]                                                              ),
      .op_d_i           (int_ipu_operand3[ipu*ELEN +: ELEN]                                                              ),
      .tag_i            (input_tag                                                                                       ),
      .carry_i          ('0                                                                                              ),
      .sew_i            (sew                                                                                             ),
      .be_o             (/* Unused */                                                                                    ),
      .result_o         (int_ipu_result[ipu*ELEN +: ELEN]                                                                ),
      .result_valid_o   (int_ipu_result_valid[ipu*ELENB +: ELENB]                                                        ),
      .result_ready_i   (int_ipu_result_ready                                                                            ),
      .saturated_o      (int_ipu_saturated[ipu*ELENB +: ELENB]                                                           ),
      .tag_o            (int_ipu_result_tag[ipu]                                                                         ),
      .busy_o           (int_ipu_busy[ipu]                                                                               )
    );
  end : gen_ipus

  ////////////
  //  FPUs  //
  ////////////

  if (FPU) begin: gen_fpu
    logic [N_FPU*ELENB-1:0] lane_in_ready, lane_result_valid;
    logic [N_FPU*ELEN-1:0] lane_operand1, lane_operand2, lane_operand3, lane_result;
    vfu_tag_t [N_FPU-1:0] lane_result_tag;
    logic lane_result_ready;
    logic [N_FU*ELEN-1:0] wide_operand1, wide_operand2, wide_operand3;
    always_comb begin: gen_widening
      automatic logic [N_FU*ELEN/2-1:0] shift_operand1 = !widening_upper_q ? operand1[N_FU*ELEN/2-1:0] : operand1[N_FU*ELEN-1:N_FU*ELEN/2];
      automatic logic [N_FU*ELEN/2-1:0] shift_operand2 = !widening_upper_q ? operand2[N_FU*ELEN/2-1:0] : operand2[N_FU*ELEN-1:N_FU*ELEN/2];

      wide_operand1 = operand1;
      wide_operand2 = operand2;
      wide_operand3 = operand3;

      case (spatz_req.vtype.vsew)
        EW_32: begin
          for (int el = 0; el < N_FU; el++) begin
            if (spatz_req.op_arith.widen_vs1 && MAXEW == EW_64)
              wide_operand1[64*el +: 64] = widen_fp32_to_fp64(shift_operand1[32*el +: 32]);

            if (spatz_req.op_arith.widen_vs2 && MAXEW == EW_64)
              wide_operand2[64*el +: 64] = widen_fp32_to_fp64(shift_operand2[32*el +: 32]);
          end
        end
        EW_16: begin
          for (int el = 0; el < (MAXEW == EW_64 ? 2*N_FU : N_FU); el++) begin
            if (spatz_req.op_arith.widen_vs1)
              wide_operand1[32*el +: 32] = widen_fp16_to_fp32(shift_operand1[16*el +: 16]);

            if (spatz_req.op_arith.widen_vs2)
              wide_operand2[32*el +: 32] = widen_fp16_to_fp32(shift_operand2[16*el +: 16]);
          end
        end
        EW_8: begin
          for (int el = 0; el < (MAXEW == EW_64 ? 4*N_FU : 2*N_FU); el++) begin
            if (spatz_req.op_arith.widen_vs1)
              wide_operand1[16*el +: 16] = widen_fp8_to_fp16(shift_operand1[8*el +: 8]);

            if (spatz_req.op_arith.widen_vs2)
              wide_operand2[16*el +: 16] = widen_fp8_to_fp16(shift_operand2[8*el +: 8]);
          end
        end
        default:;
      endcase
    end: gen_widening

    if (N_FPU < N_FU) begin: gen_pipeline_fpu
      logic [N_FU*ELEN-1:0] data_d, data_q;
      logic [N_FU*ELENB-1:0] valid_d, valid_q;
      logic [idx_width(N_FU/N_FPU)-1:0] input_pnt_d, input_pnt_q, output_pnt_d, output_pnt_q;
      vfu_tag_t tag_d, tag_q;
      `FF(data_q, data_d, '0)
      `FF(valid_q, valid_d, '0)
      `FF(input_pnt_q, input_pnt_d, '0)
      `FF(output_pnt_q, output_pnt_d, '0)
      `FF(tag_q, tag_d, '0)

      always_comb begin
        data_d = data_q;
        valid_d = valid_q;
        input_pnt_d = input_pnt_q;
        output_pnt_d = output_pnt_q;
        tag_d = tag_q;
        fpu_in_ready = divsqrt_shared_active ? {N_FU{lane_in_ready[ELENB-1:0]}} : '0;
        lane_operand1 = wide_operand1[input_pnt_q*N_FPU*ELEN +: N_FPU*ELEN];
        lane_operand2 = wide_operand2[input_pnt_q*N_FPU*ELEN +: N_FPU*ELEN];
        lane_operand3 = wide_operand3[input_pnt_q*N_FPU*ELEN +: N_FPU*ELEN];
        if (!divsqrt_shared_active && spatz_req_valid && operands_ready && &lane_in_ready && is_fpu_insn) begin
          input_pnt_d = input_pnt_q + 1'b1;
          if (input_pnt_d == '0 || !(&valid_operations[input_pnt_d*N_FPU*ELENB +: N_FPU*ELENB]))
            input_pnt_d = '0;
          if (input_pnt_d == '0) fpu_in_ready = '1;
        end

        if (result_ready) begin
          data_d = '0;
          valid_d = '0;
          tag_d = '0;
        end
        // Advertise space independently of valid, allowing the FPU collector
        // to drain every lane, including short-latency exceptional results.
        lane_result_ready = !divsqrt_shared_active &&
            (!(|valid_q[output_pnt_q*N_FPU*ELENB +: N_FPU*ELENB]) || result_ready);
        if (&lane_result_valid && lane_result_ready) begin
          data_d[output_pnt_q*N_FPU*ELEN +: N_FPU*ELEN] = lane_result;
          valid_d[output_pnt_q*N_FPU*ELENB +: N_FPU*ELENB] = lane_result_valid;
          tag_d = lane_result_tag[0];
          output_pnt_d = output_pnt_q + 1'b1;
          if (tag_d.wb || tag_d.reduction) output_pnt_d = '0;
        end
      end
      assign fpu_result = divsqrt_shared_active ? {{(N_FU-N_FPU)*ELEN{1'b0}}, lane_result} : data_q;
      assign fpu_result_valid = divsqrt_shared_active ? {{(N_FU-N_FPU)*ELENB{1'b0}}, lane_result_valid} : valid_q;
      assign fpu_result_tag = divsqrt_shared_active ? lane_result_tag[0] : tag_q;
      assign fpu_gather_busy = |valid_q || input_pnt_q != '0;
    end else begin: gen_no_pipeline_fpu
      assign fpu_in_ready = lane_in_ready;
      assign lane_operand1 = wide_operand1;
      assign lane_operand2 = wide_operand2;
      assign lane_operand3 = wide_operand3;
      assign fpu_result = lane_result;
      assign fpu_result_valid = lane_result_valid;
      assign fpu_result_tag = lane_result_tag[0];
      assign lane_result_ready = result_ready;
      assign fpu_gather_busy = 1'b0;
    end

    for (genvar fpu = 0; unsigned'(fpu) < N_FPU; fpu++) begin : gen_fpnew
      logic int_fpu_result_valid;
      logic int_fpu_in_ready;
      vfu_tag_t tag;

      assign lane_in_ready[fpu*ELENB +: ELENB]     = {ELENB{int_fpu_in_ready}};
      assign lane_result_valid[fpu*ELENB +: ELENB] = {ELENB{int_fpu_result_valid}};
      assign lane_result_tag[fpu] = tag;

      elen_t fpu_operand1, fpu_operand2, fpu_operand3;

      assign fpu_operand1 = (fpu == 0 && divsqrt_shared_active)
        ? wide_operand1[divsqrt_slot_q*ELEN +: ELEN]: spatz_req.op_arith.switch_rs1_rd ? lane_operand3[fpu*ELEN +: ELEN] : lane_operand1[fpu*ELEN +: ELEN];
      assign fpu_operand2 = (fpu == 0 && divsqrt_shared_active)
        ? wide_operand2[divsqrt_slot_q*ELEN +: ELEN]:lane_operand2[fpu*ELEN +: ELEN];

      assign fpu_operand3 = (fpu_op == fpnew_pkg::ADD || spatz_req.op_arith.switch_rs1_rd) ? lane_operand1[fpu*ELEN +: ELEN] : lane_operand3[fpu*ELEN +: ELEN];

      logic int_fpu_in_valid;
      assign int_fpu_in_valid = spatz_req_valid && operands_ready && (!spatz_req.op_arith.is_scalar || fpu == 0) && is_fpu_insn;

      // Generate an FPU pipeline
      elen_t fpu_operand1_q, fpu_operand2_q, fpu_operand3_q;
      operation_e fpu_op_q;
      fp_format_e fpu_src_fmt_q, fpu_dst_fmt_q;
      int_format_e fpu_int_fmt_q;
      logic fpu_op_mode_q;
      logic fpu_vectorial_op_q;
      roundmode_e rm_q;
      vfu_tag_t input_tag_q;
      logic fpu_in_valid_q;
      logic fpu_in_ready_d;
      assign fpu_stage_valid[fpu] = fpu_in_valid_q;
      logic int_fpu_in_valid_gated;
      logic fpu_result_ready;

      assign int_fpu_in_valid_gated = int_fpu_in_valid
        && (fpu == 0 || !(divsqrt_shared_active))
        && !(fpu == 0 && divsqrt_shared_active && divsqrt_inflight_q);

      assign fpu_result_ready = (fpu == 0 && divsqrt_shared_active)? divsqrt_shared_ready : lane_result_ready;


      `FFL(fpu_operand1_q, fpu_operand1, int_fpu_in_valid && int_fpu_in_ready, '0)
      `FFL(fpu_operand2_q, fpu_operand2, int_fpu_in_valid && int_fpu_in_ready, '0)
      `FFL(fpu_operand3_q, fpu_operand3, int_fpu_in_valid && int_fpu_in_ready, '0)
      `FFL(fpu_op_q, fpu_op, int_fpu_in_valid && int_fpu_in_ready, fpnew_pkg::FMADD)
      `FFL(fpu_src_fmt_q, fpu_src_fmt, int_fpu_in_valid && int_fpu_in_ready, fpnew_pkg::FP32)
      `FFL(fpu_dst_fmt_q, fpu_dst_fmt, int_fpu_in_valid && int_fpu_in_ready, fpnew_pkg::FP32)
      `FFL(fpu_int_fmt_q, fpu_int_fmt, int_fpu_in_valid && int_fpu_in_ready, fpnew_pkg::INT8)
      `FFL(fpu_op_mode_q, fpu_op_mode, int_fpu_in_valid && int_fpu_in_ready, 1'b0)
      `FFL(fpu_vectorial_op_q, fpu_vectorial_op, int_fpu_in_valid && int_fpu_in_ready, 1'b0)
      `FFL(rm_q, (spatz_req.op == VFCMP && spatz_req.rm == fpnew_pkg::RUP) ? fpnew_pkg::RDN : spatz_req.rm, int_fpu_in_valid && int_fpu_in_ready, fpnew_pkg::RNE)
      `FFL(input_tag_q, input_tag, int_fpu_in_valid && int_fpu_in_ready, '{vsew: EW_8, default: '0})
      `FFL(fpu_in_valid_q, int_fpu_in_valid_gated, int_fpu_in_ready, 1'b0)


      assign int_fpu_in_ready = !fpu_in_valid_q || fpu_in_valid_q && fpu_in_ready_d;
      assign fpu_load_ready[fpu] = int_fpu_in_valid_gated && int_fpu_in_ready;

      localparam fpu_implementation_t FPUImpl = (FDivSqrt && (!divsqrt_is_shared || fpu == 0)) ? FPUImplementation : without_divsqrt(FPUImplementation);


      fpnew_top #(
        .Features                   (FPUFeatures           ),
        .Implementation             (FPUImpl               ),
        .TagType                    (vfu_tag_t             ),
        .StochasticRndImplementation(fpnew_pkg::DEFAULT_RSR)
      ) i_fpu (
        .clk_i         (clk_i                                                  ),
        .rst_ni        (rst_ni                                                 ),
        .hart_id_i     ((hart_id_i << $clog2(N_FPU)) | 32'(fpu)),
        .flush_i       (1'b0                                                   ),
        .busy_o        (fpu_busy_d[fpu]                                        ),
        .operands_i    ({fpu_operand3_q, fpu_operand2_q, fpu_operand1_q}       ),
        // Only the FPU0 executes scalar instructions
        .in_valid_i    (fpu_in_valid_q                                         ),
        .in_ready_o    (fpu_in_ready_d                                         ),
        .op_i          (fpu_op_q                                               ),
        .src_fmt_i     (fpu_src_fmt_q                                          ),
        .dst_fmt_i     (fpu_dst_fmt_q                                          ),
        .int_fmt_i     (fpu_int_fmt_q                                          ),
        .vectorial_op_i(fpu_vectorial_op_q                                     ),
        .op_mod_i      (fpu_op_mode_q                                          ),
        .tag_i         (input_tag_q                                            ),
        .simd_mask_i   ('1                                                     ),
        .rnd_mode_i    (rm_q                                                   ),
        .result_o      (lane_result[fpu*ELEN +: ELEN]                          ),
        .out_valid_o   (int_fpu_result_valid                                   ),
        .out_ready_i   (fpu_result_ready                                       ),
        .status_o      (fpu_status_d[fpu]                                      ),
        .tag_o         (tag                                                    )
      );

    end : gen_fpnew
  end: gen_fpu else begin: gen_no_fpu
    assign fpu_stage_valid = '0;
    assign fpu_gather_busy = 1'b0;
    assign is_fpu_busy      = 1'b0;
    assign fpu_in_ready     = '0;
    assign fpu_result       = '0;
    assign fpu_result_valid = '0;
    assign fpu_result_tag   = '0;
    assign fpu_status_o     = '0;
  end: gen_no_fpu

  if (DimcSectionWidth % VRFWordWidth != 0)
    $error("[spatz_vfu] DIMC section width must be an integer multiple of VRF word width.");

endmodule : spatz_vfu
