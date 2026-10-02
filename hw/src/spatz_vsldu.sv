// Copyright 2023 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Author: Matheus Cavalcante, ETH Zurich
//
// The vector slide unit executes all slide instructions and vcompress.vm

module spatz_vsldu
  import spatz_pkg::*;
  import rvv_pkg::*;
  import cf_math_pkg::idx_width; (
    input  logic             clk_i,
    input  logic             rst_ni,
    // Spatz request
    input  spatz_req_t       spatz_req_i,
    input  logic             spatz_req_valid_i,
    output logic             spatz_req_ready_o,
    // VSLDU response
    output logic             vsldu_rsp_valid_o,
    output vsldu_rsp_t       vsldu_rsp_o,
    // VRF interface
    output vrf_addr_t        vrf_waddr_o,
    output vrf_data_t        vrf_wdata_o,
    output logic             vrf_we_o,
    output vrf_be_t          vrf_wbe_o,
    input  logic             vrf_wvalid_i,
    output spatz_id_t  [1:0] vrf_id_o,
    output vrf_addr_t        vrf_raddr_o,
    output logic             vrf_re_o,
    input  vrf_data_t        vrf_rdata_i,
    input  logic             vrf_rvalid_i
  );

// Include FF
`include "common_cells/registers.svh"

  ///////////////////////
  //  Operation queue  //
  ///////////////////////

  spatz_req_t spatz_req_d;

  spatz_req_t spatz_req;
  logic       spatz_req_valid;
  logic       spatz_req_ready;

  spill_register #(
    .T(spatz_req_t)
  ) i_operation_queue (
    .clk_i  (clk_i                                          ),
    .rst_ni (rst_ni                                         ),
    .data_i (spatz_req_d                                    ),
    .valid_i(spatz_req_valid_i && spatz_req_i.ex_unit == SLD),
    .ready_o(spatz_req_ready_o                              ),
    .data_o (spatz_req                                      ),
    .valid_o(spatz_req_valid                                ),
    .ready_i(spatz_req_ready                                )
  );

// Is the current instruction a vcompress?
  logic is_compress;
  assign is_compress = spatz_req_valid && (spatz_req.op == VCOMPRESS);

  vrf_data_t rs1_masked;
  always_comb begin: rs1_proc
    rs1_masked = '0;
    unique case (spatz_req.vtype.vsew)
      EW_8 : rs1_masked = vrf_data_t'(spatz_req.rs1[7:0]);
      EW_16: rs1_masked = vrf_data_t'(spatz_req.rs1[15:0]);
      EW_32: rs1_masked = vrf_data_t'(spatz_req.rs1[31:0]);
      default: rs1_masked = vrf_data_t'(spatz_req.rs1);  // EW_64
    endcase
  end

  // Convert the vl to number of bytes for all element widths
  always_comb begin: proc_spatz_req
    spatz_req_d = spatz_req_i;

    unique case (spatz_req_i.vtype.vsew)
      EW_8: begin
        spatz_req_d.vl     = spatz_req_i.vl;
        spatz_req_d.vstart = spatz_req_i.vstart;
        if (spatz_req_i.op_sld.vmv && spatz_req_i.op_sld.insert)
          spatz_req_d.rs1 = MAXEW == EW_32 ? {4{spatz_req_i.rs1[7:0]}} : {8{spatz_req_i.rs1[7:0]}};
      end
      EW_16: begin
        spatz_req_d.vl     = spatz_req_i.vl << 1;
        spatz_req_d.vstart = spatz_req_i.vstart << 1;
        if (spatz_req_i.op_sld.vmv && spatz_req_i.op_sld.insert)
          spatz_req_d.rs1 = MAXEW == EW_32 ? {2{spatz_req_i.rs1[15:0]}} : {4{spatz_req_i.rs1[15:0]}};
      end
      EW_32: begin
        spatz_req_d.vl     = spatz_req_i.vl << 2;
        spatz_req_d.vstart = spatz_req_i.vstart << 2;
        if (spatz_req_i.op_sld.vmv && spatz_req_i.op_sld.insert)
          spatz_req_d.rs1 = MAXEW == EW_32 ? {1{spatz_req_i.rs1[31:0]}} : {2{spatz_req_i.rs1[31:0]}};
      end
      default: begin
        spatz_req_d.vl     = spatz_req_i.vl << MAXEW;
        spatz_req_d.vstart = spatz_req_i.vstart << MAXEW;
        if (spatz_req_i.op_sld.vmv && spatz_req_i.op_sld.insert)
          spatz_req_d.rs1 = spatz_req_i.rs1;
      end
    endcase
  end: proc_spatz_req

  ///////////////////////
  //  Output Register  //
  ///////////////////////

  typedef struct packed {
    vrf_addr_t waddr;
    vrf_data_t wdata;
    vrf_be_t wbe;
  } vrf_req_t;

  vrf_req_t vrf_req_d, vrf_req_q;
  logic     vrf_req_valid_d, vrf_req_ready_d;
  logic     vrf_req_valid_q, vrf_req_ready_q;

  spill_register #(
    .T(vrf_req_t)
  ) i_vrf_req_register (
    .clk_i  (clk_i          ),
    .rst_ni (rst_ni         ),
    .data_i (vrf_req_d      ),
    .valid_i(vrf_req_valid_d),
    .ready_o(vrf_req_ready_d),
    .data_o (vrf_req_q      ),
    .valid_o(vrf_req_valid_q),
    .ready_i(vrf_req_ready_q)
  );

  assign vrf_waddr_o     = vrf_req_q.waddr;
  assign vrf_wdata_o     = vrf_req_q.wdata;
  assign vrf_wbe_o       = vrf_req_q.wbe;
  assign vrf_we_o        = vrf_req_valid_q;
  assign vrf_id_o[0]     = spatz_req.id; // ID of the instruction currently reading elements
  assign vrf_req_ready_q = vrf_wvalid_i;

  /////////////
  // Signals //
  /////////////

  // Is the register file operation valid? (slide path)
  logic vreg_operations_finished;
  // Compress path finished
  logic cmp_finished;
  // Either path finished -- drives the shared completion logic
  logic ops_finished;
  assign ops_finished = is_compress ? cmp_finished : vreg_operations_finished;

  // Vector register file counter signals (byte index of the current VRF word,
  // used by both the slide and the compress paths)
  logic  vreg_counter_en;
  vlen_t vreg_counter_delta;
  vlen_t vreg_counter_d;
  vlen_t vreg_counter_q;
  `FF(vreg_counter_q, vreg_counter_d, '0)

  // Is the vector length zero (no active instruction)
  logic is_vl_zero;
  assign is_vl_zero = spatz_req.vl == 'd0;

  // Is the instruction slide up
  logic is_slide_up;
  assign is_slide_up = spatz_req.op == VSLIDEUP;

  // Instruction currently committing results
  spatz_id_t op_id_q, op_id_d;
  `FF(op_id_q, op_id_d, '0)

  // Number of bytes we slide up or down
  vlen_t slide_amount_q, slide_amount_d;
  `FF(slide_amount_q, slide_amount_d, '0)

  // Are we doing a vregfile read prefetch (when we slide down)
  logic prefetch_q, prefetch_d;
  `FF(prefetch_q, prefetch_d, 1'b0);

  ////////////////////////
  // Compressor signals //
  ////////////////////////

  // Byte-enable of the valid (body) bytes of the current source word
  logic [VRFWordBWidth-1:0] cmp_tail_be;

  // Destination bytes written by the previous source words. After the last source word
  // it holds the total number of destination bytes, used by the final flush
  vlen_t cmp_base_q, cmp_base_d;
  `FF(cmp_base_q, cmp_base_d, '0)

  // Byte-enable mask of the current source word
  logic [VRFWordBWidth-1:0] cmp_chunk_be;

  // Is the current source word the last one?
  logic  cmp_last_word;
  logic  cmp_last_word_q, cmp_last_word_d;
  `FF(cmp_last_word_q, cmp_last_word_d, 1'b0)


  // Per-word derived quantities
  vlen_t                            cmp_chunk_bytes;
  vlen_t                            cmp_end_byte;
  logic [$clog2(VRFWordBWidth)-1:0] cmp_offset;
  logic                             cmp_emit_word;

  // the compacted word is aligned by the slider shifter and the
  // partial destination word is accumulated in the slider overflow register
  vrf_data_t compact_data;

  // Control
  logic      cmp_need_flush, cmp_stall, cmp_advance;
  logic      cmp_re, cmp_req_valid;
  vrf_addr_t cmp_raddr, cmp_waddr;
  vrf_be_t   cmp_wbe;

  ///////////////////
  // State Handler //
  ///////////////////

  // Currently running instructions
  logic [NrParallelInstructions-1:0] running_d, running_q;
  `FF(running_q, running_d, '0)

  // Respond to controller if we are finished executing
  typedef enum logic {
    VSLDU_RUNNING,    // Running an instruction
    VSLDU_WAIT_WVALID // Waiting for the last wvalid to acknowledge the instruction
   } state_t;
   state_t state_q, state_d;
  `FF(state_q, state_d, VSLDU_RUNNING)

  // New instruction
  // Initialize the internal state one cycle in advance
  logic new_vsldu_request, new_vsldu_request_q;
  assign new_vsldu_request = spatz_req_valid && !running_q[spatz_req.id];

  `FF(new_vsldu_request_q, new_vsldu_request, '0)

  logic new_compress_request;
  assign new_compress_request = new_vsldu_request && is_compress;

  typedef enum logic[1:0] {
    VREG_READ_V0_t, // Read the needed VRF words of v0.t
    VREG_IDLE,
    VREG_WAIT_FIRST_WRITE
  } vreg_operation_first_t;
  vreg_operation_first_t vreg_operation_first_q, vreg_operation_first_d;
  `FF(vreg_operation_first_q, vreg_operation_first_d, VREG_IDLE)

  typedef enum logic [2:0] {
    CMP_IDLE,
    CMP_READ_MASK,
    CMP_RUN,
    CMP_FLUSH
  } cmp_state_e;
  cmp_state_e cmp_state_q, cmp_state_d;
  `FF(cmp_state_q, cmp_state_d, CMP_IDLE)

  // Accept a new operation or clear req register if we are finished
  always_comb begin : vsldu_new_request_proc
    slide_amount_d = slide_amount_q;
    prefetch_d     = prefetch_q;
    running_d      = running_q;

    // Spatz SLDU ready when empty
    spatz_req_ready = !spatz_req_valid;

    // New request?
    if (new_vsldu_request) begin
      // Mark the instruction as running
      running_d[spatz_req.id] = 1'b1;

      if (spatz_req.op == VCOMPRESS) begin
        // vcompress has no scalar slide amount and needs no prefetch
        slide_amount_d = '0;
        prefetch_d     = 1'b0;
      end else begin
        slide_amount_d = spatz_req.op_sld.insert ? (spatz_req.op_sld.vmv ? 'd0 : 'd1) : spatz_req.rs1;
        slide_amount_d <<= spatz_req.vtype.vsew;

        prefetch_d = spatz_req.op == VSLIDEUP ? spatz_req.vstart >= VRFWordBWidth : 1'b1;
      end
    end

    // Finished an instruction
    if (ops_finished) begin
      // We are handling an instruction
      spatz_req_ready = 1'b1;

      // No longer running this instruction
      running_d[spatz_req.id] = 1'b0;
    end

    // Clear the prefetch register
    if (prefetch_q && vrf_re_o && vrf_rvalid_i && vreg_operation_first_q != VREG_READ_V0_t)
      prefetch_d = 1'b0;
  end : vsldu_new_request_proc

  ////////////////////
  //  Mask operand  //
  ////////////////////

  // Only the words holding the bits of the first vl elements are read.
  localparam int unsigned MaskWordIdxWidth = (NrWordsPerVector > 1) ? $clog2(NrWordsPerVector) : 1;
  typedef logic [MaskWordIdxWidth-1:0] mask_word_idx_t;

  // Mask word being read, and the last one needed by the current instruction
  mask_word_idx_t mask_word_q, mask_word_d, mask_last_word;
  `FF(mask_word_q, mask_word_d, '0)

  logic mask_word_ready; // A mask word is arriving from the VRF
  logic mask_read_last;  // It is the last needed one

  vlen_t vl_elem_last;
  assign vl_elem_last = (spatz_req.vl >> spatz_req.vtype.vsew) - vlen_t'(1);

  logic v0_t_is_ready, cmp_mask_is_ready;

  // Mask operand register
  logic [VLEN-1:0] operand_mask_q, operand_mask_d;
  `FF(operand_mask_q, operand_mask_d, '0)

  always_comb begin : mask_last_word_proc
    mask_last_word = mask_word_idx_t'(NrWordsPerVector-1);
    if ((vl_elem_last >> $clog2(VRFWordWidth)) < vlen_t'(NrWordsPerVector))
      mask_last_word = mask_word_idx_t'(vl_elem_last >> $clog2(VRFWordWidth));
  end : mask_last_word_proc

  assign v0_t_is_ready     = (vreg_operation_first_q == VREG_READ_V0_t) && vrf_rvalid_i;
  assign cmp_mask_is_ready = (cmp_state_q == CMP_READ_MASK) && vrf_rvalid_i;
  assign mask_word_ready   = v0_t_is_ready || cmp_mask_is_ready;
  assign mask_read_last    = (mask_word_q == mask_last_word);

  always_comb begin : mask_word_counter_proc
    mask_word_d = mask_word_q;
    if (new_vsldu_request)
      mask_word_d = '0;
    else if (mask_word_ready)
      mask_word_d = mask_read_last ? '0 : mask_word_q + mask_word_idx_t'(1);
  end : mask_word_counter_proc

  logic trig_v0_t_read_done;
  assign trig_v0_t_read_done = v0_t_is_ready && mask_read_last;

  logic v0_t_read_done;
  `FFLARNC(v0_t_read_done, 1'b1, trig_v0_t_read_done, ops_finished, 1'b0, clk_i, rst_ni);

  always_comb begin : mask_reg_proc
    operand_mask_d = operand_mask_q;

    if (mask_word_ready) begin
      for (int w = 0; w < NrWordsPerVector; w++)
        if (mask_word_q == mask_word_idx_t'(w))
          operand_mask_d[w*VRFWordWidth +: VRFWordWidth] = vrf_rdata_i;
    end else if (cmp_advance)
      unique case (spatz_req.vtype.vsew)
        EW_8   : operand_mask_d = operand_mask_q >> (VRFWordBWidth);
        EW_16  : operand_mask_d = operand_mask_q >> (VRFWordBWidth/2);
        EW_32  : operand_mask_d = operand_mask_q >> (VRFWordBWidth/4);
        default: operand_mask_d = operand_mask_q >> (VRFWordBWidth/8);
      endcase
  end : mask_reg_proc

  //////////////////////
  // Compress control //
  //////////////////////

  // Only the last source word can be partially filled
  always_comb begin : cmp_tail_be_proc
    cmp_tail_be = '1;
    if (cmp_last_word && (spatz_req.vl[$clog2(VRFWordBWidth)-1:0] != '0))
      for (int b = 0; b < VRFWordBWidth; b++)
        cmp_tail_be[b] = ($clog2(VRFWordBWidth))'(b) < spatz_req.vl[$clog2(VRFWordBWidth)-1:0];
  end

  always_comb begin : cmp_last_word_proc
    cmp_last_word_d = cmp_last_word_q;

    if (new_compress_request)
      cmp_last_word_d = spatz_req.vl <= vlen_t'(VRFWordBWidth);
    else if (cmp_advance)
      cmp_last_word_d = (vreg_counter_q + vlen_t'(2*VRFWordBWidth)) >= spatz_req.vl;
  end : cmp_last_word_proc

  assign cmp_last_word = cmp_last_word_q;

  assign cmp_end_byte   = cmp_base_q + cmp_chunk_bytes;
  assign cmp_offset     = cmp_base_q[$clog2(VRFWordBWidth)-1:0];
  assign cmp_emit_word  = (cmp_end_byte >> $clog2(VRFWordBWidth)) != (cmp_base_q >> $clog2(VRFWordBWidth));

  // flush when last word is emitted and there are remaining bytes, so when the 2 more writes are needed
  assign cmp_need_flush = cmp_emit_word && (cmp_end_byte[$clog2(VRFWordBWidth)-1:0] != '0);

  // A write that the output register cannot take freezes the read counter, the prefix sum and the accumulator
  assign cmp_stall   = (cmp_state_q == CMP_RUN) && vrf_rvalid_i && (cmp_emit_word || cmp_last_word) && !vrf_req_ready_d;
  assign cmp_advance = (cmp_state_q == CMP_RUN) && vrf_rvalid_i && !cmp_stall;

  assign cmp_finished = (cmp_advance && cmp_last_word && !cmp_need_flush) || ((cmp_state_q == CMP_FLUSH) && vrf_req_ready_d);

  always_comb begin : cmp_fsm_state_evolution
    cmp_state_d = cmp_state_q;

    unique case (cmp_state_q)
      CMP_IDLE: begin
        if (new_compress_request && !is_vl_zero)
          cmp_state_d = CMP_READ_MASK;
      end

      CMP_READ_MASK: begin
        if (cmp_mask_is_ready && mask_read_last)
          cmp_state_d = CMP_RUN;
      end

      CMP_RUN: begin
        if (cmp_advance && cmp_last_word)
          cmp_state_d = cmp_need_flush ? CMP_FLUSH : CMP_IDLE;
      end

      CMP_FLUSH: begin
        if (vrf_req_ready_d)
          cmp_state_d = CMP_IDLE;
      end

      default: cmp_state_d = CMP_IDLE;
    endcase
  end : cmp_fsm_state_evolution

  /////////////////////
  //  Slide control  //
  /////////////////////

  // Count how many VRFWords we have already committed
  logic [$bits(vlen_t)-$clog2(VRFWordBWidth):0] vreg_counter_mod_wordBwidth;
  assign vreg_counter_mod_wordBwidth = vreg_counter_q >> $clog2(VRFWordBWidth);

  // Are we on the first/last VRF operation?
  logic vreg_operation_first;
  logic vreg_operation_last;

  // Generate masking based on v0.t
  logic [VLEN-1:0] vm_masking;

  always_comb begin : vsldu_vm_masking_proc
    vm_masking = '1;
    if(!spatz_req.op_sld.vm) begin
      case (spatz_req.vtype.vsew)
        // i < (VLEN/vsew)*8 where 8 --> max lmul
        EW_8:for(int i=0;i<VLEN;i=i+1)begin
          vm_masking[i*1+:1] = {1{operand_mask_q[i]}};
        end
        EW_16:for(int i=0;i<(VLEN/2);i=i+1)begin
          vm_masking[i*2+:2] = {2{operand_mask_q[i]}};
        end
        EW_32: for(int i=0;i<(VLEN/4);i=i+1)begin
          vm_masking[i*4+:4] = {4{operand_mask_q[i]}};
        end
        default: if (MAXEW == EW_64) for(int i=0;i<(VLEN/8);i=i+1)begin
          vm_masking[i*8+:8] = {8{operand_mask_q[i]}};
        end
      endcase
    end
  end : vsldu_vm_masking_proc

  always_comb begin: vsldu_vreg_counter_proc
    // How many elements are left to do
    automatic int unsigned delta = spatz_req.vl - vreg_counter_q;

    // Default assignments
    vreg_counter_en        = 1'b0;
    vreg_counter_d         = vreg_counter_q;
    vreg_counter_delta     = '0;
    vreg_operation_first_d = vreg_operation_first_q;

    // Do we have a new request?
    if (new_vsldu_request) begin
      // Load vstart into the counter
      vreg_counter_d = spatz_req.vstart;
      if (!spatz_req.op_sld.insert && spatz_req.vstart < slide_amount_d && is_slide_up)
        vreg_counter_d = slide_amount_d;
    end

    // Is this the first/last operation?
    case (vreg_operation_first_q)
      VREG_IDLE: begin
        // Wait until our first write operation
        vreg_operation_first = spatz_req_valid && !prefetch_q && new_vsldu_request_q;

        if(spatz_req_valid && !spatz_req.op_sld.vm && !v0_t_read_done)
          vreg_operation_first_d = VREG_READ_V0_t;
        else begin
          if (spatz_req_valid && vreg_counter_q <= slide_amount_q)
            vreg_operation_first_d = VREG_WAIT_FIRST_WRITE;

          if (vrf_req_valid_d && vrf_req_ready_d)
            vreg_operation_first_d = VREG_IDLE;
        end
      end

      VREG_READ_V0_t: begin
        vreg_operation_first = 0;
        if (v0_t_is_ready && mask_read_last)
          vreg_operation_first_d = VREG_WAIT_FIRST_WRITE;
      end

      VREG_WAIT_FIRST_WRITE: begin
        vreg_operation_first = spatz_req_valid && !prefetch_q;
        if (vrf_req_valid_d && vrf_req_ready_d)
          vreg_operation_first_d = VREG_IDLE;
      end
      default:;
    endcase

    // vcompress does not use this FSM, so it is frozen in the idle state
    if (is_compress) begin
      vreg_operation_first   = 1'b0;
      vreg_operation_first_d = VREG_IDLE;
    end

    vreg_operation_last = spatz_req_valid && !prefetch_q && (delta <= (VRFWordBWidth - vreg_counter_q[idx_width(VRFWordBWidth)-1:0]));

    // How many operations are we calculating now?
    if (spatz_req_valid) begin
      if (vreg_operation_last)
        vreg_counter_delta = delta;
      else if (vreg_operation_first)
        vreg_counter_delta = VRFWordBWidth - vreg_counter_d[idx_width(VRFWordBWidth)-1:0];
      else
        vreg_counter_delta = VRFWordBWidth;
    end

    // Do we have to increment the counter?
    vreg_counter_en = !is_compress && (vreg_operation_first_q != VREG_READ_V0_t) && ((spatz_req.use_vs2 && vrf_re_o && vrf_rvalid_i) || !spatz_req.use_vs2) && ((spatz_req.use_vd && vrf_req_valid_d && vrf_req_ready_d) || !spatz_req.use_vd);
    if (vreg_counter_en) begin
      if (vreg_operation_last)
        // Reset the counter
        vreg_counter_d = '0;
      else
        // Increment the counter
        vreg_counter_d = vreg_counter_q + vreg_counter_delta;
    end

    // vcompress reads one full source word per advance
    if (is_compress) begin
      if (new_vsldu_request)
        vreg_counter_d = '0;
      else if (cmp_advance)
        vreg_counter_d = cmp_last_word ? '0 : vreg_counter_q + vlen_t'(VRFWordBWidth);
    end

    // Did we finish?
    vreg_operations_finished = vreg_operation_last && vreg_counter_en;
  end: vsldu_vreg_counter_proc

  always_comb begin: vsldu_rsp
    // Maintain state
    state_d = state_q;
    op_id_d = op_id_q;

    // Do not acknowledge anything
    vsldu_rsp_valid_o = 1'b0;
    vsldu_rsp_o       = '0;

    // ID of the instruction currently writing elements
    vrf_id_o[1] = spatz_req.id;

    case (state_q)
      VSLDU_RUNNING: begin
        // Did we finish the execution of an instruction?
        if (!is_vl_zero && ops_finished && spatz_req_valid) begin
          op_id_d = spatz_req.id;
          state_d = VSLDU_WAIT_WVALID;
        end
      end

      VSLDU_WAIT_WVALID: begin
        vrf_id_o[1] = op_id_q; // ID of the instruction currently writing to the VRF

        if (vrf_wvalid_i) begin
          vsldu_rsp_valid_o = 1'b1;
          vsldu_rsp_o.id    = op_id_q;
          state_d           = VSLDU_RUNNING;

          // Did we finish *another* instruction?
          if (!is_vl_zero && ops_finished && spatz_req_valid) begin
            op_id_d = spatz_req.id;
            state_d = VSLDU_WAIT_WVALID;
          end
        end
      end // case: VSLDU_WAIT_WVALID

      default:;
    endcase
  end: vsldu_rsp

  ///////////////////////
  // Compress datapath //
  ///////////////////////

  always_comb begin : cmp_counters_proc
    cmp_base_d = cmp_base_q;

    if (new_compress_request)
      cmp_base_d = '0;
    else if (cmp_advance)
      cmp_base_d = cmp_end_byte;
  end : cmp_counters_proc

  always_comb begin : cmp_chunk_be_proc
    cmp_chunk_be = '0;
    unique case (spatz_req.vtype.vsew)
      EW_8:    for (int b = 0; b < VRFWordBWidth; b++) cmp_chunk_be[b] = operand_mask_q[b];
      EW_16:   for (int b = 0; b < VRFWordBWidth; b++) cmp_chunk_be[b] = operand_mask_q[b/2];
      EW_32:   for (int b = 0; b < VRFWordBWidth; b++) cmp_chunk_be[b] = operand_mask_q[b/4];
      default: for (int b = 0; b < VRFWordBWidth; b++) cmp_chunk_be[b] = operand_mask_q[b/8];
    endcase
    cmp_chunk_be &= cmp_tail_be;
  end : cmp_chunk_be_proc

  // Compaction network: every selected byte moves down by the number of discarded
  // bytes before it (its shift amount). The move is split in log2(VRFWordBWidth)
  // stages: stage k moves a byte down by 2^k if bit k of its shift amount is set.
  // It is a synthesis optimized algorithm for performing the compaction of a word with
  // variable numember of selected bytes
  localparam int unsigned CmpNetStages = $clog2(VRFWordBWidth);

  typedef struct packed {
    logic [CmpNetStages-1:0] shamt; // how far the byte has to move down
    logic [7:0] data; // the byte value, or zero if the position is empty
  } cmp_lane_t;

  // [stage][byte position]: stage 0 is the input, stage CmpNetStages the output.
  // An empty position is all zeros: with a zero shift amount it never moves.
  cmp_lane_t [CmpNetStages:0][VRFWordBWidth-1:0] cmp_net;

  always_comb begin : cmp_compaction_proc
    // Discarded bytes seen so far. One bit wider than the shift amounts,
    // as the whole word can be discarded
    automatic logic [CmpNetStages:0] discarded;
    discarded = '0;

    // Input: a selected byte gets the number of discarded bytes before it as shift
    // amount, a discarded byte becomes an empty position
    for (int i = 0; i < VRFWordBWidth; i++) begin
      cmp_net[0][i] = '0;
      if (cmp_chunk_be[i]) begin
        cmp_net[0][i].shamt = discarded[CmpNetStages-1:0];
        cmp_net[0][i].data  = vrf_rdata_i[i*8 +: 8];
      end else begin
        discarded = discarded + 1'b1;
      end
    end

    // Selected bytes of the word = destination bytes produced by this source word
    cmp_chunk_bytes = vlen_t'(VRFWordBWidth) - vlen_t'(discarded);

    // At the stage k, the only byte that can be moved to position p is the only one that is located
    // 2^k positions above it, so p+(1<<k), because at this stage bytes move down by 2^k or they remain in place 
    for (int k = 0; k < CmpNetStages; k++)
      for (int p = 0; p < VRFWordBWidth; p++) begin
        // if the byte 2^k position above exists in the word and its related shamt is 1
        // it moves down to this position, otherwise the byte here stays or the position is empty
        if ((p + (1 << k) < VRFWordBWidth) && cmp_net[k][p + (1 << k)].shamt[k])
          cmp_net[k+1][p] = cmp_net[k][p + (1 << k)]; // the byte 2^k above is moved down
        else if (!cmp_net[k][p].shamt[k])
          cmp_net[k+1][p] = cmp_net[k][p]; // the byte doen't move
        else
          cmp_net[k+1][p] = '0;
      end

    for (int b = 0; b < VRFWordBWidth; b++)
      compact_data[b*8 +: 8] = cmp_net[CmpNetStages][b].data;
  end : cmp_compaction_proc

  // Partial destination words (wbe != '1) are written by the flush, or directly by a last
  // source word that does not emit
  always_comb begin : cmp_wbe_proc
    cmp_wbe = '1;
    for (int b = 0; b < VRFWordBWidth; b++)
      if (cmp_state_q == CMP_FLUSH)
        cmp_wbe[b] = $clog2(VRFWordBWidth)'(b) < cmp_base_q[$clog2(VRFWordBWidth)-1:0];
      else if (cmp_last_word && !cmp_emit_word)
        cmp_wbe[b] = $clog2(VRFWordBWidth)'(b) < cmp_end_byte[$clog2(VRFWordBWidth)-1:0];
  end : cmp_wbe_proc

  assign cmp_req_valid = ((cmp_state_q == CMP_RUN) && vrf_rvalid_i && (cmp_emit_word || cmp_last_word)) ||
                         (cmp_state_q == CMP_FLUSH);

  assign cmp_re = (cmp_state_q == CMP_READ_MASK) || (cmp_state_q == CMP_RUN);

  ////////////
  // Slider //
  ////////////

  // Shift overflow register
  vrf_data_t shift_overflow_q, shift_overflow_d;
  `FF(shift_overflow_q, shift_overflow_d, '0)

  // Number of bytes we have to shift the elements around
  // inside the register element
  logic [$clog2(VRFWordBWidth)-1:0] in_elem_offset, in_elem_flipped_offset;
  // vcompress shifts the compacted word up by the destination byte offset
  assign in_elem_offset         = is_compress ? cmp_offset : slide_amount_d[$clog2(VRFWordBWidth)-1:0];
  assign in_elem_flipped_offset = VRFWordBWidth - in_elem_offset;

  // Data signals for different stages of the shift
  vrf_data_t data_in, data_out, data_low, data_high;
  vrf_be_t slide_wbe; // Used for monitor wbe signals before vm_masking

  // Slide-path outputs, muxed with the compress path further down (wdata is shared)
  vrf_req_t  sld_req;
  logic      sld_req_valid;
  logic      sld_re;
  vrf_addr_t sld_raddr;

  logic [$bits(vlen_t):0] ins_pos;
  assign ins_pos = (($bits(vlen_t)+1)'(vreg_counter_q[$clog2(VRFWordBWidth)-1:0]) +
                    ($bits(vlen_t)+1)'(vreg_counter_delta) -
                    ($bits(vlen_t)+1)'(4'b0001<<spatz_req.vtype.vsew));

  always_comb begin : vsldu_slider_proc
    shift_overflow_d = shift_overflow_q;

    data_in   = '0;
    data_out  = '0;
    data_high = '0;
    data_low  = '0;

    sld_req.wbe   = '0;
    sld_req.wdata = '0;

    slide_wbe = '0;

    // Is there a vector instruction executing now?
    if (!is_vl_zero) begin
        if (is_compress) begin
          // vcompress uses the slide up datapath: flip the compacted bytes around (d[-i] = d[i])
          if (cmp_state_q == CMP_RUN)
            for (int b_src = 0; b_src < VRFWordBWidth; b_src++)
              data_in[(VRFWordBWidth-b_src-1)*8 +: 8] = compact_data[b_src*8 +: 8];
        end
        else if (is_slide_up && spatz_req.op_sld.insert && spatz_req.op_sld.vmv) begin
          for (int b_src = 0; b_src < VRFWordBWidth; b_src++)
            data_in[(VRFWordBWidth-b_src-1)*8 +: 8] = spatz_req.rs1[b_src*8%ELEN +: 8];
        end
        else if (is_slide_up) begin
          // If we have a slide up operation, flip all bytes around (d[-i] = d[i])
          for (int b_src = 0; b_src < VRFWordBWidth; b_src++)
            data_in[(VRFWordBWidth-b_src-1)*8 +: 8] = (vreg_operation_first_q == VREG_READ_V0_t)? data_in[(VRFWordBWidth-b_src-1)*8 +: 8] : vrf_rdata_i[b_src*8 +: 8];
        end else begin
          data_in = (vreg_operation_first_q == VREG_READ_V0_t)? data_in : vrf_rdata_i;

        // If we are already over the MAXVL, all continuing elements are zero
        if ((vreg_counter_q >= MAXVL - slide_amount_q) || (vreg_operation_last && spatz_req.op_sld.insert))
          data_in = '0;
      end

      // Shift direct elements into the correct position
      for (int b_src = 0; b_src < VRFWordBWidth; b_src++)
        if (b_src >= in_elem_offset) begin
          // high elements
          for (int b_dst = 0; b_dst <= b_src; b_dst++)
            if (b_src-b_dst == in_elem_offset)
              data_high[b_dst*8 +: 8] = data_in[b_src*8 +: 8];
        end else begin
          // low elements
          for (int b_dst = b_src; b_dst < VRFWordBWidth; b_dst++)
            if (b_dst-b_src == in_elem_flipped_offset)
              data_low[b_dst*8 +: 8] = data_in[b_src*8 +: 8];
        end

      // Combine overflow and direct elements together
      if (is_compress) begin
        // The overflow register holds the partial destination word: the bytes spilling
        // into the next word replace it on emit, otherwise the new bytes are merged in
        if (cmp_advance)
          shift_overflow_d = cmp_emit_word ? data_low : (shift_overflow_q | data_high);
        data_out = data_high | shift_overflow_q;
      end else if (is_slide_up) begin
        if (vreg_counter_en || prefetch_q)
          shift_overflow_d = data_low;
        data_out = data_high | shift_overflow_q;
      end else begin
        if (vreg_counter_en || prefetch_q)
          shift_overflow_d = data_high;
        data_out = data_low | shift_overflow_q;

        // Insert rs1 element at the last position
        if (spatz_req.op_sld.insert && vreg_operation_last) begin
          for (int b = 0; b < VRFWordBWidth; b++)
            if (($bits(vlen_t)+1)'(b) >= ins_pos)
              data_out[b*8 +: 8] = data_low[b*8 +: 8];
          data_out = data_out | (vrf_data_t'(rs1_masked) << {ins_pos, 3'b000});
        end
      end

      // If we have a slide up operation, flip all bytes back around (d[i] = d[-i])
      if (is_slide_up || is_compress) begin
        for (int b_src = 0; b_src < VRFWordBWidth; b_src++)
          sld_req.wdata[(VRFWordBWidth-b_src-1)*8 +: 8] = data_out[b_src*8 +: 8];

        // Insert rs1 element at the first position
        if (spatz_req.op_sld.insert && !spatz_req.op_sld.vmv && vreg_operation_first && spatz_req.vstart == 'd0)
          // fill the LSB with rs1_masked
          sld_req.wdata = sld_req.wdata | vrf_data_t'(rs1_masked);
      end else begin
        sld_req.wdata = data_out;
      end

      // Create byte enable mask
      for (int i = 0; i < VRFWordBWidth; i++)
        slide_wbe[i] = i < vreg_counter_delta;

      // Special byte enable mask case when we are operating on the first register element.
      if (vreg_operation_first && is_slide_up)
        for (int i = 0; i < VRFWordBWidth; i++)
          slide_wbe[i] = (spatz_req.op_sld.insert || (i >= slide_amount_d[$clog2(VRFWordBWidth)-1:0])) & (i < (vreg_counter_q[$clog2(VRFWordBWidth)-1:0] + vreg_counter_delta));
    end

    // Reset overflow register when finished
    if (vreg_operations_finished || new_compress_request || cmp_finished)
      shift_overflow_d = '0;

    sld_req.wbe = slide_wbe & vm_masking[vreg_counter_mod_wordBwidth*VRFWordBWidth +:VRFWordBWidth];
  end : vsldu_slider_proc

  // VRF signals
  assign sld_re        = (vreg_operation_first_q == VREG_READ_V0_t)||(spatz_req.use_vs2 && (spatz_req_valid || prefetch_q) && running_q[spatz_req.id]);
  assign sld_req_valid = (vreg_operation_first_q != VREG_READ_V0_t)&& spatz_req_valid && spatz_req.use_vd && (vrf_re_o || !spatz_req.use_vs2) && (vrf_rvalid_i || !spatz_req.use_vs2) && !prefetch_q;

  ////////////////////////
  // Address Generation //
  ////////////////////////

  vlen_t sld_offset_rd;
  localparam int zero_fill_idx = (NrWordsPerVector > 1) ? $clog2(NrWordsPerVector) : 0;
  vrf_addr_t base_raddr, base_waddr, base_vs1_raddr;

  always_comb begin: addr_gen_proc
    base_raddr = '0;
    base_waddr = '0;
    base_vs1_raddr = '0;

    base_raddr[$bits(vrf_addr_t)-1:zero_fill_idx] = spatz_req.vs2;
    base_waddr[$bits(vrf_addr_t)-1:zero_fill_idx] = spatz_req.vd;
    //vcompress: mask is in vs1
    base_vs1_raddr[$bits(vrf_addr_t)-1:zero_fill_idx] = spatz_req.vs1;

    sld_offset_rd   = is_slide_up ? (prefetch_q ? -slide_amount_q[$bits(vlen_t)-1:$clog2(VRFWordBWidth)] - 1 : -slide_amount_q[$bits(vlen_t)-1:$clog2(VRFWordBWidth)]) : prefetch_q ? slide_amount_q[$bits(vlen_t)-1:$clog2(VRFWordBWidth)] : slide_amount_q[$bits(vlen_t)-1:$clog2(VRFWordBWidth)] + 1;
    sld_raddr       = (vreg_operation_first_q == VREG_READ_V0_t) ? vrf_addr_t'(mask_word_q) : base_raddr + vreg_counter_q[$bits(vlen_t)-1:$clog2(VRFWordBWidth)] + sld_offset_rd;
    sld_req.waddr   = base_waddr + vreg_counter_q[$bits(vlen_t)-1:$clog2(VRFWordBWidth)];

    cmp_waddr     = base_waddr + vrf_addr_t'(cmp_base_q >> $clog2(VRFWordBWidth));

    unique case (cmp_state_q)
      CMP_READ_MASK: cmp_raddr = base_vs1_raddr + vrf_addr_t'(mask_word_q);
      default:       cmp_raddr = base_raddr + vreg_counter_q[$bits(vlen_t)-1:$clog2(VRFWordBWidth)];
    endcase
  end: addr_gen_proc

  /////////////////
  //  Output mux //
  /////////////////

  assign vrf_re_o        = is_compress ? cmp_re        : sld_re;
  assign vrf_raddr_o     = is_compress ? cmp_raddr     : sld_raddr;
  assign vrf_req_d.wdata = sld_req.wdata;
  assign vrf_req_d.waddr = is_compress ? cmp_waddr     : sld_req.waddr;
  assign vrf_req_d.wbe   = is_compress ? cmp_wbe       : sld_req.wbe;
  assign vrf_req_valid_d = is_compress ? cmp_req_valid : sld_req_valid;

endmodule : spatz_vsldu
