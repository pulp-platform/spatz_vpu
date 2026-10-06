// Copyright 2026 University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Synthesizable DIMC macro model used by the Spatz VFU custom sf.vqmmacc path.

module DIMC #(
  parameter int unsigned SECTION_WIDTH = 512,
  parameter int unsigned NUM_SECTIONS  = 2
) (
  input  logic                         RCK,
  input  logic                         RESETn,

  output logic                         READYN,
  input  logic                         COMPE,
  input  logic                         FCSN,
  input  logic [1:0]                   MODE,

  input  logic [1:0]                   FA,
  input  logic [SECTION_WIDTH-1:0]     FD,
  input  logic [23:0]                  ADDIN,
  output logic                         SOUT,
  output logic [2:0]                   RES_OUT,
  output logic [23:0]                  PSOUT,
  output logic [SECTION_WIDTH-1:0]     Q,
  input  logic [SECTION_WIDTH-1:0]     D,
  input  logic [6:0]                   RA,
  input  logic [6:0]                   WA,

  input  logic                         RCSN,
  input  logic                         RCSN0,
  input  logic                         RCSN1,
  input  logic                         RCSN2,
  input  logic                         RCSN3,
  input  logic                         WCK,
  input  logic                         WCSN,
  input  logic                         WEN,

  input  logic [SECTION_WIDTH-1:0]     M,
  input  logic [7:0]                   MCT
);

  localparam int unsigned RowWidth    = SECTION_WIDTH * NUM_SECTIONS;
  localparam int unsigned NumSections = NUM_SECTIONS;
  localparam int unsigned ValidBitsWidth = RowWidth > 1 ? $clog2(RowWidth + 1) : 1;

  logic [SECTION_WIDTH-1:0] kernel_mem [31:0][NumSections-1:0];
  logic [SECTION_WIDTH-1:0] feature_buf [NumSections-1:0];

  logic                     compute_trigger;
  logic                     mem_read_en;
  logic                     mem_write_en;
  logic [ValidBitsWidth-1:0] valid_bits;

  logic [RowWidth-1:0]      kernel_row;
  logic [RowWidth-1:0]      feature_row;
  logic [RowWidth-1:0]      masked_kernel;
  logic [RowWidth-1:0]      masked_feature;
  logic [23:0]              comp_result;
  logic [23:0]              psum;
  logic [3:0]               result_4bit;

  assign compute_trigger = COMPE & ~RCSN & ~RCSN0 & ~RCSN1 & ~RCSN2 & ~RCSN3;
  assign mem_read_en     = ~COMPE & ~RCSN;
  assign mem_write_en    = ~COMPE & ~WCSN & ~WEN;

  always_comb begin
    valid_bits = ValidBitsWidth'(RowWidth) - ValidBitsWidth'({MCT, 2'b00});
    if (MCT == '0) begin
      valid_bits = ValidBitsWidth'(RowWidth);
    end
  end

  always_comb begin
    kernel_row  = '0;
    feature_row = '0;
    for (int unsigned section = 0; section < NumSections; section++) begin
      kernel_row[section*SECTION_WIDTH +: SECTION_WIDTH]  = kernel_mem[RA[6:2]][section];
      feature_row[section*SECTION_WIDTH +: SECTION_WIDTH] = feature_buf[section];
    end
  end

  always_comb begin
    masked_kernel  = kernel_row;
    masked_feature = feature_row;
    for (int unsigned bit_idx = 0; bit_idx < RowWidth; bit_idx++) begin
      if (bit_idx >= valid_bits) begin
        masked_kernel[bit_idx]  = 1'b0;
        masked_feature[bit_idx] = 1'b0;
      end
    end
  end

  always_comb begin
    comp_result = '0;
    unique case (MODE)
      2'b00: begin
        for (int unsigned bit_idx = 0; bit_idx < RowWidth; bit_idx++) begin
          comp_result = comp_result + (masked_kernel[bit_idx] & masked_feature[bit_idx]);
        end
      end
      2'b01: begin
        for (int unsigned elem = 0; elem < RowWidth / 2; elem++) begin
          logic [1:0] k_val;
          logic [1:0] f_val;
          k_val = masked_kernel[elem*2 +: 2];
          f_val = masked_feature[elem*2 +: 2];
          comp_result = comp_result + (k_val * f_val);
        end
      end
      2'b10: begin
        for (int unsigned elem = 0; elem < RowWidth / 4; elem++) begin
          logic [3:0] k_val;
          logic [3:0] f_val;
          k_val = masked_kernel[elem*4 +: 4];
          f_val = masked_feature[elem*4 +: 4];
          comp_result = comp_result + (k_val * f_val);
        end
      end
      default: begin
        for (int unsigned elem = 0; elem < RowWidth / 8; elem++) begin
          logic [7:0] k_val;
          logic [7:0] f_val;
          k_val = masked_kernel[elem*8 +: 8];
          f_val = masked_feature[elem*8 +: 8];
          comp_result = comp_result + (k_val * f_val);
        end
      end
    endcase
  end

  always_comb begin
    psum = comp_result + ADDIN;

    if (psum[23]) begin
      result_4bit = 4'b0000;
    end else if (|psum[23:4]) begin
      result_4bit = 4'b1111;
    end else begin
      result_4bit = psum[3:0];
    end
  end

  always_ff @(posedge RCK or negedge RESETn) begin
    if (!RESETn) begin
      feature_buf <= '{default: '0};
      kernel_mem  <= '{default: '{default: '0}};
      Q           <= '0;
    end else begin
      if (!FCSN) begin
        feature_buf[FA] <= FD;
      end

      if (mem_read_en) begin
        Q <= kernel_mem[RA[6:2]][RA[1:0]];
      end

      if (mem_write_en) begin
        for (int unsigned bit_idx = 0; bit_idx < SECTION_WIDTH; bit_idx++) begin
          if (M[bit_idx]) begin
            kernel_mem[WA[6:2]][WA[1:0]][bit_idx] <= D[bit_idx];
          end
        end
      end
    end
  end

  always_comb begin
    PSOUT           = psum;
    SOUT            = result_4bit[0];
    RES_OUT         = result_4bit[3:1];
    READYN          = !compute_trigger;
  end

endmodule : DIMC
