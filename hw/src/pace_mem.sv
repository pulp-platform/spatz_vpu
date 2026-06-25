// Copyright 2026 ETH Zurich and University of Bologna.
//
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License. You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// SPDX-License-Identifier: SHL-0.51

// Author: Arpan Suravi Prasad <prasadar@iis.ee.ethz.ch>
module pace_mem #(
  parameter  int unsigned BufDepth   = 2,
  parameter  int unsigned BufWidth   = 256,
  parameter  int unsigned ParamWidth = 256,
  localparam int unsigned AddrWidth  = $clog2(BufDepth)
)(
  input logic                   clk_i,
  input logic                   rst_ni,
  input logic                   we_i,
  output logic                  done_o,
  input logic                   init_i,
  input logic  [BufWidth-1:0]   data_i,
  output logic [ParamWidth-1:0] data_o
);
  logic [BufDepth-1:0][BufWidth-1:0] mem_d, mem_q;
  logic [AddrWidth-1:0] cnt_d, cnt_q;

  always_comb begin
    mem_d = mem_q;
    cnt_d = cnt_q;
    mem_d[cnt_q] = we_i ? data_i : mem_q[cnt_q];
    cnt_d = (init_i == 1'b0) ? '0 : we_i ? cnt_q + 1 : cnt_q;
  end

  assign done_o = (cnt_q == BufDepth);

  always_ff @(posedge clk_i, negedge rst_ni) begin
    if(~rst_ni) begin
      cnt_q <= '0;
      mem_q <= '0;
    end else begin
      cnt_q <= cnt_d;
      mem_q <= mem_d;
    end
  end

  localparam NumParamWords = ParamWidth / BufWidth;
  localparam RemParamWords = ParamWidth % BufWidth;

  if(NumParamWords > 0) begin
    for(genvar ii=0; ii<NumParamWords; ii++) begin
      assign data_o[ii*BufWidth+:BufWidth] = mem_q[ii];
    end
  end
  if(RemParamWords > 0) begin
    localparam int unsigned RemWidth = ParamWidth - NumParamWords*BufWidth;
    assign data_o[NumParamWords*BufWidth+:RemWidth] = mem_q[NumParamWords][0+:RemWidth];
  end
endmodule
