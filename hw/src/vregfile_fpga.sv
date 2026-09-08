// Copyright 2023 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Author: Cyril Koenig, ETH Zurich
// Author: Axel Vanoni, ETH Zurich

`include "common_cells/registers.svh"

module vregfile import spatz_pkg::*; #(
    parameter int  unsigned NrReadPorts = 0,
    parameter int  unsigned NrWords     = NRVREG,
    parameter int  unsigned WordWidth   = VRFWordWidth,
    // Derived parameters.  Do not change!
    parameter type          addr_t      = logic[$clog2(NrWords)-1:0],
    parameter type          data_t      = logic [WordWidth-1:0],
    parameter type          strb_t      = logic [WordWidth/8-1:0]
  ) (
    input  logic                    clk_i,
    input  logic                    rst_ni,
    input  logic                    testmode_i,
    // Write ports
    input  addr_t                   waddr_i,
    input  data_t                   wdata_i,
    input  logic                    we_i,
    input  strb_t                   wbe_i,
    // Read ports
    input  addr_t [NrReadPorts-1:0] raddr_i,
    output data_t [NrReadPorts-1:0] rdata_o
  );

  localparam int NumBytes = WordWidth / 8;

  logic [NrWords-1:0][WordWidth/8-1:0][7:0] mem_d;
  logic [NrWords-1:0][WordWidth/8-1:0][7:0] mem_q;


  for (genvar i = 0; i < NrReadPorts; i++) begin : gen_read_port
    assign rdata_o[i] = mem_q[raddr_i[i]];
  end

  always_comb begin
    mem_d = mem_q;

    if (we_i) begin
      for (int word = 0; word < NrWords; word++) begin
        for (int i = 0; i < NumBytes; i++) begin
          if (word == waddr_i && wbe_i[i]) begin
            mem_d[word][i] = wdata_i[8*i+:8];
          end
        end
      end
    end

  end // always_comb

  `FF(mem_q, mem_d, '0);

endmodule : vregfile
