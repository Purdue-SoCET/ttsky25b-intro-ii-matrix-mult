/*
 * Copyright (c) 2024 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_intro_ii_matrix_mult (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

  // All output pins must be assigned. If not used, assign to 0.
  assign uo_out[7:3]  = '0;  // Unused dedicated outputs
  assign uio_out = '0;  // Not using bidirectional output path
  assign uio_oe  = '0;  // Will use ALL bidirectional pins as inputs

  // List all unused inputs to prevent warnings
  wire _unused = &{ena, 1'b0};

  // Instantiate Project
  matrix_mult matrix_mult_proj (
    .clk(clk),
    .n_rst(rst_n),
    .confirmation(ui_in[0]),
    .serial_in(ui_in[1]),
    .bit_period({uio_in, ui_in[7:2]}),
    .data_read(uo_out[0]),
    .serial_out(uo_out[1]),
    .tx_busy(uo_out[2])
  );

endmodule
