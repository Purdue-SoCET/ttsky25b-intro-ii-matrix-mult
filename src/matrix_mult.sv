`timescale 1ns / 10ps

module matrix_mult (
    input logic clk, n_rst,
    input logic confirmation,
    input logic serial_in,
    input logic [13:0] bit_period,
    output logic data_read,
    output logic serial_out,
    output logic tx_busy
);

    logic overrun_error, framing_error, data_ready, load_en1, load_en2, load_en3, load_en4, send_en;
    logic [7:0]rx_data;
    logic [7:0]parallel_out1, parallel_out2, parallel_out3, parallel_out4;
    logic [7:0]product1, product2, final_sum;

    controller cntrl(
        .clk(clk),
        .n_rst(n_rst),
        .start_bit(serial_in == 1'b0),
        .overrun_error(overrun_error),
        .framing_error(framing_error),
        .data_ready(data_ready),
        .confirmation(confirmation),
        .data_read(data_read),
        .load_en1(load_en1),
        .load_en2(load_en2),
        .load_en3(load_en3),
        .load_en4(load_en4),
        .send_en(send_en)
    );

    rcv_block uart_rcv(
        .clk(clk),
        .n_rst(n_rst),
        .serial_in(serial_in),
        .data_read(data_read),
        .rx_data(rx_data),
        .data_ready(data_ready),
        .overrun_error(overrun_error),
        .framing_error(framing_error),
        .bit_period(bit_period)
    );

    flex_sr #(
        .SIZE(8),
        .MSB_FIRST(0)
    ) sr_1 (
        .clk(clk),
        .n_rst(n_rst),
        .shift_enable(1'b0),
        .load_enable(load_en1),
        .serial_in(1'b1),
        .parallel_in(rx_data),
        .serial_out(),
        .parallel_out(parallel_out1)
    );

    flex_sr #(
        .SIZE(8),
        .MSB_FIRST(0)
    ) sr_2 (
        .clk(clk),
        .n_rst(n_rst),
        .shift_enable(1'b0),
        .load_enable(load_en2),
        .serial_in(1'b1),
        .parallel_in(rx_data),
        .serial_out(),
        .parallel_out(parallel_out2)
    );

    flex_sr #(
        .SIZE(8),
        .MSB_FIRST(0)
    ) sr_3 (
        .clk(clk),
        .n_rst(n_rst),
        .shift_enable(1'b0),
        .load_enable(load_en3),
        .serial_in(1'b1),
        .parallel_in(product1),
        .serial_out(),
        .parallel_out(parallel_out3)
    );

    flex_sr #(
        .SIZE(8),
        .MSB_FIRST(0)
    ) sr_4 (
        .clk(clk),
        .n_rst(n_rst),
        .shift_enable(1'b0),
        .load_enable(load_en4),
        .serial_in(1'b1),
        .parallel_in(product2),
        .serial_out(),
        .parallel_out(parallel_out4)
    );

    wallace_tree wt_1(
        .a(parallel_out1[3:0]),
        .b(parallel_out1[7:4]),
        .product(product1)
    );

    wallace_tree wt_2(
        .a(parallel_out2[3:0]),
        .b(parallel_out2[7:4]),
        .product(product2)
    );

    n_bit_adder #(
        .SIZE(8)
    ) prod_adder(
        .a(parallel_out3),
        .b(parallel_out4),
        .carry_in(1'b0),
        .sum(final_sum),
        .carry_out()
    );

    tx_block uart_tx(
        .clk(clk),
        .n_rst(n_rst),
        .tx_data(final_sum),
        .tx_start(send_en),
        .bit_period(bit_period),
        .serial_out(serial_out),  
        .tx_busy(tx_busy)
    );

endmodule

//RCV Block

module rcv_block (
    input logic clk,
    input logic n_rst,
    input logic serial_in,
    input logic data_read,
    input logic [13:0] bit_period,
    output logic [7:0] rx_data,
    output logic data_ready,
    output logic overrun_error,
    output logic framing_error
);

    logic frame_err;
    //logic [7:0] rx_data1;

    logic shift_strobe, enable_timer, packet_done;
    timer tim(.clk(clk),
              .n_rst(n_rst),
              .enable_timer(enable_timer),
              .shift_strobe(shift_strobe),
              .bit_period(bit_period),
              .packet_done(packet_done));
    
    logic [7:0] packet_data;
    logic stop_bit;
    sr_9bit sr9(.clk(clk),
                .n_rst(n_rst),
                .shift_strobe(shift_strobe),
                .serial_in(serial_in),
                .packet_data(packet_data),
                .stop_bit(stop_bit));
    
    logic new_packet_detected;
    logic sbc_clear, sbc_enable, load_buffer;
    rcu control(.clk(clk), .n_rst(n_rst),
                .new_packet_detected(new_packet_detected),
                .packet_done(packet_done), .framing_error(frame_err),
                .sbc_clear(sbc_clear), .sbc_enable(sbc_enable), 
                .load_buffer(load_buffer), .enable_timer(enable_timer));
    
    rx_data_buff data_buff(.clk(clk), .n_rst(n_rst),
                           .load_buffer(load_buffer), .packet_data(packet_data),
                           .data_read(data_read), .rx_data(rx_data), 
                           .data_ready(data_ready), .overrun_error(overrun_error));
    
    start_bit_det sbd(.clk(clk), .n_rst(n_rst), 
                      .serial_in(serial_in), .new_packet_detected(new_packet_detected));
    
    stop_bit_chk sbc(.clk(clk), .n_rst(n_rst),
                     .sbc_clear(sbc_clear), .sbc_enable(sbc_enable),
                     .stop_bit(stop_bit), .framing_error(frame_err));
    
    assign framing_error = frame_err;
endmodule


module timer (
    input logic clk,
    input logic n_rst,
    input logic enable_timer,
    input logic [13:0] bit_period,
    output logic shift_strobe,
    output logic packet_done
);
    /* verilator lint_off UNUSEDSIGNAL */
    logic bit_shift_clk, pckt_done;
    logic [3:0] bit_cnt;
    logic [13:0] clk_cnt;
    flex_counter #(.SIZE(14)) clock_count (.clk(clk),
                              .n_rst(n_rst),
                              .clear(pckt_done),
                              .count_enable(enable_timer),
                              .rollover_val(bit_period),
                              .rollover_flag(bit_shift_clk),
                              .count_out(clk_cnt));
    
    assign shift_strobe = bit_shift_clk;
    flex_counter #(.SIZE(4)) bit_count (.clk(clk),
                            .n_rst(n_rst),
                            .rollover_val(4'b1001),
                            .clear(pckt_done),
                            .count_enable(bit_shift_clk),
                            .rollover_flag(pckt_done),
                            .count_out(bit_cnt));
    /* verilator lint_on UNUSEDSIGNAL */
    assign packet_done = pckt_done;

endmodule



module flex_counter #(parameter SIZE = 4) (
    input logic clk,
    input logic n_rst,
    input logic clear,
    input logic count_enable,
    input logic [SIZE-1:0] rollover_val,
    output logic [SIZE-1:0] count_out,
    output logic rollover_flag
    );

    logic [SIZE-1:0] cnt_out_ff, next_cnt_out;
    logic roflag_ff, next_roflag;

    always_ff @( posedge clk, negedge n_rst ) begin : flex_count_ff
        if(!n_rst) begin
            cnt_out_ff <= 0;
        end else begin
            cnt_out_ff <= next_cnt_out;
        end
    end

    always_ff @(posedge clk, negedge n_rst) begin : flex_count_opt_ff
        if(!n_rst) begin
            roflag_ff <= 0;
        end else begin 
            roflag_ff <= next_roflag;
        end
    end


    always_comb begin : flex_count_comb
        next_cnt_out = cnt_out_ff;
        if(clear) begin
            next_cnt_out = 0;
            next_roflag = 1'b0;
        end 
        else if(count_enable) begin
            if(cnt_out_ff == rollover_val) begin
                next_cnt_out = 'b1;
            end
            else begin
                next_cnt_out = cnt_out_ff + 1'b1;
            end
        end
        next_roflag = (next_cnt_out == rollover_val);
    end

    assign count_out = cnt_out_ff;
    assign rollover_flag = roflag_ff;

endmodule


module flex_sr #(
    parameter SIZE = 8,
    parameter MSB_FIRST = 0
) (
    input logic clk,
    input logic n_rst,
    input logic shift_enable,
    input logic load_enable,
    input logic serial_in,
    input logic [SIZE-1:0]parallel_in,
    output logic serial_out,
    output logic [SIZE-1:0]parallel_out
);
    logic [SIZE-1:0]sr_temp;
    logic [SIZE-1:0]sr_next;

    always_ff @(posedge clk, negedge n_rst) begin
        if(!n_rst) begin
            sr_temp <= {SIZE{1'b1}};
        end else begin
            sr_temp <= sr_next;
        end
    end

    always_comb begin
        if(load_enable) begin
            sr_next = parallel_in;
        end else if(shift_enable) begin
            if(MSB_FIRST) begin
                sr_next = {sr_temp[SIZE-2:0], serial_in};
            end else if(!MSB_FIRST) begin
                sr_next = {serial_in, sr_temp[SIZE-1:1]};
            end
        end else begin
            sr_next = sr_temp;
        end
    end

    assign serial_out = MSB_FIRST ? sr_temp[SIZE-1] : sr_temp[0];
    assign parallel_out = sr_temp;

endmodule



module rcu (
    input logic clk,
    input logic n_rst,
    input logic new_packet_detected,
    input logic packet_done,
    input logic framing_error,
    output logic sbc_clear,
    output logic sbc_enable,
    output logic load_buffer,
    output logic enable_timer
);
    typedef enum logic [2:0] {IDLE, CLEAR_ERRORS, RCV_DATA, CHECK_STOP_BIT, ERROR_CHECK, LOAD_BUFFER} state_t;
    logic [2:0]state;
    logic [2:0]next_state;

    always_ff @(posedge clk, negedge n_rst) begin
        if(!n_rst) begin
            state <= IDLE;
        end else begin
            state <= next_state;
        end
    end

    always_comb begin
        next_state = state;
        case(state)
            IDLE: next_state = new_packet_detected ? CLEAR_ERRORS : IDLE;
            CLEAR_ERRORS: next_state = RCV_DATA;
            RCV_DATA: next_state = packet_done ? CHECK_STOP_BIT : RCV_DATA;
            CHECK_STOP_BIT: next_state = ERROR_CHECK;
            ERROR_CHECK: next_state = framing_error ? IDLE : LOAD_BUFFER;
            LOAD_BUFFER: next_state = IDLE;
            default: next_state = IDLE;
        endcase
    end

    always_comb begin
        case(state)
            IDLE: begin
                sbc_clear = 0;
                enable_timer = 0;
                sbc_enable = 0;
                load_buffer = 0;
            end
            CLEAR_ERRORS: begin
                sbc_clear = 1;
                enable_timer = 0;
                sbc_enable = 0;
                load_buffer = 0;
            end
            RCV_DATA: begin
                sbc_clear = 0;
                enable_timer = 1;
                sbc_enable = 0;
                load_buffer = 0;
            end
            CHECK_STOP_BIT: begin
                sbc_clear = 0;
                enable_timer = 0;
                sbc_enable = 1;
                load_buffer = 0;
            end
            ERROR_CHECK: begin
                sbc_clear = 0;
                enable_timer = 0;
                sbc_enable = 0;
                load_buffer = 0;
            end
            LOAD_BUFFER: begin
                sbc_clear = 0;
                enable_timer = 0;
                sbc_enable = 0;
                load_buffer = 1;
            end
            default: begin
                sbc_clear = 0;
                enable_timer = 0;
                sbc_enable = 0;
                load_buffer = 0;
            end
        endcase
    end

endmodule


module stop_bit_chk
(
  input  wire clk,
  input  wire n_rst,
  input  wire sbc_clear,
  input  wire sbc_enable,
  input  wire stop_bit,
  output reg  framing_error
);

  reg nxt_framing_error;
  
  always @ (negedge n_rst, posedge clk)
  begin : REG_LOGIC
    if(1'b0 == n_rst)
    begin
      framing_error  <= 1'b0; // Initialize to inactive value
    end
    else
    begin
      framing_error <= nxt_framing_error;
    end
  end
  
  always @ (framing_error, sbc_clear, sbc_enable, stop_bit)
  begin : NXT_LOGIC
    // Set default value(s)
    nxt_framing_error <= framing_error;
    
    // Define override condition(s)
    if(1'b1 == sbc_clear) // Synchronus clear/reset takes top priority for value
    begin
      nxt_framing_error <= 1'b0;
    end
    else if(1'b1 == sbc_enable) // Stop bit checker is enabled
    begin
      if(1'b1 == stop_bit) // Proper stop bit -> framming error flag should be inactive
      begin
        nxt_framing_error <= 1'b0;
      end
      else // Improper stop bit -> framing error flag should be asserted
      begin
        nxt_framing_error <= 1'b1;
      end
    end
  end
  
  // No additional output logic is needed since output port is used as flip-flop


endmodule


module sr_9bit (
    input logic clk,
    input logic n_rst,
    input logic shift_strobe,
    input logic serial_in,
    output logic [7:0] packet_data,
    output logic stop_bit
);
    /* verilator lint_off PINCONNECTEMPTY */
    flex_sr #(.SIZE(9), .MSB_FIRST(0)) sr_9 (
                  .clk(clk),
                  .n_rst(n_rst),
                  .shift_enable(shift_strobe),
                  .load_enable(1'b0),
                  .serial_in(serial_in),
                  .parallel_in(9'b0),
                  .serial_out(),
                  .parallel_out({stop_bit, packet_data}));
    /* verilator lint_on PINCONNECTEMPTY */
endmodule


module rx_data_buff
(
  input  wire clk,
  input  wire n_rst,
  input  wire load_buffer,
  input  wire [7:0] packet_data,
  input  wire data_read,
  output reg  [7:0] rx_data,
  output reg  data_ready,
  output reg  overrun_error
);

  reg [7:0] nxt_rx_data;
  reg nxt_overrun_error;
  reg nxt_data_ready;
  
  always @ (negedge n_rst, posedge clk)
  begin : REG_LOGIC
    if(1'b0 == n_rst)
    begin
      rx_data       <= '1;  // Initialize the rx_data buffer to have all bits be the idle line value
      data_ready    <= 1'b0;        // Initialize the data_ready flag to be inactive
      overrun_error <= 1'b0;        // Initialize the overrun_error flag to be inactive
    end
    else
    begin
      rx_data       <= nxt_rx_data;
      data_ready    <= nxt_data_ready;
      overrun_error <= nxt_overrun_error;
    end
  end
  
  always @ (rx_data, data_ready, overrun_error, packet_data, load_buffer, data_read)
  begin : NXT_LOGIC
    // Assign default values
    nxt_rx_data        <= rx_data;
    nxt_data_ready    <= data_ready;
    nxt_overrun_error  <= overrun_error;
    
    // Define override condition(s)
    // RX data logic
    if(1'b1 == load_buffer)
    begin
      nxt_rx_data <= packet_data;
    end
    
    // Data ready logic
    if(1'b1 == load_buffer)  // New data will be loaded on the next clock edge -> should always cause data_ready to be asserted
    begin
      nxt_data_ready <= 1'b1;
    end
    else if (1'b1 == data_read) // If new data is not going to be loaded on the next clk edge and the currently stored data is being read -> deassert the data ready flag
    begin
      nxt_data_ready <= 1'b0;
    end
    
    // Overrun Error logic
    if((1'b1 == load_buffer) && (1'b1 == data_ready) && (1'b0 == data_read)) // Loading new data, already have data loaded, and current data is not being read -> overrun will occur
    begin
      nxt_overrun_error <= 1'b1;
    end
    else if (1'b1 == data_read) // Currently stored data is being read -> clear any prior overrun error
    begin
      nxt_overrun_error <= 1'b0;
    end
  end  
endmodule


module start_bit_det
(
  input  wire clk,
  input  wire n_rst,
  input  wire serial_in,
  output wire new_packet_detected
);

  reg old_sample;
  reg new_sample;
  reg sync_phase;
  
  always @ (negedge n_rst, posedge clk)
  begin : REG_LOGIC
    if(1'b0 == n_rst)
    begin
      old_sample  <= 1'b1; // Reset value to idle line value
      new_sample  <= 1'b1; // Reset value to idle line value
      sync_phase  <= 1'b1; // Reset value to idle line value
    end
    else
    begin
      old_sample  <= new_sample;
      new_sample  <= sync_phase;
      sync_phase  <= serial_in;
    end
  end
  
  // Output logic
  assign new_packet_detected = old_sample & (~new_sample); // Detect a falling edge -> new sample must be '0' and old sample must be '1'

  
endmodule

//TX BLOCK
module tx_block (
   input  logic         clk,
   input  logic         n_rst,
   input  logic [7:0]   tx_data,    // adder output
   input  logic         tx_start,   // send_en on controller
   input  logic [13:0]  bit_period, // Bit period TBD
   output logic         serial_out,  
   output logic         tx_busy    
);


    logic count_start, count_finish;
    /* verilator lint_off UNUSEDSIGNAL */
    logic [13:0] clk_cnt;
    flex_counter #(.SIZE(14)) count (.clk(clk),
                              .n_rst(n_rst),
                              .clear(count_start),
                              .count_enable(1'b1),
                              .rollover_val(bit_period),
                              .rollover_flag(count_finish),
                              .count_out(clk_cnt));
    /* verilator lint_on UNUSEDSIGNAL */

    tcu tx_ctrl (.clk(clk), .n_rst(n_rst), .tx_data(tx_data), 
                 .tx_start(tx_start), .serial_out(serial_out), .tx_busy(tx_busy),
                 .count_finish(count_finish), .count_start(count_start));

endmodule

module tcu (
   input  logic         clk,
   input  logic         n_rst,
   input  logic [7:0]   tx_data,    // Data to be transmitted
   input  logic         tx_start,   // Pulse to initiate transmission
   output logic         serial_out,  // Serial output line
   output logic         tx_busy,     // High when transmission is in progress
   output logic count_start,
   input logic count_finish
);
    typedef enum logic [4:0] {
        IDLE = 5'b0,
        SEND_START = 5'h1,
        WAIT_START = 5'h2,
        SEND1 = 5'h3,
        WAIT1 = 5'h4,
        SEND2 = 5'h5,
        WAIT2 = 5'h6,
        SEND3 = 5'h7,
        WAIT3 = 5'h8,
        SEND4 = 5'h9,
        WAIT4 = 5'hA,
        SEND5 = 5'hB,
        WAIT5 = 5'hC,
        SEND6 = 5'hD,
        WAIT6 = 5'hE,
        SEND7 = 5'hF,
        WAIT7 = 5'h10,
        SEND8 = 5'h11,
        WAIT8 = 5'h12,
        SEND_STOP = 5'h13,
        WAIT_STOP = 5'h14
    } state_t;

    state_t state, next_state;

    always_ff @( posedge clk, negedge n_rst ) begin : ff_tcu
        if(!n_rst) begin
            state <= IDLE;
        end else begin
            state <= next_state;
        end        
    end

    always_comb begin : state_comb
        next_state = state;
        tx_busy = 0;
        serial_out = 1;
        count_start = 0;
        case (state)
            IDLE: begin
                tx_busy = 0;
                if(tx_start) begin
                    next_state = SEND_START;
                    count_start = 1;
                end
            end
            SEND_START: begin
                serial_out = 0; //start bit
                next_state = WAIT_START;
            end
            WAIT_START: begin
                serial_out = 0;
                if(count_finish) next_state = SEND1;
            end
            SEND1: begin
                serial_out = tx_data[0];
                //count_start = 1;
                next_state = WAIT1;
            end
            WAIT1: begin
                serial_out = tx_data[0];
                if(count_finish) next_state = SEND2;
            end
            SEND2: begin
                serial_out = tx_data[1];
                //count_start = 1;
                next_state = WAIT2;
            end
            WAIT2: begin
                serial_out = tx_data[1];
                if(count_finish) next_state = SEND3;
            end
            SEND3: begin
                serial_out = tx_data[2];
                //count_start = 1;
                next_state = WAIT3;
            end
            WAIT3: begin
                serial_out = tx_data[2];
                if(count_finish) next_state = SEND4;
            end
            SEND4: begin
                serial_out = tx_data[3];
                //count_start = 1;
                next_state = WAIT4;
            end
            WAIT4: begin
                serial_out = tx_data[3];
                if(count_finish) next_state = SEND5;
            end
            SEND5: begin
                serial_out = tx_data[4];
                //count_start = 1;
                next_state = WAIT5;
            end
            WAIT5: begin
                serial_out = tx_data[4];
                if(count_finish) next_state = SEND6;
            end
            SEND6: begin
                serial_out = tx_data[5];
                //count_start = 1;
                next_state = WAIT6;
            end
            WAIT6: begin
                serial_out = tx_data[5];
                if(count_finish) next_state = SEND7;
            end
            SEND7: begin
                serial_out = tx_data[6];
                //count_start = 1;
                next_state = WAIT7;
            end
            WAIT7: begin
                serial_out = tx_data[6];
                if(count_finish) next_state = SEND8;
            end
            SEND8: begin
                serial_out = tx_data[7];
                //count_start = 1;
                next_state = WAIT8;
            end
            WAIT8: begin
                serial_out = tx_data[7];
                if(count_finish) next_state = SEND_STOP;
            end
            SEND_STOP: begin
                serial_out = 1;
                //count_start = 1;
                next_state = WAIT_STOP;
            end
            WAIT_STOP: begin
                serial_out = 1;
                if(count_finish) next_state = IDLE;
            end
            default: next_state = IDLE;
        endcase
    end
endmodule

module wallace_tree (
    input logic [3:0] a,
    input logic [3:0] b,
    output logic [7:0] product
);

    logic [3:0] pp0, pp1, pp2, pp3;
    assign pp0 = a[0] ? b : 4'b0; 
    assign pp1 = a[1] ? b : 4'b0;  
    assign pp2 = a[2] ? b : 4'b0;  
    assign pp3 = a[3] ? b : 4'b0;  // partial products

    assign product[0] = pp0[0];

    logic s1, c1;
    full_adder fa1(.a(pp0[1]), .b(pp1[0]), .carry_in(1'b0), .sum(s1), .carry_out(c1));
    assign product[1] = s1;

    logic s2, c2, s2_final, c2_ha;
    full_adder fa2(.a(pp0[2]), .b(pp1[1]), .carry_in(pp2[0]), .sum(s2), .carry_out(c2));
    half_adder ha2(.a(s2), .b(c1), .sum(s2_final), .carry_out(c2_ha));
    assign product[2] = s2_final;
    
    logic col2_carry;
    full_adder fa2_carry(.a(c2), .b(c2_ha), .carry_in(1'b0), .sum(col2_carry), .carry_out());

    logic s3_1, c3_1, s3, c3;
    full_adder fa3_1(.a(pp0[3]), .b(pp1[2]), .carry_in(pp2[1]), .sum(s3_1), .carry_out(c3_1));
    full_adder fa3_2(.a(s3_1), .b(pp3[0]), .carry_in(col2_carry), .sum(s3), .carry_out(c3));
    assign product[3] = s3;
    
    logic col3_carry;
    full_adder fa3_carry(.a(c3_1), .b(c3), .carry_in(1'b0), .sum(col3_carry), .carry_out());

    logic s4, c4, s4_final, c4_ha;
    full_adder fa4(.a(pp1[3]), .b(pp2[2]), .carry_in(pp3[1]), .sum(s4), .carry_out(c4));
    half_adder ha4(.a(s4), .b(col3_carry), .sum(s4_final), .carry_out(c4_ha));
    assign product[4] = s4_final;

    logic col4_carry;
    full_adder fa4_carry(.a(c4), .b(c4_ha), .carry_in(1'b0), .sum(col4_carry), .carry_out());

    logic s5, c5;
    full_adder fa5(.a(pp2[3]), .b(pp3[2]), .carry_in(col4_carry), .sum(s5), .carry_out(c5));
    assign product[5] = s5;

    logic s6, c6;
    half_adder ha6(.a(pp3[3]), .b(c5), .sum(s6), .carry_out(c6));
    assign product[6] = s6;

    assign product[7] = c6; // final

endmodule

module full_adder(
    input logic a,
    input logic b,
    input logic carry_in,
    output logic carry_out,
    output logic sum
);
    assign sum = a^b^carry_in;
    assign carry_out = (a&b)|(b&carry_in)|(carry_in&a);
endmodule

module half_adder (
    input logic a, b,
    output logic sum, carry_out
);
    assign sum = a ^ b;
    assign carry_out = a & b;
endmodule

module n_bit_adder #(
    parameter SIZE = 16
) (
    input logic [SIZE-1:0]a,
    input logic [SIZE-1:0]b,
    input logic carry_in,
    output logic [SIZE-1:0]sum,
    output logic carry_out
);
    logic carry_temp[SIZE:0];
    assign carry_temp[0] = carry_in;
    generate
        genvar i;
        for(i=0; i<SIZE; i++) begin: gen_for_loop
            full_adder adder_n(
                .a(a[i]),
                .b(b[i]),
                .carry_in(carry_temp[i]),
                .carry_out(carry_temp[i+1]),
                .sum(sum[i])
            );
        end
    endgenerate
    assign carry_out = carry_temp[SIZE];
    
endmodule

module controller (
    input logic clk,
    input logic n_rst,
    input logic start_bit,
    input logic overrun_error,
    input logic framing_error,
    input logic data_ready,
    input logic confirmation, //mcu data read
    output logic data_read,
    output logic load_en1, //inital 2 numbers
    output logic load_en2, //^^
    output logic load_en3, //1st product
    output logic load_en4, //2nd product
    //output logic add_en, //sum
    output logic send_en //transmitter (tx start)
);

    typedef enum logic [4:0] {
        IDLE,
        SAMPLE1,
        STORE1,
        WAIT1, //1 cycle, could be more, waiting for start bit
        SAMPLE2,
        STORE2,
        WAIT2, //1 cycle
        MULT1,
        STORE3,
        WAIT3, //1 cycle
        MULT2,
        STORE4,
        WAIT4, //1 cycle
        ADD,
        STORE5,
        WAIT5, //1 cycle
        SEND,
        WAIT6, //many cycles, confirmation from MCU
        EIDLE
    } state_t;

    state_t state, next_state;

    always_ff @( posedge clk, negedge n_rst ) begin : controller_ff
        if(!n_rst) begin
            state <= IDLE;
        end
        else begin
            state <= next_state;
        end
    end

    always_comb begin : next_state_logic
        next_state = state;
        data_read = 1'b0;
        load_en1 = 0;
        load_en2 = 0;
        load_en3 = 0;
        load_en4 = 0;
        send_en = 0;
        //add_en = 0;
        case(state)
        IDLE: begin
            if(start_bit) next_state = SAMPLE1;
        end
        SAMPLE1: begin
            if(data_ready) next_state = STORE1;
            if(overrun_error | framing_error) next_state = EIDLE;
        end
        STORE1: begin
            load_en1 = 1;
            next_state = WAIT1;
            data_read = 1;
        end
        WAIT1: begin
            load_en1 = 0;
            data_read = 0;
            if(start_bit) next_state = SAMPLE2;
        end
        SAMPLE2: begin
            if(data_ready) next_state = STORE2;            
            if(overrun_error | framing_error) next_state = EIDLE;
        end
        STORE2: begin
            load_en2 = 1;
            next_state = WAIT2;
            data_read = 1;
        end
        WAIT2: begin
            load_en2 = 0;
            data_read = 0;
            next_state = MULT1;
        end
        MULT1: begin
            next_state = STORE3;
        end
        STORE3: begin
            load_en3 = 1;
            next_state = WAIT3;
        end
        WAIT3: begin
            next_state = MULT2;
        end
        MULT2: begin
            next_state = STORE4;
        end
        STORE4: begin
            load_en4 = 1;
            next_state = WAIT4;
        end
        WAIT4: begin
            next_state = ADD;
        end
        ADD: begin
            //add_en = 1;
            next_state = WAIT5;
        end
        WAIT5: begin
            next_state = SEND;
        end
        SEND: begin
            send_en = 1;
            next_state = WAIT6;
        end
        WAIT6: begin
            send_en = 1;
            if(confirmation) next_state = IDLE;
        end
        EIDLE: begin
            if(start_bit) next_state = SAMPLE1;
        end
        default: begin
            next_state = EIDLE;
        end
        endcase

    end

endmodule

/* verilator lint_on DECLFILENAME */

