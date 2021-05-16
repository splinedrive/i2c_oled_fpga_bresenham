/*
 *  top.v
 *  copyright (c) 2021  hirosh dabui <hirosh@dabui.de>
 *
 *  permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  the software is provided "as is" and the author disclaims all warranties
 *  with regard to this software including all implied warranties of
 *  merchantability and fitness. in no event shall the author be liable for
 *  any special, direct, indirect, or consequential damages or any damages
 *  whatsoever resulting from loss of use, data or profits, whether in an
 *  action of contract, negligence or other tortious action, arising out of
 *  or in connection with the use or performance of this software.
 *
 */
`timescale 1 ns/10 ps
`default_nettype none
`include "i2c_api.vh"
`ifdef SIM
module top_tb;
inout  sda;
`else
/*
module top(input CLK,
inout P2_1,
output P2_2);
// these are the led holding registers, whatever you write to these appears on the led display
wire clk = CLK;
wire scl_pin = P2_1;
wire sda = P2_2;
*/
module top(input clk,
           inout sda,
           output scl_pin);
reg [7:0] reg_file[3:0];
`endif



localparam TRANSFER_RATE   = 400_000;
localparam CLK_FREQ        = 25_000_000;
localparam CLK_PERIOD      = 1/$itor(CLK_FREQ);
localparam PERIOD_NS       = $rtoi(CLK_PERIOD*10.0e9);
localparam SCL_CYCLES      = CLK_FREQ / TRANSFER_RATE;

wire [7:0] data_rx;
wire i2c_ready;
wire stopped;
wire sda_oe;
`ifdef SIM
reg sda_in;
`else
wire sda_in;
`endif
wire sda_out;
wire scl;
reg enable;

wire error;
wire valid;
wire [7:0] rx_data;
wire rx_valid;

i2c_api
    #(
        .TRANSFER_RATE(TRANSFER_RATE),
        .CLK_FREQ(CLK_FREQ)
    ) i2c_api_i (
        .clk(clk),
        .resetn(resetn),
        .enable(enable),
        .slave_addr(slave_addr),
        .device_register(device_register),
        .data_tx(mux_data_tx),
        .data_rx(data_rx),
        .scl(scl),
        .sda_oe(sda_oe),
        .i2c_function(mux_i2c_function),
`ifdef SIM
        .sda(sda),
`endif
        .sda_in(sda_in),
        .sda_out(sda_out),
        .done(done),
        .ready(i2c_ready)
    );

`ifndef SIM
SB_IO #(
          .PIN_TYPE(6'b1010_01),
          .PULLUP(1'b0)
      ) sda_i (
          .PACKAGE_PIN(sda),
          .OUTPUT_ENABLE(sda_oe),
          .D_OUT_0(sda_out),
          .D_IN_0(sda_in)
      );

SB_IO #(
          .PIN_TYPE(6'b1010_01),
          .PULLUP(1'b0)
      ) sdc_i (
          .PACKAGE_PIN(scl_pin),
          .OUTPUT_ENABLE(1'b1),
          .D_OUT_0(scl)
      );
`endif


`ifdef SIM
reg clk = 0;
always #(PERIOD_NS>>1) clk = ~clk;

initial begin
    $dumpfile("testbench.vcd");
    $dumpvars(0, top_tb);
    $dumpon;
end

initial begin
    sda_in = 1'b0;
end
`endif

reg [5:0] reset_cnt = 0;
wire resetn = &reset_cnt;

always @(posedge clk) begin
    reset_cnt <= reset_cnt + !resetn;
end

/* oled part */
localparam SETCONTRAST         = 8'h81;
localparam DISPLAYALLON_RESUME = 8'hA4;
localparam DISPLAYALLON        = 8'hA5;
localparam NORMALDISPLAY       = 8'hA6;
localparam INVERTDISPLAY       = 8'hA7;
localparam DISPLAYOFF          = 8'hAE;
localparam DISPLAYON           = 8'hAF;
localparam SETDISPLAYOFFSET    = 8'hD3;
localparam SETCOMPINS          = 8'hDA;
localparam SETVCOMDETECT       = 8'hDB;
localparam SETDISPLAYCLOCKDIV  = 8'hD5;
localparam SETPRECHARGE        = 8'hD9;
localparam SETMULTIPLEX        = 8'hA8;
localparam SETLOWCOLUMN        = 8'h00;
localparam SETHIGHCOLUMN       = 8'h10;
localparam SETSTARTLINE        = 8'h40;
localparam MEMORYMODE          = 8'h20;
localparam COLUMNADDR          = 8'h21;
localparam PAGEADDR            = 8'h22;
localparam COMSCANINC          = 8'hC0;
localparam COMSCANDEC          = 8'hC8;
localparam SEGREMAP            = 8'hA0;
localparam CHARGEPUMP          = 8'h8D;


reg signed [7:0] x0 = 0; // 0 - 15
reg signed [7:0] y0 = 0; // 0 - 3
reg signed [7:0] x1 = 127; // 0 - 15
reg signed [7:0] y1 = 63; // 0 - 3
reg [3:0] x; // 0 - 15
reg [1:0] y; // 0 - 3
reg [7:0] i;
reg  [9:0] ssd1306_addr;
wire [7:0] ssd1306_out;
wire ready_gfx;
wire done_gfx;
reg render;
reg rd;
reg and_fb;

gfx_unit_bresenham gfx_unit_i(
                       .clk(clk),
                       .and_fb(and_fb),
                       .x0(x0), // 0 - 15
                       .y0(y0), // 0 - 3
                       .x1(x1), // 0 - 15
                       .y1(y1), // 0 - 3
                       .ssd1306_addr(ssd1306_addr),
                       .ssd1306_out(ssd1306_out),
                       .rd(rd),
                       .ready(ready_gfx),
                       .resetn(resetn),
                       .render(render),
                       .done(done_gfx)
                   );


reg [4:0] state;
reg [3:0] return_state;
reg [31:0] wait_states;
wire done;
wire [7:0] data_tx;

`I2C_API_DECLS

    /* instructions */
    localparam NOP        = 4'd00;
localparam STOP       = 4'd01;
localparam JMP        = 4'd02;
localparam LD         = 4'd03;
localparam WAIT       = 4'd04;
localparam JNZ        = 4'd05;
localparam FB_FLUSH   = 4'd06;
localparam DRAW_LINE       = 4'd07;
localparam SHOW_CLOCK = 4'd08;

wire [7:0] i2c_function;
wire [6:0] slave_addr;
wire [7:0] device_register;
wire [3:0] opcode;
wire [7:0] operand;

reg [3:0] led_idx;

reg [7:0] ip;

reg [ 8 + 7 + 8 + 8 + 4 +  8 - 1:0] ctrl;
always @(*) begin
    case (ip)
        /* i2c_function                  slave_addr   out                  reg     opcode   operand */
        /* DISPLAYOFF */
        00: ctrl <= {I2C_NOP,             7'h3c,       8'h00,                8'h00,  NOP,      8'd00};
        01: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        02: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        03: ctrl <= {I2C_WRITE_RAW,       7'h3c,       DISPLAYOFF,           8'h00,  NOP,      8'h00};
        04: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        05: ctrl <= {I2C_NOP,             7'h3c,       8'h00,                8'h00,  NOP,      8'd00};
        /* SETDISPLAYCLOCKDIV 0x80 */
        06: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        07: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        08: ctrl <= {I2C_WRITE_RAW,       7'h3c,       SETDISPLAYCLOCKDIV,   8'h00,  NOP,      8'h00};
        09: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h80,                8'h00,  NOP,      8'h00};
        10: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};

        /* SETMULTIPLEX 0x3f */
        11: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        12: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        13: ctrl <= {I2C_WRITE_RAW,       7'h3c,       SETMULTIPLEX,         8'h00,  NOP,      8'h00};
        14: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h3f,                8'h00,  NOP,      8'h00};
        15: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};


        /* SETDISPLAYOFFSET 0x0 */
        16: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        17: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        18: ctrl <= {I2C_WRITE_RAW,       7'h3c,       SETDISPLAYOFFSET,     8'h00,  NOP,      8'h00};
        19: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        20: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};

        /* SETSTARTLINE | 0 */
        21: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        21: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        22: ctrl <= {I2C_WRITE_RAW,       7'h3c,       SETSTARTLINE | 1'b0,  8'h00,  NOP,      8'h00};
        23: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};

        /* CHARGEPUMP 0x14 */
        24: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        25: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        26: ctrl <= {I2C_WRITE_RAW,       7'h3c,       CHARGEPUMP,           8'h00,  NOP,      8'h00};
        27: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h14,                8'h00,  NOP,      8'h00};
        28: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};

        /* MEMORYMODE 0x0*/
        29: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        30: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        31: ctrl <= {I2C_WRITE_RAW,       7'h3c,       MEMORYMODE,           8'h00,  NOP,      8'h00};
        32: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        33: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};

        /* SEGREMAP | 0x1 */
        34: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        35: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        36: ctrl <= {I2C_WRITE_RAW,       7'h3c,       SEGREMAP | 1'b1,      8'h00,  NOP,      8'h00};
        37: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};

        /* COMSCANDEC */
        38: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        39: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        40: ctrl <= {I2C_WRITE_RAW,       7'h3c,       COMSCANDEC,           8'h00,  NOP,      8'h00};
        41: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};

        /* SETCOMPINS 0x12 */
        42: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        43: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        44: ctrl <= {I2C_WRITE_RAW,       7'h3c,       SETCOMPINS,           8'h00,  NOP,      8'h00};
        45: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h12,                8'h00,  NOP,      8'h00};
        46: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};

        /* SETCONTRAST 0xcf*/
        47: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        48: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        49: ctrl <= {I2C_WRITE_RAW,       7'h3c,       SETCONTRAST,          8'h00,  NOP,      8'h00};
        50: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'hcf,                8'h00,  NOP,      8'h00};
        51: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};

        /* SETVCOMDETECT 0x40 */
        52: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        53: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        54: ctrl <= {I2C_WRITE_RAW,       7'h3c,       SETVCOMDETECT,        8'h00,  NOP,      8'h00};
        55: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h40,                8'h00,  NOP,      8'h00};
        56: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};

        /* DISPLAYALLON_RESUME */
        57: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        58: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        59: ctrl <= {I2C_WRITE_RAW,       7'h3c,       DISPLAYALLON_RESUME,  8'h00,  NOP,      8'h00};
        60: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};

        /* NORMALDISPLAY */
        61: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        62: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        63: ctrl <= {I2C_WRITE_RAW,       7'h3c,       NORMALDISPLAY,        8'h00,  NOP,      8'h00};
        64: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};

        /* DISPLAYON */
        65: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        66: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        67: ctrl <= {I2C_WRITE_RAW,       7'h3c,       DISPLAYON,            8'h00,  NOP,      8'h00};
        68: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'd165};

        /* nop */
        69: ctrl <= {I2C_NOP,              7'h00,       8'h00,               8'h00,  NOP,     8'd79};

        70: ctrl <= {I2C_NOP,              7'h68,       8'h00,               8'h00,  NOP,      8'h00};
        /* read seconds */
        71: ctrl <= {I2C_READ_U8,          7'h68,       8'h00,               8'h00,  LD,       8'h03};
        /* read minutes */
        72: ctrl <= {I2C_READ_U8,          7'h68,       8'h00,               8'h01,  LD,       8'h02};
        /* read hours   */
        73: ctrl <= {I2C_READ_U8,          7'h68,       8'h00,               8'h02,  LD,       8'h01};
        /* */
        74: ctrl <= {I2C_READ_U8,          7'h68,       8'h00,               8'h02,  JMP,      8'd79};
        /* write seconds */
        75: ctrl <= {I2C_WRITE_8,          7'h68,       8'h00,               8'h00,  NOP,      8'h00};
        /* write minutes */
        76: ctrl <= {I2C_WRITE_8,          7'h68,       8'h39,               8'h01,  NOP,      8'h00};
        /* write hours   */
        77: ctrl <= {I2C_WRITE_8,          7'h68,       8'h00,               8'h02,  NOP,      8'h05};
        /* wait */
        78: ctrl <= {I2C_NOP    ,          7'h68,       8'h00,               8'h00,  JMP,     8'd75};

        /* COLUMNADDR */
        79: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        80: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        81: ctrl <= {I2C_WRITE_RAW,       7'h3c,       COLUMNADDR,           8'h00,  NOP,      8'h00};
        82: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};

        /* 0 */
        83: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        84: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        85: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        86: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};

        /* 127 */
        87: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        88: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        89: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h7f,                8'h00,  NOP,      8'h00};
        90: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};

        /* PAGEADDR */
        91: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        92: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        93: ctrl <= {I2C_WRITE_RAW,       7'h3c,       PAGEADDR,             8'h00,  NOP,      8'h00};
        94: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};

        /* 0 */
        95: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        96: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        97: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        98: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};

        /* 7 */
        99:  ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        100: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        101: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h07,                8'h00,  NOP,      8'h00};
        102: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};

        103: ctrl <= {I2C_NOP,           7'h3c,       8'h00,                 8'h03,  NOP,       8'h70};
        104: ctrl <= {I2C_NOP,           7'h3c,       8'h00,                 8'h00,  NOP,       8'd00};
        105: ctrl <= {I2C_NOP,           7'h3c,       8'h00,                 8'h00,  DRAW_LINE, 8'd69};

        106: ctrl <= {I2C_START,         7'h3c,       8'h00,                 8'h00,  NOP,       8'd69};
        107: ctrl <= {I2C_WRITE_RAW,     7'h3c,       8'h40,                 8'h00,  FB_FLUSH,  8'd69};
        108: ctrl <= {I2C_STOP,          7'h3c,       8'h00,                 8'h00,  NOP,       8'd69};
        109: ctrl <= {I2C_NOP,           7'h3c,       8'h00,                 8'h00,  NOP,      8'd69};
        110: ctrl <= {I2C_NOP,           7'h3c,       8'h00,                 8'h00,  JMP,      8'd69};

        default:
            ctrl <= 0;
    endcase
end


wire [7:0] mux_data_tx = opcode == FB_FLUSH ? (rd ? ssd1306_out : 8'h40) : data_tx;
wire [7:0] mux_i2c_function = opcode == FB_FLUSH & rd ? I2C_WRITE_RAW : i2c_function;

/*       8               7           8          8              4         8 */
assign {i2c_function, slave_addr, data_tx, device_register, opcode, operand} = ctrl;

reg [7:0] i2c_function_fb_flush;
reg flag;
always @(posedge clk) begin
    if (~resetn) begin
        render <= 0;
        i <= 0;
        x <= ~0;
        y <= ~0;
        state <= 0;
        return_state <= 0;
        ip <= 0;
        enable <= 1'b1;
        led_idx <= 0;
        and_fb <= 1'b0;
        flag <= 0;
        rd <= 1'b0;

`ifndef SIM
        reg_file[0] <= 0;
        reg_file[1] <= 0;
        reg_file[2] <= 0;
        reg_file[3] <= 0;
`endif
    end else begin
        case (state)
            0: begin
                if (i2c_ready) begin
                    state <= 1;
                end
            end
            1: begin
                if (done) begin

                    case (opcode)
                        FB_FLUSH:begin
                            state <= 10;
                        end
                        JMP: begin
                            ip <= operand;
                            state <= 0;
                        end
                        LD: begin
                            led_idx <= operand;
                            ip <= ip + 1;
                            state <= 0;
                        end
                        WAIT: begin
                          `ifndef SIM
                            wait_states[22 -:8] <= operand -1;
                            return_state <= 0;
                            state <= 15;
                          `else
                            state <= 0;
                          `endif
                            ip <= ip + 1;
                        end
                        DRAW_LINE: begin
                            rd <= 0;
                            if (ready_gfx) begin
                                if (i == 0) and_fb <= and_fb ^ 1'b1;
                                // draw_bresenham(0, i, i<<1, 63)
                                x0 <= 0;
                                y0 <= i;
                                x1 <= i<<1;//(i<<1);
                                y1 <= 63;
                               // x0 <= 0;
                               // y0 <= 0;
                               // x1 <= 127;
                               // y1 <= 63;
                                render <= 1'b1;
                                state <= 5;
                            end
                        end
                        STOP: begin
                          `ifdef SIM
                            $finish;
                    `endif
                            ip <= ip;
                            state <= 0;
                        end
                        default: begin
                            ip <= ip + 1;
                            state <= 0;
                        end
                    endcase
                `ifndef SIM
                    if (opcode == LD) begin
                        reg_file[operand] <= data_rx;
                        state <= 0;
                    end
                `endif
                end
            end

            5: begin
                if (done_gfx) begin
                    render <= 1'b0;
                    state <= 6;
                    //ip <= ip + 1;
                    //i <= (i >= (63 -5)) ? 0 : i + 5;
                end
            end

            6: begin
                rd <= 0;
                if (ready_gfx) begin
                    // draw_bresenham(i<<1, 0, 127, i)
                    x0 <= i<<1;
                    y0 <= 0;
                    x1 <= 127;
                    y1 <= i;
                    render <= 1'b1;
                    state <= 7;
                end
            end

            7: begin
                if (done_gfx) begin
                    render <= 1'b0;
                    state <= 0;
                    ip <= ip + 1;
                    i <= (i >= (63-2)) ? 0 : i + 2;
                end
            end

            10: begin

                if (i2c_ready) begin
                    ssd1306_addr <= 0;
                    rd <= 1'b1;
                    state <= 11;
                end
            end

            11: begin
                if (done)begin
                    state <= 2;
                end
            end

            2: begin
                if (&ssd1306_addr) begin
                    ip <= ip + 1;
                    state <= 0;
                    rd <= 1'b0;
                end else begin
                    if (i2c_ready) begin
                        ssd1306_addr <= ssd1306_addr + 1;
                        state <= 3;
                    end
                end

            end

            3: begin
                if (done) begin
                    state <= 2;
                end
            end

            15: begin
                wait_states <= wait_states -1;
                if (wait_states == 1) state <= return_state;
            end

            default:
                state <= 0;
        endcase
    end
end

endmodule
