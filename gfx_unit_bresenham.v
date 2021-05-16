/*
 *  gfx_unit_bresenham.v
 *
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
module gfx_unit_bresenham(
           input clk,
           input and_fb,
           input signed [7:0] x0,
           input signed [7:0] y0,
           input signed [7:0] x1,
           input signed [7:0] y1,
           input  [9:0] ssd1306_addr,
           output [7:0] ssd1306_out,
           input rd,
           output ready,
           input resetn,
           input render,
           output reg done
       );

localparam XSIZE = 128;
localparam YSIZE = 64;
localparam ADDR_DEPTH = XSIZE*YSIZE/8;
localparam DATA_WIDTH = 8;

wire  [DATA_WIDTH-1:0] in;
wire [DATA_WIDTH-1:0] out;
reg  cs;
reg  we;

assign ssd1306_out = rd ? out : 'hz;

framebuffer
    #(
        .XSIZE(XSIZE),
        .YSIZE(YSIZE),
        .ADDR_DEPTH(ADDR_DEPTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) framebuffer_i(
        .clk(clk),
        .wr_addr(fb_wr_ptr),
        .rd_addr(rd ? ssd1306_addr : fb_rd_ptr),
        .in(in),
        .out(out),
        .cs(cs | rd),
        .we(we)
    );

reg render_r;
always @(posedge clk) begin
    if (~resetn) begin
        render_r <= 0;
    end else begin
        render_r <= render;
    end
end

reg signed [7:0] x;
reg signed [7:0] y;
reg [5:0] state;

/* set pixel start */
wire [9:0] index = (((y>>3)<<7) | x);
//wire [9:0] index = (((y>>3)<<7) | x);
//wire [9:0] index = ((y & (8'hF8))<<4) | x;
//wire [9:0] index = {y[5:3], {7{1'b0}}} | x;
wire [$clog2(ADDR_DEPTH)-1:0] fb_rd_ptr = index;
wire [$clog2(ADDR_DEPTH)-1:0] fb_wr_ptr = fb_rd_ptr;
wire [7:0] pixel = (1<<(y&7));
assign in = and_fb ? out & ~pixel : out | pixel;
/* set pixel stop */


/* bresenham */
reg signed [8:0] dx;
reg signed [8:0] dy;
reg signed [8:0] xstep;
reg signed [8:0] ystep;
reg signed [8:0] eps;
wire signed [8:0] eps_dy = eps + (dy<<1);
wire signed [8:0] eps_dx = eps + (dx<<1);
/* ********* */

assign ready = !state;


always @(posedge clk) begin
    if (~resetn) begin
        done <= 1'b0;
        we <= 0;
        state <= 0;
        cs <= 0;
        x <= 0;
        y <= 0;
        dx <= 0;
        dy <= 0;
        eps <= 0;
        xstep <= 0;
        ystep <= 0;
    end else begin
        case (state)

            0: begin
                done <= 1'b0;
                if (!render_r & render) begin
                    we <= 1'b0;
                    cs <= 1'b1;

                    dx <= x1 - x0;
                    dy <= y1 - y0;
                    xstep <= 1;
                    ystep <= 1;

                    state <= 1;
                end else begin
                    we <= 1'b0;
                    cs <= 1'b0;
                end
            end

            1: begin
                if (dx < 0) begin
                    dx <= -dx;
                    xstep <= -1;
                end

                if (dy < 0) begin
                    dy <= -dy;
                    ystep <= -1;
                end

                x <= x0;
                y <= y0;

                state <= 13;

            end

            13: begin
                if (dy <= dx) begin
                    eps <= -dx;
                    state <= 2;
                end else begin
                    eps <= -dy;
                    state <= 10;
                end
            end

            2: begin
                if (x == x1) begin
                    cs <= 1'b0;
                    we <= 1'b0;
                    state <= 20;
                end else begin
                    we <= 1'b0;
                    state <= 11;
                end

                if (eps_dy > 0) begin
                    y <= y + ystep;
                    eps <= eps_dy - (dx<<1);
                end else begin
                    eps <= eps_dy;
                end
                x <= x + xstep;
            end

            11: begin
                we <= 1'b1;
                state <= 2;
            end

            10: begin
                we <= 1'b1;
                if (y == y1) begin
                    cs <= 1'b0;
                    we <= 1'b0;
                    state <= 20;
                end else begin
                    we <= 1'b0;
                    state <= 12;
                end

                if (eps_dx > 0) begin
                    x <= x + ystep;
                    eps <= eps_dx - (dy<<1);
                end else begin
                    eps <= eps_dx;
                end
                y <= y + xstep;
            end

            12: begin
                we <= 1'b1;
                state <= 10;
            end

            20: begin
                done <= 1'b1;
                state <= 0;
            end

            default:
                state <= 0;
        endcase
    end
end

endmodule
