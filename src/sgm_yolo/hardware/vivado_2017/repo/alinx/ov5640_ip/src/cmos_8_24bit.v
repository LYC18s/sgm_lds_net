`timescale 1ns / 1ps

// Convert the OV5640's 8-bit DVP RGB565 byte stream into one RGB888 AXI pixel.
// Register 0x4300 = 0x60 transmits the 16-bit RGB565 word MSB first.  Expand
// 5/6-bit components by repeating their most-significant bits; unlike padding
// with zero or one, this maps black to 0 and white to 255.
//
// Xilinx AXI4-Stream Video places R in TDATA[7:0], G in [15:8], and B in
// [23:16].  Thus the Verilog word is {B,G,R}, while little-endian DDR bytes
// are R,G,B as expected by camera.frame().
module cmos_8_24bit #(
    parameter integer FRAME_WIDTH = 640
) (
    input  wire        rst,
    input  wire        pclk,
    input  wire [7:0]  pdata_i,
    input  wire        de_i,
    output reg  [23:0] pdata_o,
    output reg         de_o,
    output reg         line_last
);
    reg       byte_phase;
    reg [7:0] first_byte;
    reg [15:0] pixel_index;

    always @(posedge pclk or posedge rst) begin
        if (rst) begin
            byte_phase  <= 1'b0;
            first_byte  <= 8'd0;
            pixel_index <= 16'd0;
            pdata_o     <= 24'd0;
            de_o        <= 1'b0;
            line_last   <= 1'b0;
        end else begin
            de_o      <= 1'b0;
            line_last <= 1'b0;
            if (!de_i) begin
                byte_phase  <= 1'b0;
                pixel_index <= 16'd0;
            end else begin
                if (!byte_phase) begin
                    // RGB565[15:8] = RRRRRGGG
                    first_byte <= pdata_i;
                    byte_phase <= 1'b1;
                end else begin
                    // RGB565[7:0] = GGGBBBBB.  Output {B8,G8,R8}.
                    pdata_o <= {
                        pdata_i[4:0], pdata_i[4:2],
                        first_byte[2:0], pdata_i[7:5], first_byte[2:1],
                        first_byte[7:3], first_byte[7:5]
                    };
                    de_o       <= 1'b1;
                    line_last  <= (pixel_index == FRAME_WIDTH - 1);
                    byte_phase <= 1'b0;
                    if (pixel_index == FRAME_WIDTH - 1)
                        pixel_index <= 16'd0;
                    else
                        pixel_index <= pixel_index + 16'd1;
                end
            end
        end
    end
endmodule
