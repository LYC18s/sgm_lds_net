`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// 模块名称：sort3_comb
//
// 功能：
//   对3个无符号数进行组合排序：
//
//      lo <= mid <= hi
//
// 用于3×3中值滤波。
// 本模块没有时钟，是纯组合逻辑。
// ////////////////////////////////////////////////////////////////////////////////

module sort3_comb #(
    parameter W = 6
)(
    input  wire [W-1:0] a,
    input  wire [W-1:0] b,
    input  wire [W-1:0] c,

    output reg  [W-1:0] lo,
    output reg  [W-1:0] mid,
    output reg  [W-1:0] hi
);

    always @* begin

        // a <= b
        if (a <= b) begin

            // a <= b <= c
            if (b <= c) begin
                lo  = a;
                mid = b;
                hi  = c;
            end

            // a <= c < b
            else if (a <= c) begin
                lo  = a;
                mid = c;
                hi  = b;
            end

            // c < a <= b
            else begin
                lo  = c;
                mid = a;
                hi  = b;
            end

        end

        // b < a
        else begin

            // b < a <= c
            if (a <= c) begin
                lo  = b;
                mid = a;
                hi  = c;
            end

            // b <= c < a
            else if (b <= c) begin
                lo  = b;
                mid = c;
                hi  = a;
            end

            // c < b < a
            else begin
                lo  = c;
                mid = b;
                hi  = a;
            end

        end

    end

endmodule
