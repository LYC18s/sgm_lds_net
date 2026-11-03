`timescale 1ns/1ps
// Exact signed W8 times two unsigned A8 values (including q-zin in C00).
module lds_net_pack2_w8(
    input wire clk,rst_n,in_valid, output wire in_ready,
    input wire [7:0] a0,a1, input wire signed [7:0] weight,
    output wire out_valid,input wire out_ready,
    output reg signed [15:0] p0,p1);
    reg [2:0] valid_pipe; reg sign0,sign1;
    reg signed [24:0] packed_a;reg signed [17:0] magnitude_w;
    (* use_dsp="yes" *) reg signed [42:0] product;
    wire [7:0] mw=weight[7] ? (~weight+8'd1) : weight;
    wire step=!valid_pipe[2] || out_ready;
    assign in_ready=step && rst_n;assign out_valid=valid_pipe[2] && rst_n;
    always @(posedge clk) begin
        if(!rst_n) begin
            valid_pipe<=0;sign0<=0;sign1<=0;packed_a<=0;magnitude_w<=0;
            product<=0;p0<=0;p1<=0;
        end else if(step) begin
            valid_pipe<={valid_pipe[1:0],in_valid};
            packed_a<=$signed({1'b0,a1,8'd0,a0});
            magnitude_w<=$signed({10'd0,mw});sign0<=weight[7];
            product<=packed_a*magnitude_w;sign1<=sign0;
            p0<=sign1 ? -$signed({1'b0,product[14:0]}) : $signed({1'b0,product[14:0]});
            p1<=sign1 ? -$signed({1'b0,product[30:16]}) : $signed({1'b0,product[30:16]});
        end
    end
endmodule
