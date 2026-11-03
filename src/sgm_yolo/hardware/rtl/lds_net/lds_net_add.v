`timescale 1ns/1ps
// Align both operands before adding, then round once. No intermediate clipping.
module lds_net_add #(parameter MIN_SHIFT=0,MAX_SHIFT=63)(
    input wire clk,rst_n,in_valid,output wire in_ready,
    input wire signed [15:0] x0,x1,
    input wire signed [15:0] zero0,zero1,
    input wire signed [31:0] multiplier0,multiplier1,
    input wire [5:0] shift,input wire signed [15:0] minimum,maximum,
    output wire out_valid,input wire out_ready,output wire signed [15:0] result);
    reg valid_delta,valid0,valid1;
    reg signed [16:0] delta_q0,delta_q1;
    reg signed [31:0] multiplier_q0,multiplier_q1;
    reg [5:0] shift_delta;
    reg signed [15:0] minimum_delta,maximum_delta;
    (* use_dsp="yes" *) reg signed [48:0] product0,product1;
    reg signed [63:0] aligned;
    reg [5:0] shift0,shift1;
    reg signed [15:0] minimum0,maximum0,minimum1,maximum1;
    wire round_ready;wire ready1=!valid1 || round_ready;
    wire ready0=!valid0 || ready1;
    assign in_ready=rst_n && (!valid_delta || ready0);
    wire signed [16:0] delta0=$signed({x0[15],x0})-$signed({zero0[15],zero0});
    wire signed [16:0] delta1=$signed({x1[15],x1})-$signed({zero1[15],zero1});
    always @(posedge clk) begin
        if(!rst_n) begin
            valid_delta<=0;valid0<=0;valid1<=0;product0<=0;product1<=0;aligned<=0;
            delta_q0<=0;delta_q1<=0;multiplier_q0<=0;multiplier_q1<=0;
            shift_delta<=0;minimum_delta<=0;maximum_delta<=0;
            shift0<=0;shift1<=0;minimum0<=0;maximum0<=0;minimum1<=0;maximum1<=0;
        end else begin
            if(in_ready) begin
                valid_delta<=in_valid;delta_q0<=delta0;delta_q1<=delta1;
                multiplier_q0<=multiplier0;multiplier_q1<=multiplier1;
                shift_delta<=shift;minimum_delta<=minimum;maximum_delta<=maximum;
            end
            // Separate BRAM lane selection and zero-point subtraction from
            // the exact two-DSP multiplier, including under backpressure.
            if(ready0) begin
                valid0<=valid_delta;product0<=delta_q0*multiplier_q0;
                product1<=delta_q1*multiplier_q1;
                shift0<=shift_delta;minimum0<=minimum_delta;maximum0<=maximum_delta;
            end
            if(ready1) begin
                valid1<=valid0;
                aligned<=$signed({{15{product0[48]}},product0})+$signed({{15{product1[48]}},product1});
                shift1<=shift0;minimum1<=minimum0;maximum1<=maximum0;
            end
        end
    end
    lds_net_round_sat_compact #(.MIN_SHIFT(MIN_SHIFT),.MAX_SHIFT(MAX_SHIFT)) rounding(clk,rst_n,valid1,round_ready,aligned,shift1,
        minimum1,maximum1,out_valid,out_ready,result);
endmodule
