`timescale 1ns/1ps
// Model-bound multiplier width. W4 abs accumulator<=73019 ->18signed bits;
// W8 abs accumulator<=460690 ->20signed bits, including Bias/partial sums.
module lds_net_requant_compact #(parameter ACC_BITS=20,MIN_SHIFT=31,MAX_SHIFT=43)(
    input wire clk,rst_n,in_valid,output wire in_ready,
    input wire signed [31:0] accumulator,multiplier,input wire [5:0] shift,
    input wire signed [15:0] minimum,maximum,
    output wire out_valid,input wire out_ready,output wire signed [15:0] result);
    reg valid_input,valid_q;(* use_dsp="yes" *) reg signed [ACC_BITS+31:0] product;
    reg signed [ACC_BITS-1:0] accumulator_input;reg signed [31:0] multiplier_input;
    reg [5:0] shift_input;reg signed [15:0] minimum_input,maximum_input;
    reg [5:0] shift_q;reg signed [15:0] minimum_q,maximum_q;
    wire round_ready;wire product_ready=!valid_q || round_ready;
    assign in_ready=rst_n && (!valid_input || product_ready);
    wire signed [ACC_BITS-1:0] bounded_accumulator=accumulator[ACC_BITS-1:0];
    wire signed [63:0] reference_product={{(32-ACC_BITS){product[ACC_BITS+31]}},product};
    always @(posedge clk)begin
        if(!rst_n)begin
            valid_input<=0;valid_q<=0;product<=0;shift_q<=0;minimum_q<=0;maximum_q<=0;
            accumulator_input<=0;multiplier_input<=0;shift_input<=0;minimum_input<=0;maximum_input<=0;
        end else begin
            if(in_ready)begin
                valid_input<=in_valid;accumulator_input<=bounded_accumulator;multiplier_input<=multiplier;
                shift_input<=shift;minimum_input<=minimum;maximum_input<=maximum;
            end
            if(product_ready)begin
                valid_q<=valid_input;product<=accumulator_input*multiplier_input;
                shift_q<=shift_input;minimum_q<=minimum_input;maximum_q<=maximum_input;
            end
        end
    end
    lds_net_round_sat_compact #(.MIN_SHIFT(MIN_SHIFT),.MAX_SHIFT(MAX_SHIFT)) rounding(.clk(clk),.rst_n(rst_n),.in_valid(valid_q),.in_ready(round_ready),
        .value(reference_product),.shift(shift_q),.minimum(minimum_q),.maximum(maximum_q),.out_valid(out_valid),.out_ready(out_ready),.result(result));
endmodule
