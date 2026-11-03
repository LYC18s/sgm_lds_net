`timescale 1ns/1ps
// Exact arithmetic right-shift with a sticky low bit. MIN_SHIFT/MAX_SHIFT
// specialize only a PROVEN exported shift range; defaults cover all0..63.
// The quotient is clipped using overflow flags before a16-bit increment,
// avoiding a64-bit increment/comparison register on every quantization lane.
module lds_net_round_sat_compact #(parameter MIN_SHIFT=0,MAX_SHIFT=63)(
    input wire clk,rst_n,in_valid,output wire in_ready,
    input wire signed [63:0] value,input wire [5:0] shift,
    input wire signed [15:0] minimum,maximum,
    output wire out_valid,input wire out_ready,output reg signed [15:0] result);
    localparam WIDTH=66-MIN_SHIFT,QUOTIENT=64-MIN_SHIFT;
    localparam STAGES=$clog2(MAX_SHIFT-MIN_SHIFT+1);
    wire signed [WIDTH-1:0] jam[0:STAGES];
    generate if(MIN_SHIFT==0)begin: initial_full
        assign jam[0]={value,2'b00};
    end else if(MIN_SHIFT==1)begin: initial_one
        assign jam[0]={value[63:1],value[0],1'b0};
    end else begin: initial_fixed
        assign jam[0]={value[63:MIN_SHIFT],value[MIN_SHIFT-1],|value[MIN_SHIFT-2:0]};
    end endgenerate
    wire [5:0] delta=shift-MIN_SHIFT;
    genvar stage;
    generate for(stage=0;stage<STAGES;stage=stage+1)begin: barrel
        wire signed [WIDTH-1:0] moved=$signed(jam[stage])>>>(1<<stage);
        assign jam[stage+1]=delta[stage]?{moved[WIDTH-1:1],moved[0] || (|jam[stage][(1<<stage)-1:0])}:jam[stage];
    end endgenerate
    wire signed [QUOTIENT-1:0] quotient=jam[STAGES][WIDTH-1:2];
    wire increment=jam[STAGES][1] && (jam[STAGES][0] || jam[STAGES][2]);
    wire positive_overflow,negative_overflow;wire signed [15:0] low_quotient;
    generate if(QUOTIENT>16)begin: wide
        assign positive_overflow=!quotient[QUOTIENT-1] && |quotient[QUOTIENT-2:15];
        assign negative_overflow=quotient[QUOTIENT-1] && !( &quotient[QUOTIENT-2:15]);
        assign low_quotient=quotient[15:0];
    end else begin: narrow
        assign positive_overflow=0;assign negative_overflow=0;
        assign low_quotient={{(16-QUOTIENT){quotient[QUOTIENT-1]}},quotient};
    end endgenerate
    reg [1:0] valid_pipe;reg signed [15:0] q,lo,hi;reg inc,pos,neg;
    wire step=!valid_pipe[1] || out_ready;
    assign in_ready=step && rst_n;assign out_valid=valid_pipe[1] && rst_n;
    always @(posedge clk)begin
        if(!rst_n)begin valid_pipe<=0;q<=0;lo<=0;hi<=0;inc<=0;pos<=0;neg<=0;result<=0;end
        else if(step)begin
            valid_pipe<={valid_pipe[0],in_valid};q<=low_quotient;inc<=increment;
            pos<=positive_overflow;neg<=negative_overflow;lo<=minimum;hi<=maximum;
            if(neg)result<=lo;
            else if(pos)result<=hi;
            else if(q<$signed(lo))result<=lo;
            else if(q>=$signed(hi))result<=hi;
            else result<=q+$signed({15'd0,inc});
        end
    end
endmodule
