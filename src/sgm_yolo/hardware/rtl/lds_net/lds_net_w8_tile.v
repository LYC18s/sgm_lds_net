`timescale 1ns/1ps
// The six frozen W8 layers have K<=27. Four channels x four pixels x SIMD2.
// Exactly sixteen shared-weight DSP48E1 multipliers, with unsigned A8 inputs.
module lds_net_w8_tile(
    input wire clk,rst_n,start,active_bank,input wire [5:0] reduction,
    input wire [2:0] channels,input wire [127:0] biases,multipliers,
    input wire [23:0] shifts,input wire signed [15:0] minimum,maximum,
    input wire window_we,input wire [4:0] window_address,input wire [31:0] window_data,
    input wire weight_we,input wire [4:0] weight_address,input wire [31:0] weight_data,
    input wire load_bank,
    output wire ready,output reg done,error,
    output wire out_valid,input wire out_ready,output wire [15:0] out_data,
    output wire [1:0] out_keep,out_pixel,output wire [4:0] out_channel);
    (* ram_style="distributed" *) reg [31:0] windows0[0:31],windows1[0:31];
    (* ram_style="distributed" *) reg [31:0] weights0[0:31],weights1[0:31];
    reg [31:0] a00,a01,a10,a11,w00,w01,w10,w11;
    reg bank_q,selected,issue_q,second_q;
    reg [5:0] k,count_q;reg [2:0] channels_q;
    localparam IDLE=0,INIT=1,CALC=2,DRAIN=3,QSTART=4,QWAIT=5;
    reg [2:0] state;reg [3:0] drain;
    reg [127:0] bias_q,multiplier_q;reg [23:0] shift_q;
    reg signed [15:0] low_q,high_q;
    reg signed [19:0] acc[0:15];reg signed [16:0] delta_q[0:15];
    wire issue=state==CALC && k<count_q;
    wire [31:0] a0=selected?a10:a00,a1=selected?a11:a01;
    wire [31:0] w0=selected?w10:w00,w1=selected?w11:w01;
    wire [511:0] products;wire [15:0] product_valid;
    wire [4095:0] sums;
    genvar c,lane,pair;
    generate for(lane=0;lane<2;lane=lane+1) begin: inputs
        for(c=0;c<4;c=c+1) begin: channels_mul
            for(pair=0;pair<2;pair=pair+1) begin: pixels
                localparam INDEX=(lane*4+c)*2+pair;
                wire unused_ready;
                wire [31:0] a=lane==0?a0:a1;
                wire [31:0] w=lane==0?w0:w1;
                lds_net_pack2_w8 multiply(clk,rst_n,issue_q && (lane==0 || second_q),unused_ready,
                    a[pair*16+:8],a[pair*16+8+:8],
                    lane==1 && !second_q ? 8'd0 : w[c*8+:8],
                    product_valid[INDEX],1'b1,products[INDEX*32+:16],products[INDEX*32+16+:16]);
            end
        end
    end endgenerate
    generate for(c=0;c<128;c=c+1) begin: outputs
        if(c<16) assign sums[c*32+:32]={{12{acc[c][19]}},acc[c]};
        else assign sums[c*32+:32]=0;
    end endgenerate
    wire quant_ready,quant_done;
    lds_net_tile_requant #(.MAX_CHANNELS(4)) quantization(clk,rst_n,state==QSTART,sums,
        {896'd0,multiplier_q},{168'd0,shift_q},{3'd0,channels_q},low_q,high_q,
        quant_ready,out_valid,out_ready,out_data,out_keep,out_pixel,out_channel,quant_done);
    integer i,p;reg signed [15:0] term0,term1;
    always @(posedge clk) begin
        if(window_we) begin
            if(load_bank) windows1[window_address]<=window_data;
            else windows0[window_address]<=window_data;
        end
        if(weight_we) begin
            if(load_bank) weights1[weight_address]<=weight_data;
            else weights0[weight_address]<=weight_data;
        end
        a00<=windows0[k[4:0]];a01<=windows0[k[4:0]+5'd1];
        a10<=windows1[k[4:0]];a11<=windows1[k[4:0]+5'd1];
        w00<=weights0[k[4:0]];w01<=weights0[k[4:0]+5'd1];
        w10<=weights1[k[4:0]];w11<=weights1[k[4:0]+5'd1];
        selected<=bank_q;second_q<=k+1<count_q;
        if(!rst_n) issue_q<=0;else issue_q<=issue;
    end
    assign ready=state==IDLE;
    always @(posedge clk) begin
        if(!rst_n) begin
            state<=IDLE;done<=0;error<=0;bank_q<=0;count_q<=0;channels_q<=0;k<=0;drain<=0;
            bias_q<=0;multiplier_q<=0;shift_q<=0;low_q<=0;high_q<=0;
            for(i=0;i<16;i=i+1) begin acc[i]<=0;delta_q[i]<=0;end
        end else begin
            done<=0;
            if((window_we || weight_we) && !ready && load_bank==bank_q) error<=1;
            case(state)
                IDLE:if(start) begin
                    bank_q<=active_bank;count_q<=reduction;channels_q<=channels;
                    bias_q<=biases;multiplier_q<=multipliers;shift_q<=shifts;
                    low_q<=minimum;high_q<=maximum;k<=0;error<=reduction==0 || reduction>32 || channels==0 || channels>4;
                    state<=INIT;
                end
                INIT:begin
                    for(i=0;i<4;i=i+1) for(p=0;p<4;p=p+1) begin acc[i*4+p]<=bias_q[i*32+:32];delta_q[i*4+p]<=0;end
                    state<=CALC;
                end
                CALC:if(issue) k<=k+2;else begin state<=DRAIN;drain<=0;end
                DRAIN:begin
                    drain<=drain+1'b1;
                    if(drain==7) state<=QSTART;
                end
                QSTART:if(quant_ready) state<=QWAIT;
                QWAIT:if(quant_done) begin state<=IDLE;done<=1;end
                default:state<=IDLE;
            endcase
            if(state!=INIT) begin
                for(i=0;i<4;i=i+1) for(p=0;p<4;p=p+1) begin
                    term0=product_valid[i*2+p/2] ? $signed(products[(i*2+p/2)*32+(p%2)*16+:16]) : 16'sd0;
                    term1=product_valid[(i+4)*2+p/2] ? $signed(products[((i+4)*2+p/2)*32+(p%2)*16+:16]) : 16'sd0;
                    delta_q[i*4+p]<=$signed({term0[15],term0})+$signed({term1[15],term1});
                    acc[i*4+p]<=acc[i*4+p]+delta_q[i*4+p];
                end
            end
        end
    end
endmodule
