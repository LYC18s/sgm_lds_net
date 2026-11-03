`timescale 1ns/1ps
// Ordered two-code output stream for up to 32 channels at four pixels.
// Capture partial sums before starting the next tile; no metadata latency guess.
module lds_net_tile_requant #(parameter MAX_CHANNELS=32,ACC_BITS=20)(
    input wire clk,rst_n,start,
    input wire [4095:0] accumulators,input wire [1023:0] multipliers,
    input wire [191:0] shifts,input wire [5:0] channels,
    input wire signed [15:0] minimum,maximum,
    output wire ready,output wire out_valid,input wire out_ready,
    output wire [15:0] out_data,output wire [1:0] out_keep,
    output wire [1:0] out_pixel,output wire [4:0] out_channel,
    output reg done);
    reg running;reg [4095:0] sums;reg [1023:0] m;reg [191:0] r;
    localparam CO_BITS=$clog2(MAX_CHANNELS);
    reg [5:0] count;reg [CO_BITS-1:0] source_co,dest_co;reg [2:0] source_pixel,dest_pixel;
    reg signed [15:0] low,high;
    wire ir0,ir1,ov0,ov1;wire signed [15:0] q0,q1;
    wire input_valid=running && source_pixel<4;
    wire output_fire=out_valid && out_ready;
    assign ready=!running;
    assign out_valid=running && ov0 && ov1;
    assign out_data={q1[7:0],q0[7:0]};
    assign out_keep=dest_co+1<count ? 2'b11 : 2'b01;
    assign out_pixel=dest_pixel[1:0];assign out_channel=dest_co;
    wire [9:0] index0=source_co*4+source_pixel;
    wire [9:0] index1=(source_co+1)*4+source_pixel;
    lds_net_requant_compact #(.ACC_BITS(ACC_BITS)) lane0(clk,rst_n,input_valid && ir1,ir0,
        sums[index0*32+:32],m[source_co*32+:32],r[source_co*6+:6],low,high,
        ov0,out_ready && ov1,q0);
    lds_net_requant_compact #(.ACC_BITS(ACC_BITS)) lane1(clk,rst_n,input_valid && ir0,ir1,
        source_co+1<count ? sums[index1*32+:32] : 32'd0,
        source_co+1<count ? m[(source_co+1)*32+:32] : 32'd1,
        source_co+1<count ? r[(source_co+1)*6+:6] : 6'd0,low,high,
        ov1,out_ready && ov0,q1);
    always @(posedge clk) begin
        if(!rst_n) begin
            running<=0;sums<=0;m<=0;r<=0;count<=0;source_co<=0;dest_co<=0;
            source_pixel<=0;dest_pixel<=0;low<=0;high<=0;done<=0;
        end else begin
            done<=0;
            if(start && ready) begin
                running<=1;sums<=accumulators;m<=multipliers;r<=shifts;count<=channels>MAX_CHANNELS ? MAX_CHANNELS : channels;
                source_co<=0;dest_co<=0;source_pixel<=0;dest_pixel<=0;low<=minimum;high<=maximum;
            end
            if(input_valid && ir0 && ir1) begin
                if(source_co+2>=count) begin source_co<=0;source_pixel<=source_pixel+1'b1;end
                else source_co<=source_co+2;
            end
            if(output_fire) begin
                if(dest_co+2>=count) begin
                    dest_co<=0;dest_pixel<=dest_pixel+1'b1;
                    if(dest_pixel==3) begin running<=0;done<=1;end
                end else dest_co<=dest_co+2;
            end
        end
    end
endmodule
