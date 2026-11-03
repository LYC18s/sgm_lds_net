`timescale 1ns/1ps
// Unpack one padded DDR row into normalized eight-byte channel groups.
// mode0=A6, mode1=A8, mode2=three RGB bytes per pixel. No CPU conversion.
module lds_net_feature_unpack(
    input wire clk,rst_n,start,input wire [1:0] mode,input wire signed_codes,
    input wire [8:0] groups,
    input wire [63:0] in_data,input wire in_valid,in_last,output wire in_ready,
    output reg [63:0] out_data,output wire out_valid,input wire out_ready,
    output wire out_last,output wire ready,output reg done,error);
    reg running,last_seen;reg [1:0] mode_q;reg signed_q;
    reg [8:0] count,emitted;reg [127:0] reservoir;reg [7:0] available;
    wire [6:0] group_bits=mode_q==0?7'd48:mode_q==1?7'd64:7'd24;
    assign ready=!running;
    assign out_valid=running && emitted<count && available>=group_bits;
    assign out_last=emitted+1==count;
    assign in_ready=running && !last_seen && (emitted>=count || available<=64);
    wire incoming=in_valid && in_ready,outgoing=out_valid && out_ready;
    reg [127:0] next_bits;reg [7:0] next_count;
    integer i;reg [5:0] code6;
    always @* begin
        out_data=0;
        for(i=0;i<8;i=i+1) begin
            code6=reservoir[i*6+:6];
            if(mode_q==0) out_data[i*8+:8]={{2{signed_q && code6[5]}},code6};
            else if(mode_q==1 || i<3) out_data[i*8+:8]=reservoir[i*8+:8];
        end
        next_bits=outgoing?reservoir>>group_bits:reservoir;
        next_count=available-(outgoing?group_bits:0);
        if(incoming && emitted<count) begin
            next_bits=next_bits|({64'd0,in_data}<<next_count);
            next_count=next_count+64;
        end
        if(emitted>=count) begin next_bits=0;next_count=0;end
    end
    always @(posedge clk) begin
        if(!rst_n) begin
            running<=0;last_seen<=0;mode_q<=0;signed_q<=0;count<=0;emitted<=0;
            reservoir<=0;available<=0;done<=0;error<=0;
        end else begin
            done<=0;
            if(start && ready) begin
                last_seen<=0;mode_q<=mode;signed_q<=signed_codes;count<=groups;
                emitted<=0;reservoir<=0;available<=0;error<=0;
                if(mode>2 || groups==0 || groups>320) begin done<=1;error<=1;end
                else running<=1;
            end else if(running) begin
                reservoir<=next_bits;available<=next_count;
                if(incoming && in_last) last_seen<=1;
                if(outgoing) emitted<=emitted+1'b1;
                if(emitted==count && last_seen) begin running<=0;done<=1;end
                if(last_seen && emitted<count && available<group_bits) begin running<=0;done<=1;error<=1;end
            end
        end
    end
endmodule
