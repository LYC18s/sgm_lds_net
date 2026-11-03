`timescale 1ns/1ps
// Scatter a four-pixel/channel tile into packed HWC8 DDR without a read/modify
// write. Prefix and tail WSTRB preserve neighboring channel groups and rows.
// A command owns its tile until every B response has returned, including cancel.
module lds_net_tile_store #(parameter MAX_CHANNELS=16)(
    input wire clk,rst_n,start,cancel,
    input wire [31:0] base_address,input wire [15:0] row_bytes,
    input wire [9:0] width,height,output_x,output_y,
    input wire [7:0] total_channels,channel_start,
    input wire [$clog2(MAX_CHANNELS+1)-1:0] channels,input wire a8,
    output wire ready,output reg done,error,
    input wire [15:0] in_data,input wire [1:0] in_keep,in_pixel,
    input wire [4:0] in_channel,input wire in_valid,output wire in_ready,
    output wire [31:0] awaddr,output wire [7:0] awlen,output wire awvalid,input wire awready,
    output wire [63:0] wdata,output wire [7:0] wstrb,output wire wlast,wvalid,input wire wready,
    input wire [1:0] bresp,input wire bvalid,output wire bready,
    output reg [31:0] bytes_written);
    localparam IDLE=0,SETUP0=1,SETUP1=2,CAPTURE=3,SELECT=4,COMMAND=5,SEND=6,WAIT_RESPONSE=7,FINISH=8;
    reg [3:0] state;reg cancelled;
    reg [31:0] base;reg [15:0] stride;reg [9:0] x,y,w,h;
    reg [7:0] total,first;reg [5:0] count;reg bits8;
    reg [1:0] expected_pixel;reg [4:0] expected_channel;
    localparam PACKET_BYTES=MAX_CHANNELS+8;
    reg [2:0] pixel;reg [PACKET_BYTES*8-1:0] packet;reg [PACKET_BYTES-1:0] packet_keep;
    reg [31:0] address,pixel_address,row_address,initial_offset,minimum_row_bytes;
    reg [32:0] tensor_end;reg [5:0] span;reg [2:0] beat,beats;
    wire [8:0] pixel_bytes=((total+16'd7)>>3)*(bits8?8:6);
    wire [5:0] payload=(count*(bits8?8:6))>>3;
    wire [5:0] next_span={3'd0,pixel_address[2:0]}+payload;
    wire configuration_ok=w!=0 && h!=0 && y<h && x<w && total!=0 && count!=0 && count<=MAX_CHANNELS && first+{1'b0,count}<=total &&
        (bits8 || (first[1:0]==0 && count[1:0]==0)) && stride>=minimum_row_bytes && stride[5:0]==0 && base[5:0]==0 && tensor_end<=33'h100000000;
    wire incoming=in_valid && in_ready;
    wire lane0=in_keep[0] && in_channel<count;
    wire lane1=in_keep[1] && in_channel+1<count;
    wire ordered=in_pixel==expected_pixel && in_channel==expected_channel && in_keep==(in_channel+1<count?2'b11:2'b01);
    // Fixed byte-pair banks replace a variable bit-position write into a
    // 128-bit vector. A6/A8 layouts are STATIC wires out of these code banks.
    wire [MAX_CHANNELS*8-1:0] packed8;wire [MAX_CHANNELS*6-1:0] packed6;
    genvar co;
    generate for(co=0;co<MAX_CHANNELS/2;co=co+1)begin: code_pairs
        reg [15:0] codes[0:3];integer p;
        always @(posedge clk)begin
            if(!rst_n || state==IDLE && start)begin for(p=0;p<4;p=p+1)codes[p]<=0;end
            else if(incoming && ordered && in_channel[4:1]==co)
                codes[in_pixel]<={lane1?in_data[15:8]:8'd0,lane0?in_data[7:0]:8'd0};
        end
        wire [15:0] selected=codes[pixel[1:0]];
        assign packed8[co*16+:16]=selected;
        assign packed6[co*12+:12]={selected[13:8],selected[5:0]};
    end endgenerate
    wire [MAX_CHANNELS*8-1:0] selected_pixel=bits8?packed8:{{MAX_CHANNELS*2{1'b0}},packed6};
    wire command_ready,dma_done,dma_error,dma_busy,stream_ready;wire [31:0] dma_bytes,unused_stalls;
    lds_net_axi_write writer(.clk(clk),.rst_n(rst_n),.command_valid(state==COMMAND && !cancelled && !cancel),
        .command_ready(command_ready),.command_address(address),.command_bytes({18'd0,span}),
        .done(dma_done),.error(dma_error),.busy(dma_busy),.data(packet[beat*64+:64]),.data_keep(packet_keep[beat*8+:8]),.valid(state==SEND),.ready(stream_ready),
        .awaddr(awaddr),.awlen(awlen),.awvalid(awvalid),.awready(awready),.wdata(wdata),.wstrb(wstrb),.wlast(wlast),.wvalid(wvalid),.wready(wready),
        .bresp(bresp),.bvalid(bvalid),.bready(bready),.bytes_written(dma_bytes),.stall_cycles(unused_stalls));
    assign ready=state==IDLE;assign in_ready=state==CAPTURE && !cancelled && !cancel;
    integer j;
    always @(posedge clk)begin
        if(!rst_n)begin
            state<=IDLE;done<=0;error<=0;cancelled<=0;base<=0;stride<=0;x<=0;y<=0;w<=0;h<=0;total<=0;first<=0;
            count<=0;bits8<=0;expected_pixel<=0;expected_channel<=0;pixel<=0;packet<=0;packet_keep<=0;
            address<=0;pixel_address<=0;row_address<=0;initial_offset<=0;minimum_row_bytes<=0;tensor_end<=0;
            span<=0;beat<=0;beats<=0;bytes_written<=0;
        end else begin
            done<=0;
            if(cancel && state!=IDLE)begin cancelled<=1;error<=1;end
            case(state)
                IDLE:if(start)begin
                    error<=0;cancelled<=0;bytes_written<=0;base<=base_address;stride<=row_bytes;x<=output_x;y<=output_y;
                    w<=width;h<=height;total<=total_channels;first<=channel_start;count<=channels;bits8<=a8;
                    expected_pixel<=0;expected_channel<=0;pixel<=0;
                    if(cancel)begin error<=1;state<=FINISH;end else state<=SETUP0;
                end
                SETUP0:begin
                    row_address<=base+y*stride;
                    initial_offset<=x*pixel_bytes+(first*(bits8?8:6)>>3);
                    minimum_row_bytes<=w*pixel_bytes;tensor_end<={1'b0,base}+{1'b0,h}*stride;
                    if(cancel || cancelled)state<=FINISH;else state<=SETUP1;
                end
                SETUP1:begin
                    pixel_address<=row_address+initial_offset;
                    if(cancel || cancelled || !configuration_ok)begin error<=1;state<=FINISH;end else state<=CAPTURE;
                end
                CAPTURE:begin
                    if(cancel || cancelled)state<=FINISH;
                    else if(incoming)begin
                        if(!ordered)begin error<=1;state<=FINISH;end
                        else if(in_channel+2>=count)begin
                            expected_channel<=0;expected_pixel<=expected_pixel+1'b1;if(in_pixel==3)state<=SELECT;
                        end else expected_channel<=expected_channel+2;
                    end
                end
                SELECT:begin
                    if(cancel || cancelled || pixel==4 || x+pixel>=w)state<=FINISH;
                    else begin
                        address<={pixel_address[31:3],3'd0};span<=next_span;beats<=(next_span+7)>>3;beat<=0;
                        packet<={64'd0,selected_pixel}<<(pixel_address[2:0]*8);
                        for(j=0;j<PACKET_BYTES;j=j+1)packet_keep[j]<=j>=pixel_address[2:0] && j<next_span;
                        state<=COMMAND;
                    end
                end
                COMMAND:begin if(cancel || cancelled)state<=FINISH;else if(command_ready)state<=SEND;end
                SEND:if(stream_ready)begin beat<=beat+1'b1;if(beat+1==beats)state<=WAIT_RESPONSE;end
                WAIT_RESPONSE:if(dma_done)begin
                    bytes_written<=bytes_written+dma_bytes;pixel<=pixel+1'b1;pixel_address<=pixel_address+pixel_bytes;
                    if(dma_error)begin error<=1;state<=FINISH;end else state<=SELECT;
                end
                FINISH:begin state<=IDLE;done<=1;end
                default:begin error<=1;state<=FINISH;end
            endcase
        end
    end
endmodule
