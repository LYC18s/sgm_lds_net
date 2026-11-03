`timescale 1ns/1ps
// Export packed HWC8 A8 tensors into the exact NCHW five-port packet.
// One DDR input row is reused for every channel; no CPU transpose/copy.
module lds_net_export_operator(
    input wire clk,rst_n,start,cancel,input wire [1023:0] descriptor,
    input wire [31:0] source,destination,
    output wire ready,output reg done,error,
    output wire [31:0] araddr,output wire [7:0] arlen,output wire arvalid,input wire arready,
    input wire [63:0] rdata,input wire [1:0] rresp,input wire rlast,rvalid,output wire rready,
    output wire [31:0] awaddr,output wire [7:0] awlen,output wire awvalid,input wire awready,
    output wire [63:0] wdata,output wire [7:0] wstrb,output wire wlast,wvalid,input wire wready,
    input wire [1:0] bresp,input wire bvalid,output wire bready,
    output reg [31:0] source_bytes,destination_bytes,cycles);
    localparam IDLE=0,READ_COMMAND=1,READ_WAIT=2,SELECT_CHANNEL=3,WRITE_COMMAND=4,
        BYTE_READ=5,BYTE_CAPTURE=6,SEND_WORD=7,WRITE_WAIT=8,DRAIN=9,FINISH=10,SETUP0=11,SETUP1=12,SETUP2=13;
    reg [3:0] state;reg cancelled,read_seen,unpack_seen;
    reg [9:0] y;reg [7:0] channel;reg [8:0] load_address;
    reg [31:0] write_address;reg [9:0] byte_position,span,rounded_span;reg [2:0] prefix;
    reg [63:0] packet;reg [7:0] keep;
    reg [9:0] width,height;reg [7:0] channels;wire [1:0] groups=channels>8?2'd2:2'd1;
    reg [15:0] stride;wire aborting=cancel || cancelled;
    reg [31:0] source_q,destination_q,source_row_address,destination_row_address,channel_address;
    reg [19:0] plane_size;reg [27:0] packet_size;reg [32:0] source_end;
    wire [32:0] destination_end={1'b0,destination_q}+{5'd0,packet_size};
    wire [9:0] row_words=channels>8?width<<1:width;
    wire dma_ready,dma_done,dma_error,dma_busy,dv,dr,dl;wire [63:0] dd;wire [31:0] dma_bytes,unused_r_stalls;
    wire unpack_ready,unpack_done,unpack_error,uv,ul;wire [63:0] ud;
    wire read_issue=state==READ_COMMAND && dma_ready && unpack_ready && !aborting;
    lds_net_axi_read reader(.clk(clk),.rst_n(rst_n),.command_valid(read_issue),.command_ready(dma_ready),
        .command_address(source_row_address),.command_bytes({8'd0,stride}),.done(dma_done),.error(dma_error),.busy(dma_busy),
        .data(dd),.valid(dv),.ready(dr),.last(dl),.araddr(araddr),.arlen(arlen),.arvalid(arvalid),.arready(arready),
        .rdata(rdata),.rresp(rresp),.rlast(rlast),.rvalid(rvalid),.rready(rready),.bytes_read(dma_bytes),.stall_cycles(unused_r_stalls));
    lds_net_feature_unpack unpacker(.clk(clk),.rst_n(rst_n),.start(read_issue),.mode(2'd1),.signed_codes(1'b1),
        .groups(row_words),.in_data(dd),.in_valid(dv),.in_last(dl),.in_ready(dr),.out_data(ud),.out_valid(uv),.out_ready(1'b1),
        .out_last(ul),.ready(unpack_ready),.done(unpack_done),.error(unpack_error));
    (* ram_style="block" *) reg [63:0] row[0:319];reg [63:0] q;
    wire payload_byte=byte_position>=prefix && byte_position<span;
    wire [9:0] pixel=byte_position-prefix;
    wire writing=state==READ_WAIT && uv && !aborting;
    wire [8:0] row_pixel=channels>8?{pixel[7:0],1'b0}+{8'd0,channel[3]}:pixel[8:0];
    wire [8:0] read_address=payload_byte?row_pixel:9'd0;
    // Separate RAM write/read addresses explicitly. A muxed common address
    // inferred an incorrect write port after synthesis (all words hit row[0]).
    always @(posedge clk)begin
        if(writing)row[load_address]<=ud;
        q<=row[read_address];
    end
    wire write_ready,write_done,write_error,write_busy,stream_ready;wire [31:0] written_bytes,unused_w_stalls;
    wire [31:0] target=channel_address;
    lds_net_axi_write writer(.clk(clk),.rst_n(rst_n),.command_valid(state==WRITE_COMMAND && !aborting),
        .command_ready(write_ready),.command_address(write_address),.command_bytes({14'd0,span}),
        .done(write_done),.error(write_error),.busy(write_busy),.data(packet),.data_keep(keep),
        .valid(state==SEND_WORD),.ready(stream_ready),.awaddr(awaddr),.awlen(awlen),.awvalid(awvalid),.awready(awready),
        .wdata(wdata),.wstrb(wstrb),.wlast(wlast),.wvalid(wvalid),.wready(wready),.bresp(bresp),.bvalid(bvalid),.bready(bready),
        .bytes_written(written_bytes),.stall_cycles(unused_w_stalls));
    assign ready=state==IDLE;
    always @(posedge clk)begin
        if(!rst_n)begin
            state<=IDLE;cancelled<=0;read_seen<=0;unpack_seen<=0;y<=0;channel<=0;load_address<=0;
            write_address<=0;byte_position<=0;span<=0;rounded_span<=0;prefix<=0;packet<=0;keep<=0;
            done<=0;error<=0;source_bytes<=0;destination_bytes<=0;cycles<=0;
            width<=0;height<=0;channels<=0;stride<=0;source_q<=0;destination_q<=0;
            source_row_address<=0;destination_row_address<=0;channel_address<=0;plane_size<=0;packet_size<=0;source_end<=0;
        end else begin
            done<=0;if(!ready)cycles<=cycles+1;
            if(cancel && state!=IDLE)begin cancelled<=1;error<=1;end
            if(aborting && !write_busy && state!=IDLE && state!=DRAIN && state!=FINISH)state<=DRAIN;
            else case(state)
                IDLE:if(start)begin
                    cancelled<=0;error<=0;source_bytes<=0;destination_bytes<=0;cycles<=0;y<=0;channel<=0;
                    width<=descriptor[8*32+:10];height<=descriptor[8*32+16+:10];channels<=descriptor[9*32+16+:8];stride<=descriptor[5*32+:16];
                    source_q<=source;destination_q<=destination;source_row_address<=source;destination_row_address<=destination;channel_address<=destination;
                    if(cancel || descriptor[31:0]!=4 || descriptor[19*32+:8]!=8)begin error<=1;state<=FINISH;end
                    else state<=SETUP0;
                end
                SETUP0:begin plane_size<=width*height;source_end<={1'b0,source_q}+{1'b0,height}*stride;state<=SETUP1;end
                SETUP1:begin packet_size<=channels*plane_size;state<=SETUP2;end
                SETUP2:begin
                    if(width==0 || width>320 || (channels>8 && width>160) || height==0 || channels==0 || channels>16 || row_words>320 || source_q[5:0]!=0 ||
                       stride[5:0]!=0 || stride<({6'd0,row_words}<<3) || source_end>33'h100000000 || destination_end>33'h100000000)
                        begin error<=1;state<=FINISH;end
                    else state<=READ_COMMAND;
                end
                READ_COMMAND:if(read_issue)begin read_seen<=0;unpack_seen<=0;load_address<=0;state<=READ_WAIT;end
                READ_WAIT:begin
                    if(uv)load_address<=load_address+1'b1;
                    if(dma_done)read_seen<=1;if(unpack_done)unpack_seen<=1;
                    if((read_seen || dma_done) && (unpack_seen || unpack_done))begin
                        source_bytes<=source_bytes+dma_bytes;channel<=0;channel_address<=destination_row_address;
                        if(dma_error || unpack_error)begin error<=1;state<=DRAIN;end else state<=SELECT_CHANNEL;
                    end
                end
                SELECT_CHANNEL:begin
                    write_address<={target[31:3],3'd0};prefix<=target[2:0];span<=width+target[2:0];
                    rounded_span<=((width+target[2:0]+7)>>3)<<3;byte_position<=0;packet<=0;keep<=0;state<=WRITE_COMMAND;
                end
                WRITE_COMMAND:if(write_ready)state<=BYTE_READ;
                BYTE_READ:state<=BYTE_CAPTURE;
                BYTE_CAPTURE:begin
                    packet[byte_position[2:0]*8+:8]<=payload_byte?q[channel[2:0]*8+:8]:8'd0;
                    keep[byte_position[2:0]]<=payload_byte;byte_position<=byte_position+1'b1;
                    if(byte_position[2:0]==7)state<=SEND_WORD;else state<=BYTE_READ;
                end
                SEND_WORD:if(stream_ready)begin
                    packet<=0;keep<=0;
                    if(byte_position==rounded_span)state<=WRITE_WAIT;else state<=BYTE_READ;
                end
                WRITE_WAIT:if(write_done)begin
                    destination_bytes<=destination_bytes+written_bytes;
                    if(write_error || aborting)begin error<=1;state<=DRAIN;end
                    else if(channel+1<channels)begin channel<=channel+1'b1;channel_address<=channel_address+plane_size;state<=SELECT_CHANNEL;end
                    else if(y+1<height)begin y<=y+1'b1;source_row_address<=source_row_address+stride;destination_row_address<=destination_row_address+width;state<=READ_COMMAND;end
                    else state<=FINISH;
                end
                DRAIN:if(!dma_busy && unpack_ready && !write_busy)state<=FINISH;
                FINISH:begin state<=IDLE;done<=1;end
                default:begin error<=1;state<=DRAIN;end
            endcase
        end
    end
endmodule
