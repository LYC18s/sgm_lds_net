`timescale 1ns/1ps
// DDR Add / nearest-neighbor upsample descriptor adapter. Each input row is
// unpacked once; all channels/tiles reuse it. Upsample reuses a row twice.
// Reference Add aligns both products and rounds only after their sum.
module lds_net_vector_operator(
    input wire clk,rst_n,start,cancel,input wire [1023:0] descriptor,
    input wire [31:0] source0,source1,destination,input wire [9:0] first_row,row_count,
    output wire ready,output reg done,error,
    output wire [31:0] araddr,output wire [7:0] arlen,output wire arvalid,input wire arready,
    input wire [63:0] rdata,input wire [1:0] rresp,input wire rlast,rvalid,output wire rready,
    output wire [31:0] awaddr,output wire [7:0] awlen,output wire awvalid,input wire awready,
    output wire [63:0] wdata,output wire [7:0] wstrb,output wire wlast,wvalid,input wire wready,
    input wire [1:0] bresp,input wire bvalid,output wire bready,
    output reg [31:0] source_bytes,destination_bytes,cycles);
    localparam IDLE=0,ROW_SELECT=1,READ_COMMAND=2,READ_WAIT=3,TILE_START=4,WORD_READ=5,
        ADD_ISSUE=6,ADD_WAIT=7,COPY_SEND=8,ADVANCE=9,STORE_WAIT=10,NEXT_TILE=11,DRAIN=12,FINISH=13,VALIDATE=14,CHECK_SHAPE=15;
    reg [3:0] state;reg cancelled,port,read_seen,unpack_seen,cache_valid;wire add_pending=outstanding!=0;reg [5:0] outstanding;reg [1:0] result_pixel;reg [4:0] result_channel;
    reg [9:0] x,y,cached_y;reg [7:0] first_co;reg [1:0] pixel;reg [4:0] pair_channel;reg [8:0] load_address;
    wire [9:0] command_iw=descriptor[8*32+:10],command_ih=descriptor[8*32+16+:10];
    wire [9:0] command_ow=descriptor[10*32+:10],command_oh=descriptor[10*32+16+:10];
    wire [7:0] command_co=descriptor[9*32+:8];wire [4:0] command_groups=(command_co+16'd7)>>3;
    wire [1:0] command_fused=descriptor[23*32+:2];
    // Capture immutable shape once. The scan/address/BRAM path must not
    // recompute channel rounding from the shared controller descriptor.
    reg upsample;reg [1:0] fused;reg [9:0] iw,ih,ow,oh;reg [7:0] co;reg [4:0] groups;
    wire [9:0] read_y=(upsample || fused[port])?y>>1:y;
    wire [4:0] count=co-first_co>16?5'd16:co-first_co;
    wire [9:0] sy=upsample?y>>1:y;
    wire [9:0] sx=upsample?(x+pixel)>>1:x+pixel;
    wire [8:0] read_address=sx<iw?(fused[0]?sx>>1:sx)*groups+((first_co+pair_channel)>>3):9'd0;
    wire [8:0] read_address1=sx<iw?(fused[1]?sx>>1:sx)*groups+((first_co+pair_channel)>>3):9'd0;
    wire [2:0] channel=(first_co+pair_channel)&7;
    wire [15:0] stride=port?descriptor[6*32+:16]:descriptor[5*32+:16];
    wire [31:0] source=port?source1:source0;
    wire bits8=port?descriptor[19*32+9+:7]==8:descriptor[19*32+:8]==8;
    wire signed_q=port?descriptor[19*32+25]:descriptor[19*32+8];
    wire aborting=cancel || cancelled || state==DRAIN;
    wire [15:0] stride0=descriptor[5*32+:16],stride1=descriptor[6*32+:16];
    wire [7:0] bits0=descriptor[19*32+:8];wire [6:0] bits1=descriptor[19*32+9+:7];
    reg [25:0] tensor_bytes0,tensor_bytes1;
    reg [17:0] minimum_row0,minimum_row1;reg [14:0] row_words;reg validation_error;
    wire [32:0] end0={1'b0,source0}+tensor_bytes0,end1={1'b0,source1}+tensor_bytes1;
    wire input_ok=source0[5:0]==0 && stride0[5:0]==0 && end0<=33'h100000000 &&
        (bits0==6 || bits0==8) && stride0>=minimum_row0 &&
        (upsample || (source1[5:0]==0 && stride1[5:0]==0 && end1<=33'h100000000 &&
        (bits1==6 || bits1==8) && stride1>=minimum_row1 && ow==iw && oh==ih));
    wire dma_ready,dma_done,dma_error,dma_busy,dv,dr,dl;wire [63:0] dd;wire [31:0] dma_bytes,unused_stalls;
    wire unpack_ready,unpack_done,unpack_error,uv;wire [63:0] ud;wire ul;
    wire issue=state==READ_COMMAND && dma_ready && unpack_ready && !aborting;
    lds_net_axi_read reader(.clk(clk),.rst_n(rst_n),.command_valid(issue),.command_ready(dma_ready),
        .command_address(source+read_y*stride),.command_bytes({8'd0,stride}),.done(dma_done),.error(dma_error),.busy(dma_busy),
        .data(dd),.valid(dv),.ready(dr),.last(dl),.araddr(araddr),.arlen(arlen),.arvalid(arvalid),.arready(arready),
        .rdata(rdata),.rresp(rresp),.rlast(rlast),.rvalid(rvalid),.rready(rready),.bytes_read(dma_bytes),.stall_cycles(unused_stalls));
    lds_net_feature_unpack unpacker(.clk(clk),.rst_n(rst_n),.start(issue),.mode(bits8?2'd1:2'd0),.signed_codes(signed_q),
        .groups((fused[port]?iw>>1:iw)*groups),.in_data(dd),.in_valid(dv),.in_last(dl),.in_ready(dr),
        .out_data(ud),.out_valid(uv),.out_ready(1'b1),.out_last(ul),.ready(unpack_ready),.done(unpack_done),.error(unpack_error));
    (* ram_style="block" *) reg [63:0] row0[0:319],row1[0:319];reg [63:0] q0,q1;
    wire write0=state==READ_WAIT && uv && !port && !aborting;
    wire write1=state==READ_WAIT && uv && port && !aborting;
    always @(posedge clk)begin
        if(write0)row0[load_address]<=ud;
        if(write1)row1[load_address]<=ud;
        q0<=row0[read_address];
        q1<=row1[read_address1];
    end
    wire [7:0] x00=q0[channel*8+:8],x01=q0[(channel+1)*8+:8];
    wire [7:0] x10=q1[channel*8+:8],x11=q1[(channel+1)*8+:8];
    wire signed [15:0] a00={{8{descriptor[19*32+8] && x00[7]}},x00};
    wire signed [15:0] a01={{8{descriptor[19*32+8] && x01[7]}},x01};
    wire signed [15:0] a10={{8{descriptor[19*32+25] && x10[7]}},x10};
    wire signed [15:0] a11={{8{descriptor[19*32+25] && x11[7]}},x11};
    wire ir0,ir1,ov0,ov1;wire signed [15:0] result0,result1;
    wire store_ready,store_done,store_error,store_input_ready;wire [31:0] stored_bytes;
    wire submit=state==ADD_ISSUE && !aborting;
    wire add_output_ready=aborting || (!upsample && store_input_ready);
    lds_net_add #(.MIN_SHIFT(29),.MAX_SHIFT(32)) add0(.clk(clk),.rst_n(rst_n),.in_valid(submit && ir1),.in_ready(ir0),.x0(a00),.x1(a10),
        .zero0(descriptor[21*32+:16]),.zero1(descriptor[22*32+:16]),.multiplier0(descriptor[15*32+:32]),.multiplier1(descriptor[16*32+:32]),
        .shift(descriptor[17*32+:6]),.minimum(descriptor[18*32+:16]),.maximum(descriptor[18*32+16+:16]),
        .out_valid(ov0),.out_ready(add_output_ready && ov1),.result(result0));
    lds_net_add #(.MIN_SHIFT(29),.MAX_SHIFT(32)) add1(.clk(clk),.rst_n(rst_n),.in_valid(submit && ir0),.in_ready(ir1),.x0(a01),.x1(a11),
        .zero0(descriptor[21*32+:16]),.zero1(descriptor[22*32+:16]),.multiplier0(descriptor[15*32+:32]),.multiplier1(descriptor[16*32+:32]),
        .shift(descriptor[17*32+:6]),.minimum(descriptor[18*32+:16]),.maximum(descriptor[18*32+16+:16]),
        .out_valid(ov1),.out_ready(add_output_ready && ov0),.result(result1));
    wire store_valid=!aborting && (state==COPY_SEND || (!upsample && ov0 && ov1));
    lds_net_tile_store store(.clk(clk),.rst_n(rst_n),.start(state==TILE_START && !aborting),.cancel(aborting),
        .base_address(destination),.row_bytes(descriptor[7*32+:16]),.width(ow),.height(oh),.output_x(x),.output_y(y),
        .total_channels(co),.channel_start(first_co),.channels(count),.a8(descriptor[19*32+16+:8]==8),
        .ready(store_ready),.done(store_done),.error(store_error),.in_data(upsample?{x01,x00}:{result1[7:0],result0[7:0]}),
        .in_keep((upsample?pair_channel:result_channel)+1<count?2'b11:2'b01),.in_pixel(upsample?pixel:result_pixel),.in_channel(upsample?pair_channel:result_channel),.in_valid(store_valid),.in_ready(store_input_ready),
        .awaddr(awaddr),.awlen(awlen),.awvalid(awvalid),.awready(awready),.wdata(wdata),.wstrb(wstrb),.wlast(wlast),.wvalid(wvalid),
        .wready(wready),.bresp(bresp),.bvalid(bvalid),.bready(bready),.bytes_written(stored_bytes));
    assign ready=state==IDLE;
    always @(posedge clk)begin
        if(!rst_n)begin
            state<=IDLE;cancelled<=0;port<=0;read_seen<=0;unpack_seen<=0;cache_valid<=0;cached_y<=0;outstanding<=0;result_pixel<=0;result_channel<=0;
            x<=0;y<=0;first_co<=0;pixel<=0;pair_channel<=0;load_address<=0;
            done<=0;error<=0;source_bytes<=0;destination_bytes<=0;cycles<=0;
            tensor_bytes0<=0;tensor_bytes1<=0;
            minimum_row0<=0;minimum_row1<=0;row_words<=0;validation_error<=0;
            upsample<=0;fused<=0;iw<=0;ih<=0;ow<=0;oh<=0;co<=0;groups<=0;
        end else begin
            done<=0;if(!ready)cycles<=cycles+1;
            case({submit && ir0 && ir1,ov0 && ov1 && add_output_ready})
             2'b10:outstanding<=outstanding+1'b1;
             2'b01:outstanding<=outstanding-1'b1;
            endcase
            if(ov0 && ov1 && add_output_ready && !aborting)begin
             if(result_channel+2<count)result_channel<=result_channel+2;
             else begin result_channel<=0;result_pixel<=result_pixel+1'b1;end
            end
            if(cancel && state!=IDLE && state!=DRAIN && state!=FINISH)begin cancelled<=1;error<=1;state<=DRAIN;end
            else if(store_done && store_error && state!=IDLE && state!=DRAIN && state!=FINISH)begin error<=1;state<=DRAIN;end
            else case(state)
                IDLE:if(start)begin
                    cancelled<=0;error<=0;source_bytes<=0;destination_bytes<=0;cycles<=0;cache_valid<=0;
                    x<=0;y<=first_row;first_co<=0;port<=0;pixel<=0;pair_channel<=0;
                    upsample<=descriptor[31:0]==3;fused<=command_fused;
                    iw<=command_iw;ih<=command_ih;ow<=command_ow;oh<=command_oh;co<=command_co;groups<=command_groups;
                    tensor_bytes0<=command_ih*stride0;tensor_bytes1<=command_ih*stride1;
                    // Bounds: iw<=1023, groups<=31, so products fit 18/15 bits.
                    minimum_row0<=(command_fused[0]?command_iw>>1:command_iw)*command_groups*(bits0==8?8:6);
                    minimum_row1<=(command_fused[1]?command_iw>>1:command_iw)*command_groups*(bits1==8?8:6);
                    row_words<=command_iw*command_groups;validation_error<=0;state<=CHECK_SHAPE;
                end
                CHECK_SHAPE:begin
                    validation_error<=!input_ok || (descriptor[31:0]!=2 && !upsample) || iw==0 || row_words>320 || co==0 ||
                       (row_count!=0 && first_row+{1'b0,row_count}>oh) || (row_count==0 && first_row!=0) ||
                       (upsample && (ow!=iw*2 || oh!=ih*2));
                    state<=VALIDATE;
                end
                VALIDATE:begin
                    if(validation_error)begin error<=1;state<=FINISH;end else state<=ROW_SELECT;
                end
                ROW_SELECT:begin
                    port<=0;
                    if(cache_valid && cached_y==sy)state<=TILE_START;else state<=READ_COMMAND;
                end
                READ_COMMAND:if(issue)begin read_seen<=0;unpack_seen<=0;load_address<=0;state<=READ_WAIT;end
                READ_WAIT:begin
                    if(uv)load_address<=load_address+1'b1;
                    if(dma_done)read_seen<=1;if(unpack_done)unpack_seen<=1;
                    if((read_seen || dma_done) && (unpack_seen || unpack_done))begin
                        source_bytes<=source_bytes+dma_bytes;
                        if(dma_error || unpack_error)begin error<=1;state<=DRAIN;end
                        else if(!port && !upsample)begin port<=1;state<=READ_COMMAND;end
                        else begin cache_valid<=1;cached_y<=sy;state<=TILE_START;end
                    end
                end
                TILE_START:if(store_ready)begin pixel<=0;pair_channel<=0;result_pixel<=0;result_channel<=0;state<=WORD_READ;end
                WORD_READ:state<=upsample?COPY_SEND:ADD_ISSUE;
                ADD_ISSUE:if(ir0 && ir1)begin
                    if(pair_channel+2<count)begin pair_channel<=pair_channel+2;state<=WORD_READ;end
                    else if(pixel<3)begin pair_channel<=0;pixel<=pixel+1'b1;state<=WORD_READ;end
                    else state<=ADD_WAIT;
                end
                ADD_WAIT:if(ov0 && ov1 && store_input_ready && result_pixel==3 && result_channel+2>=count)state<=STORE_WAIT;
                COPY_SEND:if(store_input_ready)state<=ADVANCE;
                ADVANCE:begin
                    if(pair_channel+2<count)begin pair_channel<=pair_channel+2;state<=WORD_READ;end
                    else if(pixel<3)begin pair_channel<=0;pixel<=pixel+1'b1;state<=WORD_READ;end
                    else state<=STORE_WAIT;
                end
                STORE_WAIT:if(store_done)begin destination_bytes<=destination_bytes+stored_bytes;state<=NEXT_TILE;end
                NEXT_TILE:begin
                    // Active channel batches maintain first_co<co. Reuse the
                    // same remaining>16 test as count, avoiding add-then-compare.
                    if(co-first_co>16)begin first_co<=first_co+count;state<=TILE_START;end
                    else if(x+4<ow)begin first_co<=0;x<=x+4;state<=TILE_START;end
                    else if(y+1<first_row+(row_count==0?oh:row_count))begin first_co<=0;x<=0;y<=y+1;state<=ROW_SELECT;end
                    else state<=FINISH;
                end
                DRAIN:if(!dma_busy && unpack_ready && store_ready && !add_pending)state<=FINISH;
                FINISH:begin state<=IDLE;done<=1;end
                default:begin error<=1;state<=DRAIN;end
            endcase
        end
    end
endmodule
