`timescale 1ns/1ps
// Complete convolution descriptor adapter: PL loads each channel group then
// executes it over the row range. A zero row_count selects the full layer.
// Descriptor/source/destination/program_base remain stable until ready returns.
// No mocked arithmetic and no per-layer PS weight/feature-map copies.
module lds_net_conv_operator(
    input wire clk,rst_n,start,cancel,input wire [5:0] layer_index,
    input wire [1023:0] descriptor,input wire [31:0] source,destination,program_base,
    input wire [9:0] first_row,row_count,
    output wire ready,output reg done,error,
    output wire [31:0] araddr,output wire [7:0] arlen,output wire arvalid,input wire arready,
    input wire [63:0] rdata,input wire [1:0] rresp,input wire rlast,rvalid,output wire rready,
    output wire [31:0] awaddr,output wire [7:0] awlen,output wire awvalid,input wire awready,
    output wire [63:0] wdata,output wire [7:0] wstrb,output wire wlast,wvalid,input wire wready,
    input wire [1:0] bresp,input wire bvalid,output wire bready,
    output reg [31:0] source_bytes,program_bytes,destination_bytes,cycles);
    localparam IDLE=0,LOAD_START=1,LOAD_WAIT=2,GROUP_START=3,GROUP_WAIT=4,NEXT=5,ABORT=6,FINISH=7;
    reg [9:0] row_cursor;wire stem_retile=layer_index==0;wire [10:0] end_row=first_row+(row_count==0?oh:row_count);
    reg [2:0] state;reg owner,cancelled;reg [7:0] first;
    wire aborting=cancel || cancelled || state==ABORT;
    wire [7:0] co=descriptor[9*32+:8];wire w8=descriptor[11*32+24+:8]==8;
    wire [9:0] oh=descriptor[10*32+16+:10];
    wire loader_ready,loader_done,loader_error;wire [5:0] group_channels;
    wire [1023:0] biases,multipliers;wire [1151:0] initial_codes;wire [191:0] shifts;
    wire pwe,wwe;wire [3:0] pg;wire [9:0] pa;wire [15:0] pd;wire [31:0] wd;wire [4:0] wa;
    wire [31:0] lar,gar,loaded_bytes;wire [7:0] lalen,galen;wire lav,gav,lrr,grr;
    wire group_ready,group_done,group_error;wire [31:0] group_source_bytes,group_destination_bytes,unused_tiles,unused_cycles;
    lds_net_mac_parameter_loader loader(.clk(clk),.rst_n(rst_n),.start(state==LOAD_START && !aborting),.cancel(aborting),
        .engine_ready(group_ready),.layer_index(layer_index),.channel_start(first),.program_base(program_base),
        .ready(loader_ready),.done(loader_done),.error(loader_error),.channels(group_channels),.biases(biases),.multipliers(multipliers),.shifts(shifts),
        .initial_codes(initial_codes),.program_we(pwe),.program_group(pg),
        .program_address(pa),.program_data(pd),.weight_we(wwe),.weight_address(wa),.weight_data(wd),
        .araddr(lar),.arlen(lalen),.arvalid(lav),.arready(arready && !owner),.rdata(rdata),.rresp(rresp),.rlast(rlast),
        .rvalid(rvalid && !owner),.rready(lrr),.bytes_read(loaded_bytes));
    lds_net_mac_group group_engine(.clk(clk),.rst_n(rst_n),.start(state==GROUP_START && !aborting),.cancel(aborting),.new_tensor(!stem_retile || (first==0 && row_cursor==first_row)),
        .source_base(source),.destination_base(destination),.source_row_bytes(descriptor[5*32+:16]),.destination_row_bytes(descriptor[7*32+:16]),
        .width(descriptor[8*32+:10]),.height(descriptor[8*32+16+:10]),.output_width(descriptor[10*32+:10]),.output_height(oh),
        .first_output_row(stem_retile?row_cursor:first_row),.output_rows(stem_retile?10'd1:(row_count==0?oh:row_count)),.input_channels(descriptor[9*32+16+:8]),.output_channels(co),
        .channel_start(first),.channels(group_channels),.input_mode(descriptor[19*32+:8]==6?2'd0:descriptor[32]?2'd2:2'd1),
        .input_signed(descriptor[19*32+8]),.output_a8(descriptor[19*32+16+:8]==8),.kernel3(descriptor[11*32+:8]==3),
        .stride2(descriptor[11*32+8+:8]==2),.padding(descriptor[11*32+16+:8]!=0),.w8(w8),
        .input_zero(descriptor[21*32+:16]),.minimum(descriptor[18*32+:16]),.maximum(descriptor[18*32+16+:16]),
        .biases(biases),.multipliers(multipliers),.shifts(shifts),.initial_codes(initial_codes),
        .program_we(pwe),.program_group(pg),.program_address(pa),.program_data(pd),.weight_we(wwe),.weight_address(wa),.weight_data(wd),
        .ready(group_ready),.done(group_done),.error(group_error),.araddr(gar),.arlen(galen),.arvalid(gav),.arready(arready && owner),
        .rdata(rdata),.rresp(rresp),.rlast(rlast),.rvalid(rvalid && owner),.rready(grr),
        .awaddr(awaddr),.awlen(awlen),.awvalid(awvalid),.awready(awready),.wdata(wdata),.wstrb(wstrb),.wlast(wlast),
        .wvalid(wvalid),.wready(wready),.bresp(bresp),.bvalid(bvalid),.bready(bready),
        .tiles_completed(unused_tiles),.source_bytes(group_source_bytes),.destination_bytes(group_destination_bytes),.cycles(unused_cycles));
    assign araddr=owner?gar:lar;assign arlen=owner?galen:lalen;assign arvalid=owner?gav:lav;assign rready=owner?grr:lrr;
    assign ready=state==IDLE;
    always @(posedge clk)begin
        if(!rst_n)begin
            state<=IDLE;row_cursor<=0;owner<=0;cancelled<=0;first<=0;done<=0;error<=0;source_bytes<=0;program_bytes<=0;destination_bytes<=0;cycles<=0;
        end else begin
            done<=0;if(!ready)cycles<=cycles+1;
            if(cancel && state!=IDLE && state!=ABORT && state!=FINISH)begin cancelled<=1;error<=1;state<=ABORT;end
            else case(state)
                IDLE:if(start)begin
                    row_cursor<=first_row;first<=0;owner<=0;cancelled<=0;error<=0;source_bytes<=0;program_bytes<=0;destination_bytes<=0;cycles<=0;
                    if(cancel || descriptor[31:0]!=1 || co==0)begin error<=1;state<=FINISH;end else state<=LOAD_START;
                end
                LOAD_START:if(loader_ready && group_ready)begin owner<=0;state<=LOAD_WAIT;end
                LOAD_WAIT:if(loader_done)begin
                    program_bytes<=program_bytes+loaded_bytes;
                    if(loader_error)begin error<=1;state<=ABORT;end else begin owner<=1;state<=GROUP_START;end
                end
                GROUP_START:if(group_ready)state<=GROUP_WAIT;
                GROUP_WAIT:if(group_done)begin
                    source_bytes<=source_bytes+group_source_bytes;destination_bytes<=destination_bytes+group_destination_bytes;
                    if(group_error)begin error<=1;state<=ABORT;end else state<=NEXT;
                end
                NEXT:if(first+group_channels>=co)begin
                     if(stem_retile && row_cursor+1<end_row)begin row_cursor<=row_cursor+1'b1;first<=0;owner<=0;state<=LOAD_START;end
                     else state<=FINISH;
                    end
                    else begin first<=first+group_channels;owner<=0;state<=LOAD_START;end
                ABORT:if(loader_ready && group_ready)state<=FINISH;
                FINISH:begin state<=IDLE;done<=1;end
                default:begin error<=1;state<=ABORT;end
            endcase
        end
    end
endmodule
