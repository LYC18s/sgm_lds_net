`timescale 1ns/1ps
// Autonomous DDR -> row cache -> windows -> real W4/W8 MAC -> packed DDR.
// One preloaded output-channel group is replayed over a rectangular row range.
// The PL parameter loader owns the preload ports. It must hold command fields
// stable until ready returns; no host intervention occurs within a group.
// This is the correctness baseline; cross-group/layer fusion is separate.
module lds_net_mac_group(
    input wire clk,rst_n,start,cancel,new_tensor,
    input wire [31:0] source_base,destination_base,
    input wire [15:0] source_row_bytes,destination_row_bytes,
    input wire [9:0] width,height,output_width,output_height,first_output_row,output_rows,
    input wire [7:0] input_channels,output_channels,channel_start,
    input wire [5:0] channels,input wire [1:0] input_mode,
    input wire input_signed,output_a8,kernel3,stride2,padding,w8,
    input wire signed [15:0] input_zero,minimum,maximum,
    input wire [1023:0] biases,multipliers,input wire [1151:0] initial_codes,input wire [191:0] shifts,
    input wire program_we,input wire [3:0] program_group,input wire [9:0] program_address,input wire [15:0] program_data,
    input wire weight_we,input wire [4:0] weight_address,input wire [31:0] weight_data,
    output wire ready,output reg done,error,
    output wire [31:0] araddr,output wire [7:0] arlen,output wire arvalid,input wire arready,
    input wire [63:0] rdata,input wire [1:0] rresp,input wire rlast,rvalid,output wire rready,
    output wire [31:0] awaddr,output wire [7:0] awlen,output wire awvalid,input wire awready,
    output wire [63:0] wdata,output wire [7:0] wstrb,output wire wlast,wvalid,input wire wready,
    input wire [1:0] bresp,input wire bvalid,output wire bready,
    output reg [31:0] tiles_completed,source_bytes,destination_bytes,cycles);
    localparam IDLE=0,ROW_START=1,ROW_WAIT=2,WINDOW_START=3,WINDOW_WAIT=4,
        STORE_START=5,TILE_START=6,TILE_WAIT=7,STORE_WAIT=8,NEXT_TILE=9,FINISH=10,ABORT=11,PREFETCH_WAIT=12;
    reg [3:0] state;reg cancelled,initial_row;
    reg [9:0] x,y,k;
    reg active_bank,fill_bank,prefetched,window_go;
    reg [10:0] reduction;wire [10:0] reduction_input=input_channels*(kernel3?9:1);
    wire last_chunk={1'b0,k}+11'd128>=reduction;
    wire aborting=cancelled || cancel || state==ABORT;
    // A distinct inactive half of each SDP window RAM is written while DSPs
    // consume the current half. Prefetch stays within an already cached row.
    wire prefetch_next=!w8 && (!last_chunk || x+4<output_width);
    wire prefetch_start=state==TILE_START && prefetch_next && !aborting;
    reg [9:0] fill_k,fill_x;
    reg [7:0] fill_count;
    wire [9:0] requested_k=prefetch_start?(last_chunk?10'd0:k+10'd128):k;
    wire [10:0] requested_remaining=reduction-{1'b0,requested_k};
    wire row_ready,row_done,row_error,clear,load_we,load_ready,commit;
    wire [1:0] load_slot;wire [8:0] load_address,row_tag;wire [63:0] load_data;
    wire [31:0] row_misses,row_hits,row_bytes;
    lds_net_row_loader row_loader(.clk(clk),.rst_n(rst_n),.start(state==ROW_START && !aborting),.cancel(aborting),
        .new_tensor(new_tensor && initial_row),.base_address(source_base),.row_bytes(source_row_bytes),
        .width(width),.height(height),.output_y(y),.channels(input_channels),.mode(input_mode),.signed_codes(input_signed),
        .kernel3(kernel3),.stride2(stride2),.padding(padding),.ready(row_ready),.done(row_done),.error(row_error),
        .clear_rows(clear),.load_we(load_we),.load_ready(load_ready),.load_slot(load_slot),.load_address(load_address),
        .load_data(load_data),.row_commit(commit),.row_tag(row_tag),.araddr(araddr),.arlen(arlen),.arvalid(arvalid),
        .arready(arready),.rdata(rdata),.rresp(rresp),.rlast(rlast),.rvalid(rvalid),.rready(rready),
        .rows_read(row_misses),.cache_hits(row_hits),.bytes_read(row_bytes));
    wire window_ready,window_done,window_error,window_valid;
    wire [6:0] window_address;wire [23:0] window6;wire [31:0] window8;
    lds_net_window window(.clk(clk),.rst_n(rst_n),.clear_rows(clear),.load_we(load_we),.load_ready(load_ready),
        .load_slot(load_slot),.load_address(load_address),.load_data(load_data),.row_commit(commit),.row_tag(row_tag),
        .start(window_go && !aborting),.width(width),.height(height),.channels(input_channels),
        .kernel3(kernel3),.stride2(stride2),.padding(padding),.output_x(fill_x),.output_y(y),.k_start(fill_k),.k_count(fill_count),
        .input_zero(input_zero),.ready(window_ready),.done(window_done),.error(window_error),
        .out_valid(window_valid),.out_ready(1'b1),.out_address(window_address),.out_a6(window6),.out_a8(window8));
    wire w4_ready,w4_done,w4_error,w4_valid;wire [15:0] w4_data;wire [1:0] w4_keep,w4_pixel;wire [4:0] w4_channel;
    wire w8_ready,w8_done,w8_error,w8_valid;wire [15:0] w8_data;wire [1:0] w8_keep,w8_pixel;wire [4:0] w8_channel;
    wire tile_ready=w8?w8_ready:w4_ready,tile_done=w8?w8_done:w4_done,tile_error=w8?w8_error:w4_error;
    wire tile_valid=w8?w8_valid:w4_valid;wire [15:0] tile_data=w8?w8_data:w4_data;
    wire [1:0] tile_keep=w8?w8_keep:w4_keep,tile_pixel=w8?w8_pixel:w4_pixel;
    wire [4:0] tile_channel=w8?w8_channel:w4_channel;
    wire store_ready,store_done,store_error,store_input_ready;
    wire tile_output_ready=aborting || store_input_ready;
    lds_net_mac_tile w4_engine(.clk(clk),.rst_n(rst_n),.start(state==TILE_START && !w8 && !aborting),
        .first_chunk(k==0),.last_chunk(last_chunk),.signed_activation(input_signed),.active_window(active_bank),
        .initial_codes(initial_codes),.multipliers(multipliers),.shifts(shifts),.channels(channels),.minimum(minimum),.maximum(maximum),
        .window_we(window_valid && !w8 && !aborting),.window_bank(fill_bank),.window_address(window_address),.window_data(window6),
        .program_we(program_we && ready),.program_group(program_group),.program_address(program_address),.program_data(program_data),
        .ready(w4_ready),.done(w4_done),.error(w4_error),.out_valid(w4_valid),.out_ready(tile_output_ready),
        .out_data(w4_data),.out_keep(w4_keep),.out_pixel(w4_pixel),.out_channel(w4_channel));
    lds_net_w8_tile w8_engine(.clk(clk),.rst_n(rst_n),.start(state==TILE_START && w8 && !aborting),
        .active_bank(1'b0),.reduction(reduction[5:0]),.channels(channels[2:0]),.biases(biases[127:0]),
        .multipliers(multipliers[127:0]),.shifts(shifts[23:0]),.minimum(minimum),.maximum(maximum),
        .window_we(window_valid && w8 && !aborting),.window_address(window_address[4:0]),.window_data(window8),
        .weight_we(weight_we && ready),.weight_address(weight_address),.weight_data(weight_data),.load_bank(1'b0),
        .ready(w8_ready),.done(w8_done),.error(w8_error),.out_valid(w8_valid),.out_ready(tile_output_ready),
        .out_data(w8_data),.out_keep(w8_keep),.out_pixel(w8_pixel),.out_channel(w8_channel));
    wire [31:0] stored_bytes;
    lds_net_tile_store #(.MAX_CHANNELS(32)) store(.clk(clk),.rst_n(rst_n),.start(state==STORE_START && !aborting),.cancel(aborting),
        .base_address(destination_base),.row_bytes(destination_row_bytes),.width(output_width),.height(output_height),
        .output_x(x),.output_y(y),.total_channels(output_channels),.channel_start(channel_start),.channels(channels),.a8(output_a8),
        .ready(store_ready),.done(store_done),.error(store_error),.in_data(tile_data),.in_keep(tile_keep),.in_pixel(tile_pixel),
        .in_channel(tile_channel),.in_valid(tile_valid && !aborting),.in_ready(store_input_ready),
        .awaddr(awaddr),.awlen(awlen),.awvalid(awvalid),.awready(awready),.wdata(wdata),.wstrb(wstrb),
        .wlast(wlast),.wvalid(wvalid),.wready(wready),.bresp(bresp),.bvalid(bvalid),.bready(bready),.bytes_written(stored_bytes));
    assign ready=state==IDLE;
    always @(posedge clk)begin
        if(!rst_n)begin
            state<=IDLE;done<=0;error<=0;cancelled<=0;initial_row<=0;x<=0;y<=0;k<=0;
            tiles_completed<=0;source_bytes<=0;destination_bytes<=0;cycles<=0;
            active_bank<=0;fill_bank<=0;prefetched<=0;window_go<=0;reduction<=0;fill_k<=0;fill_x<=0;fill_count<=0;
        end else begin
            done<=0;window_go<=0;if(!ready)cycles<=cycles+1;
            if((state==WINDOW_START && window_ready || prefetch_start) && !aborting)begin
                fill_k<=requested_k;fill_x<=prefetch_start && last_chunk?x+10'd4:x;
                fill_count<=requested_remaining>128?8'd128:requested_remaining[7:0];window_go<=1;
            end
            if(prefetch_start)begin fill_bank<=!active_bank;prefetched<=0;end
            if(window_done && state!=WINDOW_WAIT)prefetched<=1;
            if(cancel && state!=IDLE && state!=ABORT && state!=FINISH)begin cancelled<=1;error<=1;state<=ABORT;end
            else if(cancelled && state!=IDLE && state!=FINISH && state!=ABORT)state<=ABORT;
            else if((store_done && store_error || window_done && window_error) && state!=IDLE && state!=ABORT && state!=FINISH)begin error<=1;state<=ABORT;end
            else case(state)
                IDLE:if(start)begin
                    cancelled<=0;error<=0;tiles_completed<=0;source_bytes<=0;destination_bytes<=0;cycles<=0;
                    x<=0;y<=first_output_row;k<=0;initial_row<=1;
                    reduction<=reduction_input;
                    active_bank<=0;fill_bank<=0;prefetched<=0;
                    if(cancel || channels==0 || channels>(w8?4:32) || reduction_input==0 || reduction_input>1024 ||
                       (w8 && reduction_input>32) || output_rows==0 || first_output_row+{1'b0,output_rows}>output_height ||
                       (w8 && input_mode==0) || (!w8 && input_mode!=0))begin error<=1;state<=FINISH;end
                    else state<=ROW_START;
                end
                ROW_START:if(row_ready)state<=ROW_WAIT;
                ROW_WAIT:if(row_done)begin
                    source_bytes<=source_bytes+row_bytes;initial_row<=0;
                    if(row_error)begin error<=1;state<=ABORT;end else state<=WINDOW_START;
                end
                WINDOW_START:if(window_ready)state<=WINDOW_WAIT;
                WINDOW_WAIT:if(window_done)begin
                    if(window_error)begin error<=1;state<=ABORT;end
                    else if(last_chunk)state<=STORE_START;
                    else state<=TILE_START;
                end
                STORE_START:if(store_ready)state<=TILE_START;
                TILE_START:if(tile_ready)state<=TILE_WAIT;
                TILE_WAIT:if(tile_done)begin
                    if(tile_error)begin error<=1;state<=ABORT;end
                    else if(last_chunk)state<=STORE_WAIT;
                    else begin
                        k<=k+10'd128;active_bank<=!active_bank;
                        if(prefetched || window_done)begin
                            if(k+256>=reduction)state<=STORE_START;else state<=TILE_START;
                        end else state<=PREFETCH_WAIT;
                    end
                end
                STORE_WAIT:if(store_done)begin
                    destination_bytes<=destination_bytes+stored_bytes;
                    if(store_error)begin error<=1;state<=ABORT;end else state<=NEXT_TILE;
                end
                NEXT_TILE:begin
                    tiles_completed<=tiles_completed+1;k<=0;
                    if(x+4<output_width)begin
                        x<=x+4;
                        if(w8)state<=WINDOW_START;
                        else begin active_bank<=!active_bank;
                            if(prefetched || window_done)begin
                                if(reduction<=128)state<=STORE_START;else state<=TILE_START;
                            end else state<=PREFETCH_WAIT;
                        end
                    end
                    else if(y+1<first_output_row+output_rows)begin x<=0;y<=y+1;fill_bank<=active_bank;prefetched<=0;state<=ROW_START;end
                    else state<=FINISH;
                end
                PREFETCH_WAIT:if(prefetched || window_done)begin
                    if(last_chunk)state<=STORE_START;else state<=TILE_START;
                end
                ABORT:if(row_ready && window_ready && w4_ready && w8_ready && store_ready)state<=FINISH;
                FINISH:begin state<=IDLE;done<=1;end
                default:begin error<=1;state<=ABORT;end
            endcase
        end
    end
endmodule
