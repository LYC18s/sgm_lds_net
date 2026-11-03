`timescale 1ns/1ps
// Refill only missing convolution rows. Tags persist across horizontal tiles
// and K chunks; modulo-three placement reuses the two overlapping rows.
// The AXI reader drains issued traffic on cancel/error before reporting done.
module lds_net_row_loader(
    input wire clk,rst_n,start,cancel,new_tensor,
    input wire [31:0] base_address,
    input wire [15:0] row_bytes,
    input wire [9:0] width,height,output_y,
    input wire [7:0] channels,
    input wire [1:0] mode,input wire signed_codes,kernel3,stride2,padding,
    output wire ready,output reg done,error,
    output wire clear_rows,load_we,input wire load_ready,
    output wire [1:0] load_slot,output wire [8:0] load_address,
    output wire [63:0] load_data,output wire row_commit,output wire [8:0] row_tag,
    output wire [31:0] araddr,output wire [7:0] arlen,
    output wire arvalid,input wire arready,
    input wire [63:0] rdata,input wire [1:0] rresp,input wire rlast,rvalid,output wire rready,
    output reg [31:0] rows_read,cache_hits,bytes_read);
    localparam IDLE=0,CLEAR=1,SCAN=2,REQUEST=3,STREAM=4,COMMIT=5,FINISH=6,VALIDATE=7;
    reg [2:0] state;
    reg [31:0] base,saved_base;reg [15:0] stride,saved_stride;
    reg [9:0] h,saved_width,saved_height;reg [7:0] saved_channels;
    reg [1:0] encoding,saved_mode;reg signed_q,saved_signed;
    reg [8:0] groups,offset;reg [1:0] row_index,row_count;
    reg signed [12:0] first_row;
    reg [2:0] valid_rows;reg [8:0] tags[0:2];
    reg cache_configured,cancelled,read_seen,unpack_seen;
    reg [25:0] tensor_bytes_q;reg shape_ok_q,spatial_ok_q,config_change_q,new_tensor_q;
    reg [31:0] payload_bytes_q;
    wire signed [12:0] y=first_row+$signed({11'd0,row_index});
    wire outside=y<0 || y>=$signed({3'd0,h});
    // 4 == 1 (mod3). Fold base-four digits instead of a signed divider:
    // the general y%3 inferred a 26-level path through the tag lookup.
    // Only nonnegative y<512 is consulted; padding rows are skipped.
    function [1:0] modulo3;
        input [8:0] value;reg [3:0] sum;
        begin
            sum={2'd0,value[1:0]}+{2'd0,value[3:2]}+{2'd0,value[5:4]}+{2'd0,value[7:6]}+{3'd0,value[8]};
            modulo3=sum>=12?sum-12:sum>=9?sum-9:sum>=6?sum-6:sum>=3?sum-3:sum;
        end
    endfunction
    wire [1:0] slot=modulo3(y[8:0]);
    wire hit=!outside && valid_rows[slot] && tags[slot]==y;
    wire config_change=!cache_configured || base_address!=saved_base || row_bytes!=saved_stride ||
        width!=saved_width || height!=saved_height || channels!=saved_channels ||
        mode!=saved_mode || signed_codes!=saved_signed;
    wire [15:0] requested_groups=width*((channels+16'd7)>>3);
    wire [31:0] payload_bytes=mode==0?requested_groups*6:mode==1?requested_groups*8:width*3;
    wire [32:0] tensor_end={1'b0,base}+tensor_bytes_q;
    wire parameters_ok=shape_ok_q && spatial_ok_q && groups<=320 &&
        stride>=payload_bytes_q && tensor_end<=33'h100000000;
    wire dma_ready,dma_done,dma_error,dma_busy,stream_valid,stream_ready,stream_last;
    wire [63:0] stream_data;wire [31:0] dma_bytes,unused_stalls;
    wire unpack_ready,unpack_done,unpack_error,unpack_valid,unpack_last;
    wire issue=state==REQUEST && dma_ready && unpack_ready && load_ready && !cancel && !cancelled;
    wire failed=cancelled || cancel || error || dma_error || unpack_error;
    lds_net_axi_read reader(.clk(clk),.rst_n(rst_n),.command_valid(issue),.command_ready(dma_ready),
        .command_address(base+y*stride),.command_bytes({8'd0,stride}),.done(dma_done),.error(dma_error),.busy(dma_busy),
        .data(stream_data),.valid(stream_valid),.ready(stream_ready),.last(stream_last),
        .araddr(araddr),.arlen(arlen),.arvalid(arvalid),.arready(arready),.rdata(rdata),.rresp(rresp),
        .rlast(rlast),.rvalid(rvalid),.rready(rready),.bytes_read(dma_bytes),.stall_cycles(unused_stalls));
    lds_net_feature_unpack unpacker(.clk(clk),.rst_n(rst_n),.start(issue),.mode(encoding),.signed_codes(signed_q),
        .groups(groups),.in_data(stream_data),.in_valid(stream_valid),.in_last(stream_last),.in_ready(stream_ready),
        .out_data(load_data),.out_valid(unpack_valid),.out_ready(failed || load_ready),.out_last(unpack_last),
        .ready(unpack_ready),.done(unpack_done),.error(unpack_error));
    assign ready=state==IDLE;
    assign clear_rows=state==CLEAR && load_ready;
    assign load_we=state==STREAM && unpack_valid && !failed;
    assign load_slot=slot;assign load_address=offset;assign row_tag=y[8:0];
    assign row_commit=state==COMMIT && load_ready && !failed;
    integer i;
    always @(posedge clk) begin
        if(!rst_n) begin
            state<=IDLE;done<=0;error<=0;base<=0;stride<=0;h<=0;encoding<=0;signed_q<=0;groups<=0;
            row_index<=0;row_count<=0;first_row<=0;valid_rows<=0;offset<=0;cache_configured<=0;
            cancelled<=0;read_seen<=0;unpack_seen<=0;rows_read<=0;cache_hits<=0;bytes_read<=0;
            tensor_bytes_q<=0;shape_ok_q<=0;spatial_ok_q<=0;config_change_q<=0;new_tensor_q<=0;payload_bytes_q<=0;
            saved_base<=0;saved_stride<=0;saved_width<=0;saved_height<=0;saved_channels<=0;saved_mode<=0;saved_signed<=0;
            for(i=0;i<3;i=i+1) tags[i]<=0;
        end else begin
            done<=0;
            if(cancel && state!=IDLE) begin cancelled<=1;error<=1;end
            case(state)
                IDLE:if(start) begin
                    error<=0;cancelled<=0;rows_read<=0;cache_hits<=0;bytes_read<=0;
                    base<=base_address;stride<=row_bytes;h<=height;encoding<=mode;signed_q<=signed_codes;
                    groups<=mode==2?width:requested_groups;row_index<=0;row_count<=kernel3?3:1;
                    first_row<=$signed({3'd0,output_y}<<(stride2?1:0))-$signed({12'd0,padding});
                    saved_base<=base_address;saved_stride<=row_bytes;saved_width<=width;saved_height<=height;
                    saved_channels<=channels;saved_mode<=mode;saved_signed<=signed_codes;
                    tensor_bytes_q<=height*row_bytes;payload_bytes_q<=payload_bytes;
                    shape_ok_q<=width!=0 && height!=0 && height<=512 && channels!=0 &&
                        requested_groups<=320 && mode<=2 && (mode!=2 || channels==3) &&
                        row_bytes[5:0]==0 && base_address[5:0]==0;
                    spatial_ok_q<=output_y*(stride2?2:1)<height+padding;
                    config_change_q<=config_change;new_tensor_q<=new_tensor;state<=VALIDATE;
                end
                VALIDATE:begin
                    if(!parameters_ok || cancel || cancelled) begin state<=FINISH;error<=1;cache_configured<=0;valid_rows<=0;end
                    else begin
                        cache_configured<=1;
                        if(new_tensor_q || config_change_q) begin valid_rows<=0;state<=CLEAR;end
                        else state<=SCAN;
                    end
                end
                CLEAR:if(load_ready) state<=cancel || cancelled?FINISH:SCAN;
                SCAN:begin
                    if(cancel || cancelled) state<=FINISH;
                    else if(row_index==row_count) state<=FINISH;
                    else if(outside || hit) begin
                        row_index<=row_index+1'b1;
                        if(hit) cache_hits<=cache_hits+1;
                    end else state<=REQUEST;
                end
                REQUEST:begin
                    if(cancel || cancelled) state<=FINISH;
                    else if(issue) begin
                        // A partial refill must invalidate the old slot even
                        // when cancellation arrives before the first word.
                        valid_rows[slot]<=0;read_seen<=0;unpack_seen<=0;offset<=0;state<=STREAM;
                    end
                end
                STREAM:begin
                    if(unpack_valid && (failed || load_ready)) offset<=offset+1'b1;
                    if(dma_done) read_seen<=1;
                    if(unpack_done) unpack_seen<=1;
                    if(dma_error || unpack_error) error<=1;
                    if((read_seen || dma_done) && (unpack_seen || unpack_done)) begin
                        bytes_read<=bytes_read+dma_bytes;
                        if(failed) state<=FINISH;
                        else state<=COMMIT;
                    end
                end
                COMMIT:if(cancel || cancelled) state<=FINISH;
                    else if(load_ready) begin
                        valid_rows[slot]<=1;tags[slot]<=y[8:0];rows_read<=rows_read+1;
                        row_index<=row_index+1'b1;state<=SCAN;
                    end
                FINISH:begin state<=IDLE;done<=1;end
                default:begin state<=FINISH;error<=1;end
            endcase
        end
    end
endmodule
