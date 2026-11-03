`timescale 1ns/1ps
// Display ABI524c4401. The PS prepares boxes and requests publication; the NN
// owner validates slot/frame/base BEFORE the video side starts prefetching.
// Conversely a slot is retired in NN only after the actual SOF event crosses
// back. No PS timing assumption substitutes for either ownership handshake.
module lds_net_display_control(
    input wire video_clk,video_rst_n,nn_clk,nn_rst_n,
    input wire write_enable,input wire [11:0] write_address,input wire [31:0] write_data,input wire [3:0] write_strobe,
    output reg write_reject,input wire [11:0] read_address,output reg [31:0] read_data,
    output wire box_write,output wire [4:0] box_address,output wire [31:0] box_data,output wire [3:0] box_strobe,
    output wire count_write,output wire [4:0] box_count,input wire shadow_locked,
    output wire submit_valid,input wire submit_ready,output wire submit_slot,output wire [31:0] submit_frame,submit_base,output wire clear,
    input wire committed,clear_committed,committed_slot,input wire [31:0] committed_frame,
    input wire active_valid,active_slot,input wire [31:0] active_frame,input wire pending_error,
    input wire [31:0] cache_failures,read_errors,mask_misses,
    output wire authorize_valid,input wire authorize_ready,output wire authorize_slot,output wire [31:0] authorize_frame,authorize_base,input wire authorize_accept,
    output wire event_valid,input wire event_ready,output wire event_clear,event_slot,output wire [31:0] event_frame);
    localparam IDLE=0,AUTHORIZE=1,SUBMIT=2,WAIT_EVENT=3;
    reg [1:0] state;reg auth_sent,clear_requested,clear_busy,rejected,event_overflow;
    reg [31:0] frame,base,commit_count;reg slot;reg [4:0] count;
    wire locked=state!=IDLE || clear_busy || shadow_locked;
    wire control_address=write_address==4;
    wire box_range=write_address>=12'h100 && write_address<12'h180;
    wire bev_range=write_address>=12'h200 && write_address<12'h240;
    wire config_address=write_address==12'h010 || write_address==12'h014 || write_address==12'h018 || write_address==12'h01c || box_range || bev_range;
    always @*begin
        write_reject=(|write_strobe) && (write_address[1:0]!=0 || !(control_address || config_address) || (config_address && locked) ||
            (control_address && write_strobe[0] && write_data[0] && (locked || base[5:0]!=0 || {1'b0,base}+33'd57600>33'h100000000)) ||
            (write_address==12'h010 && write_strobe[0] && write_data[7:0]>16));
        case(read_address)
            0:read_data=32'h524c4401;
            4:read_data={26'd0,active_valid,event_overflow,rejected,pending_error,clear_busy,state!=IDLE};
            8:read_data=active_frame;
            12:read_data={31'd0,active_slot};
            16:read_data={27'd0,count};
            20:read_data=frame;
            24:read_data=base;
            28:read_data={31'd0,slot};
            32:read_data=cache_failures;
            36:read_data=read_errors;
            40:read_data=mask_misses;
            44:read_data=commit_count;
            48:read_data=32'h42455631; // BEV1 extension, sixteen atomic sprite descriptors
            default:read_data=0;
        endcase
    end
    wire write_ok=write_enable && !write_reject;
    // Box writes do not depend on the unrelated commit base/extent adder.
    assign box_write=write_enable && box_range && |write_strobe && !locked && write_address[1:0]==0;
    assign box_address=write_address[6:2];assign box_data=write_data;assign box_strobe=write_strobe;
    assign count_write=write_ok && write_address==16 && write_strobe[0];assign box_count=write_data[4:0];
    wire auth_source_ready,auth_done;wire [0:0] auth_response;
    wire [64:0] auth_payload;
    lds_net_cdc_rpc #(.WIDTH(65)) authorization(.source_clk(video_clk),.source_rst_n(video_rst_n),.destination_clk(nn_clk),.destination_rst_n(nn_rst_n),
        .source_valid(state==AUTHORIZE && !auth_sent),.source_ready(auth_source_ready),.source_data({slot,frame,base}),.source_done(auth_done),.source_response(auth_response),
        .destination_valid(authorize_valid),.destination_ready(authorize_ready),.destination_data(auth_payload),.destination_response(authorize_accept));
    assign {authorize_slot,authorize_frame,authorize_base}=auth_payload;
    assign submit_valid=state==SUBMIT;assign submit_slot=slot;assign submit_frame=frame;assign submit_base=base;
    assign clear=clear_requested && (state==IDLE || state==WAIT_EVENT);
    reg event_pending,event_kind,event_slot_hold;reg [31:0] event_frame_hold;
    wire event_source_ready,event_done;wire [0:0] unused_response;wire [33:0] event_payload;
    reg event_sent_kind;
    lds_net_cdc_rpc #(.WIDTH(34)) events(.source_clk(video_clk),.source_rst_n(video_rst_n),.destination_clk(nn_clk),.destination_rst_n(nn_rst_n),
        .source_valid(event_pending),.source_ready(event_source_ready),.source_data({event_kind,event_slot_hold,event_frame_hold}),.source_done(event_done),.source_response(unused_response),
        .destination_valid(event_valid),.destination_ready(event_ready),.destination_data(event_payload),.destination_response(1'b0));
    assign {event_clear,event_slot,event_frame}=event_payload;
    function [31:0] merge;
        input [31:0] a,b;input [3:0] enables;integer j;
        begin for(j=0;j<4;j=j+1)merge[j*8+:8]=enables[j]?b[j*8+:8]:a[j*8+:8];end
    endfunction
    always @(posedge video_clk)begin
        if(!video_rst_n)begin
            state<=IDLE;auth_sent<=0;clear_requested<=1;clear_busy<=1;rejected<=0;event_overflow<=0;
            frame<=0;base<=0;slot<=0;count<=0;commit_count<=0;
            event_pending<=0;event_kind<=0;event_slot_hold<=0;event_frame_hold<=0;event_sent_kind<=0;
        end else begin
            if(write_ok)begin
                case(write_address)
                    4:if(write_strobe[0])begin
                        if(write_data[0])begin state<=AUTHORIZE;auth_sent<=0;rejected<=0;end
                        if(write_data[1])begin clear_requested<=1;clear_busy<=1;end
                        if(write_data[2])begin rejected<=0;event_overflow<=0;end
                    end
                    16:if(write_strobe[0])count<=write_data[4:0];
                    20:frame<=merge(frame,write_data,write_strobe);
                    24:base<=merge(base,write_data,write_strobe);
                    28:if(write_strobe[0])slot<=write_data[0];
                endcase
            end
            if(state==AUTHORIZE && !auth_sent && auth_source_ready)auth_sent<=1;
            if(state==AUTHORIZE && auth_done)begin
                if(auth_response[0])state<=SUBMIT;
                else begin state<=IDLE;rejected<=1;end
            end
            if(state==SUBMIT && submit_ready)state<=WAIT_EVENT;
            if(event_pending && event_source_ready)begin event_pending<=0;event_sent_kind<=event_kind;end
            if(committed || clear_committed)begin
                if(event_pending && !event_source_ready)event_overflow<=1;
                else begin event_pending<=1;event_kind<=clear_committed;event_slot_hold<=committed_slot;event_frame_hold<=committed_frame;end
                if(committed)commit_count<=commit_count+1;
                if(clear_committed)clear_requested<=0;
            end
            if(event_done)begin
                if(event_sent_kind)begin clear_busy<=0;state<=IDLE;end
                else if(state==WAIT_EVENT && !clear_busy)state<=IDLE;
            end
        end
    end
endmodule
