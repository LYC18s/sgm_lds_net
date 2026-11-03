`timescale 1ns/1ps
// Two-stage elastic RGB overlay. All video state advances only on accepted
// pixels. Shadow boxes belong to the submitted mask/frame and are copied only
// after its first four rows are cached and a real video SOF retires old reads.
module lds_net_display_core #(parameter PANEL_Y=0)(
    input wire nn_clk,nn_rst_n,video_clk,video_rst_n,
    input wire box_write,input wire [4:0] box_address,input wire [31:0] box_data,input wire [3:0] box_strobe,
    input wire count_write,input wire [4:0] box_count,output wire shadow_locked,
    input wire submit_valid,output wire submit_ready,input wire submit_slot,
    input wire [31:0] submit_frame,submit_base,input wire clear,
    output wire committed,clear_committed,committed_slot,output wire [31:0] committed_frame,
    output wire active_valid,active_slot,output wire [31:0] active_frame,
    output wire pending,pending_error,output wire [31:0] cache_failures,read_errors,mask_misses,
    input wire [23:0] s_data,input wire [2:0] s_keep,input wire s_sof,s_eol,s_valid,output wire s_ready,
    output reg [23:0] m_data,output reg [2:0] m_keep,output reg m_sof,m_eol,m_valid,input wire m_ready,
    output wire [31:0] araddr,output wire [7:0] arlen,output wire arvalid,input wire arready,
    input wire [63:0] rdata,input wire [1:0] rresp,input wire rlast,rvalid,output wire rready);
    reg [23:0] in_data;reg [2:0] in_keep;reg in_eol,in_valid;
    reg in_sof;
    reg [10:0] x,y,in_x,in_y,in_image_y;reg in_panel;
    wire [10:0] accept_x=s_sof?11'd0:x,accept_y=s_sof?11'd0:y;
    always @(posedge video_clk)begin
        if(!video_rst_n)begin
            in_valid<=0;in_data<=0;in_keep<=0;in_sof<=0;in_eol<=0;
            x<=0;y<=0;in_x<=0;in_y<=0;in_image_y<=0;in_panel<=0;
        end else if(s_ready)begin
            in_valid<=s_valid;in_data<=s_data;in_keep<=s_keep;in_sof<=s_sof;in_eol<=s_eol;
            // Resolve SOF, panel offset and frame counters BEFORE the box
            // comparators. These coordinates stall with their input pixel.
            in_x<=accept_x;in_y<=accept_y;in_image_y<=accept_y-PANEL_Y;
            in_panel<=accept_y>=PANEL_Y && accept_y<PANEL_Y+480;
            if(s_valid)begin
                if(s_eol)begin x<=0;y<=accept_y==1079?11'd0:accept_y+1'b1;end
                else begin x<=accept_x+1'b1;y<=accept_y;end
            end
        end
    end
    reg [31:0] shadow[0:31];reg [4:0] shadow_count;
    reg [9:0] box_x1[0:15],box_x2[0:15],box_y1[0:15],box_y2[0:15];
    reg [10:0] inner_x1[0:15],inner_y1[0:15];
    reg [9:0] inner_x2[0:15],inner_y2[0:15];reg [15:0] box_enabled;
    wire [10:0] vx=in_x,vy=in_y;
    wire output_step=!m_valid || m_ready;
    reg stage_valid;wire step=!stage_valid || output_step;
    assign s_ready=video_rst_n && (!in_valid || step);
    wire fire=in_valid && step && video_rst_n;
    wire [10:0] image_y=in_image_y;
    wire roi=vx>=80 && vx<560 && in_panel;
    wire [8:0] roi_x=vx-80;
    wire active_group,request_valid,request_ready,request_group,request_done,request_error;
    wire [8:0] request_row;wire [31:0] request_base,request_frame;
    wire [63:0] cache_data;wire cache_valid;
    lds_net_mask_prefetch #(.PANEL_Y(PANEL_Y)) prefetch(.clk(video_clk),.rst_n(video_rst_n),.pixel_fire(fire),.pixel_sof(in_sof),.pixel_x(vx),.pixel_y(vy),
        .submit_valid(submit_valid),.submit_ready(submit_ready),.submit_slot(submit_slot),.submit_frame(submit_frame),.submit_base(submit_base),.clear(clear),
        .committed(committed),.clear_committed(clear_committed),.committed_slot(committed_slot),.committed_frame(committed_frame),
        .active_valid(active_valid),.active_group(active_group),.active_slot(active_slot),.active_frame(active_frame),.pending(pending),.pending_error(pending_error),.cache_failures(cache_failures),
        .request_valid(request_valid),.request_ready(request_ready),.request_group(request_group),.request_row(request_row),.request_base(request_base),.request_frame(request_frame),.request_done(request_done),.request_error(request_error));
    lds_net_mask_cache cache(.nn_clk(nn_clk),.nn_rst_n(nn_rst_n),.video_clk(video_clk),.video_rst_n(video_rst_n),
        .request_valid(request_valid),.request_ready(request_ready),.request_group(request_group),.request_row(request_row),.request_base(request_base),.request_frame(request_frame),.request_done(request_done),.request_error(request_error),
        .read_step(step),.read_pixel_valid(in_valid && roi && active_valid),.read_group(active_group),.read_row(image_y[8:0]),.read_word(roi_x[8:5]),.read_frame(active_frame),
        .read_data(cache_data),.read_valid(cache_valid),.araddr(araddr),.arlen(arlen),.arvalid(arvalid),.arready(arready),.rdata(rdata),.rresp(rresp),.rlast(rlast),.rvalid(rvalid),.rready(rready),.read_errors(read_errors));
    assign shadow_locked=!submit_ready;
    reg [23:0] stage_data;reg [2:0] stage_keep;reg stage_sof,stage_eol,stage_roi,stage_first;
    reg [3:0] stage_hits;
    reg [4:0] stage_label;
    reg [31:0] misses;assign mask_misses=misses;
    reg [3:0] hits;integer i;
    always @*begin
        hits=0;
        for(i=0;i<16;i=i+1)begin
            if(box_enabled[i] && vx>=box_x1[i] && vx<=box_x2[i] && image_y>=box_y1[i] && image_y<=box_y2[i] &&
                (vx<inner_x1[i] || vx>=inner_x2[i] || image_y<inner_y1[i] || image_y>=inner_y2[i]))hits[i/4]=1;
        end
    end
    function [7:0] blend;
        input [7:0] source,color;reg [9:0] numerator;
        begin numerator={source,1'b0}+source+color+2;blend=numerator[9:2];end
    endfunction
    wire [1:0] label=cache_data[{stage_label,1'b0}+:2];
    reg [23:0] mixed;
    always @*begin
        mixed=stage_data;
        if(stage_roi && cache_valid)begin
            if(label==1)mixed={blend(stage_data[23:16],8'd0),blend(stage_data[15:8],8'd255),blend(stage_data[7:0],8'd0)};
            // HDMI rgb2dvi consumes {R,G,B}; alternative road is #CC9900.
            if(label==2)mixed={blend(stage_data[23:16],8'd204),blend(stage_data[15:8],8'd153),blend(stage_data[7:0],8'd0)};
        end
        if(|stage_hits)mixed=24'h00ff00;
    end
    integer j,b;
    always @(posedge video_clk)begin
        if(!video_rst_n)begin
            shadow_count<=0;box_enabled<=0;misses<=0;
            stage_valid<=0;stage_data<=0;stage_keep<=0;stage_sof<=0;stage_eol<=0;stage_roi<=0;stage_first<=0;stage_hits<=0;stage_label<=0;
            m_valid<=0;m_data<=0;m_keep<=0;m_sof<=0;m_eol<=0;
            for(j=0;j<32;j=j+1)shadow[j]<=0;
            for(j=0;j<16;j=j+1)begin
                box_x1[j]<=0;box_x2[j]<=0;box_y1[j]<=0;box_y2[j]<=0;
                inner_x1[j]<=0;inner_x2[j]<=0;inner_y1[j]<=0;inner_y2[j]<=0;
            end
        end else begin
            if(!shadow_locked)begin
                if(count_write && box_count<=16)shadow_count<=box_count;
                if(box_write)for(b=0;b<4;b=b+1)if(box_strobe[b])shadow[box_address][b*8+:8]<=box_data[b*8+:8];
            end
            // Per-box validity and all +/-2 border arithmetic are calculated
            // once on commit, rather than sixteen times on every pixel.
            if(committed)for(j=0;j<16;j=j+1)begin
                box_x1[j]<=shadow[j*2][9:0];box_y1[j]<=shadow[j*2][25:16];
                box_x2[j]<=shadow[j*2+1][9:0];box_y2[j]<=shadow[j*2+1][25:16];
                inner_x1[j]<={1'b0,shadow[j*2][9:0]}+11'd2;
                inner_y1[j]<={1'b0,shadow[j*2][25:16]}+11'd2;
                inner_x2[j]<=shadow[j*2+1][9:0]-10'd1;
                inner_y2[j]<=shadow[j*2+1][25:16]-10'd1;
                box_enabled[j]<=j<shadow_count && shadow[j*2+1][9:0]>shadow[j*2][9:0] && shadow[j*2+1][25:16]>shadow[j*2][25:16];
            end
            if(clear_committed)box_enabled<=0;
            if(output_step)begin
                m_valid<=stage_valid;m_data<=mixed;m_keep<=stage_keep;m_sof<=stage_sof;m_eol<=stage_eol;
                if(stage_valid && stage_roi && stage_first && !cache_valid)misses<=misses+1;
            end
            if(step)begin
                stage_valid<=in_valid;stage_data<=in_data;stage_keep<=in_keep;stage_sof<=in_sof;stage_eol<=in_eol;
                stage_roi<=roi && active_valid;stage_first<=vx==80;stage_label<=roi_x[4:0];
                stage_hits<=hits & {4{active_valid && in_panel}};
            end
        end
    end
endmodule
