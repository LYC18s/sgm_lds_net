`timescale 1ns/1ps
// Video-domain ownership and row lookahead. Cache banks rotate every four
// lines. After rows476..479 are consumed they immediately refill rows0..3 for
// the next display refresh. Pending results use the other four physical banks.
// Swaps/clear ACKs require a real accepted SOF AND no outstanding DDR read.
module lds_net_mask_prefetch #(parameter PANEL_Y=0)(
    input wire clk,rst_n,
    input wire pixel_fire,pixel_sof,input wire [10:0] pixel_x,pixel_y,
    input wire submit_valid,output wire submit_ready,input wire submit_slot,
    input wire [31:0] submit_frame,submit_base,input wire clear,
    output reg committed,clear_committed,output reg committed_slot,output reg [31:0] committed_frame,
    output reg active_valid,active_group,active_slot,output reg [31:0] active_frame,
    output reg pending,pending_error,output reg [31:0] cache_failures,
    output wire request_valid,input wire request_ready,output wire request_group,
    output wire [8:0] request_row,output wire [31:0] request_base,request_frame,
    input wire request_done,request_error);
    wire [10:0] image_y=pixel_y-PANEL_Y;
    reg [31:0] active_base,pending_base,pending_frame;
    reg pending_group,pending_slot,clear_pending,clear_seen,job_busy,job_pending;
    reg [1:0] job_bank;
    reg [3:0] active_needed,pending_loaded;
    reg [8:0] active_rows[0:3];
    reg [1:0] active_pick,pending_pick;
    integer i;
    always @*begin
        active_pick=3;pending_pick=3;
        for(i=3;i>=0;i=i-1)begin
            if(active_needed[i])active_pick=i;
            if(!pending_loaded[i])pending_pick=i;
        end
    end
    wire choose_active=active_valid && |active_needed;
    wire choose_pending=pending && !pending_error && pending_loaded!=15;
    wire sof=pixel_fire && pixel_sof;
    assign submit_ready=!pending && !clear_pending && !clear;
    assign request_valid=!job_busy && !clear_pending && !clear && !sof && (choose_active || choose_pending);
    assign request_group=choose_active?active_group:pending_group;
    assign request_row=choose_active?active_rows[active_pick]:{7'd0,pending_pick};
    assign request_base=choose_active?active_base:pending_base;
    assign request_frame=choose_active?active_frame:pending_frame;
    wire accept=request_valid && request_ready;
    wire retire_safe=!job_busy && request_ready;
    integer r;
    always @(posedge clk)begin
        if(!rst_n)begin
            committed<=0;clear_committed<=0;committed_slot<=0;committed_frame<=0;
            active_valid<=0;active_group<=0;active_slot<=0;active_frame<=0;active_base<=0;
            pending<=0;pending_error<=0;pending_group<=1;pending_slot<=0;pending_frame<=0;pending_base<=0;
            clear_pending<=0;clear_seen<=0;job_busy<=0;job_pending<=0;job_bank<=0;
            active_needed<=0;pending_loaded<=0;cache_failures<=0;
            for(r=0;r<4;r=r+1)active_rows[r]<=r;
        end else begin
            committed<=0;clear_committed<=0;
            if(!clear)clear_seen<=0;
            if(clear && !clear_seen)begin clear_pending<=1;clear_seen<=1;end
            if(submit_valid && submit_ready)begin
                pending<=1;pending_error<=0;pending_group<=!active_group;
                pending_slot<=submit_slot;pending_frame<=submit_frame;pending_base<=submit_base;pending_loaded<=0;
            end
            if(accept)begin
                job_busy<=1;job_pending<=!choose_active;job_bank<=choose_active?active_pick:pending_pick;
                if(choose_active)active_needed[active_pick]<=0;
            end
            if(job_busy && request_done)begin
                job_busy<=0;
                if(request_error)begin cache_failures<=cache_failures+1;if(job_pending)pending_error<=1;end
                else if(job_pending)pending_loaded[job_bank]<=1;
            end
            if(pixel_fire && active_valid && pixel_x==559 && pixel_y>=PANEL_Y && pixel_y<PANEL_Y+480 && !clear_pending)begin
                active_needed[image_y[1:0]]<=1;
                active_rows[image_y[1:0]]<=image_y<476?image_y+4:image_y-476;
            end
            if(sof && retire_safe)begin
                if(clear_pending || (clear && !clear_seen))begin
                    active_valid<=0;pending<=0;pending_error<=0;active_needed<=0;pending_loaded<=0;
                    clear_pending<=0;clear_committed<=1;
                end else if(pending && !pending_error && pending_loaded==15)begin
                    active_valid<=1;active_group<=pending_group;active_slot<=pending_slot;
                    active_frame<=pending_frame;active_base<=pending_base;active_needed<=0;pending<=0;
                    committed<=1;committed_slot<=pending_slot;committed_frame<=pending_frame;
                end
            end
        end
    end
endmodule
