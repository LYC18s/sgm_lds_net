`timescale 1ns/1ps
// Own all two-slot lifetimes. Capture, inference, mask restore and native PS
// detection may overlap; no DMA slot is reused while any consumer owns it.
// Box+mask publication is acknowledged only by the display's actual SOF swap.
module lds_net_frame_manager(
    input wire clk,rst_n,enable,abort_all,
    input wire [63:0] input_bases,packet_bases,mask_bases,
    output wire configuration_idle,
    output wire capture_valid,input wire capture_ready,output wire [31:0] capture_address,
    input wire capture_done,capture_error,input wire [31:0] capture_frame,
    output wire network_valid,input wire network_ready,output wire [31:0] network_input,network_packet,network_frame,
    input wire network_done,network_failed,
    output wire mask_valid,input wire mask_ready,output wire [31:0] mask_logits,mask_destination,
    input wire mask_done,mask_error,
    output wire cancel_capture,cancel_network,cancel_mask,
    output wire result_valid,output wire result_slot,output wire [31:0] result_frame,result_packet,result_mask,
    input wire result_take,
    input wire host_discard,host_slot,input wire [31:0] host_frame,
    input wire display_submit,display_slot,input wire [31:0] display_frame,
    output wire display_submit_ready,output reg protocol_error,
    input wire display_committed,display_clear_committed,input wire committed_slot,input wire [31:0] committed_frame,
    output reg active_valid,active_slot,output reg [31:0] active_frame,
    output reg [31:0] captured_frames,inferred_frames,restored_frames,failed_jobs,
    output wire [3:0] input_states,output wire [5:0] output_states,
    input wire color_enable,output wire [31:0] host_rgb_address);
    localparam IFREE=0,ICAPTURE=1,IREADY=2,IREADING=3;
    localparam OFREE=0,ONETWORK=1,OMASK_WAIT=2,OMASK_BUSY=3,ORESULT=4,OHOST=5,OPENDING=6,OACTIVE=7;
    reg [1:0] input_state[0:1],next_input[0:1];
    reg [2:0] output_state[0:1],next_output[0:1];
    reg [31:0] input_frame[0:1],output_frame[0:1];
    reg output_input[0:1];
    reg capture_busy,network_busy,mask_busy,cap_slot,net_input_slot,net_output_slot,mask_slot;
    reg draining;
    wire free_input0=input_state[0]==IFREE,free_input1=input_state[1]==IFREE;
    wire free_output0=output_state[0]==OFREE,free_output1=output_state[1]==OFREE;
    wire input_ready0=input_state[0]==IREADY,input_ready1=input_state[1]==IREADY;
    wire choose_capture=free_input0?1'b0:1'b1;
    wire choose_input=input_ready0 && input_ready1?($signed(input_frame[0]-input_frame[1])<0?1'b0:1'b1):input_ready0?1'b0:1'b1;
    wire choose_output=free_output0?1'b0:1'b1;
    wire mask_wait0=output_state[0]==OMASK_WAIT,mask_wait1=output_state[1]==OMASK_WAIT;
    wire choose_mask=mask_wait0 && mask_wait1?($signed(output_frame[0]-output_frame[1])<0?1'b0:1'b1):mask_wait0?1'b0:1'b1;
    wire result0=output_state[0]==ORESULT,result1=output_state[1]==ORESULT;
    wire choose_result=result0 && result1?($signed(output_frame[0]-output_frame[1])<0?1'b0:1'b1):result0?1'b0:1'b1;
    wire pending=output_state[0]==OPENDING || output_state[1]==OPENDING;
    wire stopping=draining || abort_all;
    assign capture_valid=enable && !stopping && !capture_busy && (free_input0 || free_input1);
    assign capture_address=input_bases[choose_capture*32+:32];
    assign network_valid=enable && !stopping && !network_busy && (input_ready0 || input_ready1) && (free_output0 || free_output1);
    assign network_input=input_bases[choose_input*32+:32];assign network_packet=packet_bases[choose_output*32+:32];
    assign network_frame=input_frame[choose_input];
    assign mask_valid=!stopping && !mask_busy && (mask_wait0 || mask_wait1);
    assign mask_logits=packet_bases[choose_mask*32+:32]+32'd7000;assign mask_destination=mask_bases[choose_mask*32+:32];
    assign cancel_capture=stopping;assign cancel_network=stopping;assign cancel_mask=stopping;
    assign result_valid=(result0 || result1) && !stopping;assign result_slot=choose_result;
    assign result_frame=output_frame[choose_result];assign result_packet=packet_bases[choose_result*32+:32];assign result_mask=mask_bases[choose_result*32+:32];
    // In color mode IREADING remains held after NN completion through HOST.
    // Output/input slot numbers are independent; retain their actual mapping.
    wire host_rgb_valid=color_enable && enable && !stopping &&
        output_state[host_slot]==OHOST && output_frame[host_slot]==host_frame &&
        input_state[output_input[host_slot]]==IREADING && input_frame[output_input[host_slot]]==host_frame;
    assign host_rgb_address=host_rgb_valid?input_bases[output_input[host_slot]*32+:32]:0;
    assign display_submit_ready=!stopping && !pending && output_state[display_slot]==OHOST && output_frame[display_slot]==display_frame;
    assign configuration_idle=!capture_busy && !network_busy && !mask_busy && !active_valid &&
        input_state[0]==IFREE && input_state[1]==IFREE && output_state[0]==OFREE && output_state[1]==OFREE;
    assign input_states={input_state[1],input_state[0]};assign output_states={output_state[1],output_state[0]};
    wire cap_accept=capture_valid && capture_ready,net_accept=network_valid && network_ready,mask_accept=mask_valid && mask_ready;
    wire net_complete=network_done || network_failed;
    wire [1:0] failure_count={1'b0,capture_busy && capture_done && (capture_error || stopping)}+
        {1'b0,network_busy && net_complete && (network_failed || stopping)}+
        {1'b0,mask_busy && mask_done && (mask_error || stopping)};
    integer i;
    always @*begin
        for(i=0;i<2;i=i+1)begin next_input[i]=input_state[i];next_output[i]=output_state[i];end
        // Complete/drain events precede ownership transfers. Eligibility above
        // uses registered states, so a just-released slot cannot race its DMA.
        if(capture_busy && capture_done)next_input[cap_slot]=capture_error || stopping?IFREE:IREADY;
        if(network_busy && net_complete)begin
            next_input[net_input_slot]=color_enable && !network_failed && !stopping?IREADING:IFREE;
            next_output[net_output_slot]=network_failed || stopping?OFREE:OMASK_WAIT;
        end
        if(mask_busy && mask_done)begin
            next_output[mask_slot]=mask_error || stopping?OFREE:ORESULT;
            if(color_enable && (mask_error || stopping))next_input[output_input[mask_slot]]=IFREE;
        end
        if(cap_accept)next_input[choose_capture]=ICAPTURE;
        if(net_accept)begin next_input[choose_input]=IREADING;next_output[choose_output]=ONETWORK;end
        if(mask_accept)next_output[choose_mask]=OMASK_BUSY;
        if(result_take && result_valid)next_output[choose_result]=OHOST;
        if(host_discard && output_state[host_slot]==OHOST && output_frame[host_slot]==host_frame)begin
            next_output[host_slot]=OFREE;
            if(color_enable)next_input[output_input[host_slot]]=IFREE;
        end
        if(display_submit && display_submit_ready)begin
            next_output[display_slot]=OPENDING;
            if(color_enable)next_input[output_input[display_slot]]=IFREE;
        end
        if(display_committed && output_state[committed_slot]==OPENDING && output_frame[committed_slot]==committed_frame)begin
            if(active_valid)next_output[active_slot]=OFREE;
            next_output[committed_slot]=OACTIVE;
        end
        if(display_clear_committed)begin
            if(active_valid)next_output[active_slot]=OFREE;
            for(i=0;i<2;i=i+1)if(output_state[i]==OPENDING)next_output[i]=OFREE;
        end
        if(!enable || stopping)begin
            for(i=0;i<2;i=i+1)begin
                if(input_state[i]==IREADY)next_input[i]=IFREE;
                if(input_state[i]==IREADING && (!network_busy || net_input_slot!=i))next_input[i]=IFREE;
            end
        end
        if(stopping)begin
            for(i=0;i<2;i=i+1)if(output_state[i]==OMASK_WAIT || output_state[i]==ORESULT)next_output[i]=OFREE;
        end
    end
    integer r;
    always @(posedge clk)begin
        if(!rst_n)begin
            for(r=0;r<2;r=r+1)begin input_state[r]<=IFREE;output_state[r]<=OFREE;input_frame[r]<=0;output_frame[r]<=0;output_input[r]<=0;end
            capture_busy<=0;network_busy<=0;mask_busy<=0;cap_slot<=0;net_input_slot<=0;net_output_slot<=0;mask_slot<=0;draining<=0;
            protocol_error<=0;active_valid<=0;active_slot<=0;active_frame<=0;captured_frames<=0;inferred_frames<=0;restored_frames<=0;failed_jobs<=0;
        end else begin
            for(r=0;r<2;r=r+1)begin input_state[r]<=next_input[r];output_state[r]<=next_output[r];end
            if(failure_count!=0)failed_jobs<=failed_jobs+failure_count;
            if(abort_all)draining<=1;
            else if(draining && !capture_busy && !network_busy && !mask_busy)draining<=0;
            if(cap_accept)begin capture_busy<=1;cap_slot<=choose_capture;end
            if(capture_busy && capture_done)begin
                capture_busy<=0;input_frame[cap_slot]<=capture_frame;
                if(!capture_error && !stopping)captured_frames<=captured_frames+1;
            end
            if(net_accept)begin
                network_busy<=1;net_input_slot<=choose_input;net_output_slot<=choose_output;output_frame[choose_output]<=input_frame[choose_input];
                output_input[choose_output]<=choose_input;
            end
            if(network_busy && net_complete)begin network_busy<=0;if(!network_failed && !stopping)inferred_frames<=inferred_frames+1;end
            if(mask_accept)begin mask_busy<=1;mask_slot<=choose_mask;end
            if(mask_busy && mask_done)begin mask_busy<=0;if(!mask_error && !stopping)restored_frames<=restored_frames+1;end
            if(display_submit && !display_submit_ready)protocol_error<=1;
            if(host_discard && (output_state[host_slot]!=OHOST || output_frame[host_slot]!=host_frame))protocol_error<=1;
            if(result_take && !result_valid)protocol_error<=1;
            if(display_committed)begin
                if(output_state[committed_slot]!=OPENDING || output_frame[committed_slot]!=committed_frame)protocol_error<=1;
                else begin active_valid<=1;active_slot<=committed_slot;active_frame<=committed_frame;end
            end
            if(display_clear_committed)active_valid<=0;
        end
    end
endmodule
