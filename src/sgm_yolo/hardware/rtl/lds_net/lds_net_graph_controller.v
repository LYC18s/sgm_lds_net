`timescale 1ns/1ps
// One launch per complete inference. Adapters must consume the full descriptor,
// run the selected operator, drain all DMA responses, then assert engine_done.
// This module alone is NOT a complete neural-network accelerator.
module lds_net_graph_controller #(
    parameter WATCHDOG_CYCLES=50000000
)(
    input wire clk,rst_n,launch_valid,output wire launch_ready,
    input wire [31:0] input_base,scratch_base,packet_base,input_frame,
    input wire abort,
    output wire command_valid,input wire command_ready,
    output wire [5:0] command_index,output wire [1023:0] descriptor,
    output wire [31:0] source0,source1,destination,
    input wire engine_done,engine_error,engine_idle,output wire cancel,
    output reg frame_done,frame_failed,output reg [31:0] error_code,
    output reg [31:0] result_frame,frame_cycles,
    output reg profile_valid,output reg [5:0] profile_node,output reg [31:0] profile_cycles);
    localparam IDLE=0,FETCH=1,ADDRESS=2,ISSUE=3,WAIT=4,DRAIN=5;
    reg [2:0] state;reg [5:0] index;
    reg [1023:0] descriptor_q;wire [1023:0] descriptor_rom;
    reg [31:0] source0_q,source1_q,destination_q;
    reg [31:0] input_q,scratch_q,packet_q,frame_q,node_cycles;
    wire [5:0] command_count;wire [31:0] scratch_bytes;wire [255:0] model_sha256;
    lds_net_graph_rom constants(index,descriptor_rom,command_count,scratch_bytes,model_sha256);
    assign descriptor=descriptor_q;
    assign command_index=index;
    assign source0=source0_q;
    assign source1=source1_q;
    assign destination=destination_q;
    assign launch_ready=rst_n && state==IDLE && engine_idle;
    assign command_valid=state==ISSUE && !abort;
    assign cancel=state==DRAIN;
    wire accepted=command_valid && command_ready;
    wire completed=(state==WAIT || accepted) && engine_done;
    wire [32:0] input_end={1'b0,input_base}+33'd307200;
    wire [32:0] scratch_end={1'b0,scratch_base}+scratch_bytes;
    wire [32:0] packet_end={1'b0,packet_base}+33'd26200;
    always @(posedge clk) begin
        if(!rst_n) begin
            state<=IDLE;index<=0;input_q<=0;scratch_q<=0;packet_q<=0;frame_q<=0;node_cycles<=0;
            frame_done<=0;frame_failed<=0;error_code<=0;result_frame<=0;frame_cycles<=0;
            profile_valid<=0;profile_node<=0;profile_cycles<=0;
            descriptor_q<=0;source0_q<=0;source1_q<=0;destination_q<=0;
        end else begin
            frame_done<=0;frame_failed<=0;profile_valid<=0;
            if(launch_valid && launch_ready) begin
                error_code<=0;frame_cycles<=0;node_cycles<=0;index<=0;
                if((input_base[5:0]|scratch_base[5:0]|packet_base[5:0])!=0 ||
                    input_end>33'h100000000 || scratch_end>33'h100000000 || packet_end>33'h100000000) begin
                    error_code<=1;frame_failed<=1;
                end else begin
                    input_q<=input_base;scratch_q<=scratch_base;packet_q<=packet_base;frame_q<=input_frame;state<=FETCH;
                end
            end
            if(state==FETCH || state==ADDRESS || state==ISSUE || state==WAIT) begin
                frame_cycles<=frame_cycles+1'b1;node_cycles<=node_cycles+1'b1;
                if(abort || engine_error || node_cycles>=WATCHDOG_CYCLES) begin
                    error_code<=abort?2:engine_error?3:4;state<=DRAIN;
                end else if(completed) begin
                    profile_valid<=1;profile_node<=index;profile_cycles<=node_cycles+1'b1;node_cycles<=0;
                    if(index+1==command_count) begin state<=IDLE;frame_done<=1;result_frame<=frame_q;end
                    else begin index<=index+1'b1;state<=FETCH;end
                end else if(accepted) state<=WAIT;
                else if(state==FETCH)begin descriptor_q<=descriptor_rom;state<=ADDRESS;end
                else if(state==ADDRESS)begin
                    source0_q<=(descriptor[32]?input_q:scratch_q)+descriptor[64+:32];
                    source1_q<=(descriptor[33]?input_q:scratch_q)+descriptor[96+:32];
                    destination_q<=(descriptor[34]?packet_q:scratch_q)+descriptor[128+:32];
                    state<=ISSUE;
                end
            end
            if(state==DRAIN && engine_idle) begin state<=IDLE;frame_failed<=1;end
        end
    end
endmodule
