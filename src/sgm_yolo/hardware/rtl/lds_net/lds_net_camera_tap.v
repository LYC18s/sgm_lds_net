`timescale 1ns/1ps
// Preserve the existing RGB/SGM stream exactly. The inference branch may lose
// a WHOLE frame, but can never backpressure the camera/video path. On overflow
// only its AXIS CDC FIFO is reset; the NN capture DMA separately drains via
// branch_cancel. No AXI memory interface is reset by this signal.
module lds_net_camera_tap(
    input wire clk,rst_n,enable,
    input wire [23:0] s_data,input wire [2:0] s_keep,input wire s_sof,s_eol,s_valid,output wire s_ready,
    output wire [23:0] m_data,output wire [2:0] m_keep,output wire m_sof,m_eol,m_valid,input wire m_ready,
    output reg [23:0] nn_data,output reg [2:0] nn_keep,output reg nn_sof,nn_eol,nn_valid,input wire nn_ready,
    output wire branch_reset_n,branch_cancel,output reg [31:0] dropped_frames);
    reg forwarding,branch_in_reset;reg [4:0] reset_cycles;
    wire fire=s_valid && s_ready;
    wire step=!nn_valid || nn_ready;
    assign s_ready=m_ready;
    assign m_data=s_data;assign m_keep=s_keep;assign m_sof=s_sof;assign m_eol=s_eol;assign m_valid=s_valid;
    assign branch_reset_n=rst_n && !branch_in_reset;
    assign branch_cancel=!branch_reset_n;
    wire selected=enable && (s_sof || forwarding);
    always @(posedge clk)begin
        if(!rst_n)begin forwarding<=0;branch_in_reset<=0;reset_cycles<=0;nn_data<=0;nn_keep<=0;nn_sof<=0;nn_eol<=0;nn_valid<=0;dropped_frames<=0;end
        else begin
            if(reset_cycles!=0)begin reset_cycles<=reset_cycles-1'b1;nn_valid<=0;forwarding<=0;if(reset_cycles==1)branch_in_reset<=0;end
            else begin
                if(step)nn_valid<=0;
                if(!enable && (forwarding || nn_valid))begin forwarding<=0;nn_valid<=0;reset_cycles<=16;branch_in_reset<=1;end
                else if(fire)begin
                    if(s_sof)forwarding<=enable;
                    if(selected)begin
                        if(step)begin nn_data<=s_data;nn_keep<=s_keep;nn_sof<=s_sof;nn_eol<=s_eol;nn_valid<=1;end
                        else begin
                            nn_valid<=0;forwarding<=0;reset_cycles<=16;branch_in_reset<=1;dropped_frames<=dropped_frames+1;
                        end
                    end
                end
            end
        end
    end
endmodule
