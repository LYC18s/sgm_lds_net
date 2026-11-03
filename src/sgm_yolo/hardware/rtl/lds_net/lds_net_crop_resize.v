`timescale 1ns/1ps
// RGB888 640x480 -> crop [80,0,480,480] -> 320x320 encoded RGB INT8.
// Three source rows preserve vertical overlap. A frame tap must isolate camera
// capture from this module's backpressure and assert abort on a dropped frame.
module lds_net_crop_resize(
    input wire clk,rst_n,abort,
    input wire [23:0] s_data,input wire s_valid,s_sof,s_eol,
    output wire s_ready,
    output reg [23:0] m_data,output wire m_valid,
    output reg m_sof,m_eol,input wire m_ready,
    output reg frame_done,frame_aborted,output reg error);
    (* ram_style="block" *) reg [23:0] row0[0:479];
    (* ram_style="block" *) reg [23:0] row1[0:479];
    (* ram_style="block" *) reg [23:0] row2[0:479];
    reg [23:0] q00,q01,q10,q11,q20,q21;
    reg active,input_complete,issued_all;
    reg [9:0] ix;reg [8:0] iy,ox,oy,source_y,rows_completed;
    reg [1:0] write_bank,valid_pipe;
    reg xodd_q,yodd_q,sof_q,eol_q,eof_q,eof_out;
    wire step=!valid_pipe[1] || m_ready;
    wire inside=ix>=80 && ix<560;
    assign s_ready=rst_n && !abort && (!active ? s_sof :
        !input_complete && (!inside || iy<source_y+9'd3));
    wire input_fire=s_valid && s_ready;
    wire malformed=active && input_fire && ((s_sof && (ix!=0 || iy!=0)) || s_eol!=(ix==639));
    wire write_pixel=active && input_fire && inside;
    wire [8:0] write_x=ix-10'd80;
    wire [8:0] read_x=(ox>>1)+(ox); // floor(3*ox/2)
    wire emit=active && !issued_all && rows_completed>source_y+9'd1;
    assign m_valid=valid_pipe[1] && rst_n && !abort && !malformed;
    wire [23:0] top0=yodd_q ? q10 : q00;
    wire [23:0] top1=yodd_q ? q11 : q01;
    wire [23:0] bottom0=yodd_q ? q20 : q10;
    wire [23:0] bottom1=yodd_q ? q21 : q11;
    function [7:0] blend;
        input [7:0] a,b,c,d;input xf,yf;
        reg [9:0] top,bottom;reg [11:0] total;
        begin
            top=xf ? a+({2'd0,b}<<1)+b : ({2'd0,a}<<1)+a+b;
            bottom=xf ? c+({2'd0,d}<<1)+d : ({2'd0,c}<<1)+c+d;
            total=yf ? top+({2'd0,bottom}<<1)+bottom+12'd8 :
                       ({2'd0,top}<<1)+top+bottom+12'd8;
            blend=total[11:4]^8'h80;
        end
    endfunction
    // One write/read port and one read-only port per physical row RAM.
    always @(posedge clk) begin
        if(write_pixel && write_bank==0) row0[write_x]<=s_data;
        else if(step) q00<=row0[read_x];
        if(step) q01<=row0[read_x+1'b1];
    end
    always @(posedge clk) begin
        if(write_pixel && write_bank==1) row1[write_x]<=s_data;
        else if(step) q10<=row1[read_x];
        if(step) q11<=row1[read_x+1'b1];
    end
    always @(posedge clk) begin
        if(write_pixel && write_bank==2) row2[write_x]<=s_data;
        else if(step) q20<=row2[read_x];
        if(step) q21<=row2[read_x+1'b1];
    end
    always @(posedge clk) begin
        if(!rst_n) begin
            active<=0;input_complete<=0;issued_all<=0;ix<=0;iy<=0;ox<=0;oy<=0;
            source_y<=0;rows_completed<=0;write_bank<=0;valid_pipe<=0;
            xodd_q<=0;yodd_q<=0;sof_q<=0;eol_q<=0;eof_q<=0;eof_out<=0;
            m_data<=0;m_sof<=0;m_eol<=0;frame_done<=0;frame_aborted<=0;error<=0;
        end else begin
            frame_done<=0;frame_aborted<=0;
            if(abort || malformed) begin
                active<=0;valid_pipe<=0;input_complete<=0;issued_all<=0;
                frame_aborted<=active;if(active) error<=1;
            end else begin
                if(input_fire) begin
                    if(!active) begin
                        active<=1;input_complete<=0;issued_all<=0;ix<=1;iy<=0;
                        ox<=0;oy<=0;source_y<=0;rows_completed<=0;write_bank<=0;
                        valid_pipe<=0;error<=0;
                    end else if(ix==639) begin
                        ix<=0;iy<=iy+1'b1;rows_completed<=rows_completed+1'b1;
                        write_bank<=write_bank==2 ? 0 : write_bank+1'b1;
                        if(iy==479) input_complete<=1;
                    end else ix<=ix+1'b1;
                end
                if(step && active) begin
                    valid_pipe<={valid_pipe[0],emit};
                    xodd_q<=ox[0];yodd_q<=oy[0];
                    sof_q<=ox==0 && oy==0;eol_q<=ox==319;eof_q<=ox==319 && oy==319;
                    m_sof<=sof_q;m_eol<=eol_q;eof_out<=eof_q;
                    m_data<={blend(top0[23:16],top1[23:16],bottom0[23:16],bottom1[23:16],xodd_q,yodd_q),
                              blend(top0[15:8],top1[15:8],bottom0[15:8],bottom1[15:8],xodd_q,yodd_q),
                              blend(top0[7:0],top1[7:0],bottom0[7:0],bottom1[7:0],xodd_q,yodd_q)};
                    if(emit) begin
                        if(ox==319) begin
                            ox<=0;oy<=oy+1'b1;source_y<=source_y+(oy[0]?9'd2:9'd1);
                            if(oy==319) issued_all<=1;
                        end else ox<=ox+1'b1;
                    end
                end
                if(m_valid && m_ready && eof_out) begin
                    frame_done<=1;active<=0;valid_pipe<=0;
                end
            end
        end
    end
endmodule
