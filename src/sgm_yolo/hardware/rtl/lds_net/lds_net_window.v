`timescale 1ns/1ps
// Three normalized input rows, each <=320 eight-channel words. Four spatial
// reads share a row/channel address calculation; cached rows span all K chunks.
// DDR adapters unpack A6 into these byte lanes ONCE, preserving signed codes.
// Two physical read copies provide four synchronous reads without im2col DDR.
(* use_dsp="no" *) module lds_net_window(
    input wire clk,rst_n,clear_rows,
    input wire load_we,output wire load_ready,input wire [1:0] load_slot,
    input wire [8:0] load_address,input wire [63:0] load_data,
    input wire row_commit,input wire [8:0] row_tag,
    input wire start,input wire [9:0] width,height,
    input wire [7:0] channels,input wire kernel3,stride2,padding,
    input wire [9:0] output_x,output_y,k_start,input wire [7:0] k_count,
    input wire signed [15:0] input_zero,
    output wire ready,output reg done,error,
    output wire out_valid,input wire out_ready,
    output reg [6:0] out_address,output reg [23:0] out_a6,output reg [31:0] out_a8);
    (* ram_style="block" *) reg [63:0] copy0[0:959],copy1[0:959];
    reg [8:0] tags[0:2];reg [2:0] rows_valid;
    reg running;reg [1:0] valid_pipe;reg [9:0] w,h,ox,oy,k,first_k;
    reg [7:0] ci,count;reg k3,s2,pad;reg signed [15:0] zero;
    reg [7:0] issued;reg [3:0] padded_q;reg [2:0] channel_q;
    reg [1:0] warmup,kx,ky;reg [6:0] input_channel;reg [3:0] spatial_init;
    reg signed [12:0] column_x[0:3];reg signed [17:0] column_group[0:3];
    reg [6:0] index_q;reg end_q,end_out;
    reg [63:0] q0,q1,q2,q3;
    wire step=!valid_pipe[1] || out_ready;
    assign ready=!running && valid_pipe==0;
    assign load_ready=ready;
    wire writing=load_we && load_ready && load_slot<3 && load_address<320;
    wire [9:0] write_index=load_slot*320+load_address;
    wire issue=running && warmup==0 && issued<count;
    wire signed [12:0] source_y=$signed({3'd0,oy}<<(s2?1:0))+$signed({11'd0,ky})-$signed({12'd0,pad});
    wire [4:0] groups=(ci+7)>>3;
    reg [1:0] row_slot;reg found;
    integer j,reset_index;
    always @* begin
        row_slot=0;found=0;
        for(j=0;j<3;j=j+1) if(rows_valid[j] && tags[j]==source_y) begin row_slot=j;found=1;end
    end
    wire [39:0] addresses;wire [3:0] pad_flags;
    genvar p;
    generate for(p=0;p<4;p=p+1) begin: positions
        wire signed [12:0] sx=column_x[p]+$signed({11'd0,kx});
        wire outside=source_y<0 || source_y>=$signed({3'd0,h}) || sx<0 || sx>=$signed({3'd0,w});
        wire [9:0] addr=row_slot*320+column_group[p]+(kx==2?{groups,1'b0}:kx==1?groups:0)+(input_channel>>3);
        assign addresses[p*10+:10]=outside || !found ? 10'd0 : addr;
        assign pad_flags[p]=outside || !found;
    end endgenerate
    always @(posedge clk) begin
        // An explicit read/write branch preserves the two physical RAM ports;
        // do not put the write-address mux inside both RAM accesses.
        if(writing)begin copy0[write_index]<=load_data;copy1[write_index]<=load_data;end
        else if(step)begin q0<=copy0[addresses[9:0]];q2<=copy1[addresses[29:20]];end
        if(step) begin q1<=copy0[addresses[19:10]];q3<=copy1[addresses[39:30]];end
    end
    function [7:0] code;
        input [63:0] word;input [2:0] channel;input outside;input signed [15:0] zp;
        reg [7:0] value;
        begin value=word[channel*8+:8];code=outside?8'd0:(zp==-128?value^8'h80:value);end
    endfunction
    reg [7:0] a0,a1,a2,a3;
    assign out_valid=valid_pipe[1] && rst_n;
    always @(posedge clk) begin
        if(!rst_n) begin
            rows_valid<=0;running<=0;valid_pipe<=0;done<=0;error<=0;w<=0;h<=0;ox<=0;oy<=0;k<=0;first_k<=0;
            ci<=0;count<=0;k3<=0;s2<=0;pad<=0;zero<=0;issued<=0;padded_q<=0;channel_q<=0;index_q<=0;
            warmup<=0;kx<=0;ky<=0;input_channel<=0;spatial_init<=0;
            for(reset_index=0;reset_index<4;reset_index=reset_index+1)begin column_x[reset_index]<=0;column_group[reset_index]<=0;end
            end_q<=0;end_out<=0;out_address<=0;out_a6<=0;out_a8<=0;
            for(reset_index=0;reset_index<3;reset_index=reset_index+1) tags[reset_index]<=0;
        end else begin
            done<=0;
            if(clear_rows && ready) begin rows_valid<=0;error<=0;end
            if(load_we && !load_ready) error<=1;
            if(row_commit && load_ready && load_slot<3) begin tags[load_slot]<=row_tag;rows_valid[load_slot]<=1;end
            if(start && ready) begin
                w<=width;h<=height;ox<=output_x;oy<=output_y;k<=k_start;first_k<=k_start;
                ci<=channels;count<=k_count;k3<=kernel3;s2<=stride2;pad<=padding;zero<=input_zero;
                issued<=0;error<=0;warmup<=2;
                if(k_count==0 || k_count>128 || width*((channels+7)>>3)>320 ||
                   k_start+k_count>channels*(kernel3?9:1) || (input_zero!=0 && input_zero!=-128)) begin done<=1;error<=1;end
                else running<=1;
            end
            // Divide K only at setup; bounded kernel/channel counters then
            // advance without division on the BRAM address path.
            if(running && warmup==2)begin
                input_channel<=k3?k/9:k;spatial_init<=k3?k%9:0;warmup<=1;
                for(reset_index=0;reset_index<4;reset_index=reset_index+1)
                    column_x[reset_index]<=$signed(({3'd0,ox}+reset_index)<<(s2?1:0))-$signed({12'd0,pad});
            end
            if(running && warmup==1)begin
                ky<=spatial_init/3;kx<=spatial_init%3;warmup<=0;
                for(reset_index=0;reset_index<4;reset_index=reset_index+1)
                    column_group[reset_index]<=column_x[reset_index]*$signed({1'b0,groups});
            end
            if(step) begin
                valid_pipe<={valid_pipe[0],issue};
                padded_q<=pad_flags;channel_q<=input_channel[2:0];index_q<=issued[6:0];end_q<=issued+1==count;
                a0=code(q0,channel_q,padded_q[0],zero);a1=code(q1,channel_q,padded_q[1],zero);
                a2=code(q2,channel_q,padded_q[2],zero);a3=code(q3,channel_q,padded_q[3],zero);
                out_a6<={a3[5:0],a2[5:0],a1[5:0],a0[5:0]};out_a8<={a3,a2,a1,a0};out_address<=index_q;end_out<=end_q;
                if(issue) begin
                    issued<=issued+1'b1;k<=k+1'b1;
                    if(!k3)input_channel<=input_channel+1'b1;
                    else if(kx==2)begin
                        kx<=0;
                        if(ky==2)begin ky<=0;input_channel<=input_channel+1'b1;end
                        else ky<=ky+1'b1;
                    end else kx<=kx+1'b1;
                    if(!found && source_y>=0 && source_y<$signed({3'd0,h})) error<=1;
                end
            end
            if(out_valid && out_ready && end_out) begin running<=0;valid_pipe<=0;done<=1;end
        end
    end
endmodule
