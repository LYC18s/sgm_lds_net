`timescale 1ns/1ps
// 8/16/32 population-count lanes; all 64 candidates retained.
// E0 latch; E1/E2 count groups; E2/E3 pair sums; E3/E4 assemble.
module sgm_folded32_hamming #(parameter IMG_WIDTH=640,LANES=32)(
    input wire clk,rst_n,input wire [24:0] data_in_L,data_in_R,
    input wire [7:0] data_in_gray,input wire data_in_L_valid,data_in_R_valid,
    output wire [383:0] data_out,output wire data_out_valid,
    output wire [7:0] data_out_gray
);
    reg [9:0] x,px;
    reg [24:0] history[0:63],left;
    reg [7:0] gray,output_gray;
    localparam GROUPS=64/LANES;
    reg [3:0] phase;
    reg [2:0] group1,group2;reg v1,v2,out_valid;
    reg [383:0] costs;
    reg [7:0] gray1,gray2;
    wire fire=data_in_L_valid && data_in_R_valid;
    wire generate_counts=phase>=1 && phase<=GROUPS;
    wire [2:0] group=phase-1;
    wire [LANES*6-1:0] totals;
    function [2:0] pc5;
        input [4:0] bits;
        begin pc5={2'b0,bits[0]}+bits[1]+bits[2]+bits[3]+bits[4];end
    endfunction
    genvar d,g;
    generate for(d=0;d<LANES;d=d+1) begin:lane
        wire [24:0] rd=history[group*LANES+d];
        wire invalid=group*LANES+d>px;
        wire [24:0] diff=invalid?25'h1ffffff:left^rd;
        reg [2:0] count[0:4],tail;
        reg [3:0] pair0,pair1;
        for(g=0;g<5;g=g+1) begin:count_five
            always @(posedge clk) if(generate_counts) count[g]<=pc5(diff[g*5+:5]);
        end
        always @(posedge clk) if(v1) begin
            pair0<={1'b0,count[0]}+count[1];
            pair1<={1'b0,count[2]}+count[3];tail<=count[4];
        end
        assign totals[d*6+:6]={2'b0,pair0}+pair1+tail;
    end endgenerate
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin x<=0;phase<=0;v1<=0;v2<=0;out_valid<=0;end
        else begin
            if(fire) begin phase<=1;px<=x;x<=x==IMG_WIDTH-1?0:x+1'b1;end
            else if(phase!=0) phase<=phase==GROUPS?0:phase+1'b1;
            v1<=generate_counts;v2<=v1;out_valid<=v2 && group2==GROUPS-1;
        end
    end
    always @(posedge clk) begin
        if(rst_n && fire) begin
            left<=data_in_L;gray<=data_in_gray;history[0]<=data_in_R;
            for(i=1;i<64;i=i+1) history[i]<=history[i-1];
        end
        if(generate_counts) begin group1<=group;gray1<=gray;end
        if(v1) begin group2<=group1;gray2<=gray1;end
        if(v2) begin
            costs[group2*LANES*6+:LANES*6]<=totals;
            if(group2==GROUPS-1) output_gray<=gray2;
        end
    end
    assign data_out=costs;assign data_out_valid=out_valid;assign data_out_gray=output_gray;
`ifndef SYNTHESIS
    always @(posedge clk) if(rst_n && fire && phase!=0 && !(LANES==8 && phase==GROUPS))
        $fatal(1,"Folded Hamming input overlaps an unfinished pixel");
`endif
endmodule
