`timescale 1ns/1ps
// Eight candidates per clock, eight groups per pixel, II=8. The 8:1 tree
// has seven comparators; one accumulator comparator preserves the first tie.
module sgm_wta8 #(parameter MAX_DISP=64,COST_W=11,DISP_W=6)(
 input wire clk,rst_n,input wire [MAX_DISP*COST_W-1:0] data_in_cost,
 input wire data_in_valid,
 output reg [DISP_W-1:0] data_out_disp,
 output reg [COST_W-1:0] data_out_min_cost,output reg data_out_valid);
 reg [MAX_DISP*COST_W-1:0] remaining;
 reg [3:0] phase;
 wire issue=data_in_valid || phase!=0;
 wire [2:0] group_index=data_in_valid?3'd0:phase[2:0];
 wire [8*COST_W-1:0] candidates=data_in_valid?data_in_cost[0+:8*COST_W]:remaining[0+:8*COST_W];
 reg [8*COST_W-1:0] candidates_q;reg candidates_valid;reg [2:0] candidate_group;
 wire [COST_W-1:0] tree[1:15];
 wire [2:0] indices[1:15];
 genvar d;
 generate for(d=0;d<8;d=d+1) begin: leaf
  assign tree[8+d]=candidates_q[d*COST_W+:COST_W];assign indices[8+d]=d;
 end
 for(d=1;d<8;d=d+1) begin: compare
  assign tree[d]=tree[2*d]<=tree[2*d+1]?tree[2*d]:tree[2*d+1];
  assign indices[d]=tree[2*d]<=tree[2*d+1]?indices[2*d]:indices[2*d+1];
 end endgenerate
 reg local_valid;
 reg [2:0] local_group,local_index;
 reg [COST_W-1:0] local_cost,best_cost;
 reg [DISP_W-1:0] best_disp;
 wire replace_best=local_group==0 || local_cost<best_cost;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin phase<=0;local_valid<=0;data_out_valid<=0;candidates_valid<=0;candidates_q<=0;candidate_group<=0;end
  else begin
   if(data_in_valid) begin remaining<=data_in_cost>>(8*COST_W);phase<=1;end
   else if(phase!=0) begin remaining<=remaining>>(8*COST_W);phase<=phase==7?0:phase+1'b1;end
   candidates_valid<=issue;
   if(issue)begin candidates_q<=candidates;candidate_group<=group_index;end
   local_valid<=candidates_valid;
   if(candidates_valid) begin local_group<=candidate_group;local_index<=indices[1];local_cost<=tree[1];end
   data_out_valid<=local_valid && local_group==7;
   if(local_valid) begin
    if(replace_best) begin best_cost<=local_cost;best_disp<={local_group,local_index};end
    if(local_group==7) begin
     data_out_min_cost<=replace_best?local_cost:best_cost;
     data_out_disp<=replace_best?{local_group,local_index}:best_disp;
    end
   end
  end
 end
 initial if(MAX_DISP!=64 || DISP_W!=6) $error("sgm_wta8 requires 64 disparities and 6-bit indices");
`ifndef SYNTHESIS
 always @(posedge clk) if(rst_n && data_in_valid && phase!=0) $fatal(1,"WTA input II must be >=8");
`endif
endmodule
