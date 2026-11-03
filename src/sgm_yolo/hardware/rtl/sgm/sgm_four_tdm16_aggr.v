`timescale 1ns/1ps
// Two physical 16-lane engines serve H/V, then DL/DR. Four disparity
// groups per pair occupy all eight issue slots. Pixels enter every 8 clocks.
// Row state remains independent for all three cross-row directions.
module sgm_four_tdm16_aggr #(parameter IMG_WIDTH=640,IMG_HEIGHT=480,PATHS=4)(
 input wire clk,rst_n,input wire [383:0] data_in_cost,
 input wire [7:0] data_in_gray,input wire data_in_valid,
 output wire [703:0] data_out,output reg data_out_valid);
 localparam DEPTH=4*IMG_WIDTH,AW=$clog2(DEPTH),BANKS=(DEPTH+1023)/1024;
 reg [3:0] phase;
 reg [1:0] bgroup;
 reg pending,read_active,active_a,active_b;
 reg [9:0] nx,ny,read_x,next_y,px,py;
 reg [383:0] pending_cost,raw;
 reg [7:0] pending_gray,pg,hgray;
 reg [575:0] hstate,dlold,dlprev,rsource;
 reg [143:0] previous;
 reg [8:0] left_neighbor;
 reg [16:0] lmeta;
 reg [8:0] hmin,vmin,lmin,rmin;
 reg [7:0] vgray,lgray,rgray;
 wire accept=data_in_valid && (phase==0 || phase==8);
 wire rd_en=accept || (read_active && phase>=1 && phase<=3);
 wire [1:0] rg=accept?2'd0:phase[1:0];
 wire [9:0] rx=accept?nx:read_x;
 wire issue_a=active_a && phase>=2 && phase<=5;
 wire issue_b=active_b;
 wire issue=issue_a || issue_b;
 wire pair_sel=issue_b;
 wire [1:0] ig=issue_b?bgroup:phase-2;
 wire [143:0] rows[1:3];
 wire [16:0] metas[1:3];
 reg [2:0] valids;
 reg [1:0] groups[0:4];
 reg pairs[0:4];
 reg [9:0] xs[0:4];
 reg [7:0] grays[0:4];
 reg reduction_valid,group_valid;
 wire [143:0] results[0:1];
 reg [8:0] group_min[0:1];
 wire [8:0] final_min[0:1];
 wire [8:0] minimum[0:3];
 wire [9:0] p2[0:3];
 wire [3:0] start={py==0 || px==IMG_WIDTH-1,py==0 || px==0,py==0,px==0};
 assign minimum[0]=(group_valid && !pairs[4] && groups[4]==3)?final_min[0]:hmin;
 assign minimum[1]=vmin;assign minimum[2]=lmin;assign minimum[3]=rmin;
 function [9:0] penalty;
 input [7:0] a,b;reg [7:0] delta;
 begin delta=a>=b?a-b:b-a;penalty=delta>=8?100:delta>=3?133:delta==2?200:400;end
 endfunction
 assign p2[0]=penalty(pg,hgray);assign p2[1]=penalty(pg,vgray);
 assign p2[2]=penalty(pg,lgray);assign p2[3]=penalty(pg,rgray);
 genvar e,d,b;
 generate for(e=0;e<2;e=e+1) begin: engine
  for(d=0;d<16;d=d+1) begin: lane
   wire [8:0] self,left,right;
   if(e==0) begin: h_or_dl
    wire [575:0] source=pair_sel?dlprev:hstate;
    wire [8:0] s[0:3],l[0:3],r[0:3];
    for(b=0;b<4;b=b+1) begin: mux
     assign s[b]=source[(b*16+d)*9+:9];
     if(b*16+d==0) assign l[b]=9'd511;
     else assign l[b]=source[(b*16+d-1)*9+:9];
     if(b*16+d==63) assign r[b]=9'd511;
     else assign r[b]=source[(b*16+d+1)*9+:9];
    end
    assign self=s[ig];assign left=l[ig];assign right=r[ig];
   end else begin: v_or_dr
    wire [8:0] s[0:3],l[0:3],r[0:3];
    for(b=0;b<4;b=b+1) begin: mux
     assign s[b]=rsource[(b*16+d)*9+:9];
     if(b*16+d==0) assign l[b]=9'd511;
     else assign l[b]=rsource[(b*16+d-1)*9+:9];
     if(b*16+d==63) assign r[b]=9'd511;
     else assign r[b]=rsource[(b*16+d+1)*9+:9];
    end
    assign self=pair_sel?s[ig]:previous[d*9+:9];
    if(d==0) assign left=pair_sel?l[ig]:(ig==0?9'd511:left_neighbor);
    else assign left=pair_sel?l[ig]:previous[(d-1)*9+:9];
    if(d==15) assign right=pair_sel?r[ig]:(ig==3?9'd511:rows[1][0+:9]);
    else assign right=pair_sel?r[ig]:previous[(d+1)*9+:9];
   end
   wire [5:0] costs[0:3];
   for(b=0;b<4;b=b+1) begin: raw_mux
    assign costs[b]=raw[(b*16+d)*6+:6];
   end
   reg [9:0] adjacent_a,penalty_a,penalty_b,norm_b;
   reg [8:0] self_a,min_a,result_c;
   reg [5:0] raw_a,raw_b;
   reg first_a,illegal_a,first_b,illegal_b;
   wire [8:0] near_value=left<right?left:right;
   wire [9:0] candidate={1'b0,self_a}<adjacent_a?{1'b0,self_a}:adjacent_a;
   wire [9:0] capped=norm_b>penalty_b?penalty_b:norm_b;
   always @(posedge clk) begin
    adjacent_a<={1'b0,near_value}+10'd100;self_a<=self;
    min_a<=pair_sel?minimum[e+2]:minimum[e];
    penalty_a<=pair_sel?p2[e+2]:p2[e];raw_a<=costs[ig];
    first_a<=pair_sel?start[e+2]:start[e];illegal_a<=({ig,4'd0}+d)>px;
    norm_b<=candidate-min_a;penalty_b<=penalty_a;raw_b<=raw_a;
    first_b<=first_a;illegal_b<=illegal_a;
    result_c<=illegal_b?9'd511:first_b?{3'd0,raw_b}:capped+raw_b;
   end
   assign results[e][d*9+:9]=result_c;
  end
  wire [8:0] tree[1:31];
  reg [8:0] partial[0:3];
  for(d=0;d<16;d=d+1) begin: leaves
   assign tree[16+d]=results[e][d*9+:9];
  end
  for(d=1;d<16;d=d+1) begin: reduce
   if(d>=4 && d<8) begin: registered_quarter
    always @(posedge clk) if(valids[2]) partial[d-4]<=tree[2*d]<tree[2*d+1]?tree[2*d]:tree[2*d+1];
    assign tree[d]=partial[d-4];
   end else assign tree[d]=tree[2*d]<tree[2*d+1]?tree[2*d]:tree[2*d+1];
  end
  reg [8:0] running_a,running_b;
  wire [8:0] running=pairs[4]?running_b:running_a;
  assign final_min[e]=group_min[e]<running?group_min[e]:running;
  always @(posedge clk) begin
   if(reduction_valid) group_min[e]<=tree[1];
   if(group_valid) begin
    if(pairs[4]) running_b<=groups[4]==0?group_min[e]:final_min[e];
    else running_a<=groups[4]==0?group_min[e]:final_min[e];
   end
  end
 end
 for(e=1;e<=3;e=e+1) begin: row_path
  wire [9:0] col=e==3 && rx<IMG_WIDTH-1?rx+1'b1:rx;
  wire [AW-1:0] ra={col,rg},wa={xs[2],groups[2]};
  wire [143:0] bd[0:BANKS-1];
  reg [$clog2(BANKS+1)-1:0] rb;
  wire write_group=valids[2] && (e==1?!pairs[2]:pairs[2]);
  wire [143:0] write_data=results[e==2?0:1];
  always @(posedge clk) if(rd_en) rb<=ra>>10;
  assign rows[e]=bd[rb];
  for(b=0;b<BANKS;b=b+1) begin: ram_bank
   localparam SIZE=DEPTH-b*1024>=1024?1024:DEPTH-b*1024;
   localparam BA=SIZE>1?$clog2(SIZE):1;
   sgm_line_write_ram #(.DATA_W(144),.DEPTH(SIZE),.ADDR_W(BA)) ram(
    .clk(clk),.rd_en(rd_en && (ra>>10)==b),.rd_addr(ra[BA-1:0]),.rd_data(bd[b]),
    .wr_en(write_group && (wa>>10)==b),.wr_addr(wa[BA-1:0]),.wr_data(write_data));
  end
  sgm_line_ram #(.DATA_W(17),.DEPTH(IMG_WIDTH),.ADDR_W(10)) metadata(
   .clk(clk),.rd_en(accept),.rd_addr(col),.rd_data(metas[e]),
   .wr_en(group_valid && groups[4]==3 && (e==1?!pairs[4]:pairs[4])),
   .wr_addr(xs[4]),.wr_data({grays[4],final_min[e==2?0:1]}));
 end endgenerate
 reg [639:0] pair_sums;
 reg [703:0] sums;
 wire [159:0] first_sum;
 wire [175:0] next_sum;
 generate for(d=0;d<16;d=d+1) begin: sum_lane
  wire [9:0] a={1'b0,results[0][d*9+:9]}+{1'b0,results[1][d*9+:9]};
  wire [9:0] prior=pair_sums[(groups[2]*16+d)*10+:10];
  assign first_sum[d*10+:10]=a;
  assign next_sum[d*11+:11]=PATHS==2?{1'b0,prior}:({1'b0,prior}+{1'b0,a});
 end endgenerate
 assign data_out=sums;
 integer i;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   phase<=0;nx<=0;ny<=0;pending<=0;read_active<=0;active_a<=0;active_b<=0;
   valids<=0;reduction_valid<=0;group_valid<=0;data_out_valid<=0;
  end else begin
   if(phase!=0) phase<=phase==8?0:phase+1'b1;
   if(active_b) begin bgroup<=bgroup+1'b1;if(bgroup==3) active_b<=0;end
   if(accept) begin
    phase<=1;pending<=1;read_active<=1;active_a<=1;read_x<=nx;next_y<=ny;
    pending_cost<=data_in_cost;pending_gray<=data_in_gray;
    if(nx==IMG_WIDTH-1) begin nx<=0;ny<=ny==IMG_HEIGHT-1?0:ny+1'b1;end else nx<=nx+1'b1;
   end
   if(phase==1) begin
    if(pending) begin
     pending<=0;px<=read_x;py<=next_y;pg<=pending_gray;raw<=pending_cost;hgray<=pg;
     vmin<=metas[1][8:0];vgray<=metas[1][16:9];
     lmin<=lmeta[8:0];lgray<=lmeta[16:9];lmeta<=metas[2];dlprev<=dlold;
     rmin<=metas[3][8:0];rgray<=metas[3][16:9];
    end
   end
   if(phase==3) read_active<=0;
   // RAM outputs for groups 0..3 are available in phases 1..4.
   if((read_active && phase>=1 && phase<=3) || phase==4) begin
    left_neighbor<=previous[135+:9];previous<=rows[1];
    dlold[(phase-1)*144+:144]<=rows[2];rsource[(phase-1)*144+:144]<=rows[3];
   end
   if(phase==5 && active_a) begin active_a<=0;active_b<=1;bgroup<=0;end
   valids<={valids[1:0],issue};reduction_valid<=valids[2];group_valid<=reduction_valid;
   groups[0]<=ig;pairs[0]<=pair_sel;xs[0]<=px;grays[0]<=pg;
   for(i=1;i<5;i=i+1) begin groups[i]<=groups[i-1];pairs[i]<=pairs[i-1];xs[i]<=xs[i-1];grays[i]<=grays[i-1];end
   data_out_valid<=valids[2] && pairs[2] && groups[2]==3;
   if(valids[2]) begin
    if(!pairs[2]) begin hstate[groups[2]*144+:144]<=results[0];pair_sums[groups[2]*160+:160]<=first_sum;end
    else sums[groups[2]*176+:176]<=next_sum;
   end
   if(group_valid && !pairs[4] && groups[4]==3) hmin<=final_min[0];
  end
 end
`ifndef SYNTHESIS
 always @(posedge clk) if(rst_n && data_in_valid && !(phase==0 || phase==8)) $fatal(1,"TDM SGM input II must be >=8");
`endif
endmodule

module sgm_line_write_ram #(parameter DATA_W=144,DEPTH=1024,ADDR_W=10)(input wire clk,rd_en,input wire [ADDR_W-1:0] rd_addr,output wire [DATA_W-1:0] rd_data,input wire wr_en,input wire [ADDR_W-1:0] wr_addr,input wire [DATA_W-1:0] wr_data);
 (* max_fanout=16 *) reg we=0;reg [ADDR_W-1:0] wa;reg [DATA_W-1:0] wd;
 always @(posedge clk)begin we<=wr_en;wa<=wr_addr;wd<=wr_data;end
 sgm_line_ram #(.DATA_W(DATA_W),.DEPTH(DEPTH),.ADDR_W(ADDR_W)) mem(clk,rd_en,rd_addr,rd_data,we,wa,wd);
endmodule
