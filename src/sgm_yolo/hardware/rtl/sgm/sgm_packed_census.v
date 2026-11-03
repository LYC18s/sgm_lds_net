`timescale 1ns/1ps
// Packed synchronous four-row history. Scan ticks must be >=3 clocks apart.
// E0 read row state; E1 shift rows/window; E2 publish Census.
module sgm_packed_census #(parameter IMG_WIDTH=640,IMG_HEIGHT=480,ADDR_WIDTH=10,
 parameter USE_SCAN_ENABLE=1)(
 input wire scan_enable,clk,rst_n,input wire [7:0] data_in,
 input wire data_in_valid,output wire data_in_ready,
 output wire [24:0] data_out,output reg data_out_valid,
 output wire [7:0] center_gray_out);
reg [ADDR_WIDTH:0] vx,sx;
reg [10:0] vy,sy;
reg [7:0] current;
reg step1,region2;
wire real_pixel=vx<IMG_WIDTH && vy<IMG_HEIGHT;
wire tick=(!USE_SCAN_ENABLE || scan_enable);
wire step=tick && (real_pixel?data_in_valid:1'b1);
assign data_in_ready=rst_n && real_pixel && tick;
wire [31:0] rows;
wire [7:0] h1=(sx<IMG_WIDTH && sy>=1)?rows[7:0]:8'd0;
wire [7:0] h2=(sx<IMG_WIDTH && sy>=2)?rows[15:8]:8'd0;
wire [7:0] h3=(sx<IMG_WIDTH && sy>=3)?rows[23:16]:8'd0;
wire [7:0] h4=(sx<IMG_WIDTH && sy>=4)?rows[31:24]:8'd0;
sgm_line_ram #(.DATA_W(32),.DEPTH(IMG_WIDTH),.ADDR_W(ADDR_WIDTH)) packed_history
(.clk(clk),.rd_en(step && vx<IMG_WIDTH),.rd_addr(vx[ADDR_WIDTH-1:0]),.rd_data(rows),
 .wr_en(step1 && sx<IMG_WIDTH),.wr_addr(sx[ADDR_WIDTH-1:0]),.wr_data({rows[23:0],current}));
reg [7:0] window[0:4][0:4];integer r,c;
wire [7:0] center=window[2][2];wire [24:0] census;
genvar gy,gx;
generate for(gy=0;gy<5;gy=gy+1) begin:y
 for(gx=0;gx<5;gx=gx+1) begin:x
  assign census[gy*5+gx]=window[4-gy][4-gx]>center;
 end
end endgenerate
reg [24:0] desc;reg [7:0] gray;
always @(posedge clk or negedge rst_n) begin
 if(!rst_n) begin
  vx<=0;vy<=0;step1<=0;region2<=0;data_out_valid<=0;
  for(r=0;r<5;r=r+1) for(c=0;c<5;c=c+1) window[r][c]<=0;
 end else begin
  step1<=step;
  if(step) begin
   sx<=vx;sy<=vy;current<=real_pixel?data_in:8'd0;
   if(vx==IMG_WIDTH+1) begin vx<=0;vy<=vy==IMG_HEIGHT+1?0:vy+1'b1;end
   else vx<=vx+1'b1;
  end
  region2<=step1 && sx>=2 && sx<=IMG_WIDTH+1 && sy>=2 && sy<=IMG_HEIGHT+1;
  if(step1) begin
   window[0][0]<=current;window[1][0]<=h1;window[2][0]<=h2;window[3][0]<=h3;window[4][0]<=h4;
   for(r=0;r<5;r=r+1) for(c=1;c<5;c=c+1) window[r][c]<=sx==0?8'd0:window[r][c-1];
  end
  data_out_valid<=region2;
  if(region2) begin desc<=census;gray<=center;end
 end
end
assign data_out=data_out_valid?desc:25'd0;
assign center_gray_out=data_out_valid?gray:8'd0;
`ifndef SYNTHESIS
always @(posedge clk) if(rst_n && step && step1) $fatal(1,"packed Census requires spaced scan ticks");
`endif
endmodule
