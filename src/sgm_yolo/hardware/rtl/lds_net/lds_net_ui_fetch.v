`timescale 1ns/1ps
// One bounded UI row per bundled request. All AXI responses drain before ACK.
module lds_net_ui_fetch(
 input wire clk,rst_n,request_valid,output wire request_ready,idle,input wire [77:0] request,
 output wire [44:0] response,
 output wire memory_write,output wire [10:0] memory_address,output wire [23:0] memory_data,
 output wire [31:0] araddr,output wire [7:0] arlen,output wire arvalid,input wire arready,
 input wire [63:0] rdata,input wire [1:0] rresp,input wire rlast,rvalid,output wire rready);
 localparam IDLE=0,ADDRESS=1,ISSUE=2,STREAM=3,COMPLETE=4;
 reg [2:0] state;reg [31:0] epoch,base,address;reg [1:0] kind,segment;
 reg bank;reg [10:0] screen_row;reg [8:0] row;reg [9:0] pixel_index,pixels;
 reg [87:0] reservoir;reg [3:0] used;reg finished,failed;
 wire dma_ready,dma_done,dma_error,dma_busy,dv,dr,dl;wire [63:0] dd;
 wire [31:0] bytes_read,stalls;
 wire [9:0] width=kind==1?10'd640:10'd192;
 wire [23:0] length=kind==1?24'd1920:24'd576;
 wire pop=state==STREAM && used>=3 && pixels<width;
 wire push=dv && dr;
 assign dr=state==STREAM && used<=3;
 assign memory_write=pop;assign memory_address={bank,pixel_index};assign memory_data=reservoir[23:0];
 assign request_ready=state==COMPLETE;
 assign idle=state==IDLE;
 assign response={failed,epoch,bank,screen_row};
 lds_net_axi_read reader(.clk(clk),.rst_n(rst_n),.command_valid(state==ISSUE),.command_ready(dma_ready),
  .command_address(address),.command_bytes(length),.done(dma_done),.error(dma_error),.busy(dma_busy),
  .data(dd),.valid(dv),.ready(dr),.last(dl),.araddr(araddr),.arlen(arlen),.arvalid(arvalid),.arready(arready),
  .rdata(rdata),.rresp(rresp),.rlast(rlast),.rvalid(rvalid),.rready(rready),.bytes_read(bytes_read),.stall_cycles(stalls));
 reg [87:0] shifted;
 always @*begin
  shifted=pop?reservoir>>24:reservoir;
  if(push)case(used-(pop?4'd3:4'd0))
   0:shifted=shifted|{24'd0,dd};
   1:shifted=shifted|{16'd0,dd,8'd0};
   2:shifted=shifted|{8'd0,dd,16'd0};
   default:shifted=shifted|{dd,24'd0};
  endcase
 end
 always @(posedge clk)begin
  if(!rst_n)begin
   state<=IDLE;epoch<=0;base<=0;address<=0;kind<=0;segment<=0;bank<=0;screen_row<=0;row<=0;
   pixel_index<=0;pixels<=0;reservoir<=0;used<=0;finished<=0;failed<=0;
  end else case(state)
   IDLE:if(request_valid)begin
    {epoch,base,kind,bank,screen_row}<=request;
    row<=request[13:12]==1?request[10:0]-11'd300:
         request[13:12]==0?request[10:0]-11'd86:request[10:0]-11'd866;
    segment<=0;pixel_index<=0;failed<=0;state<=ADDRESS;
   end
   ADDRESS:begin
    address<=kind==1?base+({23'd0,row}<<10)+({23'd0,row}<<9)+({23'd0,row}<<8)+({23'd0,row}<<7):
     base+32'd921600+((kind==0?32'd3:32'd0)+segment)*32'd73728+({23'd0,row}<<9)+({23'd0,row}<<6);
    used<=0;reservoir<=0;pixels<=0;finished<=0;state<=ISSUE;
   end
   ISSUE:if(dma_ready)state<=STREAM;
   STREAM:begin
    if(dma_done)finished<=1;
    if(dma_error)failed<=1;
    if(push || pop)begin reservoir<=shifted;used<=used+(push?4'd8:4'd0)-(pop?4'd3:4'd0);end
    if(pop)begin pixels<=pixels+1'b1;pixel_index<=pixel_index+1'b1;end
    if(finished && used==0 && pixels==width)begin
     if(kind==1 || (kind==0 && segment==1) || segment==2)state<=COMPLETE;
     else begin segment<=segment+1'b1;state<=ADDRESS;end
    end
   end
   COMPLETE:state<=IDLE;
   default:state<=IDLE;
  endcase
 end
endmodule
