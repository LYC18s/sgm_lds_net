`timescale 1ns/1ps
// Causal three-point horizontal median. First two pixels of each row retain
// their raw values. No line RAM, virtual padding, or interstage FIFO is needed.
module sgm_median3_horizontal #(parameter IMG_WIDTH=640)(
 input wire clk,rst_n,input wire [5:0] din,input wire valid,
 output reg [5:0] dout,output reg out_valid);
 reg [9:0] x;
 reg [5:0] previous,older;
 wire [5:0] low=previous<older?previous:older;
 wire [5:0] high=previous<older?older:previous;
 wire [5:0] median=din<low?low:din>high?high:din;
 always @(posedge clk) begin
  if(!rst_n) begin x<=0;previous<=0;older<=0;dout<=0;out_valid<=0;end
  else begin
   out_valid<=valid;
   if(valid) begin
    dout<=x<2?din:median;
    older<=previous;previous<=din;
    x<=x==IMG_WIDTH-1?0:x+1'b1;
   end
  end
 end
endmodule
