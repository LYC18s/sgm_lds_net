`timescale 1ns/1ps
// 32 output channels x four pixels. Independent fixed-pair instruction cursors
// skip zero coefficients and stop at chunk delimiters without bank lockstep.
module lds_net_mac_tile(
 input wire clk,rst_n,start,first_chunk,last_chunk,signed_activation,active_window,
 input wire [1151:0] initial_codes,input wire [1023:0] multipliers,input wire [191:0] shifts,
 input wire [5:0] channels,input wire signed [15:0] minimum,maximum,
 input wire window_we,window_bank,input wire [6:0] window_address,input wire [23:0] window_data,
 input wire program_we,input wire [3:0] program_group,input wire [9:0] program_address,input wire [15:0] program_data,
 output wire ready,output reg done,output wire error,
 output wire out_valid,input wire out_ready,output wire [15:0] out_data,
 output wire [1:0] out_keep,out_pixel,output wire [4:0] out_channel);
 localparam IDLE=0,START=1,WAIT=2,QUANT=3,OUTPUT=4;
 reg [2:0] state;reg first_q,last_q,sa_q,window_q;reg [5:0] channels_q;
 reg [15:0] completed;wire [15:0] bank_done,bank_error;wire [4095:0] sums;
 genvar b;
 generate for(b=0;b<16;b=b+1)begin: banks
  lds_net_mac_bank mac(.clk(clk),.rst_n(rst_n),.start(state==START),.first_chunk(first_q),
   .active_window(window_q),.signed_activation(sa_q),.enabled(b*2<channels_q),.initial_codes(initial_codes[b*72+:72]),
   .window_we(window_we),.window_bank(window_bank),.window_address(window_address),.window_data(window_data),
   .program_we(program_we && program_group==b),.program_address(program_address),.program_data(program_data),
   .done(bank_done[b]),.error(bank_error[b]),.sums(sums[b*256+:256]));
 end endgenerate
 wire quant_ready,quant_done;
 lds_net_tile_requant #(.MAX_CHANNELS(32),.ACC_BITS(18)) quantization(clk,rst_n,state==QUANT,
  sums,multipliers,shifts,channels_q,minimum,maximum,quant_ready,out_valid,out_ready,out_data,out_keep,out_pixel,out_channel,quant_done);
 assign ready=state==IDLE;assign error=|bank_error;
 always @(posedge clk)begin
  if(!rst_n)begin state<=IDLE;done<=0;first_q<=0;last_q<=0;sa_q<=0;window_q<=0;channels_q<=0;completed<=0;end
  else begin
   done<=0;
   case(state)
    IDLE:if(start)begin first_q<=first_chunk;last_q<=last_chunk;sa_q<=signed_activation;window_q<=active_window;
     channels_q<=channels;completed<=0;state<=START;end
    START:state<=WAIT;
    WAIT:begin completed<=completed|bank_done;if(&(completed|bank_done))begin
     if(last_q)state<=QUANT;else begin state<=IDLE;done<=1;end end end
    QUANT:if(quant_ready)state<=OUTPUT;
    OUTPUT:if(quant_done)begin state<=IDLE;done<=1;end
    default:state<=IDLE;
   endcase
  end
 end
endmodule
