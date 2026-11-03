`timescale 1ns/1ps
// Two fixed output channels, four spatial pixels, four native DSP48E1 MACs.
// Each DSP accumulates two exact dot products separated by 18 bits. The
// compiler proves low-slot prefix bounds, including signed-input compensation.
// A single simple-dual-port RAM holds both window banks: prefetch writes the
// inactive half while computation reads the active half (no RAM duplication).
module lds_net_mac_bank(
 input wire clk,rst_n,start,first_chunk,active_window,signed_activation,enabled,
 input wire [71:0] initial_codes,
 input wire window_we,window_bank,input wire [6:0] window_address,input wire [23:0] window_data,
 input wire program_we,input wire [9:0] program_address,input wire [15:0] program_data,
 output reg done,output reg error,output wire [255:0] sums);
 (* ram_style="block" *) reg [15:0] program[0:1023];
 (* ram_style="block" *) reg [23:0] windows[0:255];
 reg [15:0] instruction;reg [23:0] activations;
 reg [9:0] cursor;reg running,issued;reg [2:0] drain;
 reg instr_valid,win_valid,sa;reg signed [3:0] weight0,weight1;reg [7:0] chunk_issued;
 wire clear=start && first_chunk;
 wire marker=instruction[15];
 wire fetch=running && !issued;
 always @(posedge clk)begin
  if(program_we)program[program_address]<=program_data;
  instruction<=program[cursor];
  if(window_we)windows[{window_bank,window_address}]<=window_data;
  activations<=windows[{active_window,instruction[6:0]}];
  weight0<=instruction[10:7];weight1<=instruction[14:11];sa<=signed_activation;
  if(!rst_n)begin cursor<=0;running<=0;issued<=0;instr_valid<=0;win_valid<=0;done<=0;error<=0;drain<=0;chunk_issued<=0;end
  else begin
   done<=0;instr_valid<=fetch;win_valid<=instr_valid && !marker;
   if(start)begin
    running<=enabled;issued<=0;drain<=0;instr_valid<=0;win_valid<=0;error<=0;chunk_issued<=0;
    if(first_chunk)cursor<=0;
    if(!enabled)begin done<=1;running<=0;end
   end else if(running)begin
    // The synchronous instruction read is requested one cycle before valid.
    // After each request cursor advances; a delimiter stops this bank alone.
    if(fetch)begin cursor<=cursor+1'b1;end
    if(instr_valid && marker)begin issued<=1;instr_valid<=0;drain<=4;cursor<=cursor;end
    if(instr_valid && !marker)begin
     chunk_issued<=chunk_issued+1'b1;
     if(chunk_issued==128)begin error<=1;issued<=1;instr_valid<=0;win_valid<=0;drain<=4;end
    end
    if(issued)begin
     if(drain!=0)drain<=drain-1'b1;
     else begin running<=0;done<=1;end
    end
   end
  end
 end
 genvar c,p;
 generate for(c=0;c<2;c=c+1)begin: channels
  for(p=0;p<2;p=p+1)begin: pairs
   reg signed [24:0] a;reg signed [3:0] w;reg valid;
   (* use_dsp="yes" *) reg signed [47:0] acc;
   wire [5:0] a0=activations[p*12+:6],a1=activations[p*12+6+:6];
   wire [5:0] low_a=sa?(a0^6'h20):a0;
   wire signed [24:0] packed_a=$signed({sa && a1[5],a1,18'd0})+$signed({19'd0,low_a});
   wire signed [35:0] initial_code=$signed(initial_codes[c*36+:36]);
   always @(posedge clk)begin
    a<=packed_a;w<=c==0?weight0:weight1;valid<=win_valid && (c==0?weight0!=0:weight1!=0);
    if(!rst_n)begin acc<=0;valid<=0;end
    else if(clear)begin acc<={{12{initial_code[35]}},initial_code};valid<=0;end
    else if(valid)acc<=acc+a*w;
   end
   wire [17:0] low_code=acc[17:0]^18'h20000;
   assign sums[(c*4+p*2)*32+:32]={{14{low_code[17]}},low_code};
   assign sums[(c*4+p*2+1)*32+:32]=$signed(acc)>>>18;
  end
 end endgenerate
endmodule
