`timescale 1ns/1ps
// One 32-channel group. DDR contains 16-bit fixed-pair sparse instructions;
// ROM contains the exact 36-bit DSP initial codes and original M/R constants.
// Cancel drains every issued read before releasing ownership of the HP port.
module lds_net_mac_parameter_loader(
 input wire clk,rst_n,start,cancel,engine_ready,
 input wire [5:0] layer_index,input wire [7:0] channel_start,input wire [31:0] program_base,
 output wire ready,output reg done,error,output reg [5:0] channels,
 output reg [1023:0] biases,multipliers,output reg [1151:0] initial_codes,output reg [191:0] shifts,
 output wire program_we,output wire [3:0] program_group,output wire [9:0] program_address,output wire [15:0] program_data,
 output wire weight_we,output wire [4:0] weight_address,output wire [31:0] weight_data,
 output wire [31:0] araddr,output wire [7:0] arlen,output wire arvalid,input wire arready,
 input wire [63:0] rdata,input wire [1:0] rresp,input wire rlast,rvalid,output wire rready,
 output reg [31:0] bytes_read);
 localparam IDLE=0,META_FETCH=1,META_CAPTURE=2,CHECK=3,PARAM_FETCH=4,PARAM_CAPTURE=5,
 W8_FETCH=6,W8_CAPTURE=7,PROGRAM_SELECT=8,PROGRAM_COMMAND=9,PROGRAM_STREAM=10,FINISH=11;
 reg [3:0] state;reg cancelled;reg [5:0] layer;reg [7:0] first;reg [31:0] base;
 reg [5:0] metadata_index;reg [31:0] meta[0:63];reg [4:0] parameter_index,w8_index;reg [3:0] bank;
 reg [10:0] written;reg [63:0] held;reg held_valid,read_seen;reg [1:0] lane;
 wire [31:0] rom_metadata,rom_weight;wire [70:0] rom_parameter;wire group_valid;wire [6:0] group_index;
 wire [10:0] reduction=meta[1][10:0];wire is8=meta[3][0];
 wire [10:0] slots=meta[5+bank*2][10:0];wire [31:0] offset=meta[4+bank*2];
 wire [32:0] address_sum={1'b0,base}+offset;
 wire [23:0] span=((slots*2+7)>>3)<<3;
 wire failed=cancel || cancelled || error || !engine_ready;
 lds_net_mac_parameter_rom constants(clk,layer,first,group_valid,group_index,metadata_index,rom_metadata,
  meta[0][10:0]+parameter_index,rom_parameter,meta[2][8:0]+w8_index,rom_weight);
 wire dma_ready,dma_done,dma_error,dma_busy,dv,dr,dl;wire [63:0] dd;wire [31:0] dma_bytes,unused_stalls;
 lds_net_axi_read reader(clk,rst_n,state==PROGRAM_COMMAND && !failed,dma_ready,address_sum[31:0],span,
  dma_done,dma_error,dma_busy,dd,dv,dr,dl,araddr,arlen,arvalid,arready,rdata,rresp,rlast,rvalid,rready,dma_bytes,unused_stalls);
 assign ready=state==IDLE;
 assign program_we=state==PROGRAM_STREAM && held_valid && written<slots && !failed && !dma_error;
 assign program_group=bank;assign program_address=written[9:0];assign program_data=held[lane*16+:16];
 assign weight_we=state==W8_CAPTURE && !failed;assign weight_address=w8_index;assign weight_data=rom_weight;
 assign dr=state==PROGRAM_STREAM && !held_valid;
 integer i;
 always @(posedge clk)begin
  if(!rst_n)begin
   state<=IDLE;done<=0;error<=0;channels<=0;biases<=0;multipliers<=0;shifts<=0;initial_codes<=0;cancelled<=0;
   layer<=0;first<=0;base<=0;metadata_index<=0;parameter_index<=0;w8_index<=0;bank<=0;written<=0;
   held<=0;held_valid<=0;lane<=0;read_seen<=0;bytes_read<=0;
   for(i=0;i<64;i=i+1)meta[i]<=0;
  end else begin
   done<=0;
   if(state!=IDLE && (cancel || !engine_ready))begin cancelled<=1;error<=1;end
   if(failed && state!=IDLE && state!=PROGRAM_STREAM && state!=FINISH)state<=FINISH;
   else case(state)
    IDLE:if(start)begin
     layer<=layer_index;first<=channel_start;base<=program_base;metadata_index<=0;parameter_index<=0;
     channels<=0;biases<=0;initial_codes<=0;multipliers<=0;shifts<=0;bytes_read<=0;cancelled<=0;error<=0;
     if(cancel || !engine_ready || program_base[5:0]!=0)begin error<=1;state<=FINISH;end else state<=META_FETCH;
    end
    META_FETCH:if(!group_valid)begin error<=1;state<=FINISH;end else state<=META_CAPTURE;
    META_CAPTURE:begin
     meta[metadata_index]<=rom_metadata;metadata_index<=metadata_index+1'b1;
     if(metadata_index==63)state<=CHECK;else state<=META_FETCH;
    end
    CHECK:begin
     channels<=meta[1][21:16];parameter_index<=0;w8_index<=0;bank<=0;
     if(meta[1][31:24]!=first || meta[1][21:16]==0 || meta[1][21:16]>32 || reduction==0 || reduction>1024 ||
       (is8 && (reduction>32 || meta[1][21:16]>4)))begin error<=1;state<=FINISH;end else state<=PARAM_FETCH;
    end
    PARAM_FETCH:state<=PARAM_CAPTURE;
    PARAM_CAPTURE:begin
     if(is8)begin
      biases[parameter_index*32+:32]<=rom_parameter[31:0];multipliers[parameter_index*32+:32]<={1'b0,rom_parameter[62:32]};
      shifts[parameter_index*6+:6]<=rom_parameter[68:63];
     end else begin
      initial_codes[parameter_index*36+:36]<=rom_parameter[35:0];
      multipliers[parameter_index*32+:32]<={1'b0,rom_parameter[66:36]};
      shifts[parameter_index*6+:6]<=6'd31+{2'd0,rom_parameter[70:67]};
     end
     parameter_index<=parameter_index+1'b1;
     if({1'b0,parameter_index}+1==channels)begin if(is8)state<=W8_FETCH;else state<=PROGRAM_SELECT;end
     else state<=PARAM_FETCH;
    end
    W8_FETCH:state<=W8_CAPTURE;
    W8_CAPTURE:begin w8_index<=w8_index+1'b1;if(w8_index+1==reduction)state<=FINISH;else state<=W8_FETCH;end
    PROGRAM_SELECT:begin
     if(bank*2>=channels)state<=FINISH;
     else if(slots==0 || slots>1024 || offset[2:0]!=0 || address_sum[32] || address_sum+span>33'h100000000)begin error<=1;state<=FINISH;end
     else state<=PROGRAM_COMMAND;
    end
    PROGRAM_COMMAND:if(dma_ready)begin written<=0;held_valid<=0;lane<=0;read_seen<=0;state<=PROGRAM_STREAM;end
    PROGRAM_STREAM:begin
     if(dma_error)error<=1;
     if(dma_done)read_seen<=1;
     if(dv && dr)begin held<=dd;held_valid<=1;lane<=0;end
     if(held_valid)begin
      if(written<slots)written<=written+1'b1;
      if(lane==3)begin held_valid<=0;lane<=0;end else lane<=lane+1'b1;
     end
     if((read_seen || dma_done) && !held_valid && !dv)begin
      bytes_read<=bytes_read+dma_bytes;
      if(failed || dma_error || bank==15)state<=FINISH;
      else begin bank<=bank+1'b1;state<=PROGRAM_SELECT;end
     end
    end
    FINISH:begin state<=IDLE;done<=1;end
    default:begin state<=FINISH;error<=1;end
   endcase
  end
 end
endmodule
