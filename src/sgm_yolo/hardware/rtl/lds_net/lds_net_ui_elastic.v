`timescale 1ns/1ps
// Two-entry video queue: READY depends only on registered occupancy.
// One pixel/cycle sustained throughput, independent SOF/EOL/KEEP preservation.
module lds_net_ui_elastic(
 input wire clk,rst_n,input wire [23:0] s_data,input wire [2:0] s_keep,
 input wire s_sof,s_eol,s_valid,output wire s_ready,
 output wire [23:0] m_data,output wire [2:0] m_keep,
 output wire m_sof,m_eol,m_valid,input wire m_ready);
 (* ram_style="registers" *) reg [28:0] pixels[0:1];
 reg head,tail;reg [1:0] used;
 assign s_ready=rst_n && used<2;
 assign m_valid=rst_n && used!=0;
 assign {m_data,m_keep,m_sof,m_eol}=pixels[head];
 wire push=s_valid && s_ready,pop=m_valid && m_ready;
 always @(posedge clk)begin
  if(push)pixels[tail]<={s_data,s_keep,s_sof,s_eol};
  if(!rst_n)begin head<=0;tail<=0;used<=0;end
  else begin
   if(push)tail<=~tail;
   if(pop)head<=~head;
   case({push,pop})
    2'b10:used<=used+1'b1;2'b01:used<=used-1'b1;default:used<=used;
   endcase
  end
 end
endmodule
