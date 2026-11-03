`timescale 1ns/1ps
// PS RGB888 UI atlas, six fixed windows, no dependence of video READY on DDR.
module lds_net_ui(
 input wire clk,rst_n,memory_clk,memory_rst_n,
 input wire write_enable,read_enable,output wire operation_ready,input wire [11:0] write_address,read_address,input wire [31:0] write_data,
 input wire [3:0] write_strobe,output wire write_reject,output reg [31:0] read_data,
 input wire [23:0] s_data,input wire [2:0] s_keep,input wire s_sof,s_eol,s_valid,output wire s_ready,
 output wire [23:0] m_data,output reg [2:0] m_keep,output reg m_sof,m_eol,m_valid,input wire m_ready,
 output wire [31:0] araddr,output wire [7:0] arlen,output wire arvalid,input wire arready,
 input wire [63:0] rdata,input wire [1:0] rresp,input wire rlast,rvalid,output wire rready);
 reg [31:0] shadow_base,shadow_sequence,shadow_detection,shadow_sgm;
 reg [31:0] pending_base,pending_sequence,pending_detection,pending_sgm;
 reg pending,pending_enable,enabled,commit_sent;reg [31:0] active_base,ack_sequence,active_detection,active_sgm;
 reg [31:0] acknowledged_base;
 reg [31:0] epoch,underruns,errors;reg [7:0] age;
 reg [10:0] x,y;
 wire [10:0] vx=s_sof?11'd0:x,vy=s_sof?11'd0:y;
 wire advance=!m_valid || m_ready;
 wire fire=s_valid && s_ready;
 assign s_ready=rst_n && advance;
 wire visible=enabled && age<120;
 wire target=write_address==12'h308 || write_address==12'h30c || write_address==12'h310 || write_address==12'h314 || write_address==12'h318;
 wire command=write_address==12'h318 && write_strobe[0];
 wire base_valid=shadow_base!=0 && shadow_base[5:0]==0 && shadow_base<=32'hffec5000;
 assign write_reject=write_enable && (!target || pending ||
   (command && (write_data[7:0]!=1 && write_data[7:0]!=2 || shadow_sequence==ack_sequence || (write_data[7:0]==1 && !base_valid))));
 function [31:0] merge;
  input [31:0] previous,value;input [3:0] strobes;integer b;
  begin merge=previous;for(b=0;b<4;b=b+1)if(strobes[b])merge[b*8+:8]=value[b*8+:8];end
 endfunction
 wire commit_ready,commit_done,commit_valid,commit_accept;
 wire [32:0] commit_data;
 wire commit_response;
 lds_net_cdc_rpc #(.WIDTH(33),.RESPONSE_WIDTH(1)) commit_rpc(
  .source_clk(memory_clk),.source_rst_n(memory_rst_n),.destination_clk(clk),.destination_rst_n(rst_n),
  .source_valid(pending && !commit_sent),.source_ready(commit_ready),.source_data({pending_enable,pending_base}),
  .source_done(commit_done),.source_response(commit_response),.destination_valid(commit_valid),
  .destination_ready(commit_accept),.destination_data(commit_data),.destination_response(1'b1));
 wire status_ready,status_done,status_valid;wire status_request;
 wire [65:0] status_response;
 wire status_read=read_enable && (read_address==12'h304 || read_address==12'h320 || read_address==12'h324);
 reg status_sent;
 lds_net_cdc_rpc #(.WIDTH(1),.RESPONSE_WIDTH(66)) status_rpc(
  .source_clk(memory_clk),.source_rst_n(memory_rst_n),.destination_clk(clk),.destination_rst_n(rst_n),
  .source_valid(status_read && !status_sent),.source_ready(status_ready),.source_data(1'b0),
  .source_done(status_done),.source_response(status_response),.destination_valid(status_valid),
  .destination_ready(1'b1),.destination_data(status_request),.destination_response({visible,!row_ready,errors,underruns}));
 assign operation_ready=!status_read || status_done;
 always @*begin case(read_address)
  12'h300:read_data=32'h55494631;
  12'h304:read_data={28'd0,status_response[63:32]!=0,status_response[64],status_response[65],pending};
  12'h308:read_data=shadow_base;12'h30c:read_data=shadow_sequence;
  12'h310:read_data=shadow_detection;12'h314:read_data=shadow_sgm;
  12'h31c:read_data=ack_sequence;12'h320:read_data=status_response[31:0];12'h324:read_data=status_response[63:32];
  12'h328:read_data=acknowledged_base;12'h32c:read_data=active_detection;12'h330:read_data=active_sgm;
  12'h334:read_data=32'd1290240;default:read_data=0;
 endcase end
 // All software-visible registers, validation and ownership accounting run at 100 MHz.
 always @(posedge memory_clk)begin
  if(!memory_rst_n)begin
   shadow_base<=0;shadow_sequence<=0;shadow_detection<=0;shadow_sgm<=0;
   pending_base<=0;pending_sequence<=0;pending_detection<=0;pending_sgm<=0;pending<=0;pending_enable<=0;commit_sent<=0;
   acknowledged_base<=0;ack_sequence<=0;active_detection<=0;active_sgm<=0;status_sent<=0;
  end else begin
   if(status_read && !status_sent && status_ready)status_sent<=1;
   if(status_done)status_sent<=0;
   // Shadow writes do not depend on the unrelated address/sequence validation cone.
   if(write_enable && !pending)case(write_address)
    12'h308:shadow_base<=merge(shadow_base,write_data,write_strobe);
    12'h30c:shadow_sequence<=merge(shadow_sequence,write_data,write_strobe);
    12'h310:shadow_detection<=merge(shadow_detection,write_data,write_strobe);
    12'h314:shadow_sgm<=merge(shadow_sgm,write_data,write_strobe);
    12'h318:if(command && !write_reject)begin
     pending<=1;pending_enable<=write_data[7:0]==1;pending_base<=shadow_base;pending_sequence<=shadow_sequence;
     pending_detection<=shadow_detection;pending_sgm<=shadow_sgm;
    end
   endcase
   if(pending && !commit_sent && commit_ready)commit_sent<=1;
   if(commit_done)begin
    pending<=0;commit_sent<=0;ack_sequence<=pending_sequence;acknowledged_base<=pending_base;
    active_detection<=pending_detection;active_sgm<=pending_sgm;
   end
  end
 end
 wire row_ready,row_done,rpc_ready,memory_idle;reg row_valid;reg [77:0] row_request;
 (* ASYNC_REG="TRUE" *) reg busy_sync0,busy_sync1;
 always @(posedge clk)begin
  if(!rst_n)begin busy_sync0<=1;busy_sync1<=1;end
  else begin busy_sync0<=!memory_idle;busy_sync1<=busy_sync0;end
 end
 // A video-only reset may abandon RPC bookkeeping while AXI is still active.
 // Independent memory-idle synchronization prevents premature buffer release.
 assign row_ready=rpc_ready && !busy_sync1;
 wire [44:0] row_response;wire destination_valid,destination_ready;
 wire [77:0] destination_data;wire [44:0] destination_response;
 lds_net_cdc_rpc #(.WIDTH(78),.RESPONSE_WIDTH(45)) row_rpc(
  .source_clk(clk),.source_rst_n(rst_n),.destination_clk(memory_clk),.destination_rst_n(memory_rst_n),
  .source_valid(row_valid),.source_ready(rpc_ready),.source_data(row_request),.source_done(row_done),.source_response(row_response),
  .destination_valid(destination_valid),.destination_ready(destination_ready),.destination_data(destination_data),.destination_response(destination_response));
 wire memory_write;wire [10:0] memory_address;wire [23:0] memory_data;
 lds_net_ui_fetch fetch(.clk(memory_clk),.rst_n(memory_rst_n),.request_valid(destination_valid),.request_ready(destination_ready),.idle(memory_idle),
  .request(destination_data),.response(destination_response),.memory_write(memory_write),.memory_address(memory_address),.memory_data(memory_data),
  .araddr(araddr),.arlen(arlen),.arvalid(arvalid),.arready(arready),.rdata(rdata),.rresp(rresp),.rlast(rlast),.rvalid(rvalid),.rready(rready));
 (* ram_style="block" *) reg [23:0] lines[0:2047];
 always @(posedge memory_clk)if(memory_write)lines[memory_address]<=memory_data;
 reg [31:0] bank_epoch[0:1];reg [10:0] bank_row[0:1];reg [1:0] bank_valid;
 reg row_ok;
 wire current_line=(vy>=86 && vy<214) || (vy>=300 && vy<780) || (vy>=866 && vy<994);
 reg [10:0] next_y;reg next_line;reg [1:0] next_kind;
 assign commit_accept=fire && s_sof && row_ready && !row_valid;
 reg inside;reg [9:0] column;
 always @*begin
  inside=0;column=0;
  if(vy>=300 && vy<780 && vx>=1280 && vx<1920)begin inside=1;column=vx-1280;end
  else if(vy>=86 && vy<214)begin
   if(vx>=224 && vx<416)begin inside=1;column=vx-224;end
   else if(vx>=1504 && vx<1696)begin inside=1;column=vx-1504+192;end
  end else if(vy>=866 && vy<994)begin
   if(vx>=224 && vx<416)begin inside=1;column=vx-224;end
   else if(vx>=864 && vx<1056)begin inside=1;column=vx-864+192;end
   else if(vx>=1504 && vx<1696)begin inside=1;column=vx-1504+384;end
  end
 end
 reg [23:0] line_rgb,pass_rgb;reg select_ui,select_valid;
 assign m_data=select_ui?(select_valid?line_rgb:24'd0):pass_rgb;
 always @(posedge clk)begin
  if(!rst_n)begin
   active_base<=0;enabled<=0;epoch<=0;age<=120;
   next_y<=1;next_line<=0;next_kind<=0;
   x<=0;y<=0;underruns<=0;errors<=0;row_valid<=0;row_request<=0;row_ok<=0;bank_valid<=0;
   bank_epoch[0]<=0;bank_epoch[1]<=0;bank_row[0]<=0;bank_row[1]<=0;
   m_valid<=0;m_keep<=0;m_sof<=0;m_eol<=0;pass_rgb<=0;select_ui<=0;select_valid<=0;
  end else begin
   row_valid<=0;
   if(row_done)begin
    bank_epoch[row_response[11]]<=row_response[43:12];bank_row[row_response[11]]<=row_response[10:0];
    bank_valid[row_response[11]]<=!row_response[44];
    if(row_response[44])errors<=errors+1'b1;
   end
   if(advance)begin
    m_valid<=s_valid;m_keep<=s_keep;m_sof<=s_sof;m_eol<=s_eol;pass_rgb<=s_data;
    select_ui<=inside && visible;select_valid<=row_ok;
    line_rgb<=lines[{vy[0],column}];
   end
   if(fire)begin
    if(s_eol)begin x<=0;y<=vy==1079?11'd0:vy+1'b1;end else begin x<=vx+1'b1;y<=vy;end
    if(s_sof)begin
     next_y<=1;next_line<=0;next_kind<=0;
     epoch<=epoch+1'b1;bank_valid<=0;
     if(age<120)age<=age+1'b1;
     // No old request or AXI response survives this acknowledgement.
     if(commit_valid && commit_accept)begin
      enabled<=commit_data[32];active_base<=commit_data[31:0];age<=0;
     end
    end
    if(s_eol)begin
     next_y<=vy==1079?11'd1:vy+11'd2;
     next_line<=(vy>=84 && vy<212) || (vy>=298 && vy<778) || (vy>=864 && vy<992);
     next_kind<=vy<212?2'd0:vy<778?2'd1:2'd2;
    end
    if(vx==0)begin
     row_ok<=bank_valid[vy[0]] && bank_epoch[vy[0]]==epoch && bank_row[vy[0]]==vy;
     if(visible && current_line && !(bank_valid[vy[0]] && bank_epoch[vy[0]]==epoch && bank_row[vy[0]]==vy))underruns<=underruns+1'b1;
     if(visible && next_line && row_ready && !row_valid)begin
      row_valid<=1;row_request<={epoch,active_base,next_kind,next_y[0],next_y};bank_valid[next_y[0]]<=0;
     end
    end
   end
  end
 end
endmodule
