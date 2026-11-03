`timescale 1ns/1ps
// Preserve legacy video-domain registers; route only UIF1 transactions to 100 MHz.
module lds_net_ui_axil(
 input wire clk,rst_n,memory_clk,memory_rst_n,
 input wire [11:0] s_axi_awaddr,input wire [2:0] s_axi_awprot,input wire s_axi_awvalid,output wire s_axi_awready,
 input wire [31:0] s_axi_wdata,input wire [3:0] s_axi_wstrb,input wire s_axi_wvalid,output wire s_axi_wready,
 output reg [1:0] s_axi_bresp,output reg s_axi_bvalid,input wire s_axi_bready,
 input wire [11:0] s_axi_araddr,input wire [2:0] s_axi_arprot,input wire s_axi_arvalid,output wire s_axi_arready,
 output reg [31:0] s_axi_rdata,output wire [1:0] s_axi_rresp,output reg s_axi_rvalid,input wire s_axi_rready,
 output wire write_enable,output wire [11:0] write_address,read_address,output wire [31:0] write_data,
 output wire [3:0] write_strobe,input wire write_reject,input wire [31:0] read_data,
 output wire ui_write_enable,ui_read_enable,output wire [11:0] ui_address,output wire [31:0] ui_write_data,
 output wire [3:0] ui_write_strobe,input wire ui_write_reject,ui_ready,input wire [31:0] ui_read_data);
 reg have_aw,have_w,have_ar,ui_busy,ui_writing;
 reg [11:0] aw,ar;reg [31:0] wd;reg [3:0] ws;
 wire read_ui=ar>=12'h300 && ar<12'h338;
 wire write_ui=aw>=12'h300 && aw<12'h338;
 wire read_candidate=have_ar && !s_axi_rvalid && !ui_busy;
 wire write_candidate=have_aw && have_w && !s_axi_bvalid && !ui_busy && !have_ar;
 wire source_ready,source_done,destination_valid;wire [48:0] request;wire [32:0] response;
 wire start_ui=source_ready && ((read_candidate && read_ui) || (write_candidate && write_ui));
 wire start_write=!read_candidate;
 assign s_axi_awready=rst_n && !have_aw && !s_axi_bvalid;
 assign s_axi_wready=rst_n && !have_w && !s_axi_bvalid;
 assign s_axi_arready=rst_n && !have_ar && !s_axi_rvalid;
 assign s_axi_rresp=0;
 assign write_enable=rst_n && write_candidate && !write_ui;
 assign write_address=aw;assign read_address=ar;assign write_data=wd;assign write_strobe=ws;
 assign {ui_write_enable,ui_read_enable}={destination_valid && request[48],destination_valid && !request[48]};
 assign {ui_address,ui_write_data,ui_write_strobe}=request[47:0];
 lds_net_cdc_rpc #(.WIDTH(49),.RESPONSE_WIDTH(33)) ui_bus_rpc(
  .source_clk(clk),.source_rst_n(rst_n),.destination_clk(memory_clk),.destination_rst_n(memory_rst_n),
  .source_valid(start_ui),.source_ready(source_ready),.source_data({start_write,start_write?aw:ar,wd,ws}),
  .source_done(source_done),.source_response(response),.destination_valid(destination_valid),
  .destination_ready(ui_ready),.destination_data(request),.destination_response({ui_write_reject,ui_read_data}));
 always @(posedge clk)begin
  if(!rst_n)begin
   have_aw<=0;have_w<=0;have_ar<=0;ui_busy<=0;ui_writing<=0;aw<=0;ar<=0;wd<=0;ws<=0;
   s_axi_bresp<=0;s_axi_bvalid<=0;s_axi_rdata<=0;s_axi_rvalid<=0;
  end else begin
   if(s_axi_awvalid && s_axi_awready)begin have_aw<=1;aw<=s_axi_awaddr;end
   if(s_axi_wvalid && s_axi_wready)begin have_w<=1;wd<=s_axi_wdata;ws<=s_axi_wstrb;end
   if(s_axi_arvalid && s_axi_arready)begin have_ar<=1;ar<=s_axi_araddr;end
   if(s_axi_bvalid && s_axi_bready)s_axi_bvalid<=0;
   if(s_axi_rvalid && s_axi_rready)s_axi_rvalid<=0;
   if(write_enable)begin have_aw<=0;have_w<=0;s_axi_bvalid<=1;s_axi_bresp<=write_reject?2:0;end
   if(read_candidate && !read_ui)begin have_ar<=0;s_axi_rvalid<=1;s_axi_rdata<=read_data;end
   if(start_ui)begin
    ui_busy<=1;ui_writing<=start_write;
    if(start_write)begin have_aw<=0;have_w<=0;end else have_ar<=0;
   end
   if(source_done)begin
    ui_busy<=0;
    if(ui_writing)begin s_axi_bvalid<=1;s_axi_bresp<=response[32]?2:0;end
    else begin s_axi_rvalid<=1;s_axi_rdata<=response[31:0];end
   end
  end
 end
endmodule
