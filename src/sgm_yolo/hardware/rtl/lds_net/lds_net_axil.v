`timescale 1ns/1ps
// Independent AXI-Lite AW/W capture; responses remain stable under backpressure.
module lds_net_axil(input wire clk,rst_n,
    input wire [11:0] s_axi_awaddr,input wire [2:0] s_axi_awprot,input wire s_axi_awvalid,output wire s_axi_awready,
    input wire [31:0] s_axi_wdata,input wire [3:0] s_axi_wstrb,input wire s_axi_wvalid,output wire s_axi_wready,
    output reg [1:0] s_axi_bresp,output reg s_axi_bvalid,input wire s_axi_bready,
    input wire [11:0] s_axi_araddr,input wire [2:0] s_axi_arprot,input wire s_axi_arvalid,output wire s_axi_arready,
    output reg [31:0] s_axi_rdata,output wire [1:0] s_axi_rresp,output reg s_axi_rvalid,input wire s_axi_rready,
    output wire write_enable,output wire [11:0] write_address,output wire [31:0] write_data,output wire [3:0] write_strobe,
    input wire write_reject,output wire [11:0] read_address,input wire [31:0] read_data);
    reg have_aw,have_w;reg [11:0] aw;reg [31:0] wd;reg [3:0] ws;
    assign s_axi_awready=rst_n && !have_aw && !s_axi_bvalid;
    assign s_axi_wready=rst_n && !have_w && !s_axi_bvalid;
    assign s_axi_arready=rst_n && !s_axi_rvalid;
    assign write_enable=have_aw && have_w && !s_axi_bvalid;
    assign write_address=aw;assign write_data=wd;assign write_strobe=ws;
    assign read_address=s_axi_araddr;assign s_axi_rresp=0;
    always @(posedge clk) begin
        if(!rst_n) begin have_aw<=0;have_w<=0;s_axi_bvalid<=0;s_axi_bresp<=0;s_axi_rvalid<=0;s_axi_rdata<=0;aw<=0;wd<=0;ws<=0;end
        else begin
            if(s_axi_awvalid && s_axi_awready) begin have_aw<=1;aw<=s_axi_awaddr;end
            if(s_axi_wvalid && s_axi_wready) begin have_w<=1;wd<=s_axi_wdata;ws<=s_axi_wstrb;end
            if(write_enable) begin have_aw<=0;have_w<=0;s_axi_bvalid<=1;s_axi_bresp<=write_reject?2:0;end
            else if(s_axi_bvalid && s_axi_bready) s_axi_bvalid<=0;
            if(s_axi_arvalid && s_axi_arready) begin s_axi_rvalid<=1;s_axi_rdata<=read_data;end
            else if(s_axi_rvalid && s_axi_rready) s_axi_rvalid<=0;
        end
    end
endmodule
