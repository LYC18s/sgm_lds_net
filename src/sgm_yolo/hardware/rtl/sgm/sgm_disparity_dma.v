`timescale 1ns/1ps
// Streamed result DDR cache. The SGM/camera INPUT never accesses DDR.
// Three private output buffers; publish only complete, validated frames after
// all AXI write responses. Software parks the reader and acknowledges its
// active frame before another result can be published. No full-frame BRAM.
module sgm_disparity_dma #(parameter IMG_WIDTH=640,IMG_HEIGHT=480,TRANSACTIONAL_RESULT=1,SNAPSHOT_TIMEOUT_CYCLES=100000000)(
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME aclk, ASSOCIATED_BUSIF S_AXI_LITE:S_AXIS:M_AXI, ASSOCIATED_RESET aresetn" *)
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 aclk CLK" *)
    input wire aclk,
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME aresetn, POLARITY ACTIVE_LOW" *)
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 aresetn RST" *)
    input wire aresetn,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE AWADDR" *)
    input wire [8:0] s_axi_awaddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE AWVALID" *)
    input wire s_axi_awvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE AWREADY" *)
    output wire s_axi_awready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE WDATA" *)
    input wire [31:0] s_axi_wdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE WSTRB" *)
    input wire [3:0] s_axi_wstrb,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE WVALID" *)
    input wire s_axi_wvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE WREADY" *)
    output wire s_axi_wready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE BRESP" *)
    output wire [1:0] s_axi_bresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE BVALID" *)
    output wire s_axi_bvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE BREADY" *)
    input wire s_axi_bready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE ARADDR" *)
    input wire [8:0] s_axi_araddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE ARVALID" *)
    input wire s_axi_arvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE ARREADY" *)
    output wire s_axi_arready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE RDATA" *)
    output wire [31:0] s_axi_rdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE RRESP" *)
    output wire [1:0] s_axi_rresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE RVALID" *)
    output wire s_axi_rvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE RREADY" *)
    input wire s_axi_rready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TDATA" *)
    input wire [7:0] s_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TKEEP" *)
    input wire s_axis_tkeep,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TUSER" *)
    input wire s_axis_tuser,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TLAST" *)
    input wire s_axis_tlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TVALID" *)
    input wire s_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TREADY" *)
    output wire s_axis_tready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI AWADDR" *)
    output wire [31:0] m_axi_awaddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI AWLEN" *)
    output wire [7:0] m_axi_awlen,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI AWSIZE" *)
    output wire [2:0] m_axi_awsize,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI AWBURST" *)
    output wire [1:0] m_axi_awburst,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI AWLOCK" *)
    output wire m_axi_awlock,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI AWCACHE" *)
    output wire [3:0] m_axi_awcache,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI AWPROT" *)
    output wire [2:0] m_axi_awprot,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI AWVALID" *)
    output wire m_axi_awvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI AWREADY" *)
    input wire m_axi_awready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI WDATA" *)
    output wire [63:0] m_axi_wdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI WSTRB" *)
    output wire [7:0] m_axi_wstrb,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI WLAST" *)
    output wire m_axi_wlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI WVALID" *)
    output wire m_axi_wvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI WREADY" *)
    input wire m_axi_wready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI BRESP" *)
    input wire [1:0] m_axi_bresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI BVALID" *)
    input wire m_axi_bvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI BREADY" *)
    output wire m_axi_bready
);

    localparam N=IMG_WIDTH*IMG_HEIGHT;
    localparam SEEK=0,FILL=1,ADDRESS=2,SEND=3,RESPONSE=4,WAIT_ACK=5;
    reg [2:0] state=SEEK;
    reg [31:0] control=0, stride=0,hsize=0,vsize=0;
    reg [31:0] base_address[0:2];
    reg configured=0,reset_pending=0,completed=0,has_published=0;
    reg [31:0] sequence=0,ack_sequence=0,aborted=0;
    reg [1:0] published=0,write_index=1;
    reg [31:0] errors=0,byte_offset=0;
    reg snapshot_pending=0,snapshot_capture=0,snapshot_ready=0,snapshot_error=0,snapshot_timeout=0;
    reg [31:0] snapshot_tag=0,snapshot_id=0,snapshot_sequence=0,snapshot_age=0;
    reg [1:0] snapshot_index=0;
    reg [19:0] pixel_count=0;
    reg [9:0] x=0;
    reg [2:0] pack_slot=0;
    reg [4:0] fill_word=0,send_word=0;
    reg [63:0] pack=0;
    reg frame_ended=0,frame_bad=0;
    (* ram_style="distributed" *) reg [63:0] burst[0:31];
    reg [63:0] packed_word;
    always @* begin
        packed_word=pack;
        packed_word[pack_slot*8+:8]=s_axis_tdata;
    end
    wire enabled=control[0] && configured && !reset_pending;
    wire axis_fire=s_axis_tvalid && s_axis_tready;
    assign s_axis_tready=enabled && (state==SEEK || state==FILL);
    always @(posedge aclk)
        if(axis_fire && state==FILL && !s_axis_tuser && pack_slot==7)
            burst[fill_word]<=packed_word;
    assign m_axi_awaddr=base_address[write_index]+byte_offset;
    assign m_axi_awlen=8'd31;
    assign m_axi_awsize=3'd3;
    assign m_axi_awburst=2'b01;
    assign m_axi_awlock=0;
    assign m_axi_awcache=4'b0011;
    assign m_axi_awprot=0;
    assign m_axi_awvalid=state==ADDRESS;
    assign m_axi_wdata=burst[send_word];
    assign m_axi_wstrb=8'hff;
    assign m_axi_wlast=send_word==31;
    assign m_axi_wvalid=state==SEND;
    assign m_axi_bready=state==RESPONSE;

    // AXI-Lite AW and W may arrive independently. Hold responses until accepted.
    reg have_aw=0,have_w=0,bvalid=0,rvalid=0;
    reg [8:0] awaddr=0;
    reg [31:0] wdata=0,rdata=0;
    reg [3:0] wstrb=0;
    assign s_axi_awready=!have_aw && !bvalid;
    assign s_axi_wready=!have_w && !bvalid;
    assign s_axi_bvalid=bvalid;
    assign s_axi_bresp=0;
    assign s_axi_arready=!rvalid;
    assign s_axi_rvalid=rvalid;
    assign s_axi_rdata=rdata;
    assign s_axi_rresp=0;
    function [31:0] merge;
        input [31:0] old_value,new_value;
        input [3:0] strobe;
        integer j;
        begin
            merge=old_value;
            for(j=0;j<4;j=j+1) if(strobe[j]) merge[j*8+:8]=new_value[j*8+:8];
        end
    endfunction
    wire register_write=have_aw && have_w && !bvalid;
    wire soft_reset=register_write && awaddr==9'h030 && wstrb[0] && wdata[2];
    wire snapshot_cancel=register_write && awaddr==9'h114 && wstrb==4'hf && wdata==snapshot_id;
    // Reset/halt never cancels an AXI transaction already issued. Finish its B
    // response first; discarded frame buffers are never published to the reader.
    wire can_reset=state==SEEK || state==FILL || state==WAIT_ACK;
    integer i;
    always @(posedge aclk) begin
        if(!aresetn) begin
            state<=SEEK;control<=0;configured<=0;reset_pending<=0;
            have_aw<=0;have_w<=0;bvalid<=0;rvalid<=0;
            completed<=0;has_published<=0;sequence<=0;ack_sequence<=0;
            snapshot_pending<=0;snapshot_capture<=0;snapshot_ready<=0;snapshot_error<=0;snapshot_timeout<=0;
            snapshot_tag<=0;snapshot_id<=0;snapshot_sequence<=0;snapshot_age<=0;snapshot_index<=0;
            published<=0;write_index<=1;errors<=0;aborted<=0;
            pixel_count<=0;pack_slot<=0;fill_word<=0;pack<=0;
            byte_offset<=0;frame_ended<=0;frame_bad<=0;send_word<=0;
            stride<=0;hsize<=0;vsize<=0;
            for(i=0;i<3;i=i+1) base_address[i]<=0;
        end else begin
            if(s_axi_awvalid && s_axi_awready) begin have_aw<=1;awaddr<=s_axi_awaddr;end
            if(s_axi_wvalid && s_axi_wready) begin have_w<=1;wdata<=s_axi_wdata;wstrb<=s_axi_wstrb;end
            if(bvalid && s_axi_bready) bvalid<=0;
            if(register_write) begin
                have_aw<=0;have_w<=0;bvalid<=1;
                case(awaddr)
                    9'h030: begin control<=merge(control,wdata,wstrb)&32'hfffffffb;if(soft_reset) reset_pending<=1;end
                    9'h034: begin if(wdata[12]) completed<=0;errors<=errors&~merge(0,wdata,wstrb);end
                    9'h0a0: begin
                        vsize<=merge(vsize,wdata,wstrb);
                        configured<=hsize==IMG_WIDTH && stride==IMG_WIDTH && wdata==IMG_HEIGHT
                            && base_address[0][7:0]==0 && base_address[1][7:0]==0 && base_address[2][7:0]==0;
                        if(hsize!=IMG_WIDTH || stride!=IMG_WIDTH || wdata!=IMG_HEIGHT) errors<=32'h10;
                    end
                    9'h0a4:hsize<=merge(hsize,wdata,wstrb);
                    9'h0a8:stride<=merge(stride,wdata,wstrb)&32'h00ffffff;
                    9'h0ac:base_address[0]<=merge(base_address[0],wdata,wstrb);
                    9'h0b0:base_address[1]<=merge(base_address[1],wdata,wstrb);
                    9'h0b4:base_address[2]<=merge(base_address[2],wdata,wstrb);
                    9'h0e8:ack_sequence<=merge(ack_sequence,wdata,wstrb);
                    9'h0f8:if(!snapshot_pending && !snapshot_ready)snapshot_tag<=merge(snapshot_tag,wdata,wstrb);
                    9'h0f4:if(wstrb[0] && wdata[0])begin
                        if(!snapshot_pending && !snapshot_ready)begin
                            snapshot_pending<=1;snapshot_capture<=0;snapshot_id<=snapshot_tag;
                            snapshot_age<=0;snapshot_error<=0;snapshot_timeout<=0;
                        end else snapshot_error<=1;
                    end
                    // Matching token cancels a pending request or releases its lease.
                    9'h114:if(wstrb==4'hf && wdata==snapshot_id)begin
                        snapshot_pending<=0;snapshot_capture<=0;snapshot_ready<=0;
                    end
                endcase
            end
            if(rvalid && s_axi_rready) rvalid<=0;
            if(s_axi_arvalid && s_axi_arready) begin
                rvalid<=1;
                case(s_axi_araddr)
                    9'h000:rdata<=32'h53474d31; // SGM1: transactional DMA ABI.
                    9'h028:rdata<={6'b0,write_index,24'b0};
                    9'h030:rdata<=control | (reset_pending?32'h4:0);
                    9'h034:rdata<=32'h00010000 | errors | (completed?32'h1000:0) | (enabled?0:1);
                    9'h0a0:rdata<=vsize;
                    9'h0a4:rdata<=hsize;
                    9'h0a8:rdata<=stride;
                    9'h0e0:rdata<={has_published,29'b0,published};
                    9'h0e4:rdata<=sequence;
                    9'h0e8:rdata<=ack_sequence;
                    9'h0ec:rdata<=aborted;
                    9'h0f0:rdata<=32'h534e5031; // SNP1: next-output-SOF frame lease
                    9'h0f8:rdata<=snapshot_tag;
                    9'h0fc:rdata<={27'd0,snapshot_timeout,snapshot_error,snapshot_capture,snapshot_ready,snapshot_pending};
                    9'h100:rdata<={30'd0,snapshot_index};
                    9'h104:rdata<=snapshot_sequence;
                    9'h10c:rdata<=snapshot_id;
                    default:rdata<=0;
                endcase
            end
            if(reset_pending && can_reset) begin
                state<=has_published && ack_sequence!=sequence ? WAIT_ACK : SEEK;
                reset_pending<=0;configured<=0;control<=0;
                completed<=0;errors<=0;pixel_count<=0;pack_slot<=0;fill_word<=0;
                frame_bad<=0;frame_ended<=0;
                snapshot_pending<=0;snapshot_capture<=0;snapshot_ready<=0;
                // Keep the last committed frame and sequence across soft reset.
            end else begin
                case(state)
                    SEEK: if(axis_fire && s_axis_tuser) begin
                        write_index<=published==2?0:published+1'b1;
                        pixel_count<=1;x<=1;pack<={56'b0,s_axis_tdata};pack_slot<=1;
                        fill_word<=0;byte_offset<=0;frame_ended<=0;frame_bad<=s_axis_tlast || !s_axis_tkeep;
                        snapshot_capture<=snapshot_pending;
                        state<=FILL;
                    end
                    FILL: if(axis_fire) begin
                        if(s_axis_tuser) begin
                            // Replace an incomplete frame in its private buffer.
                            aborted<=aborted+1'b1;pixel_count<=1;x<=1;
                            pack<={56'b0,s_axis_tdata};pack_slot<=1;fill_word<=0;
                            byte_offset<=0;frame_ended<=0;frame_bad<=s_axis_tlast || !s_axis_tkeep;
                            snapshot_capture<=snapshot_pending;
                        end else begin
                            pixel_count<=pixel_count+1'b1;x<=x==IMG_WIDTH-1?0:x+1'b1;
                            if(s_axis_tlast!=(x==IMG_WIDTH-1) || !s_axis_tkeep) frame_bad<=1;
                            pack<=packed_word;pack_slot<=pack_slot+1'b1;
                            if(pack_slot==7) begin
                                pack<=0;fill_word<=fill_word+1'b1;
                                if(fill_word==31) begin state<=ADDRESS;send_word<=0;end
                            end
                            if(pixel_count==N-1) frame_ended<=1;
                        end
                    end
                    ADDRESS:if(m_axi_awready) state<=SEND;
                    SEND:if(m_axi_wready) begin
                        send_word<=send_word+1'b1;
                        if(send_word==31) state<=RESPONSE;
                    end
                    RESPONSE:if(m_axi_bvalid) begin
                        if(m_axi_bresp!=0) begin errors<=32'h20;frame_bad<=1;end
                        if(frame_ended) begin
                            if(!frame_bad && m_axi_bresp==0 && !reset_pending && control[0]) begin
                                // Only after the FINAL successful DDR write response.
                                published<=write_index;has_published<=1;
                                sequence<=sequence+1'b1;completed<=1;state<=WAIT_ACK;
                                if(snapshot_capture && snapshot_pending && !snapshot_cancel && !soft_reset)begin
                                    snapshot_ready<=1;snapshot_pending<=0;snapshot_capture<=0;
                                    snapshot_sequence<=sequence+1'b1;snapshot_index<=write_index;
                                    snapshot_age<=0;
                                end
                            end else begin aborted<=aborted+1'b1;state<=SEEK;snapshot_capture<=0;end
                        end else begin byte_offset<=byte_offset+256;state<=FILL;end
                    end
                    WAIT_ACK:if(ack_sequence==sequence && !snapshot_ready) state<=SEEK;
                    default:state<=SEEK;
                endcase
            end
            // Bounded failure recovery. A reader must revalidate the lease
            // after copying: expiry invalidates its data, never a silent reuse.
            if(snapshot_pending || snapshot_ready)begin
                snapshot_age<=snapshot_age+1'b1;
                if(snapshot_age>=SNAPSHOT_TIMEOUT_CYCLES-1)begin
                    snapshot_pending<=0;snapshot_capture<=0;snapshot_ready<=0;
                    snapshot_timeout<=1;snapshot_age<=0;
                end
            end
        end
    end
    initial begin
        if(N%256!=0) $error("Transactional result DMA requires whole 256-byte bursts");
    end
endmodule
