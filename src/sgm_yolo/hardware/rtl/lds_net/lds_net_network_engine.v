`timescale 1ns/1ps
// Correctness baseline: one frame launch executes all 48 real operators and
// five NCHW exports in PL. This revision deliberately spills between nodes;
// it is not the fused/dual-array performance implementation or a board image.
module lds_net_network_engine(
    input wire clk,rst_n,launch_valid,output wire launch_ready,
    input wire [31:0] input_base,scratch_base,packet_base,program_base,input_frame,
    input wire abort,
    output wire frame_done,frame_failed,output wire [31:0] error_code,result_frame,frame_cycles,
    output wire profile_valid,output wire [5:0] profile_node,output wire [31:0] profile_cycles,
    output wire [31:0] araddr,output wire [7:0] arlen,output wire arvalid,input wire arready,
    input wire [63:0] rdata,input wire [1:0] rresp,input wire rlast,rvalid,output wire rready,
    output wire [31:0] awaddr,output wire [7:0] awlen,output wire awvalid,input wire awready,
    output wire [63:0] wdata,output wire [7:0] wstrb,output wire wlast,wvalid,input wire wready,
    input wire [1:0] bresp,input wire bvalid,output wire bready,
    output reg [31:0] profile_source_bytes,profile_program_bytes,profile_destination_bytes);
    wire command_valid,command_ready,cancel;wire [5:0] command_index;wire [1023:0] descriptor;
    wire [31:0] source0,source1,destination;
    reg active,completion,completion_error;reg [1:0] owner;
    wire [2:0] op_ready,op_done,op_error;
    wire [95:0] op_araddr,op_awaddr,op_source_bytes,op_destination_bytes;
    wire [23:0] op_arlen,op_awlen,op_wstrb;
    wire [191:0] op_wdata;
    wire [2:0] op_arvalid,op_rready,op_awvalid,op_wlast,op_wvalid,op_bready;
    wire [31:0] conv_program_bytes;
    wire engine_idle=!active && (&op_ready);
    wire engine_error=completion && completion_error;
    lds_net_graph_controller controller(.clk(clk),.rst_n(rst_n),.launch_valid(launch_valid),.launch_ready(launch_ready),
        .input_base(input_base),.scratch_base(scratch_base),.packet_base(packet_base),.input_frame(input_frame),.abort(abort),
        .command_valid(command_valid),.command_ready(command_ready),.command_index(command_index),.descriptor(descriptor),
        .source0(source0),.source1(source1),.destination(destination),.engine_done(completion),.engine_error(engine_error),
        .engine_idle(engine_idle),.cancel(cancel),.frame_done(frame_done),.frame_failed(frame_failed),.error_code(error_code),
        .result_frame(result_frame),.frame_cycles(frame_cycles),.profile_valid(profile_valid),.profile_node(profile_node),.profile_cycles(profile_cycles));
    assign command_ready=engine_idle && !cancel;
    wire accept=command_valid && command_ready;
    wire [31:0] opcode=descriptor[31:0];
    lds_net_conv_operator convolution(.clk(clk),.rst_n(rst_n),.start(accept && opcode==1),.cancel(cancel),
        .layer_index(descriptor[24*32+:6]),.descriptor(descriptor),.source(source0),.destination(destination),.program_base(program_base),
        .first_row(10'd0),.row_count(10'd0),.ready(op_ready[0]),.done(op_done[0]),.error(op_error[0]),
        .araddr(op_araddr[0+:32]),.arlen(op_arlen[0+:8]),.arvalid(op_arvalid[0]),.arready(arready && owner==0),
        .rdata(rdata),.rresp(rresp),.rlast(rlast),.rvalid(rvalid && owner==0),.rready(op_rready[0]),
        .awaddr(op_awaddr[0+:32]),.awlen(op_awlen[0+:8]),.awvalid(op_awvalid[0]),.awready(awready && owner==0),
        .wdata(op_wdata[0+:64]),.wstrb(op_wstrb[0+:8]),.wlast(op_wlast[0]),.wvalid(op_wvalid[0]),.wready(wready && owner==0),
        .bresp(bresp),.bvalid(bvalid && owner==0),.bready(op_bready[0]),.source_bytes(op_source_bytes[0+:32]),
        .program_bytes(conv_program_bytes),.destination_bytes(op_destination_bytes[0+:32]),.cycles());
    lds_net_vector_operator vector_ops(.clk(clk),.rst_n(rst_n),.start(accept && (opcode==2 || opcode==3)),.cancel(cancel),
        .descriptor(descriptor),.source0(source0),.source1(source1),.destination(destination),.first_row(10'd0),.row_count(10'd0),
        .ready(op_ready[1]),.done(op_done[1]),.error(op_error[1]),
        .araddr(op_araddr[32+:32]),.arlen(op_arlen[8+:8]),.arvalid(op_arvalid[1]),.arready(arready && owner==1),
        .rdata(rdata),.rresp(rresp),.rlast(rlast),.rvalid(rvalid && owner==1),.rready(op_rready[1]),
        .awaddr(op_awaddr[32+:32]),.awlen(op_awlen[8+:8]),.awvalid(op_awvalid[1]),.awready(awready && owner==1),
        .wdata(op_wdata[64+:64]),.wstrb(op_wstrb[8+:8]),.wlast(op_wlast[1]),.wvalid(op_wvalid[1]),.wready(wready && owner==1),
        .bresp(bresp),.bvalid(bvalid && owner==1),.bready(op_bready[1]),.source_bytes(op_source_bytes[32+:32]),
        .destination_bytes(op_destination_bytes[32+:32]),.cycles());
    lds_net_export_operator export_packet(.clk(clk),.rst_n(rst_n),.start(accept && opcode==4),.cancel(cancel),
        .descriptor(descriptor),.source(source0),.destination(destination),.ready(op_ready[2]),.done(op_done[2]),.error(op_error[2]),
        .araddr(op_araddr[64+:32]),.arlen(op_arlen[16+:8]),.arvalid(op_arvalid[2]),.arready(arready && owner==2),
        .rdata(rdata),.rresp(rresp),.rlast(rlast),.rvalid(rvalid && owner==2),.rready(op_rready[2]),
        .awaddr(op_awaddr[64+:32]),.awlen(op_awlen[16+:8]),.awvalid(op_awvalid[2]),.awready(awready && owner==2),
        .wdata(op_wdata[128+:64]),.wstrb(op_wstrb[16+:8]),.wlast(op_wlast[2]),.wvalid(op_wvalid[2]),.wready(wready && owner==2),
        .bresp(bresp),.bvalid(bvalid && owner==2),.bready(op_bready[2]),.source_bytes(op_source_bytes[64+:32]),
        .destination_bytes(op_destination_bytes[64+:32]),.cycles());
    assign araddr=op_araddr[owner*32+:32];assign arlen=op_arlen[owner*8+:8];assign arvalid=op_arvalid[owner];
    assign rready=op_rready[owner];assign awaddr=op_awaddr[owner*32+:32];assign awlen=op_awlen[owner*8+:8];
    assign awvalid=op_awvalid[owner];assign wdata=op_wdata[owner*64+:64];assign wstrb=op_wstrb[owner*8+:8];
    assign wlast=op_wlast[owner];assign wvalid=op_wvalid[owner];assign bready=op_bready[owner];
    always @(posedge clk)begin
        if(!rst_n)begin
            active<=0;completion<=0;completion_error<=0;owner<=0;
            profile_source_bytes<=0;profile_program_bytes<=0;profile_destination_bytes<=0;
        end else begin
            completion<=0;
            if(accept)begin
                completion_error<=0;
                if(opcode==1 || opcode==2 || opcode==3 || opcode==4)begin
                    active<=1;owner<=opcode==1?0:opcode==4?2:1;
                end else begin completion<=1;completion_error<=1;end
            end
            if(active && op_done[owner])begin
                active<=0;completion<=1;completion_error<=op_error[owner];
                profile_source_bytes<=op_source_bytes[owner*32+:32];
                profile_program_bytes<=owner==0?conv_program_bytes:0;
                profile_destination_bytes<=op_destination_bytes[owner*32+:32];
            end
        end
    end
endmodule
