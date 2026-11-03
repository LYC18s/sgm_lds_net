`timescale 1ns/1ps
// Autonomous capture -> complete integer network -> PL road restore. These
// three masters deliberately stay independent: Vivado interconnect routes
// network traffic to HP3, capture/mask traffic to HP0. No pixel/feature memcpy
// or per-layer command is performed by PS.
module lds_net_perception_core(
    input wire clk,rst_n,
    input wire write_enable,input wire [11:0] write_address,input wire [31:0] write_data,input wire [3:0] write_strobe,
    output wire write_reject,input wire [11:0] read_address,output wire [31:0] read_data,
    input wire [23:0] s_data,input wire s_sof,s_eol,s_valid,output wire s_ready,
    input wire branch_cancel,input wire [31:0] raw_drops,output wire tap_enable,
    input wire authorize_valid,output wire authorize_ready,input wire authorize_slot,input wire [31:0] authorize_frame,authorize_base,output wire authorize_accept,
    input wire display_event_valid,output wire display_event_ready,input wire display_event_clear,display_event_slot,input wire [31:0] display_event_frame,
    output wire [31:0] net_araddr,output wire [7:0] net_arlen,output wire net_arvalid,input wire net_arready,
    input wire [63:0] net_rdata,input wire [1:0] net_rresp,input wire net_rlast,net_rvalid,output wire net_rready,
    output wire [31:0] net_awaddr,output wire [7:0] net_awlen,output wire net_awvalid,input wire net_awready,
    output wire [63:0] net_wdata,output wire [7:0] net_wstrb,output wire net_wlast,net_wvalid,input wire net_wready,
    input wire [1:0] net_bresp,input wire net_bvalid,output wire net_bready,
    output wire [31:0] cap_awaddr,output wire [7:0] cap_awlen,output wire cap_awvalid,input wire cap_awready,
    output wire [63:0] cap_wdata,output wire [7:0] cap_wstrb,output wire cap_wlast,cap_wvalid,input wire cap_wready,
    input wire [1:0] cap_bresp,input wire cap_bvalid,output wire cap_bready,
    output wire [31:0] mask_araddr,output wire [7:0] mask_arlen,output wire mask_arvalid,input wire mask_arready,
    input wire [63:0] mask_rdata,input wire [1:0] mask_rresp,input wire mask_rlast,mask_rvalid,output wire mask_rready,
    output wire [31:0] mask_awaddr,output wire [7:0] mask_awlen,output wire mask_awvalid,input wire mask_awready,
    output wire [63:0] mask_wdata,output wire [7:0] mask_wstrb,output wire mask_wlast,mask_wvalid,input wire mask_wready,
    input wire [1:0] mask_bresp,input wire mask_bvalid,output wire mask_bready);
    wire enable,abort_all,configuration_idle,protocol_error;
    wire color_enable;wire [31:0] host_rgb_address;
    wire [63:0] input_bases,packet_bases,mask_bases;wire [31:0] scratch_base,program_base;
    wire [3:0] input_states;wire [5:0] output_states;
    wire cap_valid,cap_ready,cap_unit_ready,cap_done,cap_error,net_valid,net_ready,net_done,net_failed,mask_valid,mask_ready,mask_done,mask_error;
    wire [31:0] cap_address,cap_frame,source_counter,net_input,net_packet,net_frame,mask_logits,mask_destination;
    wire cancel_capture,cancel_network,cancel_mask,result_valid,result_slot,result_take,host_discard,host_slot;
    wire [31:0] result_frame,result_packet,result_mask,host_frame;
    wire submit_ready,active_valid,active_slot;wire [31:0] active_frame;
    wire [31:0] captured_frames,inferred_frames,restored_frames,failed_jobs,crop_drops;
    wire [31:0] unused_cap_bytes,unused_mask_read,unused_mask_write,mask_cycles,error_code,network_result_frame,frame_cycles;
    wire profile_valid;wire [5:0] profile_node;wire [31:0] profile_cycles,profile_source,profile_program,profile_destination;
    wire [255:0] model_sha;wire [1023:0] unused_descriptor;wire [5:0] unused_count;wire [31:0] unused_scratch;
    lds_net_graph_rom identity(6'd0,unused_descriptor,unused_count,unused_scratch,model_sha);
    assign tap_enable=enable;
    assign cap_ready=cap_unit_ready && !branch_cancel;
    assign authorize_ready=1'b1;
    assign authorize_accept=enable && submit_ready && authorize_base==mask_bases[authorize_slot*32+:32];
    wire publish=authorize_valid && authorize_ready && authorize_accept;
    assign display_event_ready=1'b1;
    lds_net_perception_registers registers(.clk(clk),.rst_n(rst_n),.write_enable(write_enable),.write_address(write_address),.write_data(write_data),.write_strobe(write_strobe),
        .write_reject(write_reject),.read_address(read_address),.read_data(read_data),.model_sha256(model_sha),.enable(enable),.abort_all(abort_all),
        .input_bases(input_bases),.packet_bases(packet_bases),.mask_bases(mask_bases),.scratch_base(scratch_base),.program_base(program_base),
        .configuration_idle(configuration_idle),.protocol_error(protocol_error),.input_states(input_states),.output_states(output_states),
        .result_valid(result_valid),.result_slot(result_slot),.result_frame(result_frame),.result_packet(result_packet),.result_mask(result_mask),.result_take(result_take),
        .host_discard(host_discard),.host_slot(host_slot),.host_frame(host_frame),.host_published(publish),
        .active_valid(active_valid),.active_slot(active_slot),.active_frame(active_frame),.captured_frames(captured_frames),.inferred_frames(inferred_frames),.restored_frames(restored_frames),
        .failed_jobs(failed_jobs),.raw_drops(raw_drops),.crop_drops(crop_drops),.capture_counter(source_counter),.mask_cycles(mask_cycles),
        .frame_done(net_done),.frame_failed(net_failed),.error_code(error_code),.network_frame(network_result_frame),.frame_cycles(frame_cycles),
        .profile_valid(profile_valid),.profile_node(profile_node),.profile_cycles(profile_cycles),.profile_source_bytes(profile_source),.profile_program_bytes(profile_program),.profile_destination_bytes(profile_destination),
        .color_enable(color_enable),.host_rgb_address(host_rgb_address));
    lds_net_frame_manager ownership(.clk(clk),.rst_n(rst_n),.enable(enable),.abort_all(abort_all),.input_bases(input_bases),.packet_bases(packet_bases),.mask_bases(mask_bases),
        .configuration_idle(configuration_idle),.capture_valid(cap_valid),.capture_ready(cap_ready),.capture_address(cap_address),.capture_done(cap_done),.capture_error(cap_error),.capture_frame(cap_frame),
        .network_valid(net_valid),.network_ready(net_ready),.network_input(net_input),.network_packet(net_packet),.network_frame(net_frame),.network_done(net_done),.network_failed(net_failed),
        .mask_valid(mask_valid),.mask_ready(mask_ready),.mask_logits(mask_logits),.mask_destination(mask_destination),.mask_done(mask_done),.mask_error(mask_error),
        .cancel_capture(cancel_capture),.cancel_network(cancel_network),.cancel_mask(cancel_mask),
        .result_valid(result_valid),.result_slot(result_slot),.result_frame(result_frame),.result_packet(result_packet),.result_mask(result_mask),.result_take(result_take),
        .host_discard(host_discard),.host_slot(host_slot),.host_frame(host_frame),.display_submit(publish),.display_slot(authorize_slot),.display_frame(authorize_frame),.display_submit_ready(submit_ready),.protocol_error(protocol_error),
        .display_committed(display_event_valid && display_event_ready && !display_event_clear),.display_clear_committed(display_event_valid && display_event_ready && display_event_clear),.committed_slot(display_event_slot),.committed_frame(display_event_frame),
        .active_valid(active_valid),.active_slot(active_slot),.active_frame(active_frame),.captured_frames(captured_frames),.inferred_frames(inferred_frames),.restored_frames(restored_frames),.failed_jobs(failed_jobs),
        .input_states(input_states),.output_states(output_states),.color_enable(color_enable),.host_rgb_address(host_rgb_address));
    lds_net_input_capture capture(.clk(clk),.rst_n(rst_n),.command_valid(cap_valid && !branch_cancel),.command_ready(cap_unit_ready),.command_address(cap_address),.cancel(cancel_capture || branch_cancel),
        .s_data(s_data),.s_valid(s_valid),.s_sof(s_sof),.s_eol(s_eol),.s_ready(s_ready),.done(cap_done),.error(cap_error),.frame_id(cap_frame),.source_counter(source_counter),
        .awaddr(cap_awaddr),.awlen(cap_awlen),.awvalid(cap_awvalid),.awready(cap_awready),.wdata(cap_wdata),.wstrb(cap_wstrb),.wlast(cap_wlast),.wvalid(cap_wvalid),.wready(cap_wready),
        .bresp(cap_bresp),.bvalid(cap_bvalid),.bready(cap_bready),.bytes_written(unused_cap_bytes),.dropped_frames(crop_drops));
    lds_net_network_engine network(.clk(clk),.rst_n(rst_n),.launch_valid(net_valid),.launch_ready(net_ready),.input_base(net_input),.scratch_base(scratch_base),.packet_base(net_packet),.program_base(program_base),.input_frame(net_frame),.abort(cancel_network),
        .frame_done(net_done),.frame_failed(net_failed),.error_code(error_code),.result_frame(network_result_frame),.frame_cycles(frame_cycles),.profile_valid(profile_valid),.profile_node(profile_node),.profile_cycles(profile_cycles),
        .profile_source_bytes(profile_source),.profile_program_bytes(profile_program),.profile_destination_bytes(profile_destination),
        .araddr(net_araddr),.arlen(net_arlen),.arvalid(net_arvalid),.arready(net_arready),.rdata(net_rdata),.rresp(net_rresp),.rlast(net_rlast),.rvalid(net_rvalid),.rready(net_rready),
        .awaddr(net_awaddr),.awlen(net_awlen),.awvalid(net_awvalid),.awready(net_awready),.wdata(net_wdata),.wstrb(net_wstrb),.wlast(net_wlast),.wvalid(net_wvalid),.wready(net_wready),.bresp(net_bresp),.bvalid(net_bvalid),.bready(net_bready));
    lds_net_mask_dma restore(.clk(clk),.rst_n(rst_n),.start(mask_valid),.cancel(cancel_mask),.logits_address(mask_logits),.mask_address(mask_destination),.ready(mask_ready),.done(mask_done),.error(mask_error),
        .araddr(mask_araddr),.arlen(mask_arlen),.arvalid(mask_arvalid),.arready(mask_arready),.rdata(mask_rdata),.rresp(mask_rresp),.rlast(mask_rlast),.rvalid(mask_rvalid),.rready(mask_rready),
        .awaddr(mask_awaddr),.awlen(mask_awlen),.awvalid(mask_awvalid),.awready(mask_awready),.wdata(mask_wdata),.wstrb(mask_wstrb),.wlast(mask_wlast),.wvalid(mask_wvalid),.wready(mask_wready),.bresp(mask_bresp),.bvalid(mask_bvalid),.bready(mask_bready),
        .bytes_read(unused_mask_read),.bytes_written(unused_mask_write),.cycles(mask_cycles));
endmodule
