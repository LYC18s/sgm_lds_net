`timescale 1ns/1ps
// Autonomous perception ABI524c0003. Configuration is immutable until all
// capture/network/mask/host/display owners have released their slots. Enabling
// checks all eight DDR extents with one shared sequential comparator.
module lds_net_perception_registers #(parameter CLOCK_HZ=100000000)(
    input wire clk,rst_n,write_enable,input wire [11:0] write_address,
    input wire [31:0] write_data,input wire [3:0] write_strobe,output reg write_reject,
    input wire [11:0] read_address,output reg [31:0] read_data,input wire [255:0] model_sha256,
    output reg enable,abort_all,output wire [63:0] input_bases,packet_bases,mask_bases,output wire [31:0] scratch_base,program_base,
    input wire configuration_idle,protocol_error,input wire [3:0] input_states,input wire [5:0] output_states,
    input wire result_valid,result_slot,input wire [31:0] result_frame,result_packet,result_mask,output wire result_take,
    output wire host_discard,output reg host_slot,output reg [31:0] host_frame,input wire host_published,
    input wire active_valid,active_slot,input wire [31:0] active_frame,
    input wire [31:0] captured_frames,inferred_frames,restored_frames,failed_jobs,raw_drops,crop_drops,capture_counter,mask_cycles,
    input wire frame_done,frame_failed,input wire [31:0] error_code,network_frame,frame_cycles,
    input wire profile_valid,input wire [5:0] profile_node,input wire [31:0] profile_cycles,profile_source_bytes,profile_program_bytes,profile_destination_bytes,
    output reg color_enable,input wire [31:0] host_rgb_address);
    reg [31:0] addresses[0:7];
    assign input_bases={addresses[1],addresses[0]};assign scratch_base=addresses[2];assign program_base=addresses[3];
    assign packet_bases={addresses[5],addresses[4]};assign mask_bases={addresses[7],addresses[6]};
    reg host_valid;reg [31:0] host_packet,host_mask,last_cycles,last_frame,last_error,configuration_error;
    reg validating;reg [2:0] check_i,check_j;reg [5:0] profile_index;
    wire config_address=write_address>=12'h010 && write_address<=12'h02c;
    wire control_address=write_address==4;
    wire profile_address=write_address==12'h030;
    wire color_address=write_address==12'h0c0;
    wire config_locked=enable || validating || !configuration_idle || host_valid;
    wire low_write=write_enable && write_strobe[0];
    wire stop_request=low_write && control_address && (!write_data[0] || write_data[1]);
    always @*begin
        write_reject=(|write_strobe) && (write_address[1:0]!=0 || !(config_address || control_address || profile_address || color_address) ||
            (color_address && (config_locked || (write_strobe[0] && |write_data[7:1]))) ||
            (config_address && config_locked) || (profile_address && write_strobe[0] && write_data[5:0]>52) ||
            (control_address && write_strobe[0] && (
                (write_data[0] && !enable && (validating || !configuration_idle || host_valid)) ||
                (write_data[2] && (!result_valid || host_valid)) || (write_data[3] && !host_valid) || (write_data[2] && write_data[3]))));
    end
    assign result_take=write_enable && !write_reject && control_address && write_strobe[0] && write_data[2];
    assign host_discard=write_enable && !write_reject && control_address && write_strobe[0] && write_data[3];
    function [31:0] merge;
        input [31:0] a,b;input [3:0] strobes;integer j;
        begin for(j=0;j<4;j=j+1)merge[j*8+:8]=strobes[j]?b[j*8+:8]:a[j*8+:8];end
    endfunction
    function [31:0] extent;
        input [2:0] index;
        begin case(index)0,1:extent=307200;2:extent=614400;3:extent=376384;4,5:extent=26240;default:extent=57600;endcase end
    endfunction
    wire [32:0] end_i={1'b0,addresses[check_i]}+{1'b0,extent(check_i)};
    wire [32:0] end_j={1'b0,addresses[check_j]}+{1'b0,extent(check_j)};
    wire bad_alignment=addresses[check_i]==0 || addresses[check_i][5:0]!=0;
    wire overflow=end_i>33'h100000000;
    wire overlap=check_j<check_i && {1'b0,addresses[check_i]}<end_j && {1'b0,addresses[check_j]}<end_i;
    // Double-banked profiles expose one COMPLETE successful inference. The
    // writer fills the other bank; readers retry if register0xbc changes.
    (* ram_style="distributed" *) reg [31:0] node_cycles[0:127],node_source[0:127],node_program[0:127],node_destination[0:127];
    reg read_bank,write_bank,profile_available;
    wire [6:0] profile_read_index={read_bank,profile_index};
    always @(posedge clk)if(profile_valid && profile_node<53)begin
        node_cycles[{write_bank,profile_node}]<=profile_cycles;node_source[{write_bank,profile_node}]<=profile_source_bytes;
        node_program[{write_bank,profile_node}]<=profile_program_bytes;node_destination[{write_bank,profile_node}]<=profile_destination_bytes;
    end
    always @*begin
        read_data=0;
        case(read_address)
            12'h000:read_data=32'h524c0003;
            12'h004:read_data={25'd0,protocol_error,active_valid,host_valid,result_valid && !host_valid,configuration_idle,validating,enable};
            12'h008:read_data=configuration_error!=0?configuration_error:last_error;
            12'h00c:read_data=32'h0000001f;
            12'h010,12'h014,12'h018,12'h01c,12'h020,12'h024,12'h028,12'h02c:read_data=addresses[read_address[5:2]-4];
            12'h030:read_data={26'd0,profile_index};
            12'h034:read_data=profile_available?node_cycles[profile_read_index]:0;
            12'h038:read_data=profile_available?node_source[profile_read_index]:0;
            12'h03c:read_data=profile_available?node_program[profile_read_index]:0;
            12'h040:read_data=profile_available?node_destination[profile_read_index]:0;
            12'h044:read_data=CLOCK_HZ;
            12'h048:read_data=captured_frames;
            12'h04c:read_data=inferred_frames;
            12'h050:read_data=restored_frames;
            12'h054:read_data=failed_jobs;
            12'h058:read_data={28'd0,input_states};
            12'h05c:read_data={26'd0,output_states};
            12'h060:read_data=host_frame;
            12'h064:read_data={31'd0,host_slot};
            12'h068:read_data=host_packet;
            12'h06c:read_data=host_mask;
            12'h070:read_data=last_cycles;
            12'h074:read_data=active_frame;
            12'h078:read_data={31'd0,active_slot};
            12'h07c:read_data=capture_counter;
            12'h080:read_data=model_sha256[0+:32];
            12'h084:read_data=model_sha256[32+:32];
            12'h088:read_data=model_sha256[64+:32];
            12'h08c:read_data=model_sha256[96+:32];
            12'h090:read_data=model_sha256[128+:32];
            12'h094:read_data=model_sha256[160+:32];
            12'h098:read_data=model_sha256[192+:32];
            12'h09c:read_data=model_sha256[224+:32];
            12'h0a0:read_data=307200;
            12'h0a4:read_data=614400;
            12'h0a8:read_data=26200;
            12'h0ac:read_data=57600;
            12'h0b0:read_data=raw_drops;
            12'h0b4:read_data=crop_drops;
            12'h0b8:read_data=mask_cycles;
            12'h0bc:read_data=last_frame;
            12'h0c0:read_data={31'd0,color_enable};
            12'h0c4:read_data=32'h52474231; // RGB1 owned inference input
            12'h0c8:read_data=host_valid?host_rgb_address:0;
            12'h0cc:read_data=host_valid && host_rgb_address!=0?host_frame:0;
        endcase
    end
    integer r;
    always @(posedge clk)begin
        if(!rst_n)begin
            for(r=0;r<8;r=r+1)addresses[r]<=0;
            enable<=0;abort_all<=0;host_valid<=0;host_slot<=0;host_frame<=0;host_packet<=0;host_mask<=0;
            color_enable<=0;
            validating<=0;check_i<=0;check_j<=0;profile_index<=0;last_cycles<=0;last_frame<=0;last_error<=0;configuration_error<=0;
            read_bank<=0;write_bank<=1;profile_available<=0;
        end else begin
            abort_all<=0;
            if(write_enable && !write_reject)begin
                if(config_address)addresses[write_address[5:2]-4]<=merge(addresses[write_address[5:2]-4],write_data,write_strobe);
                if(profile_address && write_strobe[0])profile_index<=write_data[5:0];
                if(color_address && write_strobe[0])color_enable<=write_data[0];
                if(control_address && write_strobe[0])begin
                    if(write_data[4])begin last_error<=0;configuration_error<=0;end
                    if(write_data[0] && !enable && !validating)begin validating<=1;check_i<=0;check_j<=0;configuration_error<=0;end
                    if(result_take)begin host_valid<=1;host_slot<=result_slot;host_frame<=result_frame;host_packet<=result_packet;host_mask<=result_mask;end
                    if(host_discard)host_valid<=0;
                end
            end
            if(host_published)host_valid<=0;
            if(stop_request && !write_reject)begin enable<=0;validating<=0;abort_all<=1;end
            else if(validating)begin
                if(bad_alignment || overflow || overlap)begin
                    validating<=0;enable<=0;configuration_error<=bad_alignment?32'h1001:overflow?32'h1002:32'h1003;
                end else if(check_j==check_i)begin
                    if(check_i==7)begin validating<=0;enable<=1;end
                    else begin check_i<=check_i+1'b1;check_j<=0;end
                end else check_j<=check_j+1'b1;
            end
            if(frame_done)begin
                last_cycles<=frame_cycles;last_frame<=network_frame;read_bank<=write_bank;write_bank<=!write_bank;profile_available<=1;
            end
            if(frame_failed)last_error<=error_code;
        end
    end
endmodule
