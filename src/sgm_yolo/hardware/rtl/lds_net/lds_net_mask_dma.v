`timescale 1ns/1ps
// Read only the 19200 NCHW road logits, restore/argmax in PL, write the57600B
// two-bit ROI mask. Issued AXI traffic is drained before an aborted slot frees.
module lds_net_mask_dma(
    input wire clk,rst_n,start,cancel,input wire [31:0] logits_address,mask_address,
    output wire ready,output reg done,error,
    output wire [31:0] araddr,output wire [7:0] arlen,output wire arvalid,input wire arready,
    input wire [63:0] rdata,input wire [1:0] rresp,input wire rlast,rvalid,output wire rready,
    output wire [31:0] awaddr,output wire [7:0] awlen,output wire awvalid,input wire awready,
    output wire [63:0] wdata,output wire [7:0] wstrb,output wire wlast,wvalid,input wire wready,
    input wire [1:0] bresp,input wire bvalid,output wire bready,
    output wire [31:0] bytes_read,bytes_written,output reg [31:0] cycles);
    localparam IDLE=0,LOAD_BEGIN=1,READ_COMMAND=2,READ_WAIT=3,WRITE_COMMAND=4,RESTORE_START=5,RESTORE_WAIT=6,DRAIN=7,FINISH=8;
    reg [3:0] state;reg cancelled,read_seen,restore_seen;
    reg [31:0] source,destination;
    reg [63:0] held;reg held_valid;reg [2:0] byte_index;
    reg [127:0] reservoir;reg [7:0] available;reg [17:0] labels;reg [12:0] submitted;
    reg beat_valid;reg [63:0] beat_data;reg [7:0] beat_keep;
    wire failing=cancel || cancelled || state==DRAIN;
    wire read_ready,read_done,read_error,read_busy,dv,dr,dl;wire [63:0] dd;wire [31:0] unused_r_stalls;
    wire write_ready,write_done,write_error,write_busy,stream_ready;wire [31:0] unused_w_stalls;
    lds_net_axi_read reader(.clk(clk),.rst_n(rst_n),.command_valid(state==READ_COMMAND && !failing),.command_ready(read_ready),
        .command_address(source),.command_bytes(24'd19200),.done(read_done),.error(read_error),.busy(read_busy),
        .data(dd),.valid(dv),.ready(dr),.last(dl),.araddr(araddr),.arlen(arlen),.arvalid(arvalid),.arready(arready),
        .rdata(rdata),.rresp(rresp),.rlast(rlast),.rvalid(rvalid),.rready(rready),.bytes_read(bytes_read),.stall_cycles(unused_r_stalls));
    wire core_load_ready,core_ready,core_valid,core_sof,core_eol,core_done,core_error;
    wire [1:0] core_label;
    wire core_output_ready=state==RESTORE_WAIT && !failing && available<=126;
    lds_net_road_restore restore(.clk(clk),.rst_n(rst_n && !failing),.load_begin(state==LOAD_BEGIN && !failing),
        .load_valid(state==READ_WAIT && held_valid && !failing),.load_ready(core_load_ready),.load_data(held[byte_index*8+:8]),
        .start(state==RESTORE_START && !failing),.ready(core_ready),.out_valid(core_valid),.out_ready(core_output_ready),
        .out_label(core_label),.out_sof(core_sof),.out_eol(core_eol),.done(core_done),.error(core_error));
    assign dr=failing || !held_valid;
    wire candidate=write_busy && submitted<7200 && (failing || available>=64);
    wire beat_ready=!beat_valid || stream_ready;
    wire enqueue=candidate && beat_ready,incoming=core_valid && core_output_ready;
    lds_net_axi_write writer(.clk(clk),.rst_n(rst_n),.command_valid(state==WRITE_COMMAND && !failing),.command_ready(write_ready),
        .command_address(destination),.command_bytes(24'd57600),.done(write_done),.error(write_error),.busy(write_busy),
        .data(beat_data),.data_keep(beat_keep),.valid(beat_valid),.ready(stream_ready),
        .awaddr(awaddr),.awlen(awlen),.awvalid(awvalid),.awready(awready),.wdata(wdata),.wstrb(wstrb),.wlast(wlast),.wvalid(wvalid),
        .wready(wready),.bresp(bresp),.bvalid(bvalid),.bready(bready),.bytes_written(bytes_written),.stall_cycles(unused_w_stalls));
    reg [127:0] next_data;reg [7:0] next_available;
    always @*begin
        next_data=enqueue && !failing?reservoir>>64:reservoir;
        next_available=available-(enqueue && !failing?64:0);
        if(incoming)begin next_data=next_data|({126'd0,core_label}<<next_available);next_available=next_available+2;end
    end
    assign ready=state==IDLE;
    always @(posedge clk)begin
        if(!rst_n)begin
            state<=IDLE;cancelled<=0;read_seen<=0;restore_seen<=0;source<=0;destination<=0;held<=0;held_valid<=0;byte_index<=0;
            reservoir<=0;available<=0;labels<=0;submitted<=0;beat_valid<=0;beat_data<=0;beat_keep<=0;
            done<=0;error<=0;cycles<=0;
        end else begin
            done<=0;if(!ready)cycles<=cycles+1;
            reservoir<=next_data;available<=next_available;
            if(beat_ready)begin beat_valid<=candidate;if(candidate)begin beat_data<=failing?64'd0:reservoir[63:0];beat_keep<=failing?8'd0:8'hff;end end
            if(enqueue)submitted<=submitted+1'b1;
            if(incoming)labels<=labels+1'b1;
            if(core_done)restore_seen<=1;
            if(cancel && state!=IDLE && state!=DRAIN && state!=FINISH)begin cancelled<=1;error<=1;state<=DRAIN;end
            else case(state)
                IDLE:if(start)begin
                    cancelled<=0;error<=0;read_seen<=0;restore_seen<=0;cycles<=0;source<=logits_address;destination<=mask_address;
                    held_valid<=0;byte_index<=0;reservoir<=0;available<=0;labels<=0;submitted<=0;beat_valid<=0;
                    if(cancel || logits_address[2:0]!=0 || mask_address[5:0]!=0 ||
                        {1'b0,logits_address}+33'd19200>33'h100000000 || {1'b0,mask_address}+33'd57600>33'h100000000)
                        begin error<=1;state<=FINISH;end
                    else state<=LOAD_BEGIN;
                end
                LOAD_BEGIN:if(core_ready)state<=READ_COMMAND;
                READ_COMMAND:if(read_ready)state<=READ_WAIT;
                READ_WAIT:begin
                    if(dv && dr)begin held<=dd;held_valid<=1;byte_index<=0;end
                    if(held_valid && core_load_ready)begin
                        byte_index<=byte_index+1'b1;
                        if(byte_index==7)held_valid<=0;
                    end
                    if(read_done)read_seen<=1;
                    if((read_seen || read_done) && !held_valid && !dv)begin
                        if(read_error)begin error<=1;cancelled<=1;state<=DRAIN;end else state<=WRITE_COMMAND;
                    end
                end
                WRITE_COMMAND:if(write_ready)state<=RESTORE_START;
                RESTORE_START:if(core_ready)state<=RESTORE_WAIT;
                RESTORE_WAIT:begin
                    if(core_error)begin error<=1;cancelled<=1;state<=DRAIN;end
                    else if(write_done)begin
                        if(write_error || labels!=230400 || available!=0 || submitted!=7200 || !restore_seen)error<=1;
                        state<=FINISH;
                    end
                end
                DRAIN:begin
                    held_valid<=0;
                    if(!read_busy && !write_busy)state<=FINISH;
                end
                FINISH:begin state<=IDLE;done<=1;end
                default:begin error<=1;cancelled<=1;state<=DRAIN;end
            endcase
        end
    end
endmodule
