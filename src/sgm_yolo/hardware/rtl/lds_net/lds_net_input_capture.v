`timescale 1ns/1ps
// Camera stream -> fixed central crop/resize -> RGB3 q=p-128 DDR input.
// A passive/drop-capable upstream tap must assert cancel when a frame cannot
// be delivered. Once AW is issued, cancellation fills remaining W beats with
// zero strobes and drains every B response before releasing the input slot.
module lds_net_input_capture(
    input wire clk,rst_n,command_valid,output wire command_ready,
    input wire [31:0] command_address,input wire cancel,
    input wire [23:0] s_data,input wire s_valid,s_sof,s_eol,output wire s_ready,
    output reg done,error,output reg [31:0] frame_id,output wire [31:0] source_counter,
    output wire [31:0] awaddr,output wire [7:0] awlen,output wire awvalid,input wire awready,
    output wire [63:0] wdata,output wire [7:0] wstrb,output wire wlast,wvalid,input wire wready,
    input wire [1:0] bresp,input wire bvalid,output wire bready,
    output wire [31:0] bytes_written,output reg [31:0] dropped_frames);
    localparam IDLE=0,ARMED=1,CAPTURE=2,DRAIN=3,FINISH=4;
    reg [2:0] state;reg [31:0] base,source_frame;
    assign source_counter=source_frame;
    reg [127:0] reservoir;reg [7:0] available;reg [16:0] pixels;
    reg [15:0] submitted;reg cancelled;
    reg beat_valid;reg [63:0] beat_data;reg [7:0] beat_keep;
    wire crop_s_ready,crop_valid,crop_sof,crop_eol,crop_done,crop_error,crop_aborted;
    wire [23:0] crop_data;
    wire dma_ready,dma_done,dma_error,dma_busy,stream_ready;wire [31:0] unused_stalls;
    wire failing=cancelled || cancel || state==DRAIN || (state==CAPTURE && (crop_error || crop_aborted));
    wire begin_frame=state==ARMED && s_valid && s_sof && crop_s_ready && dma_ready && !cancel;
    assign command_ready=state==IDLE;
    assign s_ready=state==CAPTURE?crop_s_ready:state==ARMED && s_sof?crop_s_ready && dma_ready:1'b1;
    wire crop_in=(state==CAPTURE || begin_frame) && s_valid && !failing;
    wire crop_ready=state==CAPTURE && !failing && available<=96;
    lds_net_crop_resize crop(.clk(clk),.rst_n(rst_n),.abort(failing),
        .s_data(s_data),.s_valid(crop_in),.s_ready(crop_s_ready),.s_sof(s_sof),.s_eol(s_eol),
        .m_data(crop_data),.m_valid(crop_valid),.m_ready(crop_ready),.m_sof(crop_sof),.m_eol(crop_eol),
        .frame_done(crop_done),.error(crop_error),.frame_aborted(crop_aborted));
    wire incoming=crop_valid && crop_ready;
    wire stream_valid=dma_busy && submitted<38400 && (failing || available>=64);
    wire beat_ready=!beat_valid || stream_ready;
    wire outgoing=stream_valid && beat_ready;
    lds_net_axi_write writer(.clk(clk),.rst_n(rst_n),.command_valid(begin_frame),.command_ready(dma_ready),
        .command_address(base),.command_bytes(24'd307200),.done(dma_done),.error(dma_error),.busy(dma_busy),
        .data(beat_data),.data_keep(beat_keep),.valid(beat_valid),.ready(stream_ready),
        .awaddr(awaddr),.awlen(awlen),.awvalid(awvalid),.awready(awready),.wdata(wdata),.wstrb(wstrb),.wlast(wlast),.wvalid(wvalid),
        .wready(wready),.bresp(bresp),.bvalid(bvalid),.bready(bready),.bytes_written(bytes_written),.stall_cycles(unused_stalls));
    reg [127:0] next_data;reg [7:0] next_available;
    always @*begin
        next_data=outgoing && !failing?reservoir>>64:reservoir;
        next_available=available-(outgoing && !failing?64:0);
        if(incoming)begin next_data=next_data|({104'd0,crop_data}<<next_available);next_available=next_available+24;end
    end
    always @(posedge clk)begin
        if(!rst_n)begin
            state<=IDLE;base<=0;source_frame<=0;reservoir<=0;available<=0;pixels<=0;submitted<=0;cancelled<=0;
            beat_valid<=0;beat_data<=0;beat_keep<=0;
            done<=0;error<=0;frame_id<=0;dropped_frames<=0;
        end else begin
            done<=0;
            if(beat_ready)begin
                beat_valid<=stream_valid;
                if(stream_valid)begin beat_data<=failing?64'd0:reservoir[63:0];beat_keep<=failing?8'd0:8'hff;end
            end
            if(s_valid && s_ready && s_sof)begin
                source_frame<=source_frame+1;
                if(!begin_frame)dropped_frames<=dropped_frames+1;
            end
            if(state==CAPTURE || state==DRAIN)begin
                reservoir<=next_data;available<=next_available;
                if(incoming)pixels<=pixels+1'b1;
                if(outgoing)submitted<=submitted+1'b1;
            end
            if((cancel || crop_aborted || crop_error) && state==CAPTURE)begin cancelled<=1;error<=1;state<=DRAIN;end
            else case(state)
                IDLE:if(command_valid)begin
                    base<=command_address;error<=0;cancelled<=0;reservoir<=0;available<=0;pixels<=0;submitted<=0;
                    if(cancel || command_address[5:0]!=0 || {1'b0,command_address}+33'd307200>33'h100000000)begin error<=1;state<=FINISH;end
                    else state<=ARMED;
                end
                ARMED:if(cancel)begin error<=1;state<=FINISH;end
                    else if(begin_frame)begin frame_id<=source_frame+1;state<=CAPTURE;end
                CAPTURE:if(dma_done)begin
                    if(dma_error || pixels!=102400 || available!=0 || submitted!=38400)error<=1;
                    state<=FINISH;
                end
                DRAIN:if(dma_done || !dma_busy)state<=FINISH;
                FINISH:begin state<=IDLE;done<=1;end
                default:begin error<=1;cancelled<=1;state<=DRAIN;end
            endcase
        end
    end
endmodule
