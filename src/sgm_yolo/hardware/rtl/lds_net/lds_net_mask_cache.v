`timescale 1ns/1ps
// Eight cached480-label rows: four for the active mask and four for a pending
// mask. One SDP BRAM, independent NN/video clocks. The video client requests
// only banks whose previous row is no longer being read. First four pending
// rows must be acknowledged BEFORE the SOF atomic box/mask swap.
// Four-phase bundled-data CDC keeps addresses/tags stable until acknowledgement.
module lds_net_mask_cache(
    input wire nn_clk,nn_rst_n,video_clk,video_rst_n,
    input wire request_valid,output wire request_ready,input wire request_group,
    input wire [8:0] request_row,input wire [31:0] request_base,request_frame,
    output reg request_done,request_error,
    input wire read_step,read_pixel_valid,read_group,input wire [8:0] read_row,
    input wire [3:0] read_word,input wire [31:0] read_frame,
    output reg [63:0] read_data,output reg read_valid,
    output wire [31:0] araddr,output wire [7:0] arlen,output wire arvalid,input wire arready,
    input wire [63:0] rdata,input wire [1:0] rresp,input wire rlast,rvalid,output wire rready,
    output reg [31:0] read_errors);
    reg req_level,video_pending;
    reg group_hold;reg [8:0] row_hold;reg [31:0] base_hold,frame_hold;
    (* ASYNC_REG="TRUE" *) reg ack_sync0,ack_sync1,busy_sync0,busy_sync1;
    reg [3:0] startup;
    reg [7:0] valid_rows;reg [8:0] row_tags[0:7];reg [31:0] frame_tags[0:7];
    wire [2:0] read_bank={read_group,read_row[1:0]};
    wire [2:0] request_bank={request_group,request_row[1:0]};
    (* ram_style="block" *) reg [63:0] cache[0:127];
    reg nn_ack,nn_error,nn_group;reg [8:0] nn_row;reg [31:0] nn_frame;
    reg [31:0] base;reg [3:0] word_index;
    localparam IDLE=0,ISSUE=1,WAIT=2,ACK=3;
    reg [1:0] state;
    (* ASYNC_REG="TRUE" *) reg req_sync0,req_sync1;
    wire busy=state!=IDLE;
    wire dma_ready,dma_done,dma_error,dma_busy,dv,dl;wire [63:0] dd;wire [31:0] unused_bytes,unused_stalls;
    wire [31:0] row_offset=({23'd0,nn_row}<<7)-({23'd0,nn_row}<<3);
    lds_net_axi_read reader(.clk(nn_clk),.rst_n(nn_rst_n),.command_valid(state==ISSUE),.command_ready(dma_ready),
        .command_address(base+row_offset),.command_bytes(24'd120),.done(dma_done),.error(dma_error),.busy(dma_busy),
        .data(dd),.valid(dv),.ready(1'b1),.last(dl),.araddr(araddr),.arlen(arlen),.arvalid(arvalid),.arready(arready),
        .rdata(rdata),.rresp(rresp),.rlast(rlast),.rvalid(rvalid),.rready(rready),.bytes_read(unused_bytes),.stall_cycles(unused_stalls));
    always @(posedge nn_clk)if(state==WAIT && dv)cache[{nn_group,nn_row[1:0],word_index}]<=dd;
    always @(posedge video_clk)if(read_step)read_data<=cache[{read_group,read_row[1:0],read_word}];
    assign request_ready=video_rst_n && startup==15 && !video_pending && !ack_sync1 && !busy_sync1;
    integer i;
    always @(posedge video_clk)begin
        if(!video_rst_n)begin
            req_level<=0;video_pending<=0;group_hold<=0;row_hold<=0;base_hold<=0;frame_hold<=0;
            ack_sync0<=0;ack_sync1<=0;busy_sync0<=0;busy_sync1<=0;startup<=0;valid_rows<=0;
            request_done<=0;request_error<=0;read_valid<=0;read_errors<=0;
            for(i=0;i<8;i=i+1)begin row_tags[i]<=0;frame_tags[i]<=0;end
        end else begin
            request_done<=0;ack_sync0<=nn_ack;ack_sync1<=ack_sync0;busy_sync0<=busy;busy_sync1<=busy_sync0;
            if(startup!=15)startup<=startup+1'b1;
            if(read_step)read_valid<=read_pixel_valid && valid_rows[read_bank] && row_tags[read_bank]==read_row && frame_tags[read_bank]==read_frame;
            if(request_valid && request_ready)begin
                group_hold<=request_group;row_hold<=request_row;base_hold<=request_base;frame_hold<=request_frame;
                valid_rows[request_bank]<=0;req_level<=1;video_pending<=1;request_error<=0;
            end
            if(video_pending && ack_sync1)begin
                request_done<=1;video_pending<=0;req_level<=0;
                if(nn_error || nn_group!=group_hold || nn_row!=row_hold || nn_frame!=frame_hold)begin
                    request_error<=1;read_errors<=read_errors+1;valid_rows[{group_hold,row_hold[1:0]}]<=0;
                end else begin
                    valid_rows[{group_hold,row_hold[1:0]}]<=1;row_tags[{group_hold,row_hold[1:0]}]<=row_hold;frame_tags[{group_hold,row_hold[1:0]}]<=frame_hold;
                end
            end
        end
    end
    always @(posedge nn_clk)begin
        if(!nn_rst_n)begin state<=IDLE;req_sync0<=0;req_sync1<=0;nn_ack<=0;nn_error<=0;nn_group<=0;nn_row<=0;nn_frame<=0;base<=0;word_index<=0;end
        else begin
            req_sync0<=req_level;req_sync1<=req_sync0;
            case(state)
                IDLE:if(req_sync1)begin
                    nn_group<=group_hold;nn_row<=row_hold;nn_frame<=frame_hold;base<=base_hold;word_index<=0;nn_error<=0;
                    if(row_hold>=480 || base_hold[5:0]!=0 || {1'b0,base_hold}+33'd57600>33'h100000000)begin nn_error<=1;nn_ack<=1;state<=ACK;end
                    else state<=ISSUE;
                end
                ISSUE:if(dma_ready)state<=WAIT;
                WAIT:begin
                    if(dv)word_index<=word_index+1'b1;
                    if(dma_done)begin nn_error<=dma_error || word_index!=15;nn_ack<=1;state<=ACK;end
                end
                ACK:if(!req_sync1)begin nn_ack<=0;state<=IDLE;end
            endcase
        end
    end
endmodule
