`timescale 1ns/1ps
// Publish only complete disparity frames. This is on-chip BRAM, not DDR;
// camera pixels still feed the SGM pipeline directly. Nine bytes per 72-bit
// word use the native SDP BRAM width efficiently (67 BRAM36 for VGA).
module sgm_result_frame #(parameter W=640, H=480, PACKED_DISPARITY=0)(
    input wire write_clk, write_reset, capture_reset,
    input wire [7:0] pixel, input wire pixel_valid,
    output wire available, output reg overflow,
    input wire read_clk, read_reset,
    output wire [7:0] data, output wire valid, user, last,
    input wire ready
);
    localparam BITS=PACKED_DISPARITY?6:8, SLOTS=72/BITS;
    localparam N=W*H, WORDS=(N+SLOTS-1)/SLOTS, AW=$clog2(WORDS)>0?$clog2(WORDS):1;
    reg request=0, acknowledge=0;
    wire request_sync, acknowledge_sync;
    xpm_cdc_single #(.DEST_SYNC_FF(3), .SRC_INPUT_REG(0)) req_cdc
        (.src_clk(write_clk), .src_in(request), .dest_clk(read_clk), .dest_out(request_sync));
    xpm_cdc_single #(.DEST_SYNC_FF(3), .SRC_INPUT_REG(0)) ack_cdc
        (.src_clk(read_clk), .src_in(acknowledge), .dest_clk(write_clk), .dest_out(acknowledge_sync));
    assign available=(request==acknowledge_sync) && !write_reset;
    reg [AW-1:0] write_address=0;
    reg [3:0] write_slot=0;
    reg [$clog2(N):0] written=0;
    reg [71:0] pack=0;
    wire write_fire=pixel_valid && !capture_reset && available;
    wire complete=written==N-1;
    reg [71:0] write_word;
    integer i;
    always @* begin
        write_word=pack;
        for(i=0;i<SLOTS;i=i+1)
            if(write_slot==i) write_word[i*BITS+:BITS]=PACKED_DISPARITY ? pixel>>2 : pixel;
    end
    always @(posedge write_clk or posedge write_reset) begin
        if(write_reset) begin
            request<=0; overflow<=0;
        end else if(pixel_valid && !capture_reset && !available)
            overflow<=1;
        else if(write_fire && complete) request<=~request;
        if(write_reset || capture_reset) begin
            write_address<=0; write_slot<=0; written<=0; pack<=0;
        end else if(write_fire) begin
            written<=written+1'b1;
            if(write_slot==SLOTS-1 || complete) begin
                write_address<=write_address+1'b1; write_slot<=0; pack<=0;
            end else begin
                write_slot<=write_slot+1'b1; pack<=write_word;
            end
        end
    end
    localparam IDLE=0, LOAD=1, LOAD_WAIT=2, LOAD_WAIT2=3, RUN=4;
    reg [2:0] state=IDLE;
    reg [AW-1:0] read_address=0;
    reg [3:0] read_slot=0;
    reg [71:0] current_word=0, next_word=0;
    reg [1:0] prefetch_pending=0;
    reg [$clog2(W):0] x=0;
    reg [$clog2(H):0] y=0;
    wire fire=valid && ready;
    wire final_pixel=(x==W-1 && y==H-1);
    wire prefetch=(state==RUN && fire && read_slot==SLOTS-4 && read_address<WORDS-1);
    wire [AW-1:0] ram_read_address=prefetch ? read_address+1'b1 : read_address;
    wire [71:0] ram_data;
    sgm_result_ram #(.WORDS(WORDS), .AW(AW)
    ) frame_ram (
        .write_clk(write_clk), .write_enable(write_fire && (write_slot==SLOTS-1 || complete)),
        .write_address(write_address), .write_data(write_word),
        .read_clk(read_clk), .read_enable(state==LOAD || prefetch),
        .read_address(ram_read_address), .read_data(ram_data)
    );
    assign valid=(state==RUN) && !read_reset;
    wire [BITS-1:0] stored_pixel=current_word[read_slot*BITS+:BITS];
    assign data=PACKED_DISPARITY ? {stored_pixel[5:0],stored_pixel[5:4]} : stored_pixel;
    assign user=(x==0 && y==0);
    assign last=(x==W-1);
    always @(posedge read_clk or posedge read_reset) begin
        if(read_reset) begin
            state<=IDLE; acknowledge<=0; read_address<=0; read_slot<=0;
            x<=0; y<=0; current_word<=0; next_word<=0; prefetch_pending<=0;
        end else begin
            prefetch_pending<={prefetch_pending[0],prefetch};
            if(prefetch_pending[1]) next_word<=ram_data;
            case(state)
                IDLE: if(request_sync!=acknowledge) begin
                    read_address<=0; read_slot<=0; x<=0; y<=0; state<=LOAD;
                end
                LOAD: state<=LOAD_WAIT;
                LOAD_WAIT: state<=LOAD_WAIT2;
                LOAD_WAIT2: begin current_word<=ram_data; state<=RUN; end
                RUN: if(fire) begin
                    if(final_pixel) begin
                        acknowledge<=request_sync; state<=IDLE;
                    end else begin
                        if(read_slot==SLOTS-1) begin
                            current_word<=next_word; read_address<=read_address+1'b1; read_slot<=0;
                        end else read_slot<=read_slot+1'b1;
                        if(x==W-1) begin x<=0; y<=y+1'b1; end
                        else x<=x+1'b1;
                    end
                end
            endcase
        end
    end
endmodule
