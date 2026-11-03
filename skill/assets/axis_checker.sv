// Simulation checker: single clock, no interleaved TID/TDEST. Bind or instantiate.
// Set STRB=KEEP if TSTRB is absent. Zero-width optional signals must be tied off.
module axis_checker #(
    parameter DATA_WIDTH=32, USER_WIDTH=1, ID_WIDTH=1, DEST_WIDTH=1,
    parameter PACKET_BYTES=0, FATAL_ON_ERROR=1
) (
    input wire clk, resetn, valid, ready, last,
    input wire [DATA_WIDTH-1:0] data,
    input wire [DATA_WIDTH/8-1:0] keep, strb,
    input wire [USER_WIDTH-1:0] user,
    input wire [ID_WIDTH-1:0] id,
    input wire [DEST_WIDTH-1:0] dest,
    output integer errors=0,
    output integer accepted=0,
    output integer packets=0
);
    reg pending=0;
    reg [DATA_WIDTH-1:0] held_data;
    reg [DATA_WIDTH/8-1:0] held_keep, held_strb;
    reg held_last;
    reg [USER_WIDTH-1:0] held_user;
    reg [ID_WIDTH-1:0] held_id;
    reg [DEST_WIDTH-1:0] held_dest;
    integer lane, bytes_in_packet=0, count;
    task violation(input [255:0] rule);
        begin
            errors=errors+1;
            $display("AXIS_ERROR %0s time=%0t",rule,$time);
            if(FATAL_ON_ERROR) $fatal(1,"AXI-Stream checker failure");
        end
    endtask
    always @(posedge clk) begin
        if(resetn === 1'b0) begin
            if(valid !== 1'b0) violation("VALID_DURING_RESET");
            pending=0;
            bytes_in_packet=0;
        end else if(resetn !== 1'b1 || (valid !== 1'b0 && valid !== 1'b1) ||
                    (ready !== 1'b0 && ready !== 1'b1)) violation("UNKNOWN_CONTROL");
        else begin
            if(pending) begin
                if(valid !== 1'b1 || keep !== held_keep || strb !== held_strb ||
                   last !== held_last || user !== held_user || id !== held_id || dest !== held_dest)
                    violation("STALL_SIDEBAND_OR_VALID");
                for(lane=0;lane<DATA_WIDTH/8;lane=lane+1)
                    if(held_keep[lane] && held_strb[lane] && data[lane*8+:8] !== held_data[lane*8+:8])
                        violation("STALL_DATA");
            end
            if(valid) begin
                if((^keep === 1'bx) || (^strb === 1'bx) || (last !== 1'b0 && last !== 1'b1))
                    violation("UNKNOWN_PAYLOAD_CONTROL");
                if((strb & ~keep) != 0) violation("STRB_WITHOUT_KEEP");
            end
            pending=valid && !ready;
            held_data=data; held_keep=keep; held_strb=strb; held_last=last;
            held_user=user; held_id=id; held_dest=dest;
            if(valid && ready) begin
                accepted=accepted+1;
                count=0;
                for(lane=0;lane<DATA_WIDTH/8;lane=lane+1) if(keep[lane]) count=count+1;
                bytes_in_packet=bytes_in_packet+count;
                if(last) begin
                    if(PACKET_BYTES>0 && bytes_in_packet!=PACKET_BYTES) violation("PACKET_LENGTH");
                    packets=packets+1;
                    bytes_in_packet=0;
                end
            end
        end
    end
endmodule
