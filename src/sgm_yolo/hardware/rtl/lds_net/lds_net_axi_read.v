`timescale 1ns/1ps
// 64-bit INCR DMA, four outstanding ordered bursts, 4KiB splitting.
// Same AXI ID is used: HP/interconnect must return bursts in address order.
module lds_net_axi_read(
    input wire clk,rst_n,command_valid,output wire command_ready,
    input wire [31:0] command_address,input wire [23:0] command_bytes,
    output reg done,error,output reg busy,
    output wire [63:0] data,output wire valid,input wire ready,output wire last,
    output wire [31:0] araddr,output wire [7:0] arlen,
    output wire arvalid,input wire arready,
    input wire [63:0] rdata,input wire [1:0] rresp,input wire rlast,rvalid,output wire rready,
    output reg [31:0] bytes_read,stall_cycles);
    reg [31:0] address;reg [21:0] remaining,total,received;
    reg priming;reg [31:0] command_address_q;reg [23:0] command_bytes_q;
    reg prepared,ar_pending;reg [4:0] burst_q;
    reg [2:0] outstanding;reg [6:0] reserved;
    reg [4:0] lengths[0:3];reg [1:0] head,tail;reg [4:0] beat;
    (* ram_style="distributed" *) reg [64:0] fifo[0:63];
    reg [5:0] read_pointer,write_pointer;reg [6:0] used;
    // Accepted addresses are eight-byte aligned. Keep the low bits constant
    // and calculate the 4 KiB boundary in words, without a byte subtract/shift.
    wire [9:0] boundary=10'd512-{1'b0,address[11:3]};
    wire [4:0] capped=remaining>16 ? 5'd16 : remaining[4:0];
    wire [4:0] burst=boundary<capped ? boundary[4:0] : capped;
    assign command_ready=rst_n && !busy;
    assign araddr=address;assign arlen={3'd0,burst_q}-1'b1;
    assign arvalid=busy && !priming && ar_pending;
    assign rready=busy && !priming && outstanding!=0 && used<64;
    assign data=fifo[read_pointer][63:0];assign last=fifo[read_pointer][64];
    assign valid=busy && !priming && used!=0;
    wire af=arvalid && arready,rf=rvalid && rready,of=valid && ready;
    wire burst_end=rf && beat+1==lengths[head];
    wire [32:0] end_address={1'b0,command_address_q}+command_bytes_q;
    always @(posedge clk) begin
        if(!rst_n) begin
            done<=0;error<=0;busy<=0;address<=0;remaining<=0;total<=0;received<=0;
            outstanding<=0;reserved<=0;head<=0;tail<=0;beat<=0;
            read_pointer<=0;write_pointer<=0;used<=0;bytes_read<=0;stall_cycles<=0;
            priming<=0;command_address_q<=0;command_bytes_q<=0;
            prepared<=0;ar_pending<=0;burst_q<=0;
        end else begin
            done<=0;
            if(command_valid && command_ready) begin
                error<=0;bytes_read<=0;stall_cycles<=0;
                command_address_q<=command_address;command_bytes_q<=command_bytes;
                busy<=1;priming<=1;
                prepared<=0;ar_pending<=0;
            end
            // Isolate the caller's row-address multiply from validation and
            // the high-fanout DMA state enables. Busy owns this priming cycle.
            if(priming) begin
                priming<=0;
                if(command_address_q[2:0]!=0 || command_bytes_q[2:0]!=0 || command_bytes_q==0 || end_address>33'h100000000) begin
                    error<=1;done<=1;busy<=0;
                end else begin
                    address<={command_address_q[31:3],3'b0};remaining<=command_bytes_q>>3;total<=command_bytes_q>>3;received<=0;
                    outstanding<=0;reserved<=0;head<=0;tail<=0;beat<=0;read_pointer<=0;write_pointer<=0;used<=0;
                end
            end
            if(busy && !priming) begin
                // Three short stages replace boundary -> burst -> capacity ->
                // AR handshake -> reservation arithmetic in one clock.
                // No other address is issued while prepared/pending; capacity
                // can only improve before AR acceptance. A pending AR stays
                // asserted with unchanged address/length under backpressure.
                if(!prepared && remaining!=0)begin burst_q<=burst;prepared<=1;end
                if(prepared && !ar_pending && outstanding<4 && {1'b0,reserved}+{3'b0,burst_q}<=8'd64)ar_pending<=1;
                case({af,burst_end})
                    2'b10:outstanding<=outstanding+1'b1;
                    2'b01:outstanding<=outstanding-1'b1;
                    default:outstanding<=outstanding;
                endcase
                reserved<=reserved+(af?{2'b0,burst_q}:7'd0)-(of?7'd1:7'd0);
                case({rf,of})
                    2'b10:used<=used+1'b1;2'b01:used<=used-1'b1;default:used<=used;
                endcase
                if(af) begin
                    prepared<=0;ar_pending<=0;
                    lengths[tail]<=burst_q;tail<=tail+1'b1;
                    remaining<=remaining-burst_q;address<={address[31:3]+burst_q,3'b0};
                end
                if(rf) begin
                    fifo[write_pointer]<={received+1==total,rdata};write_pointer<=write_pointer+1'b1;received<=received+1'b1;
                    if(rresp!=0 || rlast!=(beat+1==lengths[head])) error<=1;
                    if(burst_end) begin head<=head+1'b1;beat<=0;end else beat<=beat+1'b1;
                end
                if(of) begin
                    read_pointer<=read_pointer+1'b1;bytes_read<=bytes_read+8;
                    if(last) begin busy<=0;done<=1;end
                end
                if((outstanding!=0 && !rvalid) || (valid && !ready)) stall_cycles<=stall_cycles+1'b1;
            end
        end
    end
endmodule
