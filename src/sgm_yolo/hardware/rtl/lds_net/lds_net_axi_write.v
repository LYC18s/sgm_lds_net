`timescale 1ns/1ps
// Independent 64-bit writer; up to four AW/B transactions, ordered W stream.
// Final byte strobe preserves adjacent storage for non-eight-byte output sizes.
module lds_net_axi_write(
    input wire clk,rst_n,command_valid,output wire command_ready,
    input wire [31:0] command_address,input wire [23:0] command_bytes,
    output reg done,error,output reg busy,
    input wire [63:0] data,input wire [7:0] data_keep,input wire valid,output wire ready,
    output wire [31:0] awaddr,output wire [7:0] awlen,output wire awvalid,input wire awready,
    output wire [63:0] wdata,output wire [7:0] wstrb,output wire wlast,wvalid,input wire wready,
    input wire [1:0] bresp,input wire bvalid,output wire bready,
    output reg [31:0] bytes_written,stall_cycles);
    reg [31:0] address;reg [21:0] remaining,total,accepted,sent;
    reg [2:0] outstanding,write_bursts;reg [2:0] final_bytes;
    reg [4:0] lengths[0:3];reg [1:0] head,tail;reg [4:0] beat;
    (* ram_style="distributed" *) reg [71:0] fifo[0:63];
    reg [5:0] read_pointer,write_pointer;reg [6:0] used;
    wire [9:0] boundary=(13'd4096-{1'b0,address[11:0]})>>3;
    wire [4:0] capped=remaining>16?5'd16:remaining[4:0];
    wire [4:0] burst=boundary<capped?boundary[4:0]:capped;
    assign command_ready=rst_n && !busy;
    assign awaddr=address;assign awlen={3'd0,burst}-1'b1;
    assign awvalid=busy && remaining!=0 && outstanding<4;
    assign ready=busy && accepted<total && used<64;
    assign wdata=fifo[read_pointer][63:0];
    assign wstrb=fifo[read_pointer][71:64] & (sent+1==total && final_bytes!=0 ? (8'h01<<final_bytes)-1'b1 : 8'hff);
    assign wlast=beat+1==lengths[head];
    assign wvalid=busy && write_bursts!=0 && used!=0;
    assign bready=busy && outstanding!=0;
    wire af=awvalid && awready,df=valid && ready,wf=wvalid && wready,bf=bvalid && bready;
    wire burst_end=wf && wlast;
    wire [32:0] end_address={1'b0,command_address}+command_bytes;
    function [3:0] popcount;
        input [7:0] value;integer i;
        begin popcount=0;for(i=0;i<8;i=i+1) popcount=popcount+value[i];end
    endfunction
    always @(posedge clk) begin
        if(!rst_n) begin
            done<=0;error<=0;busy<=0;address<=0;remaining<=0;total<=0;accepted<=0;sent<=0;
            outstanding<=0;write_bursts<=0;final_bytes<=0;head<=0;tail<=0;beat<=0;
            read_pointer<=0;write_pointer<=0;used<=0;bytes_written<=0;stall_cycles<=0;
        end else begin
            done<=0;
            if(command_valid && command_ready) begin
                error<=0;bytes_written<=0;stall_cycles<=0;
                if(command_address[2:0]!=0 || command_bytes==0 || end_address>33'h100000000) begin error<=1;done<=1;end
                else begin
                    busy<=1;address<=command_address;remaining<=(command_bytes+7)>>3;total<=(command_bytes+7)>>3;
                    accepted<=0;sent<=0;outstanding<=0;write_bursts<=0;final_bytes<=command_bytes[2:0];
                    head<=0;tail<=0;beat<=0;read_pointer<=0;write_pointer<=0;used<=0;
                end
            end
            if(busy) begin
                case({af,bf})
                    2'b10:outstanding<=outstanding+1'b1;2'b01:outstanding<=outstanding-1'b1;default:outstanding<=outstanding;
                endcase
                case({af,burst_end})
                    2'b10:write_bursts<=write_bursts+1'b1;2'b01:write_bursts<=write_bursts-1'b1;default:write_bursts<=write_bursts;
                endcase
                case({df,wf})
                    2'b10:used<=used+1'b1;2'b01:used<=used-1'b1;default:used<=used;
                endcase
                if(af) begin
                    lengths[tail]<=burst;tail<=tail+1'b1;remaining<=remaining-burst;address<=address+(burst<<3);
                end
                if(df) begin fifo[write_pointer]<={data_keep,data};write_pointer<=write_pointer+1'b1;accepted<=accepted+1'b1;end
                if(wf) begin
                    read_pointer<=read_pointer+1'b1;sent<=sent+1'b1;
                    bytes_written<=bytes_written+popcount(wstrb);
                    if(burst_end) begin head<=head+1'b1;beat<=0;end else beat<=beat+1'b1;
                end
                if(bf && bresp!=0) error<=1;
                if(remaining==0 && outstanding==0 && sent==total) begin busy<=0;done<=1;end
                if((wvalid && !wready) || (awvalid && !awready) || (ready && !valid)) stall_cycles<=stall_cycles+1'b1;
            end
        end
    end
endmodule
