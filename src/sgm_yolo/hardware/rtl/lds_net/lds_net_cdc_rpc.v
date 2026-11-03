`timescale 1ns/1ps
// One outstanding bundled-data request/response, four-phase handshake.
// The source holds the entire payload through destination acknowledgement;
// response data is held until the request level falls. No pulse crosses clocks.
module lds_net_cdc_rpc #(parameter WIDTH=65,RESPONSE_WIDTH=1)(
    input wire source_clk,source_rst_n,destination_clk,destination_rst_n,
    input wire source_valid,output wire source_ready,input wire [WIDTH-1:0] source_data,
    output reg source_done,output reg [RESPONSE_WIDTH-1:0] source_response,
    output wire destination_valid,input wire destination_ready,output wire [WIDTH-1:0] destination_data,
    input wire [RESPONSE_WIDTH-1:0] destination_response);
    reg request_level,source_busy;
    reg [WIDTH-1:0] payload;
    reg acknowledge;reg [RESPONSE_WIDTH-1:0] response;
    (* ASYNC_REG="TRUE" *) reg ack0,ack1,request0,request1;
    reg [3:0] startup;
    assign source_ready=source_rst_n && startup==15 && !source_busy && !ack1;
    assign destination_valid=destination_rst_n && request1 && !acknowledge;
    assign destination_data=payload;
    always @(posedge source_clk)begin
        if(!source_rst_n)begin
            request_level<=0;source_busy<=0;payload<=0;ack0<=0;ack1<=0;startup<=0;source_done<=0;source_response<=0;
        end else begin
            ack0<=acknowledge;ack1<=ack0;source_done<=0;
            if(startup!=15)startup<=startup+1'b1;
            if(source_valid && source_ready)begin payload<=source_data;request_level<=1;source_busy<=1;end
            if(source_busy && ack1)begin source_response<=response;source_done<=1;source_busy<=0;request_level<=0;end
        end
    end
    always @(posedge destination_clk)begin
        if(!destination_rst_n)begin request0<=0;request1<=0;acknowledge<=0;response<=0;end
        else begin
            request0<=request_level;request1<=request0;
            if(destination_valid && destination_ready)begin response<=destination_response;acknowledge<=1;end
            if(acknowledge && !request1)acknowledge<=0;
        end
    end
endmodule
