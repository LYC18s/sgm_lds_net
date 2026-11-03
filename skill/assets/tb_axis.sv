`timescale 1ns/1ps
// Ready-before-valid, bubbles, continuous packets, deterministic short/long stalls.
// Compile alongside axis_checker.sv. +FAULT deliberately changes stalled data.
module tb_axis;
    reg clk=0,resetn=0,valid=0,ready=0,last=0,user=0;
    reg [31:0] data=0;
    reg [3:0] keep=0;
    wire [31:0] errors,accepted,packets;
    integer ticks=0,p,b,hold_cycles,fault;
    always #5 clk=~clk;
    always @(negedge clk) begin
        ticks=ticks+1;
        // Runs of five low cycles, single-cycle stalls and receptive idle cycles.
        ready=(ticks%13>=5) && (ticks%7!=0);
    end
    axis_checker #(.PACKET_BYTES(10),.FATAL_ON_ERROR(0)) checker_inst(
        .clk(clk),.resetn(resetn),.valid(valid),.ready(ready),.last(last),
        .data(data),.keep(keep),.strb(keep),.user(user),.id(1'b0),.dest(1'b0),
        .errors(errors),.accepted(accepted),.packets(packets));
    initial begin
        $dumpfile("axis.vcd"); $dumpvars(0,tb_axis);
        fault=$test$plusargs("FAULT");
        repeat(3) @(negedge clk);
        #1 resetn=1;
        for(p=0;p<20;p=p+1) begin
            for(b=0;b<3;b=b+1) begin
                @(negedge clk); #1;
                valid=1;data=(p*3+b+1)*32'h01010101;
                keep=b==2?4'h3:4'hf;last=b==2;hold_cycles=0;
                @(posedge clk);
                while(!ready) begin
                    hold_cycles=hold_cycles+1;
                    if(fault && p==0 && b==0 && hold_cycles==1) begin
                        @(negedge clk); #1 data=data^1;
                    end
                    @(posedge clk);
                end
                @(negedge clk); #1 valid=0;
                if(b==0) repeat(2) @(negedge clk);
            end
        end
        repeat(2) @(negedge clk);
        if(accepted!=60 || packets!=20) $fatal(1,"Unexpected transfer counts");
        if((!fault && errors!=0) || (fault && errors==0)) $fatal(1,"Checker detection failed");
        $display("PASS: axis checker accepted=%0d packets=%0d errors=%0d fault=%0d",accepted,packets,errors,fault);
        $finish;
    end
    initial begin #100000; $fatal(1,"Bounded bench timeout"); end
endmodule
