`timescale 1ns/1ps
// Exact 11200x8, three 4096x9 RAMB36 banks. XPM rounded 14-bit addresses
// to four banks, wasting eight BRAM36 across the two four-parity camera RAMs.
module sgm_rectification_row_ram #(parameter DEPTH=11200)(
    input wire clk,
    input wire write_enable,input wire [13:0] write_address,input wire [7:0] write_data,
    input wire read_enable,read_register_enable,input wire [13:0] read_address,
    output wire [7:0] read_data
);
    localparam BANKS=(DEPTH+4095)/4096;
    wire [BANKS*8-1:0] bytes;
    reg [1:0] bank1=0,bank2=0;
    always @(posedge clk) begin
        if(read_enable) bank1<=read_address[13:12];
        if(read_register_enable) bank2<=bank1;
    end
    assign read_data=bytes[bank2*8+:8];
    genvar b;
    generate for(b=0;b<BANKS;b=b+1) begin: bank
        wire [31:0] output_word;
        wire we=write_enable && write_address[13:12]==b;
        RAMB36E1 #(.RAM_MODE("TDP"),.READ_WIDTH_A(9),.WRITE_WIDTH_A(0),
            .READ_WIDTH_B(0),.WRITE_WIDTH_B(9),.DOA_REG(1),.DOB_REG(0),
            .WRITE_MODE_A("READ_FIRST"),.WRITE_MODE_B("READ_FIRST"),
            .EN_ECC_READ("FALSE"),.EN_ECC_WRITE("FALSE"),
            .SIM_DEVICE("7SERIES"),.SIM_COLLISION_CHECK("ALL")) ram(
            .CLKARDCLK(clk),.CLKBWRCLK(clk),
            .ENARDEN(read_enable && read_address[13:12]==b),.ENBWREN(we),
            .ADDRARDADDR({1'b0,read_address[11:0],3'b0}),
            .ADDRBWRADDR({1'b0,write_address[11:0],3'b0}),
            .WEA(4'b0),.WEBWE({7'b0,we}),.DIADI(32'b0),.DIBDI({24'b0,write_data}),
            .DIPADIP(4'b0),.DIPBDIP(4'b0),.DOADO(output_word),.DOBDO(),.DOPADOP(),.DOPBDOP(),
            .REGCEAREGCE(read_register_enable),.REGCEB(1'b1),
            .RSTRAMARSTRAM(1'b0),.RSTRAMB(1'b0),.RSTREGARSTREG(1'b0),.RSTREGB(1'b0),
            .CASCADEINA(1'b0),.CASCADEINB(1'b0),.CASCADEOUTA(),.CASCADEOUTB(),
            .INJECTDBITERR(1'b0),.INJECTSBITERR(1'b0),.DBITERR(),.SBITERR(),.ECCPARITY(),.RDADDRECC());
        assign bytes[b*8+:8]=output_word[7:0];
    end endgenerate
endmodule
