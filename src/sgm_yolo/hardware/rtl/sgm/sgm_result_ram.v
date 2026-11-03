`timescale 1ns/1ps
// Explicit 512x72 SDP banks: avoid XPM's independent-clock NO_CHANGE
// inference, which maps this width to twice as many true dual-port BRAMs.
// A two-stage bank mux bounds the 142.857 MHz read-domain routing delay.
module sgm_result_ram #(parameter WORDS=34134, AW=$clog2(WORDS))(
    input wire write_clk, write_enable,
    input wire [AW-1:0] write_address,
    input wire [71:0] write_data,
    input wire read_clk, read_enable,
    input wire [AW-1:0] read_address,
    output wire [71:0] read_data
);
    localparam BANKS=(WORDS+511)/512, GROUPS=(BANKS+7)/8;
    wire [15:0] wa=write_address, ra=read_address;
    wire [GROUPS*8*72-1:0] bank_data;
    genvar b,g;
    generate for(b=0;b<GROUPS*8;b=b+1) begin : bank
        if(b<BANKS) begin : present
            wire selected_write=write_enable && (wa[15:9]==b);
            RAMB36E1 #(
                .RAM_MODE("SDP"), .READ_WIDTH_A(72), .WRITE_WIDTH_B(72),
                .READ_WIDTH_B(0), .WRITE_WIDTH_A(0), .DOA_REG(0), .DOB_REG(0),
                .WRITE_MODE_A("WRITE_FIRST"), .WRITE_MODE_B("WRITE_FIRST"),
                .EN_ECC_READ("FALSE"), .EN_ECC_WRITE("FALSE"),
                .SIM_DEVICE("7SERIES"), .SIM_COLLISION_CHECK("ALL")
            ) ram (
                .CLKARDCLK(read_clk), .CLKBWRCLK(write_clk),
                .ENARDEN(read_enable && (ra[15:9]==b)), .ENBWREN(selected_write),
                .ADDRARDADDR({1'b0,ra[8:0],6'b0}), .ADDRBWRADDR({1'b0,wa[8:0],6'b0}),
                .WEA(4'b0), .WEBWE({8{selected_write}}),
                .DIADI(write_data[31:0]), .DIBDI(write_data[63:32]),
                .DIPADIP(write_data[67:64]), .DIPBDIP(write_data[71:68]),
                .DOADO(bank_data[b*72+:32]), .DOBDO(bank_data[b*72+32+:32]),
                .DOPADOP(bank_data[b*72+64+:4]), .DOPBDOP(bank_data[b*72+68+:4]),
                .REGCEAREGCE(1'b1), .REGCEB(1'b1),
                .RSTRAMARSTRAM(1'b0), .RSTRAMB(1'b0), .RSTREGARSTREG(1'b0), .RSTREGB(1'b0),
                .CASCADEINA(1'b0), .CASCADEINB(1'b0), .CASCADEOUTA(), .CASCADEOUTB(),
                .INJECTDBITERR(1'b0), .INJECTSBITERR(1'b0), .DBITERR(), .SBITERR(), .ECCPARITY(), .RDADDRECC()
            );
        end else begin : padding
            assign bank_data[b*72+:72]=0;
        end
    end endgenerate
    reg [6:0] selected_bank=0;
    reg [3:0] selected_group=0;
    wire [GROUPS*72-1:0] group_data;
    always @(posedge read_clk) begin
        if(read_enable) selected_bank<=ra[15:9];
        selected_group<=selected_bank[6:3];
    end
    generate for(g=0;g<GROUPS;g=g+1) begin : read_group
        reg [71:0] word_q=0;
        always @(posedge read_clk)
            word_q<=bank_data[g*8*72+selected_bank[2:0]*72+:72];
        assign group_data[g*72+:72]=word_q;
    end endgenerate
    assign read_data=group_data[selected_group*72+:72];
endmodule
