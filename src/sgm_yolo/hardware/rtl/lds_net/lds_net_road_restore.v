`timescale 1ns/1ps
// Exact 80->320->480 bilinear logits, THEN argmax. Three signed channels.
// Composite four-tap coefficients per axis retain BOTH interpolation stages.
// Four memory taps/cycle, one label per four cycles before backpressure.
module lds_net_road_restore(
    input wire clk,rst_n,load_begin,
    input wire load_valid,output wire load_ready,input wire [7:0] load_data,
    input wire start,output wire ready,
    output reg out_valid,input wire out_ready,output reg [1:0] out_label,
    output reg out_sof,out_eol,output reg done,output reg error);
    reg loading,running,all_issued;reg [14:0] loaded;
    reg [12:0] load_address;reg [1:0] load_channel;
    reg [8:0] x,y;reg [1:0] phase;
    reg [4:0] valid_pipe;
    reg [23:0] x_coeff_q;
    reg [5:0] y_coeff_q,y_coeff2,y_coeff3;reg [1:0] phase0,phase1,phase2,phase3,phase4;
    reg sof0,sof1,sof2,sof3,sof4,eol0,eol1,eol2,eol3,eol4,eof0,eof1,eof2,eof3,eof4,eof_out;
    wire step=!out_valid || out_ready;
    wire issue=running && !all_issued;
    assign ready=!running;
    assign load_ready=rst_n && loading && !running && loaded<19200;
    wire write=load_valid && load_ready;
    // All divisions are fixed constants; no floating arithmetic or rounding.
    function [51:0] coefficients;
        input [8:0] index;
        integer coordinate,lo,hi,fraction,source0,source1,a,b,c,d,f0,f1;
        reg [5:0] w0,w1,w2,w3;
        begin
            coordinate=index==0 ? 0 : index*4-1;
            lo=coordinate/6;if(lo>319) lo=319;
            hi=lo<319?lo+1:319;fraction=coordinate-lo*6;
            source0=lo*2<3?0:lo*2-3;source1=hi*2<3?0:hi*2-3;
            a=source0/8;if(a>79) a=79;b=a<79?a+1:79;f0=source0-a*8;
            c=source1/8;if(c>79) c=79;d=c<79?c+1:79;f1=source1-c*8;
            w0=(6-fraction)*(8-f0);w1=(6-fraction)*f0;
            w2=fraction*(8-f1);w3=fraction*f1;
            coefficients={w3,w2,w1,w0,d[6:0],c[6:0],b[6:0],a[6:0]};
        end
    endfunction
    // Fixed two-stage interpolation coefficients are immutable ROM data.
    // A registered dual-port lookup replaces division on the address path.
    (* rom_style="block" *) reg [51:0] coefficient_rom[0:479];
    integer coefficient_index;
    initial for(coefficient_index=0;coefficient_index<480;coefficient_index=coefficient_index+1)
        coefficient_rom[coefficient_index]=coefficients(coefficient_index);
    reg [51:0] cx,cy;
    always @(posedge clk)if(step)begin cx<=coefficient_rom[x];cy<=coefficient_rom[y];end
    wire [6:0] row=cy[phase0*7+:7];
    wire [12:0] row_address=({6'd0,row}<<6)+({6'd0,row}<<4);
    wire [12:0] address0=row_address+cx[6:0],address1=row_address+cx[13:7];
    wire [12:0] address2=row_address+cx[20:14],address3=row_address+cx[27:21];
    wire [59:0] weighted_rows;reg signed [19:0] sums[0:2];
    reg signed [20:0] total0,total1,total2;
    genvar c;
    generate for(c=0;c<3;c=c+1) begin: classes
        
        
        wire signed [7:0] q0,q1,q2,q3;
        wire writing=write && load_channel==c;
        (* use_dsp="yes" *) reg signed [19:0] weighted;
        (* use_dsp="yes" *) reg signed [13:0] p0,p1,p2,p3;
        reg signed [15:0] horizontal;
        lds_net_dual8_ram ram0(clk,step,writing,load_address,address0,address1,load_data,q0,q1);
        lds_net_dual8_ram ram1(clk,step,writing,load_address,address2,address3,load_data,q2,q3);
        always @(posedge clk) begin
            if(step)begin
                p0<=q0*$signed({1'b0,x_coeff_q[5:0]});p1<=q1*$signed({1'b0,x_coeff_q[11:6]});
                p2<=q2*$signed({1'b0,x_coeff_q[17:12]});p3<=q3*$signed({1'b0,x_coeff_q[23:18]});
                horizontal<=($signed({{2{p0[13]}},p0})+$signed({{2{p1[13]}},p1}))+ 
                            ($signed({{2{p2[13]}},p2})+$signed({{2{p3[13]}},p3}));
                weighted<=horizontal*$signed({1'b0,y_coeff3});
            end
        end
        assign weighted_rows[c*20+:20]=weighted;
    end endgenerate
    always @* begin
        total0=$signed({sums[0][19],sums[0]})+$signed({weighted_rows[19],weighted_rows[19:0]});
        total1=$signed({sums[1][19],sums[1]})+$signed({weighted_rows[39],weighted_rows[39:20]});
        total2=$signed({sums[2][19],sums[2]})+$signed({weighted_rows[59],weighted_rows[59:40]});
    end
    integer i;
    always @(posedge clk) begin
        if(!rst_n) begin
            loading<=0;running<=0;all_issued<=0;loaded<=0;load_address<=0;load_channel<=0;
            x<=0;y<=0;phase<=0;valid_pipe<=0;x_coeff_q<=0;y_coeff_q<=0;y_coeff2<=0;y_coeff3<=0;
            phase0<=0;phase1<=0;phase2<=0;phase3<=0;phase4<=0;
            sof0<=0;sof1<=0;sof2<=0;sof3<=0;sof4<=0;eol0<=0;eol1<=0;eol2<=0;eol3<=0;eol4<=0;
            eof0<=0;eof1<=0;eof2<=0;eof3<=0;eof4<=0;eof_out<=0;
            out_valid<=0;out_label<=0;out_sof<=0;out_eol<=0;done<=0;error<=0;
            for(i=0;i<3;i=i+1) sums[i]<=0;
        end else begin
            done<=0;
            if(load_begin && !running) begin
                loading<=1;loaded<=0;load_address<=0;load_channel<=0;error<=0;
            end
            if(write) begin
                loaded<=loaded+1'b1;
                if(load_address==6399) begin load_address<=0;load_channel<=load_channel+1'b1;end
                else load_address<=load_address+1'b1;
                if(loaded==19199) loading<=0;
            end
            if(start) begin
                if(!running && loaded==19200 && !loading) begin
                    running<=1;all_issued<=0;x<=0;y<=0;phase<=0;valid_pipe<=0;out_valid<=0;
                    for(i=0;i<3;i=i+1) sums[i]<=0;
                end else error<=1;
            end
            if(step) begin
                out_valid<=0;valid_pipe<={valid_pipe[3:0],issue};
                x_coeff_q<=cx[51:28];y_coeff_q<=cy[28+phase0*6+:6];y_coeff2<=y_coeff_q;y_coeff3<=y_coeff2;
                phase0<=phase;phase1<=phase0;phase2<=phase1;phase3<=phase2;phase4<=phase3;
                sof0<=x==0 && y==0;eol0<=x==479;eof0<=x==479 && y==479;
                sof1<=sof0;eol1<=eol0;eof1<=eof0;
                sof2<=sof1;eol2<=eol1;eof2<=eof1;sof3<=sof2;eol3<=eol2;eof3<=eof2;sof4<=sof3;eol4<=eol3;eof4<=eof3;
                if(issue) begin
                    phase<=phase+1'b1;
                    if(phase==3) begin
                        if(x==479) begin
                            x<=0;y<=y+1'b1;
                            if(y==479) all_issued<=1;
                        end else x<=x+1'b1;
                    end
                end
                if(valid_pipe[4]) begin
                    for(i=0;i<3;i=i+1)
                        sums[i]<=phase4==0 ? $signed(weighted_rows[i*20+:20]) : sums[i]+$signed(weighted_rows[i*20+:20]);
                    if(phase4==3) begin
                        out_valid<=1;out_sof<=sof4;out_eol<=eol4;eof_out<=eof4;
                        // Strict greater preserves the smaller category on ties.
                        if(total1>total0) out_label<=total2>total1 ? 2 : 1;
                        else out_label<=total2>total0 ? 2 : 0;
                    end
                end
            end
            if(out_valid && out_ready && eof_out) begin done<=1;running<=0;out_valid<=0;end
        end
    end
endmodule

module lds_net_dual8_ram(input wire clk,step,writing,input wire [12:0] wa,ra,rb,input wire [7:0] wd,output wire [7:0] qa,qb);
 wire [31:0] da[0:1],db[0:1];reg ba,bb;
 always @(posedge clk)if(step)begin ba<=ra[12];bb<=rb[12];end
 assign qa=da[ba][7:0];assign qb=db[bb][7:0];genvar b;
 generate for(b=0;b<2;b=b+1)begin: banks
 wire we=writing && wa[12]==b;wire [11:0] a=writing?wa[11:0]:ra[11:0];
 RAMB36E1 #(.RAM_MODE("TDP"),.READ_WIDTH_A(9),.WRITE_WIDTH_A(9),.READ_WIDTH_B(9),.WRITE_WIDTH_B(0),.DOA_REG(0),.DOB_REG(0),.WRITE_MODE_A("READ_FIRST"),.WRITE_MODE_B("READ_FIRST")) ram(
 .CLKARDCLK(clk),.CLKBWRCLK(clk),.ENARDEN(we || step && ra[12]==b),.ENBWREN(step && rb[12]==b),
 .ADDRARDADDR({1'b0,a,3'b0}),.ADDRBWRADDR({1'b0,rb[11:0],3'b0}),.WEA({4{we}}),.WEBWE(8'b0),
 .DIADI({24'b0,wd}),.DIBDI(32'b0),.DIPADIP(4'b0),.DIPBDIP(4'b0),.DOADO(da[b]),.DOBDO(db[b]),.DOPADOP(),.DOPBDOP(),
 .REGCEAREGCE(1'b1),.REGCEB(1'b1),.RSTRAMARSTRAM(1'b0),.RSTRAMB(1'b0),.RSTREGARSTREG(1'b0),.RSTREGB(1'b0),
 .CASCADEINA(1'b0),.CASCADEINB(1'b0),.CASCADEOUTA(),.CASCADEOUTB(),.INJECTDBITERR(1'b0),.INJECTSBITERR(1'b0),.DBITERR(),.SBITERR(),.ECCPARITY(),.RDADDRECC());
 end endgenerate
endmodule
