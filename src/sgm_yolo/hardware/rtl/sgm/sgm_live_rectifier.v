`timescale 1ns/1ps
// Fixed-rig VGA rectification. Only 48 grayscale rows are retained on-chip.
// Four parity banks fetch a bilinear neighborhood in parallel; camera inputs
// never pass through DDR. The map is a measured Q5, 32-pixel bilinear grid.
module sgm_live_rectifier(
    input wire clk,rst_n,
    input wire [7:0] left_in,right_in,
    input wire in_valid, output wire in_ready,
    output wire [7:0] left_out,right_out,
    output wire out_valid, input wire out_ready
);
    localparam W=640,H=480,RING=48,DEPTH=7680;
    reg [9:0] ix=0,iy=0,complete_rows=0,retired_y=0;
    reg [6:0] input_ring=0;
    reg [9:0] ox=0,oy=0,retired_x=0;
    reg issued_done=0;
    wire [9:0] oldest_row,needed_rows;
    sgm_rectification_row_bounds bounds(.retired_y(retired_y),.output_y(oy),
        .oldest_row(oldest_row),.complete_rows_needed(needed_rows));
    assign in_ready=rst_n && iy<H && iy<oldest_row+RING;
    wire write_fire=in_valid && in_ready;
    wire [13:0] wa=(input_ring>>1)*320+(ix>>1);
    reg [7:0] v=0;
    wire step=!v[7] || out_ready;
    wire launch=rst_n && !issued_done && complete_rows>=needed_rows;
    assign out_valid=rst_n && v[7];
    reg [7:0] result_l=0,result_r=0;
    assign left_out=result_l;
    assign right_out=result_r;

    wire [63:0] n00,n10,n01,n11;
    sgm_rectification_grid grid(.ix(ox[9:5]),.iy(oy[8:5]),
        .n00(n00),.n10(n10),.n01(n01),.n11(n11));
    reg signed [15:0] a[0:3],b[0:3],c[0:3],d[0:3];
    reg [4:0] fx0=0,fy0=0,fy1=0;
    reg signed [21:0] top1[0:3],bottom1[0:3];
    reg signed [15:0] coord2[0:3];
    genvar k;
    wire signed [27:0] coordinate_calc[0:3];
    generate for(k=0;k<4;k=k+1) begin: map_calc
        assign coordinate_calc[k]=$signed(top1[k])*32+
            ($signed(bottom1[k])-$signed(top1[k]))*$signed({1'b0,fy1});
    end endgenerate

    function [6:0] ring_index;
        input [9:0] y;
        begin
            if(y>=432) ring_index=y-432;
            else if(y>=384) ring_index=y-384;
            else if(y>=336) ring_index=y-336;
            else if(y>=288) ring_index=y-288;
            else if(y>=240) ring_index=y-240;
            else if(y>=192) ring_index=y-192;
            else if(y>=144) ring_index=y-144;
            else if(y>=96) ring_index=y-96;
            else if(y>=48) ring_index=y-48;
            else ring_index=y;
        end
    endfunction
    wire valid_l=coord2[0]>=0 && coord2[0]<639*32 && coord2[1]>=0 && coord2[1]<479*32;
    wire valid_r=coord2[2]>=0 && coord2[2]<639*32 && coord2[3]>=0 && coord2[3]<479*32;
    wire [9:0] sx[0:1],sy[0:1];
    assign sx[0]=valid_l ? coord2[0][14:5] : 0;
    assign sy[0]=valid_l ? coord2[1][14:5] : 0;
    assign sx[1]=valid_r ? coord2[2][14:5] : 0;
    assign sy[1]=valid_r ? coord2[3][14:5] : 0;
    wire [7:0] sample[0:7];
    genvar camera,bank;
    generate for(camera=0;camera<2;camera=camera+1) begin: camera_ram
        for(bank=0;bank<4;bank=bank+1) begin: parity
            wire [9:0] read_x=sx[camera]+(sx[camera][0]!=(bank%2));
            wire [9:0] read_y=sy[camera]+(sy[camera][0]!=(bank/2));
            wire [6:0] read_ring=ring_index(read_y);
            wire [13:0] ra=(read_ring>>1)*320+(read_x>>1);
            wire we=write_fire && ix[0]==(bank%2) && input_ring[0]==(bank/2);
            sgm_rectification_row_ram #(.DEPTH(DEPTH)) rows(
                .clk(clk),.write_enable(we),.write_address(wa),
                .write_data(camera==0?left_in:right_in),
                .read_enable(step && v[2]),.read_register_enable(step),.read_address(ra),
                .read_data(sample[camera*4+bank]));
        end
    end endgenerate
    reg [4:0] wx3[0:1],wy3[0:1],wx4[0:1],wy4[0:1],wy5[0:1];
    reg [1:0] parity3[0:1],parity4[0:1];
    reg [1:0] mask3=0,mask4=0,mask5=0,mask6=0;
    wire [7:0] p00[0:1],p10[0:1],p01[0:1],p11[0:1];
    generate for(camera=0;camera<2;camera=camera+1) begin: samples
        assign p00[camera]=sample[camera*4+parity4[camera]];
        assign p10[camera]=sample[camera*4+(parity4[camera]^2'b01)];
        assign p01[camera]=sample[camera*4+(parity4[camera]^2'b10)];
        assign p11[camera]=sample[camera*4+(parity4[camera]^2'b11)];
    end endgenerate
    reg [12:0] ht5[0:1],hb5[0:1];
    reg [17:0] value6[0:1];
    integer j;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            ix<=0;iy<=0;input_ring<=0;complete_rows<=0;
            ox<=0;oy<=0;issued_done<=0;retired_x<=0;retired_y<=0;
            v<=0;result_l<=0;result_r<=0;
        end else begin
            if(write_fire) begin
                if(ix==W-1) begin
                    ix<=0;iy<=iy+1'b1;complete_rows<=complete_rows+1'b1;
                    input_ring<=input_ring==RING-1?0:input_ring+1'b1;
                end else ix<=ix+1'b1;
            end
            if(out_valid && out_ready) begin
                if(retired_x==W-1) begin retired_x<=0;retired_y<=retired_y+1'b1;end
                else retired_x<=retired_x+1'b1;
            end
            if(step) begin
                v<={v[6:0],launch};
                if(launch) begin
                    for(j=0;j<4;j=j+1) begin
                        a[j]<=n00[j*16+:16];b[j]<=n10[j*16+:16];
                        c[j]<=n01[j*16+:16];d[j]<=n11[j*16+:16];
                    end
                    fx0<=ox[4:0];fy0<=oy[4:0];
                    if(ox==W-1) begin ox<=0;
                        if(oy==H-1) issued_done<=1;
                        else oy<=oy+1'b1;
                    end else ox<=ox+1'b1;
                end
                fy1<=fy0;
                for(j=0;j<4;j=j+1) begin
                    top1[j]<=$signed(a[j])*32+($signed(b[j])-$signed(a[j]))*$signed({1'b0,fx0});
                    bottom1[j]<=$signed(c[j])*32+($signed(d[j])-$signed(c[j]))*$signed({1'b0,fx0});
                    coord2[j]<=coordinate_calc[j]>>>10;
                end
                mask3<={valid_r,valid_l};mask4<=mask3;mask5<=mask4;mask6<=mask5;
                for(j=0;j<2;j=j+1) begin
                    wx3[j]<=coord2[j*2][4:0];wy3[j]<=coord2[j*2+1][4:0];
                    parity3[j]<={sy[j][0],sx[j][0]};
                    wx4[j]<=wx3[j];wy4[j]<=wy3[j];parity4[j]<=parity3[j];wy5[j]<=wy4[j];
                    ht5[j]<=p00[j]*(6'd32-{1'b0,wx4[j]})+p10[j]*wx4[j];
                    hb5[j]<=p01[j]*(6'd32-{1'b0,wx4[j]})+p11[j]*wx4[j];
                    value6[j]<=ht5[j]*(6'd32-{1'b0,wy5[j]})+hb5[j]*wy5[j];
                end
                result_l<=mask6[0]?(value6[0]+18'd512)>>10:0;
                result_r<=mask6[1]?(value6[1]+18'd512)>>10:0;
            end
        end
    end
endmodule
