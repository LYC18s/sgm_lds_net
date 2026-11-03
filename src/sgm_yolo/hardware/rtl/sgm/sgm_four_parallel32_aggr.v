`timescale 1ns/1ps
// Four independent 32-lane engines, two disparity groups, input II=4.
// E0/E1 row reads; E2/E3 all four paths calculate in parallel.
// Registered group minima reduce at E3/E4; next pixel uses them at E6.
module sgm_four_parallel32_aggr #(
    parameter IMG_WIDTH=640, IMG_HEIGHT=480
)(
    input wire clk,rst_n,
    input wire [383:0] data_in_cost,
    input wire [7:0] data_in_gray,input wire data_in_valid,
    output wire [703:0] data_out,output reg data_out_valid
);
    localparam BW=288, DEPTH=2*IMG_WIDTH, AW=$clog2(DEPTH);
    localparam BANKS=(DEPTH+1023)/1024;
    reg [1:0] phase;
    reg [9:0] nx,ny,px,py;
    reg [7:0] pg,hgray;
    reg [383:0] raw;
    reg [575:0] hstate,dlold,dlprev;
    reg [287:0] vprevious,rprevious;
    reg [703:0] sums;
    reg [16:0] lmeta,lprevmeta;
    reg [8:0] hmin,vmin,lmin,rmin,hleft,vleft,rleft;
    reg [7:0] vgray,lgray,rgray;
    reg [8:0] first_min[0:3];
    reg [287:0] minimum_leaves[0:3];
    reg minimum_pending;
    wire accept=data_in_valid && phase==0;
    wire calculate=phase==2 || phase==3;
    wire group=phase==3;
    wire rd_en=accept || phase==1;
    wire [9:0] rx=accept?nx:px;
    wire rg=phase==1;
    wire [287:0] rows[1:3];
    wire [16:0] metas[1:3];
    wire [287:0] next_path[0:3];
    wire [8:0] tree[0:3][1:63];
    wire [8:0] final_min[0:3];
    wire [351:0] next_sum;
    genvar path,bank,d;
    generate for(path=0;path<4;path=path+1) begin: minima
        assign final_min[path]=first_min[path]<tree[path][1]?first_min[path]:tree[path][1];
        for(d=0;d<32;d=d+1) begin: leaves
            assign tree[path][32+d]=minimum_leaves[path][d*9+:9];
        end
        for(d=1;d<32;d=d+1) begin: reduce
            assign tree[path][d]=tree[path][2*d]<tree[path][2*d+1]?tree[path][2*d]:tree[path][2*d+1];
        end
    end
    for(path=1;path<=3;path=path+1) begin: row_path
        // DL retains old x for the next pixel; DR fetches old x+1.
        wire [9:0] col=path==3 && rx<IMG_WIDTH-1 ? rx+1'b1 : rx;
        wire [AW-1:0] ra={col,rg},wa={px,group};
        wire [287:0] bd[0:BANKS-1];
        reg [$clog2(BANKS+1)-1:0] rb;
        always @(posedge clk) if(rd_en) rb<=ra>>10;
        assign rows[path]=bd[rb];
        for(bank=0;bank<BANKS;bank=bank+1) begin: ram_bank
            localparam SIZE=DEPTH-bank*1024>=1024?1024:DEPTH-bank*1024;
            localparam BA=SIZE>1?$clog2(SIZE):1;
            sgm_line_ram #(.DATA_W(BW),.DEPTH(SIZE),.ADDR_W(BA)) ram(
                .clk(clk),.rd_en(rd_en && (ra>>10)==bank),.rd_addr(ra[BA-1:0]),.rd_data(bd[bank]),
                .wr_en(calculate && (wa>>10)==bank),.wr_addr(wa[BA-1:0]),.wr_data(next_path[path]));
        end
        sgm_line_ram #(.DATA_W(17),.DEPTH(IMG_WIDTH),.ADDR_W(10)) metadata(
            .clk(clk),.rd_en(accept),.rd_addr(col),.rd_data(metas[path]),
            .wr_en(minimum_pending),.wr_addr(px),.wr_data({pg,final_min[path]}));
    end endgenerate
    function [9:0] penalty;
        input [7:0] a,b;reg [7:0] delta;
        begin delta=a>=b?a-b:b-a;
            penalty=delta>=8?100:delta>=3?133:delta==2?200:400;
        end
    endfunction
    function [8:0] cost;
        input [8:0] l,s,r,minimum;
        input [9:0] p2;input [5:0] raw_cost;
        input initial_path,invalid;
        reg [8:0] near;
        reg [9:0] adjacent,candidate,norm;
        begin near=l<r?l:r;adjacent={1'b0,near}+100;
            candidate={1'b0,s}<adjacent?{1'b0,s}:adjacent;
            norm=candidate-minimum;if(norm>p2) norm=p2;
            cost=invalid?9'd511:initial_path?raw_cost:norm+raw_cost;
        end
    endfunction
    wire [8:0] path_min[0:3];
    wire [9:0] p2[0:3];
    wire [3:0] start;
    assign path_min[0]=hmin;assign path_min[1]=vmin;
    assign path_min[2]=lmin;assign path_min[3]=rmin;
    assign p2[0]=penalty(pg,hgray);assign p2[1]=penalty(pg,vgray);
    assign p2[2]=penalty(pg,lgray);assign p2[3]=penalty(pg,rgray);
    assign start[0]=px==0;assign start[1]=py==0;
    assign start[2]=py==0 || px==0;assign start[3]=py==0 || px==IMG_WIDTH-1;
    generate for(d=0;d<32;d=d+1) begin: lane
        localparam [4:0] ID=d;
        wire [5:0] index={group,ID};
        // Each lane only selects between two fixed groups. Arithmetic on a
        // dynamic index used to infer wide barrel multiplexers for neighbors.
        wire [8:0] hs=group?hstate[(32+d)*9+:9]:hstate[d*9+:9];
        wire [8:0] ls=group?dlprev[(32+d)*9+:9]:dlprev[d*9+:9];
        wire [8:0] hl,hr,ll,lr;
        if(d==0) begin: first
            assign hl=group?hleft:9'd511;
            assign ll=group?dlprev[31*9+:9]:9'd511;
        end else begin: previous
            assign hl=group?hstate[(31+d)*9+:9]:hstate[(d-1)*9+:9];
            assign ll=group?dlprev[(31+d)*9+:9]:dlprev[(d-1)*9+:9];
        end
        if(d==31) begin: last
            assign hr=group?9'd511:hstate[32*9+:9];
            assign lr=group?9'd511:dlprev[32*9+:9];
        end else begin: following
            assign hr=group?hstate[(33+d)*9+:9]:hstate[(d+1)*9+:9];
            assign lr=group?dlprev[(33+d)*9+:9]:dlprev[(d+1)*9+:9];
        end
        wire [8:0] vl=d==0?(group?vleft:9'd511):vprevious[(d-1)*9+:9];
        wire [8:0] vr=d==31?(group?9'd511:rows[1][0+:9]):vprevious[(d+1)*9+:9];
        wire [8:0] rl=d==0?(group?rleft:9'd511):rprevious[(d-1)*9+:9];
        wire [8:0] rr=d==31?(group?9'd511:rows[3][0+:9]):rprevious[(d+1)*9+:9];
        wire [5:0] rc=group?raw[(32+d)*6+:6]:raw[d*6+:6];
        wire invalid=index>px;
        wire [8:0] ca[0:3];
        assign ca[0]=cost(hl,hs,hr,path_min[0],p2[0],rc,start[0],invalid);
        assign ca[1]=cost(vl,vprevious[d*9+:9],vr,path_min[1],p2[1],rc,start[1],invalid);
        assign ca[2]=cost(ll,ls,lr,path_min[2],p2[2],rc,start[2],invalid);
        assign ca[3]=cost(rl,rprevious[d*9+:9],rr,path_min[3],p2[3],rc,start[3],invalid);
        assign next_sum[d*11+:11]={2'b0,ca[0]}+{2'b0,ca[1]}+{2'b0,ca[2]}+{2'b0,ca[3]};
        for(path=0;path<4;path=path+1) begin: engine
            assign next_path[path][d*9+:9]=ca[path];
            always @(posedge clk) if(calculate) minimum_leaves[path][d*9+:9]<=ca[path];
        end
    end endgenerate
    assign data_out=sums;
    integer p;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin phase<=0;nx<=0;ny<=0;data_out_valid<=0;minimum_pending<=0;end
        else begin
            data_out_valid<=phase==3;minimum_pending<=phase==3;
            if(accept) begin phase<=1;px<=nx;py<=ny;pg<=data_in_gray;raw<=data_in_cost;
                if(nx==IMG_WIDTH-1) begin nx<=0;ny<=ny==IMG_HEIGHT-1?0:ny+1'b1;end
                else nx<=nx+1'b1;
            end else if(phase!=0) phase<=phase+1'b1;
            if(phase==1) begin
                vprevious<=rows[1];dlold[0+:288]<=rows[2];rprevious<=rows[3];
                vmin<=metas[1][8:0];vgray<=metas[1][16:9];
                lmeta<=metas[2];rmin<=metas[3][8:0];rgray<=metas[3][16:9];
                lmin<=lprevmeta[8:0];lgray<=lprevmeta[16:9];
            end
            if(phase==2) begin
                hleft<=hstate[31*9+:9];vleft<=vprevious[31*9+:9];rleft<=rprevious[31*9+:9];
                hstate[0+:288]<=next_path[0];sums[0+:352]<=next_sum;
                vprevious<=rows[1];dlold[288+:288]<=rows[2];rprevious<=rows[3];
            end
            if(phase==3) begin
                hstate[288+:288]<=next_path[0];sums[352+:352]<=next_sum;hgray<=pg;
                dlprev<=dlold;lprevmeta<=lmeta;
                for(p=0;p<4;p=p+1) first_min[p]<=tree[p][1];
            end
            if(minimum_pending) hmin<=final_min[0];
        end
    end
`ifndef SYNTHESIS
    always @(posedge clk) if(rst_n && data_in_valid && phase!=0) $fatal(1,"Four-path II must be >=4");
`endif
endmodule
