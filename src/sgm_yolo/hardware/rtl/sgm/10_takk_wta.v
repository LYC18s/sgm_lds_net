`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// 模块名称：takk_wta
//
// 功能：
//   Winner-Takes-All (WTA)。
//   对当前像素的 64 个双路径 SGM 聚合代价进行最小值搜索，
//   输出“最小代价对应的 disparity 编号”。
//
// 数学表达：
//
//   d*(x,y) = argmin_d S(x,y,d)
//
// 其中：
//   d = 0~63
//   S(x,y,d) = 双路径 SGM 聚合代价
//
// -----------------------------------------------------------------------------
// 输入接口
// -----------------------------------------------------------------------------
//
// 当前双路径 SGM 顶层输出：
//   每个 disparity 为 11 bit
//   64 × 11 = 704 bit
//
// data_in_cost[10:0]      -> d=0
// data_in_cost[21:11]     -> d=1
// ...
// data_in_cost[703:693]   -> d=63
//
// -----------------------------------------------------------------------------
// 输出接口
// -----------------------------------------------------------------------------
//
// data_out_disp：
//   最优 disparity，范围 0~63，因此 6 bit 足够。
//
// data_out_min_cost：
//   对应的最小聚合代价，主要用于调试。
//
// -----------------------------------------------------------------------------
// 比较树
// -----------------------------------------------------------------------------
//
// 按要求，每一级比较树单独写一个 always：
//
//   第1级：64 -> 32
//   第2级：32 -> 16
//   第3级：16 -> 8
//   第4级：8  -> 4
//   第5级：4  -> 2
//   第6级：2  -> 1
//
// 每一个比较节点同时传递：
//
//   1. 当前最小 cost
//   2. 该 cost 对应的 disparity index
//
// 如果两个 cost 完全相等：
//
//   保留较小 disparity。
//
// 因此 tie-break 是稳定且确定的。
//
// -----------------------------------------------------------------------------
// 性能
// -----------------------------------------------------------------------------
//
// 固定流水延迟：6 拍
// 流水填满后吞吐：1 pixel / clk
//
// WTA 不存在 SGM 那种“相邻像素递归依赖”，
// 所以非常适合使用6级深流水。
// ////////////////////////////////////////////////////////////////////////////////

module takk_wta #(
    parameter MAX_DISP = 64,
    parameter COST_W   = 11,
    parameter DISP_W   = 6
)(
    input  wire                         clk,
    input  wire                         rst_n,

    input  wire [MAX_DISP*COST_W-1:0]  data_in_cost,
    input  wire                         data_in_valid,

    output reg  [DISP_W-1:0]           data_out_disp,
    output reg  [COST_W-1:0]           data_out_min_cost,
    output wire                         data_out_valid
);

    //==========================================================================
    // 第1级：64 -> 32
    //==========================================================================
    reg [COST_W-1:0] cost_s1 [0:31];
    reg [DISP_W-1:0] disp_s1 [0:31];
    reg              valid_s1;

    integer i1;

    always @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin

            valid_s1 <= 1'b0;

            for (i1=0; i1<32; i1=i1+1) begin
                cost_s1[i1] <= {COST_W{1'b0}};
                disp_s1[i1] <= {DISP_W{1'b0}};
            end

        end
        else begin

            valid_s1 <= data_in_valid;

            if (data_in_valid) begin

                for (i1=0; i1<32; i1=i1+1) begin

                    // 左边节点对应较小 disparity。
                    // cost相等时使用 <=，因此保留较小 disparity。
                    if (data_in_cost[(2*i1)*COST_W +: COST_W]
                        <=
                        data_in_cost[(2*i1+1)*COST_W +: COST_W]) begin

                        cost_s1[i1] <=
                            data_in_cost[(2*i1)*COST_W +: COST_W];

                        disp_s1[i1] <= 2*i1;

                    end
                    else begin

                        cost_s1[i1] <=
                            data_in_cost[(2*i1+1)*COST_W +: COST_W];

                        disp_s1[i1] <= 2*i1+1;

                    end

                end

            end

        end

    end


    //==========================================================================
    // 第2级：32 -> 16
    //==========================================================================
    reg [COST_W-1:0] cost_s2 [0:15];
    reg [DISP_W-1:0] disp_s2 [0:15];
    reg              valid_s2;

    integer i2;

    always @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin

            valid_s2 <= 1'b0;

            for (i2=0; i2<16; i2=i2+1) begin
                cost_s2[i2] <= {COST_W{1'b0}};
                disp_s2[i2] <= {DISP_W{1'b0}};
            end

        end
        else begin

            valid_s2 <= valid_s1;

            if (valid_s1) begin

                for (i2=0; i2<16; i2=i2+1) begin

                    if (cost_s1[2*i2] <= cost_s1[2*i2+1]) begin

                        cost_s2[i2] <= cost_s1[2*i2];
                        disp_s2[i2] <= disp_s1[2*i2];

                    end
                    else begin

                        cost_s2[i2] <= cost_s1[2*i2+1];
                        disp_s2[i2] <= disp_s1[2*i2+1];

                    end

                end

            end

        end

    end


    //==========================================================================
    // 第3级：16 -> 8
    //==========================================================================
    reg [COST_W-1:0] cost_s3 [0:7];
    reg [DISP_W-1:0] disp_s3 [0:7];
    reg              valid_s3;

    integer i3;

    always @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin

            valid_s3 <= 1'b0;

            for (i3=0; i3<8; i3=i3+1) begin
                cost_s3[i3] <= {COST_W{1'b0}};
                disp_s3[i3] <= {DISP_W{1'b0}};
            end

        end
        else begin

            valid_s3 <= valid_s2;

            if (valid_s2) begin

                for (i3=0; i3<8; i3=i3+1) begin

                    if (cost_s2[2*i3] <= cost_s2[2*i3+1]) begin

                        cost_s3[i3] <= cost_s2[2*i3];
                        disp_s3[i3] <= disp_s2[2*i3];

                    end
                    else begin

                        cost_s3[i3] <= cost_s2[2*i3+1];
                        disp_s3[i3] <= disp_s2[2*i3+1];

                    end

                end

            end

        end

    end


    //==========================================================================
    // 第4级：8 -> 4
    //==========================================================================
    reg [COST_W-1:0] cost_s4 [0:3];
    reg [DISP_W-1:0] disp_s4 [0:3];
    reg              valid_s4;

    integer i4;

    always @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin

            valid_s4 <= 1'b0;

            for (i4=0; i4<4; i4=i4+1) begin
                cost_s4[i4] <= {COST_W{1'b0}};
                disp_s4[i4] <= {DISP_W{1'b0}};
            end

        end
        else begin

            valid_s4 <= valid_s3;

            if (valid_s3) begin

                for (i4=0; i4<4; i4=i4+1) begin

                    if (cost_s3[2*i4] <= cost_s3[2*i4+1]) begin

                        cost_s4[i4] <= cost_s3[2*i4];
                        disp_s4[i4] <= disp_s3[2*i4];

                    end
                    else begin

                        cost_s4[i4] <= cost_s3[2*i4+1];
                        disp_s4[i4] <= disp_s3[2*i4+1];

                    end

                end

            end

        end

    end


    //==========================================================================
    // 第5级：4 -> 2
    //==========================================================================
    reg [COST_W-1:0] cost_s5 [0:1];
    reg [DISP_W-1:0] disp_s5 [0:1];
    reg              valid_s5;

    integer i5;

    always @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin

            valid_s5 <= 1'b0;

            for (i5=0; i5<2; i5=i5+1) begin
                cost_s5[i5] <= {COST_W{1'b0}};
                disp_s5[i5] <= {DISP_W{1'b0}};
            end

        end
        else begin

            valid_s5 <= valid_s4;

            if (valid_s4) begin

                for (i5=0; i5<2; i5=i5+1) begin

                    if (cost_s4[2*i5] <= cost_s4[2*i5+1]) begin

                        cost_s5[i5] <= cost_s4[2*i5];
                        disp_s5[i5] <= disp_s4[2*i5];

                    end
                    else begin

                        cost_s5[i5] <= cost_s4[2*i5+1];
                        disp_s5[i5] <= disp_s4[2*i5+1];

                    end

                end

            end

        end

    end


    //==========================================================================
    // 第6级：2 -> 1
    //
    // 最终直接写入输出寄存器。
    //==========================================================================
    reg valid_s6;

    always @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin

            valid_s6          <= 1'b0;
            data_out_disp     <= {DISP_W{1'b0}};
            data_out_min_cost <= {COST_W{1'b0}};

        end
        else begin

            valid_s6 <= valid_s5;

            if (valid_s5) begin

                if (cost_s5[0] <= cost_s5[1]) begin

                    data_out_min_cost <= cost_s5[0];
                    data_out_disp     <= disp_s5[0];

                end
                else begin

                    data_out_min_cost <= cost_s5[1];
                    data_out_disp     <= disp_s5[1];

                end

            end

        end

    end


    assign data_out_valid = valid_s6;

endmodule
