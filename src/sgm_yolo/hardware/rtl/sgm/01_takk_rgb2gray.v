`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// 模块名称：takk_rgb2gray
//
// 与 MATLAB rgb2gray_fixed.m 完全相同：
//
//   Gray = (77*R + 150*G + 29*B) >> 8
//
// 两级流水：
//   第1级：三个乘法
//   第2级：求和并取高8位
//////////////////////////////////////////////////////////////////////////////////

module takk_rgb2gray(
    input  wire       clk,
    input  wire       rst_n,

    input  wire [7:0] data_in_r,
    input  wire [7:0] data_in_g,
    input  wire [7:0] data_in_b,
    input  wire       data_in_valid,

    output wire [7:0] data_out,
    output reg        data_out_valid
);

    //==================== 第1级：乘法 ====================
    reg [15:0] r_tmp;
    reg [15:0] g_tmp;
    reg [15:0] b_tmp;

    reg valid_d1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin

            r_tmp    <= 16'd0;
            g_tmp    <= 16'd0;
            b_tmp    <= 16'd0;
            valid_d1 <= 1'b0;

        end
        else begin

            valid_d1 <= data_in_valid;

            if (data_in_valid) begin
                r_tmp <= data_in_r * 8'd77;
                g_tmp <= data_in_g * 8'd150;
                b_tmp <= data_in_b * 8'd29;
            end

        end
    end


    //==================== 第2级：求和 ====================
    reg [15:0] gray_tmp;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin

            gray_tmp      <= 16'd0;
            data_out_valid <= 1'b0;

        end
        else begin

            data_out_valid <= valid_d1;

            if (valid_d1)
                gray_tmp <= r_tmp + g_tmp + b_tmp;

        end
    end

    // 除以256 = 右移8bit
    assign data_out = gray_tmp[15:8];

endmodule
