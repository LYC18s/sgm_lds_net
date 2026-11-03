`timescale 1ns / 1ps

module alinx_ov5640 #(
    parameter integer BUFFER_DEPTH = 4096,
    parameter integer DEBUG_OV5640 = 0,
    parameter integer FRAME_WIDTH = 640
) (
    input  wire        cmos_vsync,
    input  wire        cmos_href,
    input  wire        cmos_pclk,
    input  wire [9:0]  cmos_d,
    input  wire        m_axis_video_aclk,
    input  wire        m_axis_video_aresetn,
    output wire [23:0] m_axis_video_tdata,
    output wire        m_axis_video_tvalid,
    input  wire        m_axis_video_tready,
    output wire        m_axis_video_tuser,
    output wire        m_axis_video_tlast,
    output wire [2:0]  m_axis_video_tkeep,
    output reg  [15:0] debug_line_bytes
);
    localparam integer FIFO_DATA_WIDTH = 26;
    assign m_axis_video_tkeep = 3'b111;

    (* ASYNC_REG = "TRUE" *) reg [1:0] axis_aresetn_cmos = 2'b00;
    reg [31:0] reset_cnt = 32'd0;
    reg [31:0] fifo_ready_cnt = 32'd0;
    reg cmos_aresetn = 1'b0;
    reg fifo_ready = 1'b0;

    always @(posedge cmos_pclk) begin
        axis_aresetn_cmos <= {axis_aresetn_cmos[0], m_axis_video_aresetn};
        if (!axis_aresetn_cmos[1]) begin
            reset_cnt    <= 32'd0;
            cmos_aresetn <= 1'b0;
        end else if (reset_cnt < 32'd200_000_000) begin
            reset_cnt    <= reset_cnt + 32'd1;
            cmos_aresetn <= 1'b0;
        end else begin
            cmos_aresetn <= 1'b1;
        end
    end

    always @(posedge cmos_pclk) begin
        if (!cmos_aresetn) begin
            fifo_ready_cnt <= 32'd0;
            fifo_ready     <= 1'b0;
        end else if (fifo_ready_cnt < 32'd100_000_000) begin
            fifo_ready_cnt <= fifo_ready_cnt + 32'd1;
            fifo_ready     <= 1'b0;
        end else begin
            fifo_ready     <= 1'b1;
        end
    end

    // Register the DVP pins once at their source clock.
    reg [7:0] cmos_data_q;
    reg cmos_href_q;
    reg cmos_vsync_q;
    reg cmos_vsync_qq;
    reg cmos_href_qq;
    reg [15:0] active_line_bytes;
    reg [7:0] debug_sample_first;
    reg [7:0] debug_sample_second;

    // Sample both bytes of RGB565 pixel x=440 before unpacking.  One active
    // 640-pixel line must contain exactly 1280 DVP byte clocks.
    localparam [15:0] DEBUG_SAMPLE_FIRST_BYTE = 16'd880;
    localparam [15:0] DEBUG_SAMPLE_SECOND_BYTE = 16'd881;

    always @(posedge cmos_pclk) begin
        if (!cmos_aresetn) begin
            cmos_data_q   <= 8'd0;
            cmos_href_q   <= 1'b0;
            cmos_vsync_q  <= 1'b0;
            cmos_vsync_qq <= 1'b0;
            cmos_href_qq  <= 1'b0;
            active_line_bytes <= 16'd0;
            debug_line_bytes  <= 16'd0;
            debug_sample_first <= 8'd0;
            debug_sample_second <= 8'd0;
        end else begin
            cmos_data_q   <= cmos_d[9:2];
            cmos_href_q   <= cmos_href;
            cmos_vsync_q  <= cmos_vsync;
            cmos_vsync_qq <= cmos_vsync_q;
            cmos_href_qq  <= cmos_href_q;

            // Count DVP bytes while HREF is active and sample raw byte 1/3 at
            // a known pixel before any packing, FIFO, VDMA, or DDR logic.
            if (cmos_href_q) begin
                if (active_line_bytes == DEBUG_SAMPLE_FIRST_BYTE)
                    debug_sample_first <= cmos_data_q;
                if (active_line_bytes == DEBUG_SAMPLE_SECOND_BYTE)
                    debug_sample_second <= cmos_data_q;
                active_line_bytes <= active_line_bytes + 16'd1;
            end else begin
                if (cmos_href_qq) begin
                    // [15] valid, [14] line contained exactly 1280 bytes,
                    // [13:7] first raw byte[7:1], [6:0] second byte[7:1].
                    // Seven bits per sample are enough to distinguish red
                    // (255,0) from a duplicated channel while fitting both
                    // cameras into the existing 32-bit GPIO debug register.
                    debug_line_bytes <= {
                        1'b1,
                        (active_line_bytes == 16'd1280),
                        debug_sample_first[7:1],
                        debug_sample_second[7:1]
                    };
                end
                active_line_bytes <= 16'd0;
                debug_sample_first <= 8'd0;
                debug_sample_second <= 8'd0;
            end
        end
    end

    wire [23:0] pixel_data;
    wire pixel_valid;
    wire pixel_last;
    cmos_8_24bit #(.FRAME_WIDTH(FRAME_WIDTH)) cmos_8_24bit_m0 (
        .rst(~cmos_aresetn),
        .pclk(cmos_pclk),
        .pdata_i(cmos_data_q),
        .de_i(cmos_href_q),
        .pdata_o(pixel_data),
        .de_o(pixel_valid),
        .line_last(pixel_last)
    );

    // A falling VSYNC marks the next active pixel as AXI start-of-frame.
    reg frame_start_pending;
    always @(posedge cmos_pclk) begin
        if (!cmos_aresetn)
            frame_start_pending <= 1'b0;
        else if (cmos_vsync_qq && !cmos_vsync_q)
            frame_start_pending <= 1'b1;
        else if (pixel_valid)
            frame_start_pending <= 1'b0;
    end

    wire s_axis_tuser = frame_start_pending && pixel_valid;
    wire s_axis_tlast = pixel_last && pixel_valid;
    wire fifo_empty;
    wire fifo_full;
    wire fifo_write = pixel_valid && fifo_ready && !fifo_full;

    (* ASYNC_REG = "TRUE" *) reg [1:0] fifo_ready_axis = 2'b00;
    always @(posedge m_axis_video_aclk) begin
        if (!m_axis_video_aresetn)
            fifo_ready_axis <= 2'b00;
        else
            fifo_ready_axis <= {fifo_ready_axis[0], fifo_ready};
    end

    // FWFT keeps data stable under back-pressure.  TVALID is independent of
    // TREADY, as required by AXI4-Stream; only rd_en uses the handshake.
    assign m_axis_video_tvalid = m_axis_video_aresetn && fifo_ready_axis[1] && !fifo_empty;

    xpm_fifo_async #(
        .FIFO_MEMORY_TYPE    ("auto"),
        .ECC_MODE            ("no_ecc"),
        .RELATED_CLOCKS      (0),
        .FIFO_WRITE_DEPTH    (BUFFER_DEPTH),
        .WRITE_DATA_WIDTH    (FIFO_DATA_WIDTH),
        .WR_DATA_COUNT_WIDTH (12),
        .PROG_FULL_THRESH    (10),
        .FULL_RESET_VALUE    (0),
        .USE_ADV_FEATURES    ("0707"),
        .READ_MODE           ("fwft"),
        .FIFO_READ_LATENCY   (0),
        .READ_DATA_WIDTH     (FIFO_DATA_WIDTH),
        .RD_DATA_COUNT_WIDTH (12),
        .PROG_EMPTY_THRESH   (10),
        .DOUT_RESET_VALUE    ("0"),
        .CDC_SYNC_STAGES     (2),
        .WAKEUP_TIME         (0)
    ) xpm_fifo_async_inst (
        .rst           (~cmos_aresetn),
        .wr_clk        (cmos_pclk),
        .wr_en         (fifo_write),
        .din           ({pixel_data, s_axis_tlast, s_axis_tuser}),
        .full          (fifo_full),
        .overflow      (),
        .prog_full     (),
        .wr_data_count (),
        .almost_full   (),
        .wr_ack        (),
        .wr_rst_busy   (),
        .rd_clk        (m_axis_video_aclk),
        .rd_en         (m_axis_video_tready && m_axis_video_tvalid),
        .dout          ({m_axis_video_tdata, m_axis_video_tlast, m_axis_video_tuser}),
        .empty         (fifo_empty),
        .underflow     (),
        .rd_rst_busy   (),
        .prog_empty    (),
        .rd_data_count (),
        .almost_empty  (),
        .data_valid    (),
        .sleep         (1'b0),
        .injectsbiterr (1'b0),
        .injectdbiterr (1'b0),
        .sbiterr       (),
        .dbiterr       ()
    );
endmodule
