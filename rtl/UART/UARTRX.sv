`timescale 1ns / 1ps
`include "../sys_define.svh"

// ============================================================
//  UARTRX — 串口接收（8N1，LSB first）
// ------------------------------------------------------------
//  rxd 先过两级同步器；检测起始位下降沿后，等半个 bit 周期在起始位
//  中点复采样确认（滤掉毛刺），随后每 1 个 bit 周期在中点采样一次，
//  最后校验停止位。收完一个字节 valid 拉高一拍。
// ============================================================
module UARTRX #(
        parameter int CLK_FREQ = 125_000_000,
        parameter int BAUD     = 115_200
    )(
        input  logic       clk,
        input  logic       rst_n,

        input  logic       rxd,
        output logic [7: 0] data,
        output logic        valid
    );

    localparam int DIV      = (CLK_FREQ + BAUD/2) / BAUD;
    localparam int HALF_DIV = DIV / 2;

    localparam [1:0] S_IDLE  = 2'd0,
                     S_START = 2'd1,
                     S_DATA  = 2'd2,
                     S_STOP  = 2'd3;

    // ---- 输入同步 ----
    logic rx_s0, rx_s1;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_s0 <= 1'b1;
            rx_s1 <= 1'b1;
        end
        else begin
            rx_s0 <= rxd;
            rx_s1 <= rx_s0;
        end
    end

    logic [1: 0]  state;
    logic [15: 0] cnt;
    logic [2: 0]  bit_idx;
    logic [7: 0]  data_r;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= S_IDLE;
            cnt     <= 16'd0;
            bit_idx <= 3'd0;
            data_r  <= 8'd0;
            data    <= 8'd0;
            valid   <= 1'b0;
        end
        else begin
            valid <= 1'b0;
            case (state)
                S_IDLE: begin
                    cnt <= 16'd0;
                    if (!rx_s1) state <= S_START;      // 起始位下降沿
                end
                S_START: begin
                    if (cnt == HALF_DIV[15: 0] - 16'd1) begin
                        cnt     <= 16'd0;
                        bit_idx <= 3'd0;   // 必须清零，否则从第 2 个字节起只采到 bit7
                        // 起始位中点仍为 0 才认为是真起始位
                        state <= rx_s1 ? S_IDLE : S_DATA;
                    end
                    else cnt <= cnt + 16'd1;
                end
                S_DATA: begin
                    if (cnt == DIV[15: 0] - 16'd1) begin
                        cnt        <= 16'd0;
                        data_r[bit_idx] <= rx_s1;
                        if (bit_idx == 3'd7) state <= S_STOP;
                        else                 bit_idx <= bit_idx + 3'd1;
                    end
                    else cnt <= cnt + 16'd1;
                end
                S_STOP: begin
                    if (cnt == DIV[15: 0] - 16'd1) begin
                        cnt   <= 16'd0;
                        state <= S_IDLE;
                        if (rx_s1) begin               // 停止位为 1 才收下
                            data  <= data_r;
                            valid <= 1'b1;
                        end
                    end
                    else cnt <= cnt + 16'd1;
                end
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
