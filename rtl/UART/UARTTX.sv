`timescale 1ns / 1ps
`include "../sys_define.svh"

// ============================================================
//  UARTTX — 串口发送（8N1，LSB first）
// ------------------------------------------------------------
//  ready=1 表示空闲可接收；valid 拉高的那一拍把 data 取走，
//  随后依次发送：起始位(0) → 8 个数据位(LSB first) → 停止位(1)
// ============================================================
module UARTTX #(
        parameter int CLK_FREQ = 125_000_000,
        parameter int BAUD     = 115_200
    )(
        input  logic       clk,
        input  logic       rst_n,

        input  logic [7: 0] data,
        input  logic        valid,
        output logic        ready,

        output logic        txd
    );

    localparam int DIV = (CLK_FREQ + BAUD/2) / BAUD;   // 每 bit 的时钟数（四舍五入）

    localparam [1:0] S_IDLE  = 2'd0,
                     S_START = 2'd1,
                     S_DATA  = 2'd2,
                     S_STOP  = 2'd3;

    logic [1: 0]  state;
    logic [15: 0] cnt;
    logic [2: 0]  bit_idx;
    logic [7: 0]  data_r;

    assign ready = (state == S_IDLE);
    assign txd   = (state == S_IDLE)  ? 1'b1 :
                   (state == S_START) ? 1'b0 :
                   (state == S_DATA)  ? data_r[bit_idx] : 1'b1;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= S_IDLE;
            cnt     <= 16'd0;
            bit_idx <= 3'd0;
            data_r  <= 8'd0;
        end
        else begin
            case (state)
                S_IDLE: begin
                    cnt <= 16'd0;
                    if (valid) begin
                        data_r  <= data;
                        bit_idx <= 3'd0;
                        state   <= S_START;
                    end
                end
                S_START: begin
                    if (cnt == DIV[15: 0] - 16'd1) begin
                        cnt   <= 16'd0;
                        state <= S_DATA;
                    end
                    else cnt <= cnt + 16'd1;
                end
                S_DATA: begin
                    if (cnt == DIV[15: 0] - 16'd1) begin
                        cnt <= 16'd0;
                        if (bit_idx == 3'd7) state <= S_STOP;
                        else                 bit_idx <= bit_idx + 3'd1;
                    end
                    else cnt <= cnt + 16'd1;
                end
                S_STOP: begin
                    if (cnt == DIV[15: 0] - 16'd1) begin
                        cnt   <= 16'd0;
                        state <= S_IDLE;
                    end
                    else cnt <= cnt + 16'd1;
                end
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
