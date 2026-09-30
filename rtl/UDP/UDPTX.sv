`timescale 1ns / 1ps
`include "../sys_define.svh"

// ============================================================
//  UDPTX — UDP 首部组装
// ------------------------------------------------------------
//  上游：APP 的发送缓冲区（FWFT 读口：buf_data 就是当前队首字节，
//        buf_rd_en 一拍把它弹掉）
//  下游：IPTX
//
//  组装的 8 字节首部：
//      [0..1] 源端口
//      [2..3] 目的端口
//      [4..5] UDP 长度 = 8 + 数据长度
//      [6..7] 校验和 = 0x0000
//
//  关于校验和：RFC 768 允许 IPv4 下把 UDP 校验和置 0 表示"未计算"，
//  接收端必须接受。这里就采用 0，省掉"先算一遍再回填首部"的两遍访问。
//  需要严格校验和时，在 APP 里预扫一遍缓冲区即可补上。
//
//  请求用**电平** start：APP 拉起后一直保持，直到看到 busy 再撤，
//  这样即使下游 MAC 正忙（在发 ARP）也不会丢请求。
// ============================================================
module UDPTX (
        // System Interface
        input  logic         clk,
        input  logic         rst_n,

        // 请求（来自 APP）
        input  logic         start,
        input  logic [15: 0] src_port,
        input  logic [15: 0] dst_port,
        input  logic [15: 0] payload_len,      // UDP 数据长度

        // 发送缓冲区读口（FWFT）
        input  logic [7: 0]  buf_data,
        input  logic         buf_empty,
        output logic         buf_rd_en,

        // 下游：IPTX
        output logic [7: 0]  tx_data,
        output logic         tx_valid,
        output logic         tx_sop,
        output logic         tx_eop,
        input  logic         tx_ready,

        // 状态
        output logic         busy,
        output logic         done,
        output logic [15: 0] stat_tx
    );

    // ----------------------------------------
    //  状态
    // ----------------------------------------
    localparam [1:0] S_IDLE = 2'd0,
                     S_HDR  = 2'd1,
                     S_DATA = 2'd2;

    logic [1: 0]  state;
    logic [2: 0]  idx;              // 首部字节序号 0..7
    logic [15: 0] cnt;              // 已送出的数据字节数
    logic [63: 0] hdr_r;
    logic [15: 0] pay_len_r;

    // ----------------------------------------
    //  组合组装首部
    // ----------------------------------------
    wire [63: 0] hdr_c = {src_port, dst_port, (16'd8 + payload_len), 16'h0000};

    wire [2: 0] hidx = 3'd7 - idx;
    wire [7: 0] hdr_byte = hdr_r[{hidx, 3'b000} +: 8];

    // ----------------------------------------
    //  输出
    // ----------------------------------------
    assign tx_data    = (state == S_HDR) ? hdr_byte : buf_data;
    assign tx_valid   = (state == S_HDR) || ((state == S_DATA) && !buf_empty);
    assign tx_sop     = (state == S_HDR) && (idx == 3'd0);
    assign tx_eop     = (state == S_DATA) && !buf_empty && (cnt == (pay_len_r - 16'd1));
    assign buf_rd_en  = (state == S_DATA) && tx_ready && tx_valid;
    assign busy       = (state != S_IDLE);

    // ----------------------------------------
    //  FSM
    // ----------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            idx       <= 3'd0;
            cnt       <= 16'd0;
            hdr_r     <= 64'd0;
            pay_len_r <= 16'd0;
            done      <= 1'b0;
        end
        else begin
            done <= 1'b0;
            case (state)
                S_IDLE: begin
                    if (start) begin
                        state     <= S_HDR;
                        hdr_r     <= hdr_c;
                        pay_len_r <= payload_len;
                        idx       <= 3'd0;
                        cnt       <= 16'd0;
                    end
                end
                S_HDR: begin
                    if (tx_ready) begin
                        if (idx == 3'd7) begin
                            state <= S_DATA;
                            cnt   <= 16'd0;
                        end
                        else begin
                            idx <= idx + 3'd1;
                        end
                    end
                end
                S_DATA: begin
                    if (tx_ready && tx_valid) begin
                        if (cnt == (pay_len_r - 16'd1)) begin
                            state <= S_IDLE;
                            done  <= 1'b1;
                        end
                        else begin
                            cnt <= cnt + 16'd1;
                        end
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end

    // ----------------------------------------
    //  统计
    // ----------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) stat_tx <= 16'd0;
        else if ((state == S_IDLE) && start && (stat_tx != 16'hFFFF)) stat_tx <= stat_tx + 16'd1;
    end

endmodule
