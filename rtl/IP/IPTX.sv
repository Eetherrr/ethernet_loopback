`timescale 1ns / 1ps
`include "../sys_define.svh"

// ============================================================
//  IPTX — IPv4 首部组装
// ------------------------------------------------------------
//  上游：UDPTX 的字节流（UDP 首部 + UDP 数据）
//  下游：MACTX 用户接口
//
//  组装的 20 字节固定首部（无选项）：
//      [ 0]    0x45                version=4, IHL=5
//      [ 1]    0x00                TOS
//      [ 2..3] 20 + payload_len    total length
//      [ 4..5] 报文 ID（每包 +1）
//      [ 6..7] 0x0000              flags/fragment
//      [ 8]    64                  TTL
//      [ 9]    protocol（UDP=17）
//      [10..11] 首部校验和（在线计算）
//      [12..15] 源 IP
//      [16..19] 目的 IP
//
//  下游握手遵循 MACTX 约定：SOP 那一拍同时给出 tx_dst_mac / tx_ethertype /
//  第 1 个字节；之后 19 个首部字节 + payload_len 个载荷字节连续送出，
//  最后一个字节同拍 tx_eop。
// ============================================================
module IPTX (
        // System Interface
        input  logic         clk,
        input  logic         rst_n,

        // 请求（来自 APP）
        input  logic         start,
        input  logic [15: 0] payload_len,      // IP 载荷长度（UDP 首部 + 数据）
        input  logic [31: 0] src_ip,
        input  logic [31: 0] dst_ip,
        input  logic [7: 0]  protocol,
        input  logic [47: 0] dst_mac,

        // 上游 payload（来自 UDPTX）
        input  logic [7: 0]  up_data,
        input  logic         up_valid,
        output logic         up_ready,

        // 下游：MACTX 用户接口
        output logic [7: 0]  tx_data,
        output logic         tx_valid,
        output logic         tx_sop,
        output logic         tx_eop,
        input  logic         tx_ready,
        output logic [47: 0] tx_dst_mac,
        output logic [15: 0] tx_ethertype,

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
                     S_PAY  = 2'd2;

    logic [1: 0]  state;
    logic [4: 0]  idx;              // 首部字节序号 1..19
    logic [15: 0] pay_cnt;          // 已送出的载荷字节数
    logic [159: 0] pkt_r;           // 本帧 20 字节首部
    logic [15: 0] pay_len_r;
    logic [47: 0] dst_mac_r;
    logic [15: 0] ip_id;

    // ----------------------------------------
    //  组合组装首部（SOP 拍就要用 byte0，必须组合生成）
    // ----------------------------------------
    wire [15: 0] total_len_c = 16'd20 + payload_len;

    logic [159: 0] hdr_c;
    always_comb begin
        hdr_c[159: 152] = `IP_VER_IHL;
        hdr_c[151: 144] = 8'h00;
        hdr_c[143: 128] = total_len_c;
        hdr_c[127: 112] = ip_id;
        hdr_c[111: 96]  = 16'h0000;
        hdr_c[95: 88]   = 8'd64;
        hdr_c[87: 80]   = protocol;
        hdr_c[79: 64]   = 16'h0000;      // 先填 0 再算校验和
        hdr_c[63: 32]   = src_ip;
        hdr_c[31: 0]    = dst_ip;
    end

    // 首部校验和：10 个 16bit 字求和、折叠、取反
    function automatic [15: 0] ip_csum(input [159: 0] h);
        logic [31: 0] s;
        s = 32'd0;
        for (int i = 0; i < 10; i++)
            s = s + {16'd0, h[16*(9-i) +: 16]};
        s = (s & 32'h0000_FFFF) + (s >> 16);
        s = (s & 32'h0000_FFFF) + (s >> 16);
        return ~s[15: 0];
    endfunction

    // 校验和字段先置 0 再计算
    wire [15: 0] csum_c = ip_csum(hdr_c);
    wire [159: 0] hdr_final = {hdr_c[159: 80], csum_c, hdr_c[63: 0]};

    // SOP 拍要发的字节 0
    wire [7: 0] hdr_byte0 = hdr_final[159: 152];

    // 首部字节 idx 的取值
    wire [4: 0] hidx = 5'd19 - idx;
    wire [7: 0] hdr_byte = pkt_r[{hidx, 3'b000} +: 8];

    // ----------------------------------------
    //  SOP 判定
    // ----------------------------------------
    wire kick = (state == S_IDLE) && start && tx_ready;

    // ----------------------------------------
    //  输出
    // ----------------------------------------
    assign tx_data =
        (state == S_IDLE) ? hdr_byte0 :
        (state == S_HDR)  ? hdr_byte  : up_data;

    assign tx_valid = (state == S_HDR) || (state == S_PAY) || kick;
    assign tx_sop   = kick;

    assign tx_eop   = (state == S_PAY) && up_valid && (pay_cnt == (pay_len_r - 16'd1));

    assign up_ready = (state == S_PAY);

    assign tx_dst_mac   = (state == S_IDLE) ? dst_mac : dst_mac_r;
    assign tx_ethertype = `ETHERTYPE_IP;
    assign busy         = (state != S_IDLE);

    // ----------------------------------------
    //  FSM
    // ----------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            idx       <= 5'd1;
            pay_cnt   <= 16'd0;
            pkt_r     <= 160'd0;
            pay_len_r <= 16'd0;
            dst_mac_r <= 48'd0;
            ip_id     <= 16'd0;
            done      <= 1'b0;
        end
        else begin
            done <= 1'b0;
            case (state)
                S_IDLE: begin
                    if (kick) begin
                        state     <= S_HDR;
                        pkt_r     <= hdr_final;
                        pay_len_r <= payload_len;
                        dst_mac_r <= dst_mac;
                        idx       <= 5'd1;
                        pay_cnt   <= 16'd0;
                        ip_id     <= ip_id + 16'd1;
                    end
                end
                S_HDR: begin
                    if (tx_ready) begin
                        if (idx == 5'd19) begin
                            state   <= S_PAY;
                            pay_cnt <= 16'd0;
                        end
                        else begin
                            idx <= idx + 5'd1;
                        end
                    end
                end
                S_PAY: begin
                    if (up_valid && tx_ready) begin
                        if (pay_cnt == (pay_len_r - 16'd1)) begin
                            state <= S_IDLE;
                            done  <= 1'b1;
                        end
                        else begin
                            pay_cnt <= pay_cnt + 16'd1;
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
        else if (kick && (stat_tx != 16'hFFFF)) stat_tx <= stat_tx + 16'd1;
    end

endmodule
