`timescale 1ns / 1ps
`include "../sys_define.svh"

// ============================================================
//  ARPRX — ARP 报文接收 / 学习 / 请求判定
// ------------------------------------------------------------
//  ARP 载荷（以太网 + IPv4）固定 28 字节：
//      [ 0.. 1] htype = 0x0001
//      [ 2.. 3] ptype = 0x0800
//      [ 4]     hlen  = 6
//      [ 5]     plen  = 4
//      [ 6.. 7] oper  = 1 请求 / 2 应答
//      [ 8..13] sha  发送方 MAC
//      [14..17] spa  发送方 IP
//      [18..23] tha  目标 MAC
//      [24..27] tpa  目标 IP
//
//  功能
//      1) 前 28 字节移入一个 224bit 移位寄存器，收满即组合译码
//      2) 学习：把 (spa, sha) 写入 IP/MAC 映射表
//      3) 若 oper=请求 且 tpa=本机 IP → 产生应答请求，交给 ARPTX
//
//  为什么不在收满 28 字节时立刻提交
//      以太网帧总长不足 60 字节时会补 padding，FCS 校验结果要到 MAC 帧
//      真正结束时才知道。所以这里只把"待学习/待应答"锁存下来，等
//      frame_end 那一拍、确认 frame_err=0 之后才真正提交。
//      frame_end / frame_err 由顶层接到 Arbiter 上游的 rx_eop / rx_err。
// ============================================================
module ARPRX #(
        parameter [31: 0] LOCAL_IP  = 32'hC0A8_0164,   // 192.168.1.100
        parameter [47: 0] LOCAL_MAC = 48'h00_11_22_33_44_55
    )(
        // System Interface
        input  logic         clk,
        input  logic         rst_n,

        // 来自 Arbiter 的 arp 端口
        input  logic [7: 0]  arp_data,
        input  logic         arp_valid,
        input  logic         arp_sop,
        input  logic         arp_eop,

        // MAC 帧判决（顶层接 Arbiter 上游的 rx_eop / rx_err）
        input  logic         frame_end,
        input  logic         frame_err,

        // 学习：写入 IP/MAC 映射表
        output logic         map_wr_en,
        output logic [31: 0] map_wr_ip,
        output logic [47: 0] map_wr_mac,

        // 需要回 ARP 应答（与 frame_end 同拍的一拍脉冲）
        output logic         reply_req,
        output logic [47: 0] reply_dst_mac,
        output logic [31: 0] reply_dst_ip,

        // 统计
        output logic [15: 0] stat_rx,      // 合法 ARP 报文
        output logic [15: 0] stat_req,     // 询问本机 IP 的请求
        output logic [15: 0] stat_rep,     // 应答报文
        output logic [15: 0] stat_drop     // 不是合法 ARP 的帧
    );

    // ----------------------------------------
    //  内部寄存器 / 线网声明
    // ----------------------------------------
    logic [223: 0] pkt_sr;          // 28 字节移位寄存器
    logic [5: 0]   cnt;             // 本帧已收字节数
    logic          pkt_good_seen;

    logic          pend_learn;
    logic [31: 0]  pend_ip;
    logic [47: 0]  pend_mac;

    logic          pend_reply;
    logic [47: 0]  pend_rep_mac;
    logic [31: 0]  pend_rep_ip;

    wire sop = arp_sop && arp_valid;
    wire vld = arp_valid;

    // 收满 28 字节那一拍，整包组合可见
    wire [223: 0] pkt = {pkt_sr[215: 0], arp_data};

    wire [15: 0] f_htype = pkt[223: 208];
    wire [15: 0] f_ptype = pkt[207: 192];
    wire [7: 0]  f_hlen  = pkt[191: 184];
    wire [7: 0]  f_plen  = pkt[183: 176];
    wire [15: 0] f_oper  = pkt[175: 160];
    wire [47: 0] f_sha   = pkt[159: 112];
    wire [31: 0] f_spa   = pkt[111: 80];
    wire [47: 0] f_tha   = pkt[79: 32];
    wire [31: 0] f_tpa   = pkt[31: 0];

    // 收满 28 字节那一拍
    wire pkt_full = vld && (cnt == 6'd27);

    // 报文合法性（字段格式）
    wire pkt_good = pkt_full
                    && (f_htype == `ARP_HTYPE_ETH)
                    && (f_ptype == `ARP_PTYPE_IP)
                    && (f_hlen  == `ARP_HLEN)
                    && (f_plen  == `ARP_PLEN);

    // 这条学习记录是否有效（排除全 0 的异常值）
    wire learn_ok  = pkt_good && (f_spa != 32'd0) && (f_sha != 48'd0);

    wire is_request = f_oper == `ARP_OP_REQUEST;
    wire is_reply   = f_oper == `ARP_OP_REPLY;
    wire ask_us     = (f_tpa == LOCAL_IP);

    // ----------------------------------------
    //  字节计数 + 移位接收
    //  cnt 在 28 处饱和，保证 pkt_full 每帧只触发一次
    // ----------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt           <= 6'd0;
            pkt_sr        <= 224'd0;
            pkt_good_seen <= 1'b0;
        end
        else if (vld) begin
            if (sop) begin
                cnt           <= 6'd1;
                pkt_sr        <= {216'd0, arp_data};
                pkt_good_seen <= 1'b0;
            end
            else begin
                if (cnt < 6'd28) cnt <= cnt + 6'd1;
                if (cnt < 6'd28) pkt_sr <= {pkt_sr[215: 0], arp_data};
                if (pkt_full && pkt_good) pkt_good_seen <= 1'b1;
            end
        end
    end

    // ----------------------------------------
    //  待学习 / 待应答 锁存
    // ----------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pend_learn   <= 1'b0;
            pend_ip      <= 32'd0;
            pend_mac     <= 48'd0;
            pend_reply   <= 1'b0;
            pend_rep_mac <= 48'd0;
            pend_rep_ip  <= 32'd0;
        end
        else if (pkt_full) begin
            pend_learn   <= learn_ok;
            pend_ip      <= f_spa;
            pend_mac     <= f_sha;
            pend_reply   <= pkt_good && is_request && ask_us;
            pend_rep_mac <= f_sha;
            pend_rep_ip  <= f_spa;
        end
        else if (frame_end) begin
            pend_learn <= 1'b0;
            pend_reply <= 1'b0;
        end
    end

    // MAC 帧判决：只有 CRC/帧长都好才提交
    assign map_wr_en  = frame_end && pend_learn && !frame_err;
    assign map_wr_ip  = pend_ip;
    assign map_wr_mac = pend_mac;

    assign reply_req     = frame_end && pend_reply && !frame_err;
    assign reply_dst_mac = pend_rep_mac;
    assign reply_dst_ip  = pend_rep_ip;

    // ----------------------------------------
    //  统计
    // ----------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stat_rx   <= 16'd0;
            stat_req  <= 16'd0;
            stat_rep  <= 16'd0;
            stat_drop <= 16'd0;
        end
        else begin
            if (pkt_full && pkt_good && (stat_rx != 16'hFFFF))
                stat_rx <= stat_rx + 16'd1;
            if (pkt_full && pkt_good && is_request && ask_us && (stat_req != 16'hFFFF))
                stat_req <= stat_req + 16'd1;
            if (pkt_full && pkt_good && is_reply && (stat_rep != 16'hFFFF))
                stat_rep <= stat_rep + 16'd1;
            if (arp_eop && vld && !pkt_good_seen && (stat_drop != 16'hFFFF))
                stat_drop <= stat_drop + 16'd1;
        end
    end

endmodule
