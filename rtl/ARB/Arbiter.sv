`timescale 1ns / 1ps
`include "../sys_define.svh"

// ============================================================
//  Arbiter — MAC 接收流到协议层的分流器
// ------------------------------------------------------------
//  框图位置：  MACRX ──► Arbiter ──┬──► IPRX  (EtherType 0x0800)
//                                 └──► ARPRX (EtherType 0x0806)
//
//  数据流
//      MACRX 已经把以太网帧头剥掉，送上来的是 payload 字节流
//      （rx_data/valid/sop/eop/err）+ 帧头元数据（dst/src MAC、EtherType、len）。
//      Arbiter 按 rx_ethertype 决定这一帧走 IP 还是 ARP；
//      两者都不是（IPv6、VLAN、未知类型）就整帧丢弃，并在帧尾给出
//      drop_pulse + drop_ethertype。
//
//  为什么是纯组合逻辑
//      MACRX 的 rx_ethertype 在 rx_sop 之前就已稳定，并一直保持到 rx_eop
//      （下一帧帧头才会刷新它），所以当拍就能译码，不需要锁存路由状态，
//      直通零延迟。
//
//  反压
//      MACRX 是直通流、没有反压，因此 Arbiter 也没有 ready。
//      IPRX / ARPRX 必须**每个时钟都能接收**（只解析帧头、不做长缓存）。
//      将来若某个下游需要反压，应在 MACRX 与 Arbiter 之间插一个 RX FIFO，
//      由 FIFO 的 prog_full 去控制上游，而不是在这里加 ready。
//
//  时序约定（与 MACRX 一致）
//      ip_sop / arp_sop  : 与 payload 第 1 字节同拍
//      ip_eop / arp_eop  : 与 payload 最后 1 字节同拍，ip_err/arp_err 同拍有效
//      ip_dst_mac / ip_src_mac / ip_len：sop 之前稳定，eop 时 len 有效
//      只要出现过 sop，就一定会出现 eop
// ============================================================
module Arbiter (
        // System Interface
        input  logic         clk,
        input  logic         rst_n,

        // ---------------- 上游：MACRX ----------------
        input  logic [7: 0]  rx_data,
        input  logic         rx_valid,
        input  logic         rx_sop,
        input  logic         rx_eop,
        input  logic         rx_err,
        input  logic [15: 0] rx_len,
        input  logic [47: 0] rx_dst_mac,
        input  logic [47: 0] rx_src_mac,
        input  logic [15: 0] rx_ethertype,

        // ---------------- 下游：IPRX ----------------
        output logic [7: 0]  ip_data,
        output logic         ip_valid,
        output logic         ip_sop,
        output logic         ip_eop,
        output logic         ip_err,
        output logic [15: 0] ip_len,
        output logic [47: 0] ip_dst_mac,
        output logic [47: 0] ip_src_mac,

        // ---------------- 下游：ARPRX ----------------
        output logic [7: 0]  arp_data,
        output logic         arp_valid,
        output logic         arp_sop,
        output logic         arp_eop,
        output logic         arp_err,
        output logic [15: 0] arp_len,
        output logic [47: 0] arp_dst_mac,
        output logic [47: 0] arp_src_mac,

        // ---------------- 丢弃指示 ----------------
        // drop_pulse 与 rx_eop 同拍；drop_ethertype 仅在该拍有效
        output logic         drop_pulse,
        output logic [15: 0] drop_ethertype,

        // ---------------- 统计（饱和计数，帧尾 +1）----------------
        output logic [15: 0] stat_ip_frames,
        output logic [15: 0] stat_arp_frames,
        output logic [15: 0] stat_drop_frames,
        output logic [15: 0] stat_err_frames
    );

    // ----------------------------------------
    //  按 EtherType 译码
    // ----------------------------------------
    wire is_ip  = (rx_ethertype == `ETHERTYPE_IP);
    wire is_arp = (rx_ethertype == `ETHERTYPE_ARP);

    // ----------------------------------------
    //  IP 端口
    // ----------------------------------------
    assign ip_data    = rx_data;
    assign ip_valid   = rx_valid && is_ip;
    assign ip_sop     = rx_sop   && is_ip;
    assign ip_eop     = rx_eop   && is_ip;
    assign ip_err     = rx_err;
    assign ip_len     = rx_len;
    assign ip_dst_mac = rx_dst_mac;
    assign ip_src_mac = rx_src_mac;

    // ----------------------------------------
    //  ARP 端口
    // ----------------------------------------
    assign arp_data    = rx_data;
    assign arp_valid   = rx_valid && is_arp;
    assign arp_sop     = rx_sop   && is_arp;
    assign arp_eop     = rx_eop   && is_arp;
    assign arp_err     = rx_err;
    assign arp_len     = rx_len;
    assign arp_dst_mac = rx_dst_mac;
    assign arp_src_mac = rx_src_mac;

    // ----------------------------------------
    //  未支持的类型：整帧丢弃
    // ----------------------------------------
    assign drop_pulse     = rx_eop && !is_ip && !is_arp;
    assign drop_ethertype = rx_ethertype;

    // ----------------------------------------
    //  统计
    // ----------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stat_ip_frames   <= 16'd0;
            stat_arp_frames  <= 16'd0;
            stat_drop_frames <= 16'd0;
            stat_err_frames  <= 16'd0;
        end
        else if (rx_eop) begin
            if (is_ip && (stat_ip_frames != 16'hFFFF))
                stat_ip_frames <= stat_ip_frames + 16'd1;
            else if (is_arp && (stat_arp_frames != 16'hFFFF))
                stat_arp_frames <= stat_arp_frames + 16'd1;
            else if (!is_ip && !is_arp && (stat_drop_frames != 16'hFFFF))
                stat_drop_frames <= stat_drop_frames + 16'd1;

            if (rx_err && (stat_err_frames != 16'hFFFF))
                stat_err_frames <= stat_err_frames + 16'd1;
        end
    end

endmodule
