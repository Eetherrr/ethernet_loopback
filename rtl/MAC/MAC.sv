`timescale 1ns / 1ps
`include "../sys_define.svh"

// ============================================================
//  MAC — 以太网 MAC 层顶层
// ------------------------------------------------------------
//  把 MACRX / MACTX 打包，两条通路互相独立，内部没有跨时钟域逻辑：
//      RX 通路：clk_rx（PHY 提供的 RGMII RX 时钟）
//      TX 通路：clk_tx（125MHz，RGMII TX 时钟）
//
//  MAC 与 RGMII 模块之间是 8bit SDR 信号（rgmii_rxd/rxdv/err、
//  rgmii_txd/txdv），DDR 原语在 RGMII 模块里完成。
//
//  MACRX 输出是直通流且没有反压，接上层（Arbiter）时如果上层不在
//  同一个时钟域或不能持续接收，需要在中间插入异步 FIFO。
// ============================================================
module MAC #(
        parameter [47:0] LOCAL_MAC = 48'h00_11_22_33_44_55,
        // 见 MACRX：默认按 RGMII 规范解码 RX_CTL 的下降沿
        parameter bit    ERR_ACTIVE_HIGH = 1'b0
    )(
        // System Interface — RX 时钟域（RGMII RX 时钟）
        input  logic         clk_rx,
        input  logic         rst_rx,

        // System Interface — TX 时钟域（125MHz）
        input  logic         clk_tx,
        input  logic         rst_tx,

        // User Interface — 接收（→ Arbiter）
        output logic [7: 0]  rx_data,
        output logic         rx_valid,
        output logic         rx_sop,
        output logic         rx_eop,
        output logic         rx_err,
        output logic         rx_crc_err,
        output logic [15: 0] rx_len,
        output logic [47: 0] rx_dst_mac,
        output logic [47: 0] rx_src_mac,
        output logic [15: 0] rx_ethertype,

        // User Interface — 发送（← IPTX / ARPTX 仲裁）
        input  logic [7: 0]  tx_data,
        input  logic         tx_valid,
        input  logic         tx_sop,
        input  logic         tx_eop,
        output logic         tx_ready,
        input  logic [47: 0] tx_dst_mac,
        input  logic [15: 0] tx_ethertype,
        output logic         tx_busy,
        output logic         tx_err,

        // RGMII Interface（8bit SDR，接 RGMII 模块）
        input  logic [7: 0]  rgmii_rxd,
        input  logic         rgmii_rxdv,
        input  logic         rgmii_err,
        output logic [7: 0]  rgmii_txd,
        output logic         rgmii_txdv
    );

    MACRX #(
              .LOCAL_MAC       (LOCAL_MAC),
              .ERR_ACTIVE_HIGH (ERR_ACTIVE_HIGH)
          ) MACRX_inst (
              .clk_rx        (clk_rx),
              .rst_rx        (rst_rx),
              .rx_data       (rx_data),
              .rx_valid      (rx_valid),
              .rx_sop        (rx_sop),
              .rx_eop        (rx_eop),
              .rx_err        (rx_err),
              .rx_crc_err    (rx_crc_err),
              .rx_len        (rx_len),
              .rx_dst_mac    (rx_dst_mac),
              .rx_src_mac    (rx_src_mac),
              .rx_ethertype  (rx_ethertype),
              .rgmii_rxd     (rgmii_rxd),
              .rgmii_rxdv    (rgmii_rxdv),
              .rgmii_err     (rgmii_err)
          );

    MACTX #(
              .LOCAL_MAC (LOCAL_MAC)
          ) MACTX_inst (
              .clk_tx        (clk_tx),
              .rst_tx        (rst_tx),
              .tx_data       (tx_data),
              .tx_valid      (tx_valid),
              .tx_sop        (tx_sop),
              .tx_eop        (tx_eop),
              .tx_ready      (tx_ready),
              .tx_dst_mac    (tx_dst_mac),
              .tx_ethertype  (tx_ethertype),
              .tx_busy       (tx_busy),
              .tx_err        (tx_err),
              .rgmii_txd     (rgmii_txd),
              .rgmii_txdv    (rgmii_txdv)
          );

endmodule
