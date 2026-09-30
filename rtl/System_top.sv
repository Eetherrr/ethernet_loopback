`timescale 1ns / 1ps
`include "sys_define.svh"

// ============================================================
//  System_top — 以太网 UDP 回环顶层
// ------------------------------------------------------------
//  数据通路
//      RX: PHY → RGMII → MAC(MACRX) → Arbiter ─┬→ IPRX → UDPRX → APP → UARTTX → PC
//                                              └→ ARPRX → IP/MAC 表
//      TX: PC → UARTRX → APP ─(查表)─┬→ UDPTX → IPTX ─┐
//                                    └→ ARPTX ────────┴→ TxArbiter → MACTX → RGMII → PHY
//
//  时钟方案（方案 B：全设计单时钟）
//      整个设计只用 PHY 提供的 RGMII RX 时钟 phy_rx_clk：它同时作为
//      RGMII 发送侧 ODDR 的时钟，因此没有任何跨时钟域逻辑，也就不需要
//      异步 FIFO。代价是链路 down 时 PHY 不输出 RXCLK，整个设计失去时钟。
//      实机上 phy_rx_clk 需要过 BUFG；复位要同步到该时钟域后再送进来。
//
//  回环测试怎么用
//      1) PC 先往板子的 LOCAL_IP:LOCAL_PORT 发一个 UDP 包
//         → APP 学到对端 (IP, 端口)，ARP 层把对端 MAC 写进表
//         → 该包载荷从串口原样吐出
//      2) PC 往串口写一串字节，静默 GAP_BITS 个 bit 时间后
//         → 板子把这段字节打成**一个** UDP 包发回 PC
//         → 表里若没有对端 IP 的 MAC，先发 ARP 请求、等应答填表再发 UDP
// ============================================================
module System_top #(
        parameter [47: 0] LOCAL_MAC   = 48'h00_11_22_33_44_55,
        parameter [31: 0] LOCAL_IP    = 32'hC0A8_0164,     // 192.168.1.100
        parameter [15: 0] LOCAL_PORT  = 16'd1234,
        parameter int     CLK_FREQ    = 125_000_000,
        parameter int     BAUD        = 115_200,
        parameter int     BUF_AW      = 11,                // 收发缓冲各 2KB
        parameter int     GAP_BITS    = 40,                // 串口空闲成包门限
        parameter int     ARP_TIMEOUT = 125_000_000        // ARP 等待超时（1s@125MHz）
    )(
        // ---------------- 时钟 / 复位 ----------------
        input  logic        phy_rx_clk,     // 方案 B：唯一时钟 = PHY 的 RGMII RX 时钟
        input  logic        rst_n,          // 需已同步到 phy_rx_clk

        // ---------------- PHY 接口（RGMII）----------------
        output logic        phy_tx_clk,
        output logic [3: 0] phy_txd,
        output logic        phy_tx_ctl,
        input  logic [3: 0] phy_rxd,
        input  logic        phy_rx_ctl,

        // ---------------- 串口 ----------------
        output logic        uart_txd,
        input  logic        uart_rxd
    );

    wire clk = phy_rx_clk;

    // ---------------- RGMII ↔ MAC ----------------
    logic [7: 0] rgmii_rxd;
    logic        rgmii_rxdv, rgmii_err;
    logic [7: 0] rgmii_txd;
    logic        rgmii_txdv;

    // ---------------- MAC → Arbiter ----------------
    logic [7: 0]  rx_data;
    logic         rx_valid, rx_sop, rx_eop, rx_err, rx_crc_err;
    logic [15: 0] rx_len;
    logic [47: 0] rx_dst_mac, rx_src_mac;
    logic [15: 0] rx_ethertype;

    // ---------------- Arbiter → IPRX ----------------
    logic [7: 0]  iprx_data;
    logic         iprx_valid, iprx_sop, iprx_eop, iprx_err;
    logic [15: 0] iprx_len;
    logic [47: 0] iprx_dst_mac, iprx_src_mac;

    // ---------------- Arbiter → ARPRX ----------------
    logic [7: 0]  arprx_data;
    logic         arprx_valid, arprx_sop, arprx_eop, arprx_err;
    logic [15: 0] arprx_len;
    logic [47: 0] arprx_dst_mac, arprx_src_mac;
    logic         arb_drop_pulse;
    logic [15: 0] arb_drop_ethertype;

    // ---------------- 统计（不引出，仿真可层次引用 / 实机可接 ILA）----------------
    logic [15: 0] st_arb_ip, st_arb_arp, st_arb_drop, st_arb_err;
    logic [15: 0] st_arp_rx, st_arp_req, st_arp_rep, st_arp_drop;
    logic [15: 0] st_arp_tx_req, st_arp_tx_rep;
    logic [15: 0] st_iprx, st_ip_drop_hdr, st_ip_drop_proto, st_iptx;
    logic [15: 0] st_udprx, st_udp_drop, st_udptx;
    logic [15: 0] st_app_rx_pkt, st_app_rx_byte, st_app_tx_pkt, st_app_tx_byte;
    logic [15: 0] st_app_drop, st_app_arp_wait, st_app_uart_drop;

    // ---------------- IP/MAC 表 ----------------
    logic         map_wr_en;
    logic [31: 0] map_wr_ip;
    logic [47: 0] map_wr_mac;
    logic [31: 0] map_rd_ip;
    logic         map_hit;
    logic [47: 0] map_mac;
    logic [2: 0]  map_valid;
    logic [1: 0]  map_wr_ptr;

    // ---------------- IPRX → UDPRX ----------------
    logic [7: 0]  udprx_data;
    logic         udprx_valid, udprx_sop, udprx_eop, udprx_err;
    logic [31: 0] udprx_src_ip, udprx_dst_ip;
    logic [15: 0] udprx_payload_len;
    logic [7: 0]  udprx_proto;

    // ---------------- UDPRX → APP ----------------
    logic [7: 0]  app_data;
    logic         app_valid, app_sop, app_eop, app_err;
    logic [15: 0] app_src_port, app_dst_port, app_data_len;
    logic [31: 0] app_src_ip, app_dst_ip;

    // ---------------- APP ↔ UART ----------------
    logic [7: 0]  uart_tx_data;
    logic         uart_tx_valid, uart_tx_ready;
    logic [7: 0]  uart_rx_data;
    logic         uart_rx_valid;

    // ---------------- APP 状态 ----------------
    logic [31: 0] app_peer_ip;
    logic [15: 0] app_peer_port;
    logic         app_peer_valid;

    // ---------------- APP → UDPTX / IPTX ----------------
    logic         udp_tx_start;
    logic [15: 0] udp_tx_src_port, udp_tx_dst_port, udp_tx_payload_len;
    logic [15: 0] ip_tx_payload_len;
    logic         udp_tx_busy, ip_tx_busy;

    // ---------------- APP ↔ 发送缓冲 ----------------
    logic [7: 0]  txbuf_data;
    logic         txbuf_rd_en, txbuf_empty;

    // ---------------- APP → ARPTX ----------------
    logic         arp_app_req;
    logic [31: 0] arp_app_target_ip;

    // ---------------- ARPRX → ARPTX ----------------
    logic         arp_rep_req;
    logic [47: 0] arp_rep_dst_mac;
    logic [31: 0] arp_rep_dst_ip;

    // ---------------- UDPTX → IPTX ----------------
    logic [7: 0]  udptx_data;
    logic         udptx_valid, udptx_sop, udptx_eop, udptx_ready;

    // ---------------- IPTX → TxArbiter ----------------
    logic [7: 0]  iptx_data;
    logic         iptx_valid, iptx_sop, iptx_eop;
    logic [47: 0] iptx_dst_mac;
    logic [15: 0] iptx_ethertype;
    logic         iptx_busy, iptx_done;

    // ---------------- ARPTX → TxArbiter ----------------
    logic [7: 0]  arptx_data;
    logic         arptx_valid, arptx_sop, arptx_eop;
    logic [47: 0] arptx_dst_mac;
    logic [15: 0] arptx_ethertype;
    logic         arptx_busy, arptx_done;

    // ---------------- TxArbiter → MACTX ----------------
    logic [7: 0]  mactx_data;
    logic         mactx_valid, mactx_sop, mactx_eop;
    logic         mactx_ready;
    logic [47: 0] mactx_dst_mac;
    logic [15: 0] mactx_ethertype;
    logic         mactx_busy, mactx_err;

    // ============================================================
    //  PHY / RGMII
    // ============================================================
    RGMII u_rgmii (
              .clk_sys    (clk),
              .rst_sys    (rst_n),
              .rgmii_txd  (rgmii_txd),
              .rgmii_txdv (rgmii_txdv),
              .rgmii_rxd  (rgmii_rxd),
              .rgmii_rxdv (rgmii_rxdv),
              .rgmii_err  (rgmii_err),
              .phy_tx_clk (phy_tx_clk),
              .phy_txd    (phy_txd),
              .phy_tx_ctl (phy_tx_ctl),
              .phy_rx_clk (phy_rx_clk),
              .phy_rxd    (phy_rxd),
              .phy_rx_ctl (phy_rx_ctl)
          );

    // ============================================================
    //  MAC
    // ============================================================
    MAC #(
            .LOCAL_MAC       (LOCAL_MAC),
            .ERR_ACTIVE_HIGH (1'b0)          // RGMII 规范编码
        ) u_mac (
            .clk_rx       (clk),
            .rst_rx       (rst_n),
            .clk_tx       (clk),
            .rst_tx       (rst_n),
            .rx_data      (rx_data),
            .rx_valid     (rx_valid),
            .rx_sop       (rx_sop),
            .rx_eop       (rx_eop),
            .rx_err       (rx_err),
            .rx_crc_err   (rx_crc_err),
            .rx_len       (rx_len),
            .rx_dst_mac   (rx_dst_mac),
            .rx_src_mac   (rx_src_mac),
            .rx_ethertype (rx_ethertype),
            .tx_data      (mactx_data),
            .tx_valid     (mactx_valid),
            .tx_sop       (mactx_sop),
            .tx_eop       (mactx_eop),
            .tx_ready     (mactx_ready),
            .tx_dst_mac   (mactx_dst_mac),
            .tx_ethertype (mactx_ethertype),
            .tx_busy      (mactx_busy),
            .tx_err       (mactx_err),
            .rgmii_rxd    (rgmii_rxd),
            .rgmii_rxdv   (rgmii_rxdv),
            .rgmii_err    (rgmii_err),
            .rgmii_txd    (rgmii_txd),
            .rgmii_txdv   (rgmii_txdv)
        );

    // ============================================================
    //  接收分流
    // ============================================================
    Arbiter u_arb (
                .clk              (clk),
                .rst_n            (rst_n),
                .rx_data          (rx_data),
                .rx_valid         (rx_valid),
                .rx_sop           (rx_sop),
                .rx_eop           (rx_eop),
                .rx_err           (rx_err),
                .rx_len           (rx_len),
                .rx_dst_mac       (rx_dst_mac),
                .rx_src_mac       (rx_src_mac),
                .rx_ethertype     (rx_ethertype),
                .ip_data          (iprx_data),
                .ip_valid         (iprx_valid),
                .ip_sop           (iprx_sop),
                .ip_eop           (iprx_eop),
                .ip_err           (iprx_err),
                .ip_len           (iprx_len),
                .ip_dst_mac       (iprx_dst_mac),
                .ip_src_mac       (iprx_src_mac),
                .arp_data         (arprx_data),
                .arp_valid        (arprx_valid),
                .arp_sop          (arprx_sop),
                .arp_eop          (arprx_eop),
                .arp_err          (arprx_err),
                .arp_len          (arprx_len),
                .arp_dst_mac      (arprx_dst_mac),
                .arp_src_mac      (arprx_src_mac),
                .drop_pulse       (arb_drop_pulse),
                .drop_ethertype   (arb_drop_ethertype),
                .stat_ip_frames   (st_arb_ip),
                .stat_arp_frames  (st_arb_arp),
                .stat_drop_frames (st_arb_drop),
                .stat_err_frames  (st_arb_err)
            );

    // ============================================================
    //  ARP 层
    // ============================================================
    ARPRX #(
              .LOCAL_IP  (LOCAL_IP),
              .LOCAL_MAC (LOCAL_MAC)
          ) u_arprx (
              .clk           (clk),
              .rst_n         (rst_n),
              .arp_data      (arprx_data),
              .arp_valid     (arprx_valid),
              .arp_sop       (arprx_sop),
              .arp_eop       (arprx_eop),
              .frame_end     (rx_eop),      // MAC 帧判决
              .frame_err     (rx_err),
              .map_wr_en     (map_wr_en),
              .map_wr_ip     (map_wr_ip),
              .map_wr_mac    (map_wr_mac),
              .reply_req     (arp_rep_req),
              .reply_dst_mac (arp_rep_dst_mac),
              .reply_dst_ip  (arp_rep_dst_ip),
              .stat_rx       (st_arp_rx),
              .stat_req      (st_arp_req),
              .stat_rep      (st_arp_rep),
              .stat_drop     (st_arp_drop)
          );

    ARPTX #(
              .LOCAL_IP  (LOCAL_IP),
              .LOCAL_MAC (LOCAL_MAC)
          ) u_arptx (
              .clk           (clk),
              .rst_n         (rst_n),
              .app_req       (arp_app_req),
              .app_target_ip (arp_app_target_ip),
              .rep_req       (arp_rep_req),
              .rep_dst_mac   (arp_rep_dst_mac),
              .rep_dst_ip    (arp_rep_dst_ip),
              .tx_data       (arptx_data),
              .tx_valid      (arptx_valid),
              .tx_sop        (arptx_sop),
              .tx_eop        (arptx_eop),
              .tx_ready      (mactx_ready),
              .tx_dst_mac    (arptx_dst_mac),
              .tx_ethertype  (arptx_ethertype),
              .busy          (arptx_busy),
              .done          (arptx_done),
              .stat_tx_req   (st_arp_tx_req),
              .stat_tx_rep   (st_arp_tx_rep)
          );

    ip_mac_map #(.N(3)) u_map (
                   .clk         (clk),
                   .rst_n       (rst_n),
                   .wr_en       (map_wr_en),
                   .wr_ip       (map_wr_ip),
                   .wr_mac      (map_wr_mac),
                   .rd_ip       (map_rd_ip),
                   .rd_hit      (map_hit),
                   .rd_mac      (map_mac),
                   .entry_valid (map_valid),
                   .wr_ptr      (map_wr_ptr)
               );

    // ============================================================
    //  IP 层
    // ============================================================
    IPRX u_iprx (
             .clk             (clk),
             .rst_n           (rst_n),
             .ip_data         (iprx_data),
             .ip_valid        (iprx_valid),
             .ip_sop          (iprx_sop),
             .ip_eop          (iprx_eop),
             .udp_data        (udprx_data),
             .udp_valid       (udprx_valid),
             .udp_sop         (udprx_sop),
             .udp_eop         (udprx_eop),
             .udp_err         (udprx_err),
             .src_ip          (udprx_src_ip),
             .dst_ip          (udprx_dst_ip),
             .payload_len     (udprx_payload_len),
             .protocol        (udprx_proto),
             .stat_rx         (st_iprx),
             .stat_drop_hdr   (st_ip_drop_hdr),
             .stat_drop_proto (st_ip_drop_proto)
         );

    IPTX u_iptx (
             .clk          (clk),
             .rst_n        (rst_n),
             .start        (udp_tx_start),
             .payload_len  (ip_tx_payload_len),
             .src_ip       (LOCAL_IP),
             .dst_ip       (app_peer_ip),
             .protocol     (`IP_PROTO_UDP),
             .dst_mac      (map_mac),
             .up_data      (udptx_data),
             .up_valid     (udptx_valid),
             .up_ready     (udptx_ready),
             .tx_data      (iptx_data),
             .tx_valid     (iptx_valid),
             .tx_sop       (iptx_sop),
             .tx_eop       (iptx_eop),
             .tx_ready     (mactx_ready),
             .tx_dst_mac   (iptx_dst_mac),
             .tx_ethertype (iptx_ethertype),
             .busy         (iptx_busy),
             .done         (iptx_done),
             .stat_tx      (st_iptx)
         );

    // ============================================================
    //  UDP 层
    // ============================================================
    UDPRX u_udprx (
              .clk       (clk),
              .rst_n     (rst_n),
              .udp_data  (udprx_data),
              .udp_valid (udprx_valid),
              .udp_sop   (udprx_sop),
              .udp_eop   (udprx_eop),
              .udp_err   (udprx_err),
              .ip_src    (udprx_src_ip),
              .ip_dst    (udprx_dst_ip),
              .app_data  (app_data),
              .app_valid (app_valid),
              .app_sop   (app_sop),
              .app_eop   (app_eop),
              .app_err   (app_err),
              .src_port  (app_src_port),
              .dst_port  (app_dst_port),
              .data_len  (app_data_len),
              .src_ip    (app_src_ip),
              .dst_ip    (app_dst_ip),
              .stat_rx   (st_udprx),
              .stat_drop (st_udp_drop)
          );

    UDPTX u_udptx (
              .clk         (clk),
              .rst_n       (rst_n),
              .start       (udp_tx_start),
              .src_port    (udp_tx_src_port),
              .dst_port    (udp_tx_dst_port),
              .payload_len (udp_tx_payload_len),
              .buf_data    (txbuf_data),
              .buf_empty   (txbuf_empty),
              .buf_rd_en   (txbuf_rd_en),
              .tx_data     (udptx_data),
              .tx_valid    (udptx_valid),
              .tx_sop      (udptx_sop),
              .tx_eop      (udptx_eop),
              .tx_ready    (udptx_ready),
              .busy        (udp_tx_busy),
              .done        (),
              .stat_tx     (st_udptx)
          );

    // ============================================================
    //  发送仲裁
    // ============================================================
    TxArbiter u_txarb (
                  .clk              (clk),
                  .rst_n            (rst_n),
                  .arp_tx_data      (arptx_data),
                  .arp_tx_valid     (arptx_valid),
                  .arp_tx_sop       (arptx_sop),
                  .arp_tx_eop       (arptx_eop),
                  .arp_tx_dst_mac   (arptx_dst_mac),
                  .arp_tx_ethertype (arptx_ethertype),
                  .ip_tx_data       (iptx_data),
                  .ip_tx_valid      (iptx_valid),
                  .ip_tx_sop        (iptx_sop),
                  .ip_tx_eop        (iptx_eop),
                  .ip_tx_dst_mac    (iptx_dst_mac),
                  .ip_tx_ethertype  (iptx_ethertype),
                  .tx_data          (mactx_data),
                  .tx_valid         (mactx_valid),
                  .tx_sop           (mactx_sop),
                  .tx_eop           (mactx_eop),
                  .tx_dst_mac       (mactx_dst_mac),
                  .tx_ethertype     (mactx_ethertype),
                  .tx_ready         (mactx_ready)
              );

    // ============================================================
    //  应用层
    // ============================================================
    APP #(
            .LOCAL_IP    (LOCAL_IP),
            .LOCAL_PORT  (LOCAL_PORT),
            .BUF_AW      (BUF_AW),
            .BAUD_DIV    ((CLK_FREQ + BAUD/2) / BAUD),
            .GAP_BITS    (GAP_BITS),
            .ARP_TIMEOUT (ARP_TIMEOUT)
        ) u_app (
            .clk                (clk),
            .rst_n              (rst_n),
            .udp_data           (app_data),
            .udp_valid          (app_valid),
            .udp_sop            (app_sop),
            .udp_eop            (app_eop),
            .udp_err            (app_err),
            .udp_src_port       (app_src_port),
            .udp_dst_port       (app_dst_port),
            .udp_data_len       (app_data_len),
            .udp_src_ip         (app_src_ip),
            .udp_dst_ip         (app_dst_ip),
            .frame_end          (rx_eop),      // MAC 帧判决
            .frame_err          (rx_err),
            .map_rd_ip          (map_rd_ip),
            .map_hit            (map_hit),
            .map_mac            (map_mac),
            .uart_tx_data       (uart_tx_data),
            .uart_tx_valid      (uart_tx_valid),
            .uart_tx_ready      (uart_tx_ready),
            .uart_rx_data       (uart_rx_data),
            .uart_rx_valid      (uart_rx_valid),
            .buf_data           (txbuf_data),
            .buf_rd_en          (txbuf_rd_en),
            .buf_empty          (txbuf_empty),
            .udp_tx_start       (udp_tx_start),
            .udp_tx_src_port    (udp_tx_src_port),
            .udp_tx_dst_port    (udp_tx_dst_port),
            .udp_tx_payload_len (udp_tx_payload_len),
            .udp_tx_busy        (udp_tx_busy),
            .ip_tx_busy         (ip_tx_busy),
            .arp_req            (arp_app_req),
            .arp_req_ip         (arp_app_target_ip),
            .peer_valid         (app_peer_valid),
            .peer_ip            (app_peer_ip),
            .peer_port          (app_peer_port),
            .stat_rx_pkt        (st_app_rx_pkt),
            .stat_rx_byte       (st_app_rx_byte),
            .stat_tx_pkt        (st_app_tx_pkt),
            .stat_tx_byte       (st_app_tx_byte),
            .stat_drop          (st_app_drop),
            .stat_arp_wait      (st_app_arp_wait),
            .stat_uart_drop     (st_app_uart_drop)
        );

    // IPTX 载荷长度 = UDP 首部(8) + UDP 数据长度；IPTX 忙信号回给 APP
    assign ip_tx_payload_len = udp_tx_payload_len + 16'd8;
    assign ip_tx_busy        = iptx_busy;

    // ============================================================
    //  串口
    // ============================================================
    UARTTX #(
               .CLK_FREQ (CLK_FREQ),
               .BAUD     (BAUD)
           ) u_uarttx (
               .clk   (clk),
               .rst_n (rst_n),
               .data  (uart_tx_data),
               .valid (uart_tx_valid),
               .ready (uart_tx_ready),
               .txd   (uart_txd)
           );

    UARTRX #(
               .CLK_FREQ (CLK_FREQ),
               .BAUD     (BAUD)
           ) u_uartrx (
               .clk   (clk),
               .rst_n (rst_n),
               .rxd   (uart_rxd),
               .data  (uart_rx_data),
               .valid (uart_rx_valid)
           );

endmodule
