`timescale 1ns / 1ps
`include "../rtl/sys_define.svh"

// ============================================================
//  tb_mac — MAC 层（MACRX + MACTX）验证平台
// ------------------------------------------------------------
//  覆盖用例
//    T1 正常单播帧接收（元数据 / payload / 无错误）
//    T2 广播帧接收
//    T3 CRC 错误帧 → rx_err + rx_crc_err
//    T4 目的 MAC 不匹配 → 完全不输出
//    T5 短帧（<64 字节）→ rx_err（帧长错误）
//    T6 发送一帧 → 检查线上前导码/帧头/payload/FCS
//    T7 发送短帧 → 自动 padding 到 60 字节
//    T8 连续两帧 → 检查最小帧间隔 IFG ≥ 12
//    T9 payload 断流 → tx_err（underflow）
//    T10 环回：MAC 发送 → 直接接回 MAC 接收，端到端校验
//
//  运行：make sim TB_TOP=tb_mac
// ============================================================
module tb_mac;

    // ---------------- 常量 ----------------
    localparam [47:0] LOCAL_MAC = 48'h00_11_22_33_44_55;
    localparam [47:0] PEER_MAC  = 48'hAA_BB_CC_DD_EE_FF;
    localparam [47:0] OTHER_MAC = 48'h11_22_33_44_55_66;
    localparam [15:0] TYPE_IP   = `ETHERTYPE_IP;
    localparam [15:0] TYPE_ARP  = `ETHERTYPE_ARP;

    // ---------------- 时钟 / 复位 ----------------
    logic clk   = 1'b0;
    logic rst_n = 1'b0;
    always #4 clk = ~clk;                 // 125MHz

    // ---------------- DUT 接口 ----------------
    logic [7: 0] rgmii_rxd;
    logic        rgmii_rxdv;
    logic        rgmii_err;
    logic [7: 0] rgmii_txd;
    logic        rgmii_txdv;

    logic [7: 0]  rx_data;
    logic         rx_valid, rx_sop, rx_eop, rx_err, rx_crc_err;
    logic [15: 0] rx_len;
    logic [47: 0] rx_dst_mac, rx_src_mac;
    logic [15: 0] rx_ethertype;

    logic [7: 0]  tx_data;
    logic         tx_valid, tx_sop, tx_eop, tx_ready, tx_busy, tx_err;
    logic [47: 0] tx_dst_mac;
    logic [15: 0] tx_ethertype;

    // PHY 侧激励（loopback=1 时改由 MAC 自己的 TX 驱动 RX）
    logic [7: 0] phy_rxd;
    logic        phy_rxdv, phy_err;
    logic        loopback = 1'b0;

    assign rgmii_rxd  = loopback ? rgmii_txd  : phy_rxd;
    assign rgmii_rxdv = loopback ? rgmii_txdv : phy_rxdv;
    assign rgmii_err  = loopback ? rgmii_txdv : phy_err;

    MAC #(
            .LOCAL_MAC       (LOCAL_MAC),
            .ERR_ACTIVE_HIGH (1'b0)   // RGMII 规范：err = RX_CTL 下降沿 = RXDV^RXERR
        ) dut (
            .clk_rx        (clk),
            .rst_rx        (rst_n),
            .clk_tx        (clk),
            .rst_tx        (rst_n),
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
            .tx_data       (tx_data),
            .tx_valid      (tx_valid),
            .tx_sop        (tx_sop),
            .tx_eop        (tx_eop),
            .tx_ready      (tx_ready),
            .tx_dst_mac    (tx_dst_mac),
            .tx_ethertype  (tx_ethertype),
            .tx_busy       (tx_busy),
            .tx_err        (tx_err),
            .rgmii_rxd     (rgmii_rxd),
            .rgmii_rxdv    (rgmii_rxdv),
            .rgmii_err     (rgmii_err),
            .rgmii_txd     (rgmii_txd),
            .rgmii_txdv    (rgmii_txdv)
        );

    // ============================================================
    //  监视器
    // ============================================================
    int  errors = 0;

    // ---- RX 侧：抓取 MAC 输出的 payload 流 ----
    byte unsigned rx_buf[$];
    byte unsigned rx_pay[$];          // 最近一帧的 payload
    logic [47: 0] cap_dst, cap_src;
    logic [15: 0] cap_type, cap_len;
    logic         cap_err, cap_crc;
    int           rx_frame_cnt = 0;

    always @(posedge clk) begin
        if (rx_valid) begin
            if (rx_sop) begin
                rx_buf.delete();
                cap_dst  <= rx_dst_mac;
                cap_src  <= rx_src_mac;
                cap_type <= rx_ethertype;
            end
            rx_buf.push_back(rx_data);
        end
        if (rx_eop) begin
            rx_pay       = rx_buf;
            cap_len      <= rx_len;
            cap_err      <= rx_err;
            cap_crc      <= rx_crc_err;
            rx_frame_cnt <= rx_frame_cnt + 1;
        end
    end

    // ---- TX 侧：抓取 RGMII 发出的字节流 ----
    byte unsigned tx_buf[$];
    byte unsigned tx_wire[$];         // 最近一帧的线上字节（含前导码/FCS）
    int           tx_frame_cnt = 0;
    int           tx_gap       = 0;   // 当前帧间空闲拍数
    int           tx_min_gap   = 32'h7FFF_FFFF;
    logic         tx_dv_d      = 1'b0;
    logic         tx_seen      = 1'b0;

    always @(posedge clk) begin
        tx_dv_d <= rgmii_txdv;
        if (rgmii_txdv) begin
            if (!tx_dv_d) begin                       // 帧起始
                if (tx_seen && (tx_gap < tx_min_gap)) tx_min_gap = tx_gap;
                tx_seen = 1'b1;
                tx_gap  = 0;
                tx_buf.delete();
            end
            tx_buf.push_back(rgmii_txd);
        end
        else begin
            tx_gap = tx_gap + 1;
            if (tx_dv_d) begin                        // 帧结束
                tx_wire     = tx_buf;
                tx_frame_cnt = tx_frame_cnt + 1;
            end
        end
    end

    // ============================================================
    //  共用工具（CRC / 组帧 / RGMII 激励 / 检查）
    //  需要本模块里有：clk / phy_rxd / phy_rxdv / phy_err / errors
    // ============================================================
    `include "tb_common.svh"

    // ============================================================
    //  测试用例
    // ============================================================
    byte unsigned pay[$], body[$], frame[$], exp[$];
    int           cnt0;

    // ---- T1 正常单播 ----
    task automatic test_rx_unicast();
        $display("\n---- T1 RX unicast ----");
        pay.delete();
        for (int i = 0; i < 64; i++) pay.push_back(i[7: 0]);
        make_body(LOCAL_MAC, PEER_MAC, TYPE_IP, pay, 60, body);
        frame.delete();
        foreach (body[i]) frame.push_back(body[i]);
        push_fcs(frame, body);
        body_payload(body, exp);

        cnt0 = rx_frame_cnt;
        rgmii_send(frame, 1'b0, -1);
        wait_cycles(10);

        chk_eq_int("rx frame count", rx_frame_cnt - cnt0, 1);
        chk_queue("payload", rx_pay, exp);
        chk("dst MAC", cap_dst == LOCAL_MAC);
        chk("src MAC", cap_src == PEER_MAC);
        chk("EtherType", cap_type == TYPE_IP);
        chk("rx_len", cap_len == 64);
        chk("no rx_err", cap_err == 1'b0);
    endtask

    // ---- T2 广播 ----
    task automatic test_rx_broadcast();
        $display("\n---- T2 RX broadcast ----");
        pay.delete();
        for (int i = 0; i < 46; i++) pay.push_back(8'hA0 + i[7: 0]);
        make_body(`BROADCAST_MAC, PEER_MAC, TYPE_ARP, pay, 60, body);
        frame.delete();
        foreach (body[i]) frame.push_back(body[i]);
        push_fcs(frame, body);
        body_payload(body, exp);

        cnt0 = rx_frame_cnt;
        rgmii_send(frame, 1'b0, -1);
        wait_cycles(10);

        chk_eq_int("rx frame count", rx_frame_cnt - cnt0, 1);
        chk_queue("payload", rx_pay, exp);
        chk("dst MAC is broadcast", cap_dst == `BROADCAST_MAC);
        chk("EtherType", cap_type == TYPE_ARP);
        chk("no rx_err", cap_err == 1'b0);
    endtask

    // ---- T3 CRC 错误 ----
    task automatic test_rx_crc_err();
        $display("\n---- T3 RX bad CRC ----");
        pay.delete();
        for (int i = 0; i < 46; i++) pay.push_back(i[7: 0]);
        make_body(LOCAL_MAC, PEER_MAC, TYPE_IP, pay, 60, body);
        frame.delete();
        foreach (body[i]) frame.push_back(body[i]);
        push_fcs(frame, body);

        cnt0 = rx_frame_cnt;
        rgmii_send(frame, 1'b1, -1);          // 故意破坏 FCS
        wait_cycles(10);

        chk_eq_int("rx frame count", rx_frame_cnt - cnt0, 1);
        chk("rx_err set", cap_err == 1'b1);
        chk("rx_crc_err set", cap_crc == 1'b1);
    endtask

    // ---- T4 目的 MAC 不匹配 ----
    task automatic test_rx_filter();
        $display("\n---- T4 RX dst-MAC filter ----");
        pay.delete();
        for (int i = 0; i < 46; i++) pay.push_back(i[7: 0]);
        make_body(OTHER_MAC, PEER_MAC, TYPE_IP, pay, 60, body);
        frame.delete();
        foreach (body[i]) frame.push_back(body[i]);
        push_fcs(frame, body);

        cnt0 = rx_frame_cnt;
        rgmii_send(frame, 1'b0, -1);
        wait_cycles(10);

        chk_eq_int("rx frame count (expect 0)", rx_frame_cnt - cnt0, 0);
    endtask

    // ---- T5 短帧 ----
    task automatic test_rx_runt();
        $display("\n---- T5 RX runt frame (25B < 64) ----");
        pay.delete();
        for (int i = 0; i < 7; i++) pay.push_back(8'h10 + i[7: 0]);
        make_body(LOCAL_MAC, PEER_MAC, TYPE_IP, pay, 0, body);   // 不 padding
        frame.delete();
        foreach (body[i]) frame.push_back(body[i]);
        push_fcs(frame, body);

        cnt0 = rx_frame_cnt;
        rgmii_send(frame, 1'b0, -1);
        wait_cycles(10);

        chk_eq_int("rx frame count", rx_frame_cnt - cnt0, 1);
        chk_eq_int("rx_len", cap_len, 7);
        chk("rx_err set (bad length)", cap_err == 1'b1);
        chk("CRC itself ok", cap_crc == 1'b0);
    endtask

    // ---- T6 发送一帧 ----
    task automatic test_tx_basic();
        $display("\n---- T6 TX normal frame ----");
        pay.delete();
        for (int i = 0; i < 100; i++) pay.push_back(i[7: 0]);
        cnt0 = tx_frame_cnt;
        user_send(PEER_MAC, TYPE_IP, pay);
        wait_cycles(60);

        chk_eq_int("tx frame count", tx_frame_cnt - cnt0, 1);
        chk("no tx_err", tx_err == 1'b0);

        // 期望的线上字节流
        make_body(PEER_MAC, LOCAL_MAC, TYPE_IP, pay, 60, body);
        exp.delete();
        for (int i = 0; i < `PRE_LEN; i++) exp.push_back(`PRE_CODE);
        exp.push_back(`SFD_CODE);
        foreach (body[i]) exp.push_back(body[i]);
        push_fcs(exp, body);

        chk_eq_int("wire byte count", tx_wire.size(), exp.size());
        chk("wire byte stream match", q_equal(tx_wire, exp));
        if (!q_equal(tx_wire, exp)) begin
            $write("         got : ");
            foreach (tx_wire[i]) $write("%02X ", tx_wire[i]);
            $write("\n         want: ");
            foreach (exp[i]) $write("%02X ", exp[i]);
            $write("\n");
        end
    endtask

    // ---- T7 发送短帧 → 自动 padding ----
    task automatic test_tx_pad();
        $display("\n---- T7 TX short frame auto-padding ----");
        pay.delete();
        pay.push_back(8'hDE);
        pay.push_back(8'hAD);
        pay.push_back(8'hBE);
        pay.push_back(8'hEF);
        cnt0 = tx_frame_cnt;
        user_send(PEER_MAC, TYPE_ARP, pay);
        wait_cycles(60);

        chk_eq_int("tx frame count", tx_frame_cnt - cnt0, 1);
        make_body(PEER_MAC, LOCAL_MAC, TYPE_ARP, pay, 60, body);
        chk_eq_int("body length (after padding)", body.size(), 60);
        exp.delete();
        for (int i = 0; i < `PRE_LEN; i++) exp.push_back(`PRE_CODE);
        exp.push_back(`SFD_CODE);
        foreach (body[i]) exp.push_back(body[i]);
        push_fcs(exp, body);
        chk_eq_int("wire total bytes", tx_wire.size(), 8 + 60 + `FCS_LEN);
        chk("wire byte stream match", q_equal(tx_wire, exp));
    endtask

    // ---- T8 连续两帧 → IFG ----
    task automatic test_tx_ifg();
        $display("\n---- T8 TX inter-frame gap ----");
        pay.delete();
        for (int i = 0; i < 60; i++) pay.push_back(i[7: 0]);
        cnt0 = tx_frame_cnt;
        user_send(PEER_MAC, TYPE_IP, pay);
        wait_cycles(40);
        user_send(PEER_MAC, TYPE_IP, pay);
        wait_cycles(60);

        chk_eq_int("tx frame count", tx_frame_cnt - cnt0, 2);
        chk("min IFG >= IFG", tx_min_gap >= `IFG_LEN);
        $display("  [INFO] min IFG = %0d cycles (require >= %0d)", tx_min_gap, `IFG_LEN);
    endtask

    // ---- T9 payload 断流 ----
    task automatic test_tx_underflow();
        $display("\n---- T9 TX payload underflow ----");
        // SOP + 2 个字节后撤掉 tx_valid
        @(negedge clk);
        tx_data      = 8'h11;
        tx_sop       = 1'b1;
        tx_eop       = 1'b0;
        tx_valid     = 1'b1;
        tx_dst_mac   = PEER_MAC;
        tx_ethertype = TYPE_IP;
        @(posedge clk);
        @(negedge clk);
        tx_sop   = 1'b0;
        tx_valid = 1'b0;
        while (!tx_ready) @(negedge clk);     // 等进入 payload 阶段
        tx_data  = 8'h22;
        tx_valid = 1'b1;
        @(negedge clk);
        tx_valid = 1'b0;                      // 断流，MAC 应判 underflow
        tx_data  = 8'h00;
        wait_cycles(40);

        chk("tx_err set (underflow)", tx_err == 1'b1);
        chk("frame finished (not stuck)", tx_busy == 1'b0);
    endtask

    // ---- T10 环回 ----
    task automatic test_loopback();
        $display("\n---- T10 TX->RX loopback ----");
        loopback = 1'b1;
        wait_cycles(4);
        pay.delete();
        for (int i = 0; i < 128; i++) pay.push_back((i*3) & 8'hFF);
        cnt0 = rx_frame_cnt;
        user_send(LOCAL_MAC, TYPE_IP, pay);
        wait_cycles(120);

        chk_eq_int("loopback rx frame count", rx_frame_cnt - cnt0, 1);
        chk_queue("loopback payload", rx_pay, pay);
        chk("loopback no rx_err", cap_err == 1'b0);
        chk("loopback src MAC = local MAC", cap_src == LOCAL_MAC);
        loopback = 1'b0;
    endtask

    // ---- T11 RGMII RXERR 标志（ERR_ACTIVE_HIGH=0 规范编码路径）----
    //  无错时 IDDR 的 Q2(err) 应与 Q1(rxdv) 相同；出错时 Q2 被拉反。
    //  这里在第 20 个帧字节上注入 RXERR，CRC 本身仍然正确。
    task automatic test_rx_rmii_err();
        $display("\n---- T11 RGMII RXERR flag ----");
        pay.delete();
        for (int i = 0; i < 46; i++) pay.push_back(i[7: 0]);
        make_body(LOCAL_MAC, PEER_MAC, TYPE_IP, pay, 60, body);
        frame.delete();
        foreach (body[i]) frame.push_back(body[i]);
        push_fcs(frame, body);

        cnt0 = rx_frame_cnt;
        rgmii_send(frame, 1'b0, 20);          // 第 20 字节拉 RXERR
        wait_cycles(10);

        chk_eq_int("rx frame count", rx_frame_cnt - cnt0, 1);
        chk("rx_err set (RGMII RXERR)", cap_err == 1'b1);
        chk("rx_crc_err clear (CRC ok)", cap_crc == 1'b0);
    endtask

    // ============================================================
    //  主流程
    // ============================================================
    initial begin
        phy_rxd  = 8'h00;
        phy_rxdv = 1'b0;
        phy_err  = 1'b0;
        tx_data  = 8'h00;
        tx_valid = 1'b0;
        tx_sop   = 1'b0;
        tx_eop   = 1'b0;
        tx_dst_mac   = 48'd0;
        tx_ethertype = 16'd0;

        rst_n = 1'b0;
        wait_cycles(20);
        rst_n = 1'b1;
        wait_cycles(10);

        $display("\n========================================");
        $display(" MAC layer test start");
        $display("========================================");

        test_rx_unicast();
        test_rx_broadcast();
        test_rx_crc_err();
        test_rx_filter();
        test_rx_runt();
        test_tx_basic();
        test_tx_pad();
        test_tx_ifg();
        test_tx_underflow();
        test_loopback();
        test_rx_rmii_err();

        $display("\n========================================");
        if (errors == 0) $display(" ALL TESTS PASSED");
        else             $display(" FAILED: %0d error(s)", errors);
        $display("========================================\n");
        $finish;
    end

    // 超时保护
    initial begin
        #200000;
        $display("\n[FAIL] simulation timeout");
        $finish;
    end

endmodule
