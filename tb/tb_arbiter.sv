`timescale 1ns / 1ps
`include "../rtl/sys_define.svh"

// ============================================================
//  tb_arbiter — Arbiter（MAC + Arbiter 联调）验证平台
// ------------------------------------------------------------
//  结构：RGMII 激励 ─► MAC(MACRX) ─► Arbiter ─┬─► IP 端口监视
//                                             └─► ARP 端口监视
//
//  覆盖用例
//    A1 IP  帧（0x0800）→ 只从 ip 端口出，元数据/payload 正确
//    A2 ARP 帧（0x0806）→ 只从 arp 端口出
//    A3 IPv6 帧（0x86DD）→ 两个端口都无输出，drop_pulse + drop_ethertype 正确
//    A4 IP/ARP 交替 → 路由与统计计数正确
//    A5 坏 CRC 的 IP 帧 → 照常路由，ip_err 置位
//    A6 短 IP 帧 → 照常路由，ip_err 置位（帧长非法）
//    A7 目的 MAC 不匹配 → 两个端口都无输出，且不应产生 drop_pulse
//    A8 环回：MAC TX 发 ARP 帧 → Arbiter 从 arp 端口出
//    A9 统计计数器与监视结果一致
//
//  运行：make sim TB_TOP=tb_arbiter
// ============================================================
module tb_arbiter;

    // ---------------- 常量 ----------------
    localparam [47: 0] LOCAL_MAC = 48'h00_11_22_33_44_55;
    localparam [47: 0] PEER_MAC  = 48'hAA_BB_CC_DD_EE_FF;
    localparam [47: 0] OTHER_MAC = 48'h11_22_33_44_55_66;
    localparam [15: 0] TYPE_IP   = `ETHERTYPE_IP;
    localparam [15: 0] TYPE_ARP  = `ETHERTYPE_ARP;
    localparam [15: 0] TYPE_IPV6 = `ETHERTYPE_IPV6;

    // ---------------- 时钟 / 复位 ----------------
    logic clk   = 1'b0;
    logic rst_n = 1'b0;
    always #4 clk = ~clk;                 // 125MHz

    // ---------------- MAC ↔ RGMII ----------------
    logic [7: 0] rgmii_rxd;
    logic        rgmii_rxdv;
    logic        rgmii_err;
    logic [7: 0] rgmii_txd;
    logic        rgmii_txdv;

    logic [7: 0] phy_rxd;
    logic        phy_rxdv, phy_err;
    logic        loopback = 1'b0;

    assign rgmii_rxd  = loopback ? rgmii_txd  : phy_rxd;
    assign rgmii_rxdv = loopback ? rgmii_txdv : phy_rxdv;
    assign rgmii_err  = loopback ? rgmii_txdv : phy_err;

    // ---------------- MAC ↔ Arbiter ----------------
    logic [7: 0]  rx_data;
    logic         rx_valid, rx_sop, rx_eop, rx_err, rx_crc_err;
    logic [15: 0] rx_len;
    logic [47: 0] rx_dst_mac, rx_src_mac;
    logic [15: 0] rx_ethertype;

    logic [7: 0]  ip_data;
    logic         ip_valid, ip_sop, ip_eop, ip_err;
    logic [15: 0] ip_len;
    logic [47: 0] ip_dst_mac, ip_src_mac;

    logic [7: 0]  arp_data;
    logic         arp_valid, arp_sop, arp_eop, arp_err;
    logic [15: 0] arp_len;
    logic [47: 0] arp_dst_mac, arp_src_mac;

    logic         drop_pulse;
    logic [15: 0] drop_ethertype;
    logic [15: 0] stat_ip_frames, stat_arp_frames, stat_drop_frames, stat_err_frames;

    // ---------------- 用户侧发送接口 ----------------
    logic [7: 0]  tx_data;
    logic         tx_valid, tx_sop, tx_eop, tx_ready, tx_busy, tx_err;
    logic [47: 0] tx_dst_mac;
    logic [15: 0] tx_ethertype;

    // ---------------- DUT ----------------
    MAC #(
            .LOCAL_MAC       (LOCAL_MAC),
            .ERR_ACTIVE_HIGH (1'b0)          // RGMII 规范编码
        ) dut_mac (
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

    Arbiter dut_arb (
            .clk             (clk),
            .rst_n           (rst_n),
            .rx_data         (rx_data),
            .rx_valid        (rx_valid),
            .rx_sop          (rx_sop),
            .rx_eop          (rx_eop),
            .rx_err          (rx_err),
            .rx_len          (rx_len),
            .rx_dst_mac      (rx_dst_mac),
            .rx_src_mac      (rx_src_mac),
            .rx_ethertype    (rx_ethertype),
            .ip_data         (ip_data),
            .ip_valid        (ip_valid),
            .ip_sop          (ip_sop),
            .ip_eop          (ip_eop),
            .ip_err          (ip_err),
            .ip_len          (ip_len),
            .ip_dst_mac      (ip_dst_mac),
            .ip_src_mac      (ip_src_mac),
            .arp_data        (arp_data),
            .arp_valid       (arp_valid),
            .arp_sop         (arp_sop),
            .arp_eop         (arp_eop),
            .arp_err         (arp_err),
            .arp_len         (arp_len),
            .arp_dst_mac     (arp_dst_mac),
            .arp_src_mac     (arp_src_mac),
            .drop_pulse      (drop_pulse),
            .drop_ethertype  (drop_ethertype),
            .stat_ip_frames  (stat_ip_frames),
            .stat_arp_frames (stat_arp_frames),
            .stat_drop_frames(stat_drop_frames),
            .stat_err_frames (stat_err_frames)
        );

    // ============================================================
    //  监视器
    // ============================================================
    int errors = 0;

    // ---- IP 端口 ----
    byte unsigned ip_buf[$];
    byte unsigned ip_pay[$];
    logic [47: 0] ip_cap_src, ip_cap_dst;
    logic [15: 0] ip_cap_len;
    logic         ip_cap_err;
    int           ip_frames = 0, ip_sops = 0, ip_eops = 0;

    always @(posedge clk) begin
        if (ip_valid) begin
            if (ip_sop) begin
                ip_buf.delete();
                ip_cap_src <= ip_src_mac;
                ip_cap_dst <= ip_dst_mac;
                ip_sops    <= ip_sops + 1;
            end
            ip_buf.push_back(ip_data);
        end
        if (ip_eop) begin
            ip_pay     = ip_buf;
            ip_cap_len <= ip_len;
            ip_cap_err <= ip_err;
            ip_eops    <= ip_eops + 1;
            ip_frames  <= ip_frames + 1;
        end
    end

    // ---- ARP 端口 ----
    byte unsigned arp_buf[$];
    byte unsigned arp_pay[$];
    logic [47: 0] arp_cap_src, arp_cap_dst;
    logic [15: 0] arp_cap_len;
    logic         arp_cap_err;
    int           arp_frames = 0, arp_sops = 0, arp_eops = 0;

    always @(posedge clk) begin
        if (arp_valid) begin
            if (arp_sop) begin
                arp_buf.delete();
                arp_cap_src <= arp_src_mac;
                arp_cap_dst <= arp_dst_mac;
                arp_sops    <= arp_sops + 1;
            end
            arp_buf.push_back(arp_data);
        end
        if (arp_eop) begin
            arp_pay     = arp_buf;
            arp_cap_len <= arp_len;
            arp_cap_err <= arp_err;
            arp_eops    <= arp_eops + 1;
            arp_frames  <= arp_frames + 1;
        end
    end

    // ---- 互斥 / 丢弃 ----
    int           both_valid_cnt = 0;   // ip_valid 与 arp_valid 同时有效（不允许）
    int           drop_cnt       = 0;
    logic [15: 0] drop_type_cap;

    always @(posedge clk) begin
        if (ip_valid && arp_valid) both_valid_cnt <= both_valid_cnt + 1;
        if (drop_pulse) begin
            drop_cnt      <= drop_cnt + 1;
            drop_type_cap <= drop_ethertype;
        end
    end

    // ============================================================
    //  共用工具（CRC / 组帧 / RGMII 激励 / 检查）
    // ============================================================
    `include "tb_common.svh"

    // ============================================================
    //  测试用例
    // ============================================================
    byte unsigned pay[$], body[$], frame[$], exp[$];
    int           ip0, arp0, d0, e0;

    // ---- A1 IP 帧 ----
    task automatic test_ip_frame();
        $display("\n---- A1 IP frame -> ip port ----");
        pay.delete();
        for (int i = 0; i < 64; i++) pay.push_back(i[7: 0]);
        make_body(LOCAL_MAC, PEER_MAC, TYPE_IP, pay, 60, body);
        frame.delete();
        foreach (body[i]) frame.push_back(body[i]);
        push_fcs(frame, body);
        body_payload(body, exp);

        ip0 = ip_frames; arp0 = arp_frames;
        rgmii_send(frame, 1'b0, -1);
        wait_cycles(10);

        chk_eq_int("ip frames", ip_frames - ip0, 1);
        chk_eq_int("arp frames (expect 0)", arp_frames - arp0, 0);
        chk_queue("ip payload", ip_pay, exp);
        chk("ip src MAC", ip_cap_src == PEER_MAC);
        chk("ip dst MAC", ip_cap_dst == LOCAL_MAC);
        chk("ip len = 64", ip_cap_len == 16'd64);
        chk("ip err clear", ip_cap_err == 1'b0);
        chk_eq_int("ip sop/eop balanced", ip_sops, ip_eops);
    endtask

    // ---- A2 ARP 帧 ----
    task automatic test_arp_frame();
        $display("\n---- A2 ARP frame -> arp port ----");
        pay.delete();
        for (int i = 0; i < 28; i++) pay.push_back(8'h30 + i[7: 0]);   // ARP 载荷 28 字节
        make_body(`BROADCAST_MAC, PEER_MAC, TYPE_ARP, pay, 60, body);  // 会被 padding 到 60
        frame.delete();
        foreach (body[i]) frame.push_back(body[i]);
        push_fcs(frame, body);
        body_payload(body, exp);

        ip0 = ip_frames; arp0 = arp_frames;
        rgmii_send(frame, 1'b0, -1);
        wait_cycles(10);

        chk_eq_int("arp frames", arp_frames - arp0, 1);
        chk_eq_int("ip frames (expect 0)", ip_frames - ip0, 0);
        chk_queue("arp payload", arp_pay, exp);
        chk("arp src MAC", arp_cap_src == PEER_MAC);
        chk("arp dst MAC is broadcast", arp_cap_dst == `BROADCAST_MAC);
        chk("arp len = 46 (28 + 18 pad)", arp_cap_len == 16'd46);
        chk("arp err clear", arp_cap_err == 1'b0);
        chk_eq_int("arp sop/eop balanced", arp_sops, arp_eops);
    endtask

    // ---- A3 不支持的 EtherType ----
    task automatic test_unsupported();
        $display("\n---- A3 IPv6 frame -> dropped ----");
        pay.delete();
        for (int i = 0; i < 46; i++) pay.push_back(8'h60 + i[7: 0]);
        make_body(LOCAL_MAC, PEER_MAC, TYPE_IPV6, pay, 60, body);
        frame.delete();
        foreach (body[i]) frame.push_back(body[i]);
        push_fcs(frame, body);

        ip0 = ip_frames; arp0 = arp_frames; d0 = drop_cnt;
        rgmii_send(frame, 1'b0, -1);
        wait_cycles(10);

        chk_eq_int("ip frames (expect 0)", ip_frames - ip0, 0);
        chk_eq_int("arp frames (expect 0)", arp_frames - arp0, 0);
        chk_eq_int("drop pulse", drop_cnt - d0, 1);
        chk("drop_ethertype = 0x86DD", drop_type_cap == TYPE_IPV6);
    endtask

    // ---- A4 IP/ARP 交替 ----
    task automatic test_alternating();
        $display("\n---- A4 alternating IP / ARP ----");
        ip0 = ip_frames; arp0 = arp_frames;

        pay.delete();
        for (int i = 0; i < 46; i++) pay.push_back(8'hA0 + i[7: 0]);
        make_body(LOCAL_MAC, PEER_MAC, TYPE_IP, pay, 60, body);
        frame.delete();
        foreach (body[i]) frame.push_back(body[i]);
        push_fcs(frame, body);
        rgmii_send(frame, 1'b0, -1);
        wait_cycles(20);

        pay.delete();
        for (int i = 0; i < 28; i++) pay.push_back(8'hB0 + i[7: 0]);
        make_body(LOCAL_MAC, PEER_MAC, TYPE_ARP, pay, 60, body);
        frame.delete();
        foreach (body[i]) frame.push_back(body[i]);
        push_fcs(frame, body);
        rgmii_send(frame, 1'b0, -1);
        wait_cycles(20);

        pay.delete();
        for (int i = 0; i < 46; i++) pay.push_back(8'hC0 + i[7: 0]);
        make_body(LOCAL_MAC, PEER_MAC, TYPE_IP, pay, 60, body);
        frame.delete();
        foreach (body[i]) frame.push_back(body[i]);
        push_fcs(frame, body);
        rgmii_send(frame, 1'b0, -1);
        wait_cycles(20);

        chk_eq_int("ip frames (expect 2)", ip_frames - ip0, 2);
        chk_eq_int("arp frames (expect 1)", arp_frames - arp0, 1);
        chk_eq_int("no both-valid", both_valid_cnt, 0);
    endtask

    // ---- A5 坏 CRC 的 IP 帧 ----
    task automatic test_bad_crc();
        $display("\n---- A5 IP frame with bad CRC ----");
        pay.delete();
        for (int i = 0; i < 46; i++) pay.push_back(i[7: 0]);
        make_body(LOCAL_MAC, PEER_MAC, TYPE_IP, pay, 60, body);
        frame.delete();
        foreach (body[i]) frame.push_back(body[i]);
        push_fcs(frame, body);
        body_payload(body, exp);

        ip0 = ip_frames;
        rgmii_send(frame, 1'b1, -1);          // 破坏 FCS
        wait_cycles(10);

        chk_eq_int("ip frames", ip_frames - ip0, 1);
        chk_queue("ip payload still forwarded", ip_pay, exp);
        chk("ip err set (bad CRC)", ip_cap_err == 1'b1);
    endtask

    // ---- A6 短 IP 帧 ----
    task automatic test_runt();
        $display("\n---- A6 runt IP frame ----");
        pay.delete();
        for (int i = 0; i < 7; i++) pay.push_back(8'h10 + i[7: 0]);
        make_body(LOCAL_MAC, PEER_MAC, TYPE_IP, pay, 0, body);
        frame.delete();
        foreach (body[i]) frame.push_back(body[i]);
        push_fcs(frame, body);

        ip0 = ip_frames;
        rgmii_send(frame, 1'b0, -1);
        wait_cycles(10);

        chk_eq_int("ip frames", ip_frames - ip0, 1);
        chk("ip err set (bad length)", ip_cap_err == 1'b1);
    endtask

    // ---- A7 目的 MAC 不匹配 ----
    task automatic test_filtered();
        $display("\n---- A7 filtered by dst MAC ----");
        pay.delete();
        for (int i = 0; i < 46; i++) pay.push_back(i[7: 0]);
        make_body(OTHER_MAC, PEER_MAC, TYPE_IP, pay, 60, body);
        frame.delete();
        foreach (body[i]) frame.push_back(body[i]);
        push_fcs(frame, body);

        ip0 = ip_frames; arp0 = arp_frames; d0 = drop_cnt;
        rgmii_send(frame, 1'b0, -1);
        wait_cycles(10);

        chk_eq_int("ip frames (expect 0)", ip_frames - ip0, 0);
        chk_eq_int("arp frames (expect 0)", arp_frames - arp0, 0);
        chk_eq_int("drop pulse (expect 0)", drop_cnt - d0, 0);
    endtask

    // ---- A8 环回 ----
    task automatic test_loopback();
        $display("\n---- A8 loopback: MAC TX -> Arbiter ARP port ----");
        loopback = 1'b1;
        wait_cycles(4);
        pay.delete();
        for (int i = 0; i < 28; i++) pay.push_back(8'h50 + i[7: 0]);
        arp0 = arp_frames; ip0 = ip_frames;
        user_send(LOCAL_MAC, TYPE_ARP, pay);
        wait_cycles(120);

        // MACTX 会把不足 46 字节的 payload 补 0，接收侧看到的是补齐后的内容
        exp.delete();
        foreach (pay[i]) exp.push_back(pay[i]);
        while (exp.size() < `MIN_PAYLOAD) exp.push_back(8'h00);

        chk_eq_int("arp frames", arp_frames - arp0, 1);
        chk_eq_int("ip frames (expect 0)", ip_frames - ip0, 0);
        chk_queue("loopback arp payload (padded)", arp_pay, exp);
        chk("loopback arp err clear", arp_cap_err == 1'b0);
        loopback = 1'b0;
    endtask

    // ---- A9 统计计数 ----
    task automatic test_stats();
        $display("\n---- A9 statistics counters ----");
        chk_eq_int("stat_ip_frames", stat_ip_frames, ip_frames);
        chk_eq_int("stat_arp_frames", stat_arp_frames, arp_frames);
        chk_eq_int("stat_drop_frames", stat_drop_frames, drop_cnt);
        $display("  [INFO] stat_err_frames = %0d", stat_err_frames);
        chk("no both-valid over whole run", both_valid_cnt == 0);
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
        $display(" Arbiter test start");
        $display("========================================");

        test_ip_frame();
        test_arp_frame();
        test_unsupported();
        test_alternating();
        test_bad_crc();
        test_runt();
        test_filtered();
        test_loopback();
        test_stats();

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
