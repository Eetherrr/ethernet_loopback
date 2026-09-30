`timescale 1ns / 1ps
`include "../rtl/sys_define.svh"

// ============================================================
//  tb_udp_loopback — 系统级 UDP 回环验证
// ------------------------------------------------------------
//  结构
//      PHY 模型(RGMII DDR) ─► System_top ─► 线上帧监视(内部 8bit SDR 侧)
//              ▲                  │
//              └── UART 模型 ◄────┘
//
//  字节下标约定：fbyte(f, idx) 里的 idx 是"以太网帧体"内的下标，
//  即 0..5=DST、6..11=SRC、12..13=TYPE、14 起是载荷。
//  ARP 载荷从 14 开始：oper@20、sha@22、spa@28、tha@32、tpa@38。
//
//  覆盖用例
//      S1 PC 发 UDP → 板子把载荷从串口吐出（并学到对端）
//      S2 串口收字节 → 板子打成 UDP 发回 PC（线上帧逐字节比对）
//      S3 查表未命中 → 先发 ARP 请求，PC 应答后自动补发 UDP
//      S4 PC 发 ARP 请求问本机 IP → 板子回 ARP 应答
//      S5 坏 CRC 的 UDP 帧 → 不吐串口
//      S6 IP/MAC 表只保留最近 3 条（循环替换）
//
//  运行：make sim TB_TOP=tb_udp_loopback
// ============================================================
module tb_udp_loopback;

    // ---------------- 参数 ----------------
    localparam [47: 0] LOCAL_MAC = 48'h00_11_22_33_44_55;
    localparam [31: 0] LOCAL_IP  = 32'hC0A8_0164;      // 192.168.1.100
    localparam [15: 0] LOCAL_PORT= 16'd1234;

    localparam [47: 0] PC_MAC    = 48'hAA_BB_CC_DD_EE_FF;
    localparam [31: 0] PC_IP     = 32'hC0A8_0165;      // 192.168.1.101
    localparam [15: 0] PC_PORT   = 16'd5000;
    localparam [31: 0] PC2_IP    = 32'hC0A8_0166;      // 192.168.1.102（制造查表未命中）

    localparam int CLK_FREQ    = 125_000_000;
    localparam int BAUD        = 12_500_000;           // 仿真用快串口 → DIV=10
    localparam int BIT_CYCLES  = CLK_FREQ / BAUD;      // 10
    localparam int ARP_TIMEOUT = 30_000;

    localparam [15: 0] ARP_HTYPE = `ARP_HTYPE_ETH;
    localparam [15: 0] ARP_PTYPE = `ARP_PTYPE_IP;

    // ---------------- 时钟 / 复位 ----------------
    logic clk   = 1'b0;                                // = phy_rx_clk
    logic rst_n = 1'b0;
    always #4 clk = ~clk;                              // 125MHz

    // ---------------- PHY 侧 ----------------
    logic [3: 0] phy_rxd;
    logic        phy_rx_ctl;
    logic        phy_tx_clk;
    logic [3: 0] phy_txd;
    logic        phy_tx_ctl;

    // ---------------- 串口 ----------------
    logic uart_txd;
    logic uart_rxd = 1'b1;

    // ---------------- DUT ----------------
    System_top #(
                   .LOCAL_MAC   (LOCAL_MAC),
                   .LOCAL_IP    (LOCAL_IP),
                   .LOCAL_PORT  (LOCAL_PORT),
                   .CLK_FREQ    (CLK_FREQ),
                   .BAUD        (BAUD),
                   .BUF_AW      (11),
                   .GAP_BITS    (40),
                   .ARP_TIMEOUT (ARP_TIMEOUT)
               ) dut (
                   .phy_rx_clk (clk),
                   .rst_n      (rst_n),
                   .phy_tx_clk (phy_tx_clk),
                   .phy_txd    (phy_txd),
                   .phy_tx_ctl (phy_tx_ctl),
                   .phy_rxd    (phy_rxd),
                   .phy_rx_ctl (phy_rx_ctl),
                   .uart_txd   (uart_txd),
                   .uart_rxd   (uart_rxd)
               );

    int errors = 0;
    // ============================================================
    //  监视器：线上发送的帧（直接看 RGMII 内部 8bit SDR 侧）
    // ============================================================
    byte unsigned tx_buf[$];
    byte unsigned tx_wire[$];
    int           tx_frame_cnt = 0;
    logic         tx_dv_d = 1'b0;

    always @(posedge clk) begin
        tx_dv_d <= dut.rgmii_txdv;
        if (dut.rgmii_txdv) begin
            if (!tx_dv_d) tx_buf.delete();
            tx_buf.push_back(dut.rgmii_txd);
        end
        else if (tx_dv_d) begin
            tx_wire      = tx_buf;
            tx_frame_cnt = tx_frame_cnt + 1;
        end
    end

    // ---- 串口收到的字节 ----
    byte unsigned uart_rxq[$];
    logic [7: 0]  uart_rb;

    always begin
        @(negedge uart_txd);
        repeat (BIT_CYCLES/2) @(posedge clk);
        if (uart_txd === 1'b0) begin
            for (int i = 0; i < 8; i++) begin
                repeat (BIT_CYCLES) @(posedge clk);
                uart_rb[i] = uart_txd;
            end
            repeat (BIT_CYCLES) @(posedge clk);
            uart_rxq.push_back(uart_rb);
        end
    end

    // ============================================================
    //  共用工具（只用纯函数部分）
    // ============================================================
    `define TB_COMMON_NO_BFM
    `include "tb_common.svh"

    // ============================================================
    //  PHY 模型（RGMII DDR：上升沿低半字节、下降沿高半字节）
    // ============================================================
    // 先等一个下降沿，让最后一个字节的高半字节被采样完，再撤 RXDV。
    // 否则最后一字节的高半字节会被 0 覆盖，表现就是"payload 全对但 CRC 错"。
    task automatic phy_idle(input int n);
        @(negedge clk); #1;
        phy_rx_ctl = 1'b0;
        phy_rxd    = 4'h0;
        repeat (n) @(posedge clk);
    endtask

    // 一个字节：低半字节在下降沿后换、供下一个上升沿采样；
    //           高半字节在上升沿后换、供同一个周期的下降沿采样。
    // #1 是为了避开"在采样沿同一时刻改激励"的竞争。
    task automatic phy_byte(input logic [7: 0] b);
        @(negedge clk); #1;
        phy_rxd    = b[3: 0];
        phy_rx_ctl = 1'b1;
        @(posedge clk); #1;
        phy_rxd    = b[7: 4];
    endtask

    task automatic phy_send_frame(input byte unsigned frame[$], input bit bad_fcs);
        repeat (`PRE_LEN) phy_byte(`PRE_CODE);
        phy_byte(`SFD_CODE);
        foreach (frame[i]) begin
            if (bad_fcs && (i == frame.size()-1)) phy_byte(frame[i] ^ 8'hFF);
            else                                  phy_byte(frame[i]);
        end
        phy_idle(6);
    endtask

    // ============================================================
    //  串口模型
    // ============================================================
    task automatic uart_send_byte(input logic [7: 0] b);
        @(negedge clk);
        uart_rxd = 1'b0;
        repeat (BIT_CYCLES) @(negedge clk);
        for (int i = 0; i < 8; i++) begin
            uart_rxd = b[i];
            repeat (BIT_CYCLES) @(negedge clk);
        end
        uart_rxd = 1'b1;
        repeat (BIT_CYCLES) @(negedge clk);
    endtask

    // ============================================================
    //  组包工具
    // ============================================================
    function automatic [15: 0] ip_csum(input [159: 0] h);
        logic [31: 0] s;
        s = 32'd0;
        for (int i = 0; i < 10; i++) s = s + {16'd0, h[16*(9-i) +: 16]};
        s = (s & 32'h0000_FFFF) + (s >> 16);
        s = (s & 32'h0000_FFFF) + (s >> 16);
        return ~s[15: 0];
    endfunction

    task automatic build_udp_packet(input [31: 0] s_ip, input [31: 0] d_ip,
                                    input [15: 0] s_port, input [15: 0] d_port,
                                    input [15: 0] ip_id,
                                    input byte unsigned data[$],
                                    output byte unsigned pkt[$]);
        logic [159: 0] hdr;
        logic [15: 0]  udp_len, tot_len, cs;
        pkt.delete();
        udp_len = 16'd8 + data.size();
        tot_len = 16'd20 + udp_len;

        hdr[159: 152] = `IP_VER_IHL;
        hdr[151: 144] = 8'h00;
        hdr[143: 128] = tot_len;
        hdr[127: 112] = ip_id;
        hdr[111: 96]  = 16'h0000;
        hdr[95: 88]   = 8'd64;
        hdr[87: 80]   = `IP_PROTO_UDP;
        hdr[79: 64]   = 16'h0000;
        hdr[63: 32]   = s_ip;
        hdr[31: 0]    = d_ip;
        cs            = ip_csum(hdr);
        hdr[79: 64]   = cs;

        for (int i = 0; i < 20; i++) pkt.push_back(hdr[8*(19-i) +: 8]);
        pkt.push_back(s_port[15: 8]);  pkt.push_back(s_port[7: 0]);
        pkt.push_back(d_port[15: 8]);  pkt.push_back(d_port[7: 0]);
        pkt.push_back(udp_len[15: 8]); pkt.push_back(udp_len[7: 0]);
        pkt.push_back(8'h00);          pkt.push_back(8'h00);   // UDP 校验和 = 0
        foreach (data[i]) pkt.push_back(data[i]);
    endtask

    task automatic build_arp_packet(input [15: 0] oper,
                                    input [47: 0] sha, input [31: 0] spa,
                                    input [47: 0] tha, input [31: 0] tpa,
                                    output byte unsigned pkt[$]);
        pkt.delete();
        pkt.push_back(ARP_HTYPE[15: 8]); pkt.push_back(ARP_HTYPE[7: 0]);
        pkt.push_back(ARP_PTYPE[15: 8]); pkt.push_back(ARP_PTYPE[7: 0]);
        pkt.push_back(`ARP_HLEN);             pkt.push_back(`ARP_PLEN);
        pkt.push_back(oper[15: 8]);           pkt.push_back(oper[7: 0]);
        for (int i = 0; i < 6; i++) pkt.push_back(sha[8*(5-i) +: 8]);
        for (int i = 0; i < 4; i++) pkt.push_back(spa[8*(3-i) +: 8]);
        for (int i = 0; i < 6; i++) pkt.push_back(tha[8*(5-i) +: 8]);
        for (int i = 0; i < 4; i++) pkt.push_back(tpa[8*(3-i) +: 8]);
    endtask

    task automatic wrap_frame(input [47: 0] dst, input [47: 0] src, input [15: 0] etype,
                              input byte unsigned payload[$], input int pad_to,
                              output byte unsigned frame[$]);
        byte unsigned body[$];
        make_body(dst, src, etype, payload, pad_to, body);
        frame.delete();
        foreach (body[i]) frame.push_back(body[i]);
        push_fcs(frame, body);
    endtask

    // 期望 DUT 发出的 UDP 帧（IP ID 从实际帧里取，保证只比对内容）
    task automatic expect_tx_udp(input byte unsigned actual[$],
                                 input [31: 0] d_ip, input [15: 0] d_port,
                                 input byte unsigned d[$],
                                 output byte unsigned f[$]);
        byte unsigned p[$], body[$];
        logic [15: 0] id;
        id = {fbyte(actual, 18), fbyte(actual, 19)};
        build_udp_packet(LOCAL_IP, d_ip, LOCAL_PORT, d_port, id, d, p);
        make_body(PC_MAC, LOCAL_MAC, `ETHERTYPE_IP, p, 60, body);
        f.delete();
        for (int i = 0; i < `PRE_LEN; i++) f.push_back(`PRE_CODE);
        f.push_back(`SFD_CODE);
        foreach (body[i]) f.push_back(body[i]);
        push_fcs(f, body);
    endtask

    // ---- 线上帧取字段（idx 为帧体内下标）----
    function automatic byte unsigned fbyte(input byte unsigned f[$], input int idx);
        return f[`PRE_LEN + `SFD_LEN + idx];
    endfunction

    function automatic [47: 0] wire_mac(input byte unsigned f[$], input int base);
        logic [47: 0] m;
        m = 48'd0;
        for (int i = 0; i < 6; i++) m = {m[39: 0], fbyte(f, base+i)};
        return m;
    endfunction

    function automatic [31: 0] wire_ip(input byte unsigned f[$], input int base);
        logic [31: 0] v;
        v = 32'd0;
        for (int i = 0; i < 4; i++) v = {v[23: 0], fbyte(f, base+i)};
        return v;
    endfunction

    function automatic [15: 0] wire_type(input byte unsigned f[$]);
        return {fbyte(f, 12), fbyte(f, 13)};
    endfunction

    function automatic bit map_has(input [31: 0] ip);
        if (dut.u_map.valid_r[0] && (dut.u_map.ip_tab[0] == ip)) return 1'b1;
        if (dut.u_map.valid_r[1] && (dut.u_map.ip_tab[1] == ip)) return 1'b1;
        if (dut.u_map.valid_r[2] && (dut.u_map.ip_tab[2] == ip)) return 1'b1;
        return 1'b0;
    endfunction

    task automatic wait_frames(input int n, input int max_cycles);
        int c  = 0;
        int t0 = tx_frame_cnt;
        while ((c < max_cycles) && (tx_frame_cnt < t0 + n)) begin
            @(posedge clk);
            c++;
        end
    endtask

    // 等 APP 的收包缓冲彻底排空（含 UARTTX 移位寄存器），
    // 否则上一包的尾巴会串到下一个用例里
    task automatic wait_uart_drained(input int max_cycles);
        int c = 0;
        while ((c < max_cycles) &&
               !((dut.u_app.rx_pend == 1'b0) && (dut.u_app.u_state == 2'd0) &&
                 (dut.u_uarttx.state == 2'd0))) begin
            @(posedge clk);
            c++;
        end
        repeat (BIT_CYCLES * 12) @(posedge clk);
    endtask

    task automatic wait_uart_bytes(input int n, input int max_cycles);
        int c  = 0;
        int t0 = uart_rxq.size();
        while ((c < max_cycles) && (uart_rxq.size() < t0 + n)) begin
            @(posedge clk);
            c++;
        end
    endtask

    // ============================================================
    //  测试用例
    // ============================================================
    byte unsigned data[$], pkt[$], frame[$], exp[$], got[$];
    int           t0;

    // ---- S1 PC 发 UDP → 串口吐出 ----
    task automatic test_rx_udp_to_uart();
        $display("\n---- S1 UDP in -> UART out ----");
        data.delete();
        for (int i = 0; i < 16; i++) data.push_back(8'h41 + i[7: 0]);
        build_udp_packet(PC_IP, LOCAL_IP, PC_PORT, LOCAL_PORT, 16'h0001, data, pkt);
        wrap_frame(LOCAL_MAC, PC_MAC, `ETHERTYPE_IP, pkt, 60, frame);

        uart_rxq.delete();
        phy_send_frame(frame, 1'b0);
        wait_uart_bytes(data.size(), 400000);

        chk_eq_int("uart bytes out", uart_rxq.size(), data.size());
        chk_queue("uart payload", uart_rxq, data);
        chk("peer learned", dut.u_app.peer_valid == 1'b1);
        chk("peer ip = PC", dut.u_app.peer_ip == PC_IP);
        chk("peer port = PC_PORT", dut.u_app.peer_port == PC_PORT);
    endtask

    // ---- S2 串口收字节 → 发回 UDP ----
    task automatic test_uart_to_tx_udp();
        $display("\n---- S2 UART in -> UDP out ----");
        // 先给表里放 PC_IP → PC_MAC（用一次 ARP 应答学习）
        build_arp_packet(`ARP_OP_REPLY, PC_MAC, PC_IP, LOCAL_MAC, LOCAL_IP, pkt);
        wrap_frame(LOCAL_MAC, PC_MAC, `ETHERTYPE_ARP, pkt, 60, frame);
        phy_send_frame(frame, 1'b0);
        wait_cycles(300);
        chk("map has PC_IP", map_has(PC_IP));

        data.delete();
        for (int i = 0; i < 12; i++) data.push_back(8'h61 + i[7: 0]);
        foreach (data[i]) uart_send_byte(data[i]);

        t0 = tx_frame_cnt;
        wait_frames(1, 200000);
        chk_eq_int("udp frames sent", tx_frame_cnt - t0, 1);
        chk("ethertype IP", wire_type(tx_wire) == `ETHERTYPE_IP);
        chk("dst mac = PC", wire_mac(tx_wire, 0) == PC_MAC);
        chk("src mac = local", wire_mac(tx_wire, 6) == LOCAL_MAC);
        chk("ip src = local", wire_ip(tx_wire, 26) == LOCAL_IP);
        chk("ip dst = PC", wire_ip(tx_wire, 30) == PC_IP);

        expect_tx_udp(tx_wire, PC_IP, PC_PORT, data, exp);
        chk_eq_int("wire len", tx_wire.size(), exp.size());
        chk_queue("wire frame", tx_wire, exp);
    endtask

    // ---- S3 查表未命中 → ARP 请求 → 应答 → 补发 UDP ----
    task automatic test_arp_miss_then_udp();
        $display("\n---- S3 ARP miss -> request -> reply -> UDP ----");
        // 换一个表里没有的对端：先让它发一个 UDP，让 APP 学到新 peer
        data.delete();
        for (int i = 0; i < 4; i++) data.push_back(8'h11 + i[7: 0]);
        build_udp_packet(PC2_IP, LOCAL_IP, PC_PORT, LOCAL_PORT, 16'h0002, data, pkt);
        wrap_frame(LOCAL_MAC, PC_MAC, `ETHERTYPE_IP, pkt, 60, frame);
        phy_send_frame(frame, 1'b0);
        wait_cycles(400);

        chk("peer is PC2", dut.u_app.peer_ip == PC2_IP);
        chk("map miss for PC2", !map_has(PC2_IP));

        // 串口给数据 → 应先发 ARP 请求
        data.delete();
        for (int i = 0; i < 6; i++) data.push_back(8'h71 + i[7: 0]);
        t0 = tx_frame_cnt;
        foreach (data[i]) uart_send_byte(data[i]);

        wait_frames(1, 200000);
        chk_eq_int("first frame after uart", tx_frame_cnt - t0, 1);
        chk("first is ARP", wire_type(tx_wire) == `ETHERTYPE_ARP);
        chk("oper = request",
            {fbyte(tx_wire, 20), fbyte(tx_wire, 21)} == `ARP_OP_REQUEST);
        chk("arp dst = broadcast", wire_mac(tx_wire, 0) == `BROADCAST_MAC);
        chk("spa = local ip", wire_ip(tx_wire, 28) == LOCAL_IP);
        chk("tpa = PC2 ip", wire_ip(tx_wire, 38) == PC2_IP);
        chk("tha = 0", wire_mac(tx_wire, 32) == 48'd0);
        chk("arp wait counted", dut.u_app.stat_arp_wait == 16'd1);

        // PC2 回 ARP 应答 → 表被填上 → 板子自动补发 UDP
        build_arp_packet(`ARP_OP_REPLY, PC_MAC, PC2_IP, LOCAL_MAC, LOCAL_IP, pkt);
        wrap_frame(LOCAL_MAC, PC_MAC, `ETHERTYPE_ARP, pkt, 60, frame);
        t0 = tx_frame_cnt;
        phy_send_frame(frame, 1'b0);

        wait_frames(1, 200000);
        chk_eq_int("udp sent after arp reply", tx_frame_cnt - t0, 1);
        chk("second is IP", wire_type(tx_wire) == `ETHERTYPE_IP);
        expect_tx_udp(tx_wire, PC2_IP, PC_PORT, data, exp);
        chk_queue("deferred udp frame", tx_wire, exp);
    endtask

    // ---- S4 收到 ARP 请求 → 回 ARP 应答 ----
    task automatic test_arp_reply();
        $display("\n---- S4 ARP request -> ARP reply ----");
        build_arp_packet(`ARP_OP_REQUEST, PC_MAC, PC_IP, 48'd0, LOCAL_IP, pkt);
        wrap_frame(`BROADCAST_MAC, PC_MAC, `ETHERTYPE_ARP, pkt, 60, frame);
        t0 = tx_frame_cnt;
        phy_send_frame(frame, 1'b0);
        wait_frames(1, 200000);

        chk_eq_int("arp reply frames", tx_frame_cnt - t0, 1);
        chk("ethertype ARP", wire_type(tx_wire) == `ETHERTYPE_ARP);
        chk("oper = reply", {fbyte(tx_wire, 20), fbyte(tx_wire, 21)} == `ARP_OP_REPLY);
        chk("dst mac = PC", wire_mac(tx_wire, 0) == PC_MAC);
        chk("sha = local mac", wire_mac(tx_wire, 22) == LOCAL_MAC);
        chk("spa = local ip", wire_ip(tx_wire, 28) == LOCAL_IP);
        chk("tha = pc mac", wire_mac(tx_wire, 32) == PC_MAC);
        chk("tpa = pc ip", wire_ip(tx_wire, 38) == PC_IP);
    endtask

    // ---- S5 坏 CRC → 不吐串口 ----
    task automatic test_bad_crc_dropped();
        $display("\n---- S5 bad CRC -> no UART output ----");
        data.delete();
        for (int i = 0; i < 8; i++) data.push_back(8'h99);
        build_udp_packet(PC_IP, LOCAL_IP, PC_PORT, LOCAL_PORT, 16'h0003, data, pkt);
        wrap_frame(LOCAL_MAC, PC_MAC, `ETHERTYPE_IP, pkt, 60, frame);

        wait_uart_drained(200000);             // 先让前面的包吐干净
        uart_rxq.delete();
        phy_send_frame(frame, 1'b1);           // 破坏 FCS
        wait_cycles(30000);
        chk_eq_int("uart bytes (expect 0)", uart_rxq.size(), 0);
    endtask

    // ---- S6 IP/MAC 表只留最近 3 条 ----
    task automatic test_map_3_entries();
        $display("\n---- S6 IP/MAC map keeps last 3 ----");
        for (int k = 0; k < 4; k++) begin
            build_arp_packet(`ARP_OP_REPLY, PC_MAC, 32'hC0A8_0200 + k,
                             LOCAL_MAC, LOCAL_IP, pkt);
            wrap_frame(LOCAL_MAC, PC_MAC, `ETHERTYPE_ARP, pkt, 60, frame);
            phy_send_frame(frame, 1'b0);
            wait_cycles(300);
        end

        chk("3 entries valid", dut.u_map.valid_r == 3'b111);
        chk("oldest evicted", !map_has(32'hC0A8_0200));
        chk("kept 0x201", map_has(32'hC0A8_0201));
        chk("kept 0x202", map_has(32'hC0A8_0202));
        chk("kept 0x203", map_has(32'hC0A8_0203));
    endtask

    // ============================================================
    //  主流程
    // ============================================================
    initial begin
        phy_rxd    = 4'h0;
        phy_rx_ctl = 1'b0;
        uart_rxd   = 1'b1;
        rst_n      = 1'b0;
        wait_cycles(20);
        rst_n      = 1'b1;
        wait_cycles(10);

        $display("\n========================================");
        $display(" UDP loopback system test start");
        $display("========================================");

        test_rx_udp_to_uart();
        test_uart_to_tx_udp();
        test_arp_miss_then_udp();
        test_arp_reply();
        test_bad_crc_dropped();
        test_map_3_entries();

        $display("\n========================================");
        if (errors == 0) $display(" ALL TESTS PASSED");
        else             $display(" FAILED: %0d error(s)", errors);
        $display("========================================\n");
        $finish;
    end

    initial begin
        #20000000;
        $display("\n[FAIL] simulation timeout");
        $finish;
    end

endmodule
