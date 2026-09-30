`timescale 1ns / 1ps
`include "../sys_define.svh"

// ============================================================
//  APP — 应用层
// ------------------------------------------------------------
//  职责（对应框图里 APP 的位置）
//      RX：UDPRX → 收包缓冲 → 按串口速率喂给 UARTTX
//      TX：UARTRX → 发包缓冲 → 空闲成包 → 查 IP/MAC 表
//           命中   → 直接发 UDP
//           未命中 → 先发 ARP 请求，等 ARP 应答把表填上，再发 UDP
//
//  1) 收包为什么要等 frame_end
//     以太网帧不足 60 字节会补 padding，CRC 判决要到 MAC 帧真正结束
//     才知道。所以 UDP 载荷先写进 rxbuf，等 frame_end 那一拍确认
//     frame_err=0、UDP 层也没报错，才"提交"给串口。
//
//  2) 成包规则
//     串口是字节流，没有天然边界。这里用"空闲间隔"成包：收到 1 个字节
//     就把空闲计时器清零；空闲达到 GAP_BITS 个 bit 时间且缓冲非空，
//     就把缓冲内容作为**一个** UDP 包发出去。缓冲达到 MAX_TX_PAYLOAD
//     也立即发包（防止溢出）。
//
//  3) 对端地址
//     从最近一次成功收到的 UDP 包学到 (源 IP, 源端口) 作为对端，
//     查 IP/MAC 表用的就是 peer_ip。
// ============================================================
module APP #(
        parameter [31: 0] LOCAL_IP      = 32'hC0A8_0164,   // 192.168.1.100
        parameter [15: 0] LOCAL_PORT    = 16'd1234,
        parameter int     BUF_AW        = 11,              // 缓冲 2^11 = 2048 字节
        parameter int     BAUD_DIV      = 1085,            // 每 bit 的时钟数
        parameter int     GAP_BITS      = 40,              // 空闲多少 bit 时间成包
        parameter int     ARP_TIMEOUT   = 125_000_000,     // ARP 等待超时（时钟数）
        parameter int     MAX_TX_PAYLOAD = 1472            // 单包最大 UDP 数据长度
    )(
        // System Interface
        input  logic         clk,
        input  logic         rst_n,

        // ---------------- 来自 UDPRX ----------------
        input  logic [7: 0]  udp_data,
        input  logic         udp_valid,
        input  logic         udp_sop,
        input  logic         udp_eop,
        input  logic         udp_err,
        input  logic [15: 0] udp_src_port,
        input  logic [15: 0] udp_dst_port,
        input  logic [15: 0] udp_data_len,
        input  logic [31: 0] udp_src_ip,
        input  logic [31: 0] udp_dst_ip,

        // ---------------- MAC 帧判决（顶层接 Arbiter 上游 rx_eop / rx_err）----------------
        input  logic         frame_end,
        input  logic         frame_err,

        // ---------------- IP/MAC 表查询 ----------------
        output logic [31: 0] map_rd_ip,
        input  logic         map_hit,
        input  logic [47: 0] map_mac,

        // ---------------- 与 UART 交换 ----------------
        output logic [7: 0]  uart_tx_data,
        output logic         uart_tx_valid,
        input  logic         uart_tx_ready,
        input  logic [7: 0]  uart_rx_data,
        input  logic         uart_rx_valid,

        // ---------------- 发送缓冲 FWFT 读口（给 UDPTX）----------------
        output logic [7: 0]  buf_data,
        input  logic         buf_rd_en,
        output logic         buf_empty,

        // ---------------- UDP 发送请求 ----------------
        output logic         udp_tx_start,
        output logic [15: 0] udp_tx_src_port,
        output logic [15: 0] udp_tx_dst_port,
        output logic [15: 0] udp_tx_payload_len,
        input  logic         udp_tx_busy,
        input  logic         ip_tx_busy,

        // ---------------- ARP 请求 ----------------
        output logic         arp_req,
        output logic [31: 0] arp_req_ip,

        // ---------------- 状态 / 统计 ----------------
        output logic         peer_valid,
        output logic [31: 0] peer_ip,
        output logic [15: 0] peer_port,
        output logic [15: 0] stat_rx_pkt,
        output logic [15: 0] stat_rx_byte,
        output logic [15: 0] stat_tx_pkt,
        output logic [15: 0] stat_tx_byte,
        output logic [15: 0] stat_drop,
        output logic [15: 0] stat_arp_wait,
        output logic [15: 0] stat_uart_drop
    );

    localparam int AW = BUF_AW;
    localparam int GAP_CYCLES = GAP_BITS * BAUD_DIV;

    // ============================================================
    //  声明
    // ============================================================
    // ---- 收包缓冲 ----
    logic [7: 0]    rxbuf [0: (1<<AW)-1];
    logic [AW-1: 0] rx_wr;
    logic [15: 0]   rx_wr_cnt;
    logic [15: 0]   rx_ovf;

    logic           pkt_ok;
    logic [15: 0]   pkt_len;
    logic [31: 0]   pkt_src_ip;
    logic [15: 0]   pkt_src_port;

    logic           rx_pend;
    logic [15: 0]   rx_pend_len;
    logic           rx_own_r;       // 本帧是否被允许写入收包缓冲
    logic           rx_saw_sop;     // 本帧是否来过
    logic           rx_drop_p;      // 本帧因缓冲忙被丢弃

    // ---- 对端地址 ----
    logic [31: 0]   peer_ip_r;
    logic [15: 0]   peer_port_r;
    logic           peer_valid_r;

    // ---- 串口发送 FSM ----
    localparam [1: 0] U_IDLE  = 2'd0,
                     U_FETCH = 2'd1,
                     U_SEND  = 2'd2;

    logic [1: 0]    u_state;
    logic [AW-1: 0] u_raddr;
    logic [7: 0]    u_rdata;
    logic [15: 0]   u_cnt;

    // ---- 发包缓冲 ----
    logic [7: 0]    txbuf [0: (1<<AW)-1];
    logic [AW-1: 0] tx_wr;
    logic [15: 0]   tx_wr_cnt;
    logic [AW-1: 0] tx_rd;
    logic [15: 0]   tx_remain;
    logic [7: 0]    tx_rdata;

    // ---- 发包 FSM ----
    localparam [1: 0] T_IDLE = 2'd0,
                     T_ARP  = 2'd1,
                     T_UDP  = 2'd2;

    logic [1: 0]  t_state;
    logic         udp_start_r;
    logic         udp_started;
    logic [31: 0] arp_timer;

    // ---- 空闲计时 ----
    logic [31: 0] gap_cnt;
    logic         gap_expire;
    logic         send_trigger;
    logic         tx_flush;

    // ---- 统计脉冲 ----
    logic         drop_p, arp_wait_p, rx_commit_p, tx_done_p, uart_drop_p;

    // ---- 统计计数 ----
    logic [15: 0] rx_pkt_c, rx_byte_c, tx_pkt_c, tx_byte_c, drop_c, arp_wait_c, uart_drop_c;

    // ============================================================
    //  接收：UDP 载荷写入收包缓冲
    // ============================================================
    wire rx_full = (rx_wr_cnt >= (16'd1 << AW));

    // SOP 那一拍 rx_wr 还是上一包的残留值，必须强制写地址 0，
    // 否则本包第 1 个字节会写飞、整个载荷前移一位。
    wire [AW-1:0] rx_waddr = udp_sop ? {AW{1'b0}} : rx_wr;

    // 串口比以太网慢几个数量级：上一包还在往串口喂的时候，新帧**不能**
    // 写进同一个缓冲，否则会把正在发送的包冲掉（表现为串口吐出后续帧的
    // 数据）。所以忙的时候整帧丢弃并计数。
    wire rx_can_buf = !rx_pend && (u_state == U_IDLE);
    wire rx_own     = udp_sop ? rx_can_buf : rx_own_r;

    always_ff @(posedge clk) begin
        if (udp_valid && rx_own && (udp_sop || !rx_full)) rxbuf[rx_waddr] <= udp_data;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_wr       <= {AW{1'b0}};
            rx_wr_cnt   <= 16'd0;
            rx_ovf      <= 16'd0;
            pkt_ok      <= 1'b0;
            pkt_len     <= 16'd0;
            pkt_src_ip  <= 32'd0;
            pkt_src_port<= 16'd0;
            rx_pend     <= 1'b0;
            rx_pend_len <= 16'd0;
            rx_own_r    <= 1'b0;
            rx_saw_sop  <= 1'b0;
            peer_ip_r   <= 32'd0;
            peer_port_r <= 16'd0;
            peer_valid_r<= 1'b0;
        end
        else begin
            if (udp_sop) begin
                rx_wr      <= {{(AW-1){1'b0}}, 1'b1};  // 下一字节要写的地址
                rx_wr_cnt  <= 16'd1;                   // 本拍已经写下 1 个字节
                rx_ovf     <= 16'd0;
                pkt_ok     <= 1'b0;
                pkt_len    <= 16'd0;
                rx_own_r   <= rx_can_buf;
                rx_saw_sop <= 1'b1;
            end
            else if (udp_valid && rx_own) begin
                if (rx_full) rx_ovf <= rx_ovf + 16'd1;
                else begin
                    rx_wr     <= rx_wr + {{(AW-1){1'b0}}, 1'b1};
                    rx_wr_cnt <= rx_wr_cnt + 16'd1;
                end
            end

            // UDP 层帧尾：记下长度与合法性（eop 与最后一个数据字节同拍）
            if (udp_eop) begin
                pkt_ok       <= !udp_err && (rx_ovf == 16'd0) && (rx_wr_cnt != 16'd0) && rx_own;
                pkt_len      <= rx_wr_cnt + ((rx_full || !udp_valid) ? 16'd0 : 16'd1);
                pkt_src_ip   <= udp_src_ip;
                pkt_src_port <= udp_src_port;
            end

            // MAC 帧判决：整帧无误 + 上一包已被串口取走 才提交
            if (rx_commit_p) begin
                rx_pend      <= 1'b1;
                rx_pend_len  <= pkt_len;
                peer_ip_r    <= pkt_src_ip;
                peer_port_r  <= pkt_src_port;
                peer_valid_r <= 1'b1;
            end

            // 串口把本包取完
            if ((u_state == U_SEND) && uart_tx_ready && (u_cnt == 16'd1))
                rx_pend <= 1'b0;

            // 帧结束：清本帧标记
            if (frame_end) begin
                rx_saw_sop <= 1'b0;
                rx_own_r   <= 1'b0;
            end
        end
    end

    assign rx_commit_p = frame_end && !frame_err && pkt_ok && !rx_pend;
    // 收到了 UDP 帧但因为串口还在忙而没能缓冲 → 计一次丢弃
    assign rx_drop_p   = frame_end && rx_saw_sop && !rx_own_r;

    // ============================================================
    //  串口发送：从收包缓冲取字节喂 UARTTX
    // ============================================================
    assign uart_tx_valid = (u_state == U_SEND);
    assign uart_tx_data  = u_rdata;

    always_ff @(posedge clk) u_rdata <= rxbuf[u_raddr];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            u_state <= U_IDLE;
            u_raddr <= {AW{1'b0}};
            u_cnt   <= 16'd0;
        end
        else begin
            case (u_state)
                U_IDLE: begin
                    if (rx_pend) begin
                        u_raddr <= {AW{1'b0}};
                        u_cnt   <= rx_pend_len;
                        u_state <= U_FETCH;
                    end
                end
                U_FETCH: u_state <= U_SEND;
                U_SEND: begin
                    if (uart_tx_ready) begin
                        if (u_cnt == 16'd1) u_state <= U_IDLE;
                        else begin
                            u_cnt   <= u_cnt - 16'd1;
                            u_raddr <= u_raddr + {{(AW-1){1'b0}}, 1'b1};
                            u_state <= U_FETCH;
                        end
                    end
                end
                default: u_state <= U_IDLE;
            endcase
        end
    end

    // ============================================================
    //  发送：串口字节 → 发包缓冲 + FWFT 读口
    // ============================================================
    wire tx_full    = (tx_wr_cnt >= (16'd1 << AW));
    wire tx_accept  = uart_rx_valid && !tx_full && (t_state == T_IDLE) && !tx_flush;

    always_ff @(posedge clk) begin
        if (tx_accept) txbuf[tx_wr] <= uart_rx_data;
    end

    // FWFT：buf_rd_en 时预取下一个字节，保证下游每拍都能拿到数据
    always_ff @(posedge clk) begin
        tx_rdata <= txbuf[buf_rd_en ? (tx_rd + {{(AW-1){1'b0}}, 1'b1}) : tx_rd];
    end

    assign buf_data  = tx_rdata;
    assign buf_empty = (tx_remain == 16'd0);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_wr     <= {AW{1'b0}};
            tx_wr_cnt <= 16'd0;
            tx_rd     <= {AW{1'b0}};
            tx_remain <= 16'd0;
        end
        else begin
            if (tx_accept) begin
                tx_wr     <= tx_wr + {{(AW-1){1'b0}}, 1'b1};
                tx_wr_cnt <= tx_wr_cnt + 16'd1;
            end

            if (buf_rd_en && (tx_remain != 16'd0)) begin
                tx_rd     <= tx_rd + {{(AW-1){1'b0}}, 1'b1};
                tx_remain <= tx_remain - 16'd1;
            end

            // 发包启动：复位读指针，装载剩余字节数
            if (send_trigger) begin
                tx_rd     <= {AW{1'b0}};
                tx_remain <= tx_wr_cnt;
            end

            // 发包结束 / 丢弃：清空缓冲
            if (tx_flush) begin
                tx_wr     <= {AW{1'b0}};
                tx_wr_cnt <= 16'd0;
                tx_remain <= 16'd0;
            end
        end
    end

    // ============================================================
    //  空闲成包计时
    // ============================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) gap_cnt <= 32'd0;
        else if (uart_rx_valid || (t_state != T_IDLE)) gap_cnt <= 32'd0;
        else if (gap_cnt < GAP_CYCLES[31: 0])         gap_cnt <= gap_cnt + 32'd1;
    end

    assign gap_expire   = (gap_cnt >= GAP_CYCLES[31: 0]);
    assign send_trigger = (t_state == T_IDLE) && (tx_wr_cnt != 16'd0) &&
                          (gap_expire || (tx_wr_cnt >= MAX_TX_PAYLOAD[15: 0]));

    // ============================================================
    //  发包状态机
    // ============================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            t_state     <= T_IDLE;
            udp_start_r <= 1'b0;
            udp_started <= 1'b0;
            arp_req     <= 1'b0;
            arp_req_ip  <= 32'd0;
            arp_timer   <= 32'd0;
            drop_p      <= 1'b0;
            arp_wait_p  <= 1'b0;
            tx_done_p   <= 1'b0;
            tx_flush    <= 1'b0;
        end
        else begin
            drop_p     <= 1'b0;
            arp_wait_p <= 1'b0;
            tx_done_p  <= 1'b0;
            tx_flush   <= 1'b0;

            case (t_state)
                // ---- 攒包 ----
                T_IDLE: begin
                    udp_start_r <= 1'b0;
                    udp_started <= 1'b0;
                    arp_req     <= 1'b0;

                    if (send_trigger) begin
                        if (!peer_valid_r) begin
                            // 还不知道对端是谁，只能丢
                            tx_flush <= 1'b1;
                            drop_p   <= 1'b1;
                        end
                        else if (map_hit) begin
                            udp_start_r <= 1'b1;
                            t_state     <= T_UDP;
                        end
                        else begin
                            // 查表未命中 → 先发 ARP 请求
                            arp_req    <= 1'b1;
                            arp_req_ip <= peer_ip_r;
                            arp_timer  <= 32'd0;
                            arp_wait_p <= 1'b1;
                            t_state    <= T_ARP;
                        end
                    end
                end

                // ---- 等 ARP 应答把表填上 ----
                T_ARP: begin
                    arp_req <= 1'b0;
                    if (map_hit) begin
                        udp_start_r <= 1'b1;
                        t_state     <= T_UDP;
                    end
                    else if (arp_timer >= ARP_TIMEOUT[31: 0]) begin
                        tx_flush <= 1'b1;
                        drop_p   <= 1'b1;
                        t_state  <= T_IDLE;
                    end
                    else arp_timer <= arp_timer + 32'd1;
                end

                // ---- 发 UDP ----
                T_UDP: begin
                    if (!udp_started) begin
                        udp_start_r <= 1'b1;
                        if (ip_tx_busy) begin          // 下游真的起来了才撤 start
                            udp_start_r <= 1'b0;
                            udp_started <= 1'b1;
                        end
                    end
                    else if (!udp_tx_busy && !ip_tx_busy) begin
                        tx_done_p <= 1'b1;
                        tx_flush  <= 1'b1;
                        t_state   <= T_IDLE;
                    end
                end

                default: t_state <= T_IDLE;
            endcase
        end
    end

    // 发送期间到达的串口字节会被丢掉，计数以便观察
    assign uart_drop_p = uart_rx_valid && !tx_accept;

    // ============================================================
    //  对外接口
    // ============================================================
    assign map_rd_ip = peer_ip_r;

    assign udp_tx_start       = udp_start_r;
    assign udp_tx_src_port    = LOCAL_PORT;
    assign udp_tx_dst_port    = peer_port_r;
    assign udp_tx_payload_len = tx_wr_cnt;

    assign peer_valid = peer_valid_r;
    assign peer_ip    = peer_ip_r;
    assign peer_port  = peer_port_r;

    // ============================================================
    //  统计
    // ============================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_pkt_c    <= 16'd0;
            rx_byte_c   <= 16'd0;
            tx_pkt_c    <= 16'd0;
            tx_byte_c   <= 16'd0;
            drop_c      <= 16'd0;
            arp_wait_c  <= 16'd0;
            uart_drop_c <= 16'd0;
        end
        else begin
            if (rx_commit_p && (rx_pkt_c != 16'hFFFF))    rx_pkt_c  <= rx_pkt_c + 16'd1;
            if (rx_commit_p && (rx_byte_c != 16'hFFFF))   rx_byte_c <= rx_byte_c + pkt_len;
            if (tx_done_p && (tx_pkt_c != 16'hFFFF))      tx_pkt_c  <= tx_pkt_c + 16'd1;
            if (tx_done_p && (tx_byte_c != 16'hFFFF))     tx_byte_c <= tx_byte_c + tx_wr_cnt;
            if (drop_p && (drop_c != 16'hFFFF))           drop_c    <= drop_c + 16'd1;
            if (rx_drop_p && (drop_c != 16'hFFFF))        drop_c    <= drop_c + 16'd1;
            if (arp_wait_p && (arp_wait_c != 16'hFFFF))   arp_wait_c<= arp_wait_c + 16'd1;
            if (uart_drop_p && (uart_drop_c != 16'hFFFF)) uart_drop_c <= uart_drop_c + 16'd1;
        end
    end

    assign stat_rx_pkt   = rx_pkt_c;
    assign stat_rx_byte  = rx_byte_c;
    assign stat_tx_pkt   = tx_pkt_c;
    assign stat_tx_byte  = tx_byte_c;
    assign stat_drop     = drop_c;
    assign stat_arp_wait = arp_wait_c;
    assign stat_uart_drop= uart_drop_c;

endmodule
