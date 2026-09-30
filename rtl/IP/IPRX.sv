`timescale 1ns / 1ps
`include "../sys_define.svh"

// ============================================================
//  IPRX — IPv4 首部解析
// ------------------------------------------------------------
//  输入是 Arbiter 的 ip 端口（EtherType=0x0800 的以太网载荷），
//  里面是完整的 IPv4 报文 + 可能的以太网 padding。
//
//  处理
//      1) 20 字节固定首部（无选项）移入 160bit 移位寄存器，收满即组合译码
//      2) 校验 version=4 / IHL=5 / protocol=UDP / 首部校验和 / total_length
//      3) 只把 [20, total_length-1] 这一段转给 UDPRX —— 用 total_length
//         把以太网 padding 裁掉（这是 IP 层必须做的事）
//
//  只有首部合法才转发；首部非法整包丢弃并计数。
//  若上游在 total_length 之前就 eop（截断），照样给出 eop 但 udp_err=1，
//  保证"出现过 sop 就一定有 eop"的下游约定。
// ============================================================
module IPRX (
        // System Interface
        input  logic         clk,
        input  logic         rst_n,

        // 上游：Arbiter 的 ip 端口
        input  logic [7: 0]  ip_data,
        input  logic         ip_valid,
        input  logic         ip_sop,
        input  logic         ip_eop,

        // 下游：UDPRX
        output logic [7: 0]  udp_data,
        output logic         udp_valid,
        output logic         udp_sop,
        output logic         udp_eop,
        output logic         udp_err,
        output logic [31: 0] src_ip,
        output logic [31: 0] dst_ip,
        output logic [15: 0] payload_len,      // IP 载荷长度 = total_len - 20
        output logic [7: 0]  protocol,

        // 统计
        output logic [15: 0] stat_rx,          // 合法 IPv4 报文
        output logic [15: 0] stat_drop_hdr,    // 首部非法
        output logic [15: 0] stat_drop_proto   // 非 UDP
    );

    // ----------------------------------------
    //  内部寄存器 / 线网声明
    // ----------------------------------------
    logic [159: 0] hdr_sr;
    logic [15: 0]  cnt;            // 已收字节数（寄存）
    logic [15: 0]  cnt_v;          // 当前字节序号（组合，SOP 拍为 0）
    logic [15: 0]  total_len_r;
    logic [31: 0]  src_ip_r, dst_ip_r;
    logic [7: 0]   proto_r;
    logic          hdr_ok_r;

    wire first_v = ip_sop;

    // 当前字节序号：SOP 那一拍强制为 0
    assign cnt_v = ip_sop ? 16'd0 : cnt;

    // 收满 20 字节时，整首部组合可见
    wire [159: 0] hdr_full = {hdr_sr[151: 0], ip_data};

    wire [7: 0]  f_ver_ihl = hdr_full[159: 152];
    wire [15: 0] f_total    = hdr_full[143: 128];
    wire [7: 0]  f_proto    = hdr_full[87: 80];
    wire [15: 0] f_csum     = hdr_full[79: 64];
    wire [31: 0] f_src      = hdr_full[63: 32];
    wire [31: 0] f_dst      = hdr_full[31: 0];

    wire hdr_done = ip_valid && (cnt_v == 16'd19);

    // ----------------------------------------
    //  首部校验和验证：10 个 16bit 字求和折叠后应为 0xFFFF
    // ----------------------------------------
    function automatic [31: 0] csum_fold(input [159: 0] h);
        logic [31: 0] s;
        s = 32'd0;
        for (int i = 0; i < 10; i++)
            s = s + {16'd0, h[16*(9-i) +: 16]};
        s = (s & 32'h0000_FFFF) + (s >> 16);
        s = (s & 32'h0000_FFFF) + (s >> 16);
        return s;
    endfunction

    wire [31: 0] hdr_sum = csum_fold(hdr_full);
    wire csum_ok = (hdr_sum[15: 0] == 16'hFFFF);

    wire total_ok = (f_total >= 16'd20);
    wire proto_ok = (f_proto == `IP_PROTO_UDP);
    wire hdr_good = (f_ver_ihl == `IP_VER_IHL) && csum_ok && total_ok && proto_ok;

    // ----------------------------------------
    //  计数 + 首部移位
    // ----------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt      <= 16'd0;
            hdr_sr   <= 160'd0;
        end
        else if (ip_valid) begin
            if (ip_sop) cnt <= 16'd1;
            else if (cnt != 16'hFFFF) cnt <= cnt + 16'd1;

            if (cnt_v < 16'd20) hdr_sr <= {hdr_sr[151: 0], ip_data};
        end
    end

    // ----------------------------------------
    //  首部字段锁存
    // ----------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            total_len_r <= 16'd0;
            src_ip_r    <= 32'd0;
            dst_ip_r    <= 32'd0;
            proto_r     <= 8'd0;
            hdr_ok_r    <= 1'b0;
        end
        else if (hdr_done) begin
            total_len_r <= f_total;
            src_ip_r    <= f_src;
            dst_ip_r    <= f_dst;
            proto_r     <= f_proto;
            hdr_ok_r    <= hdr_good;
        end
    end

    // ----------------------------------------
    //  载荷转发：只用 total_length 截断，padding 一律丢掉
    // ----------------------------------------
    wire in_payload = ip_valid && hdr_ok_r && (cnt_v >= 16'd20) && (cnt_v < total_len_r);
    wire last_byte  = (cnt_v == (total_len_r - 16'd1));

    assign udp_data    = ip_data;
    assign udp_valid   = in_payload;
    assign udp_sop     = in_payload && (cnt_v == 16'd20);
    assign udp_eop     = in_payload && (last_byte || ip_eop);
    assign udp_err     = udp_eop && ip_eop && !last_byte;      // 被截断

    assign src_ip      = src_ip_r;
    assign dst_ip      = dst_ip_r;
    assign payload_len = (total_len_r >= 16'd20) ? (total_len_r - 16'd20) : 16'd0;
    assign protocol    = proto_r;

    // ----------------------------------------
    //  统计
    // ----------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stat_rx         <= 16'd0;
            stat_drop_hdr   <= 16'd0;
            stat_drop_proto <= 16'd0;
        end
        else if (hdr_done) begin
            if (hdr_good) begin
                if (stat_rx != 16'hFFFF) stat_rx <= stat_rx + 16'd1;
            end
            else if (!proto_ok && (f_ver_ihl == `IP_VER_IHL) && total_ok) begin
                if (stat_drop_proto != 16'hFFFF) stat_drop_proto <= stat_drop_proto + 16'd1;
            end
            else begin
                if (stat_drop_hdr != 16'hFFFF) stat_drop_hdr <= stat_drop_hdr + 16'd1;
            end
        end
    end

endmodule
