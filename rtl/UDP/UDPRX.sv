`timescale 1ns / 1ps
`include "../sys_define.svh"

// ============================================================
//  UDPRX — UDP 首部解析
// ------------------------------------------------------------
//  输入是 IPRX 送来的 IP 载荷（已经按 total_length 裁掉以太网 padding），
//  结构为 8 字节 UDP 首部 + UDP 数据。
//
//  处理
//      1) 8 字节首部移入 64bit 移位寄存器，收满即组合译码
//      2) 用 UDP length 字段把数据段裁出来：[8, length-1]
//      3) 把数据交给 APP，并带上源/目的端口和源/目的 IP
//
//  IPv4 的 UDP 校验和是可选的（0 表示未计算），这里不做校验，
//  只把它原样透出给 APP 参考。
// ============================================================
module UDPRX (
        // System Interface
        input  logic         clk,
        input  logic         rst_n,

        // 上游：IPRX
        input  logic [7: 0]  udp_data,
        input  logic         udp_valid,
        input  logic         udp_sop,
        input  logic         udp_eop,
        input  logic         udp_err,
        input  logic [31: 0] ip_src,
        input  logic [31: 0] ip_dst,

        // 下游：APP
        output logic [7: 0]  app_data,
        output logic         app_valid,
        output logic         app_sop,
        output logic         app_eop,
        output logic         app_err,
        output logic [15: 0] src_port,
        output logic [15: 0] dst_port,
        output logic [15: 0] data_len,        // UDP 数据长度 = length - 8
        output logic [31: 0] src_ip,
        output logic [31: 0] dst_ip,

        // 统计
        output logic [15: 0] stat_rx,
        output logic [15: 0] stat_drop
    );

    // ----------------------------------------
    //  内部寄存器 / 线网声明
    // ----------------------------------------
    logic [63: 0]  hdr_sr;
    logic [15: 0]  cnt;            // 已收字节数（寄存）
    logic [15: 0]  cnt_v;          // 当前字节序号（组合，SOP 拍为 0）
    logic [15: 0]  src_port_r, dst_port_r, len_r, csum_r;
    logic [31: 0]  src_ip_r, dst_ip_r;

    assign cnt_v = udp_sop ? 16'd0 : cnt;

    wire [63: 0] hdr_full = {hdr_sr[55: 0], udp_data};

    wire [15: 0] f_src_port = hdr_full[63: 48];
    wire [15: 0] f_dst_port = hdr_full[47: 32];
    wire [15: 0] f_len      = hdr_full[31: 16];
    wire [15: 0] f_csum     = hdr_full[15: 0];

    wire hdr_done = udp_valid && (cnt_v == 16'd7);
    wire hdr_good = (f_len >= 16'd8);

    // ----------------------------------------
    //  计数 + 首部移位
    // ----------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt    <= 16'd0;
            hdr_sr <= 64'd0;
        end
        else if (udp_valid) begin
            if (udp_sop) cnt <= 16'd1;
            else if (cnt != 16'hFFFF) cnt <= cnt + 16'd1;

            if (cnt_v < 16'd8) hdr_sr <= {hdr_sr[55: 0], udp_data};
        end
    end

    // ----------------------------------------
    //  首部字段锁存
    // ----------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            src_port_r <= 16'd0;
            dst_port_r <= 16'd0;
            len_r      <= 16'd0;
            csum_r     <= 16'd0;
            src_ip_r   <= 32'd0;
            dst_ip_r   <= 32'd0;
        end
        else if (hdr_done) begin
            src_port_r <= f_src_port;
            dst_port_r <= f_dst_port;
            len_r      <= f_len;
            csum_r     <= f_csum;
            src_ip_r   <= ip_src;
            dst_ip_r   <= ip_dst;
        end
    end

    // ----------------------------------------
    //  数据段转发：用 UDP length 截断
    // ----------------------------------------
    wire in_payload = udp_valid && hdr_good && (cnt_v >= 16'd8) && (cnt_v < len_r);
    wire last_byte  = (cnt_v == (len_r - 16'd1));

    assign app_data  = udp_data;
    assign app_valid = in_payload;
    assign app_sop   = in_payload && (cnt_v == 16'd8);
    assign app_eop   = in_payload && (last_byte || udp_eop);
    assign app_err   = app_eop && ((udp_eop && !last_byte) || udp_err);

    assign src_port  = src_port_r;
    assign dst_port  = dst_port_r;
    assign data_len  = (len_r >= 16'd8) ? (len_r - 16'd8) : 16'd0;
    assign src_ip    = src_ip_r;
    assign dst_ip    = dst_ip_r;

    // ----------------------------------------
    //  统计
    // ----------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stat_rx   <= 16'd0;
            stat_drop <= 16'd0;
        end
        else if (hdr_done) begin
            if (hdr_good) begin
                if (stat_rx != 16'hFFFF) stat_rx <= stat_rx + 16'd1;
            end
            else begin
                if (stat_drop != 16'hFFFF) stat_drop <= stat_drop + 16'd1;
            end
        end
    end

endmodule
