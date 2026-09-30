`timescale 1ns / 1ps
`include "../sys_define.svh"

// ============================================================
//  ip_mac_map — ARP 的 IP → MAC 缓存映射表
// ------------------------------------------------------------
//  需求：只保留"最近 3 次接收到的 ip/mac"，动态更新即可。
//
//  实现
//      N 项（默认 3）循环表 + 一个写指针 wp：
//        · 写入时若该 IP 已在表中 → 就地更新 MAC（不占用新表项，避免
//          同一个 IP 反复刷新把表冲垮）
//        · 若不在表中 → 写到 wp 指向的表项，wp 循环 +1
//      循环替换等价于"保留最近 N 次不同的 ip/mac"。
//
//  查询是**纯组合**的：rd_ip 打进去，当拍就出 rd_hit / rd_mac，
//  这样 APP 可以在同一个时钟节拍里根据命中与否决定"直接发 UDP"
//  还是"先发 ARP 请求"。
// ============================================================
module ip_mac_map #(
        parameter int N = 3
    )(
        // System Interface
        input  logic             clk,
        input  logic             rst_n,

        // 写入（学习）：ARPRX 收到合法 ARP 报文时更新
        input  logic             wr_en,
        input  logic [31: 0]     wr_ip,
        input  logic [47: 0]     wr_mac,

        // 查询（组合读）
        input  logic [31: 0]     rd_ip,
        output logic             rd_hit,
        output logic [47: 0]     rd_mac,

        // 状态（调试/统计）
        output logic [N-1: 0]    entry_valid,
        output logic [1: 0]      wr_ptr
    );

    localparam int PW = (N <= 1) ? 1 : $clog2(N);

    logic [31: 0] ip_tab  [0: N-1];
    logic [47: 0] mac_tab [0: N-1];
    logic [N-1: 0] valid_r;

    logic [PW-1: 0] wp;
    logic [PW-1: 0] hit_idx;
    logic           hit_any;

    // ----------------------------------------
    //  组合查询
    // ----------------------------------------
    always_comb begin
        rd_hit = 1'b0;
        rd_mac = 48'd0;
        for (int i = 0; i < N; i++) begin
            if (valid_r[i] && (ip_tab[i] == rd_ip)) begin
                rd_hit = 1'b1;
                rd_mac = mac_tab[i];
            end
        end
    end

    // 写入时先看这个 IP 是否已经在表里
    always_comb begin
        hit_any = 1'b0;
        hit_idx = {PW{1'b0}};
        for (int i = 0; i < N; i++) begin
            if (valid_r[i] && (ip_tab[i] == wr_ip)) begin
                hit_any = 1'b1;
                hit_idx = i[PW-1: 0];
            end
        end
    end

    // ----------------------------------------
    //  表项更新
    // ----------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_r <= {N{1'b0}};
            wp      <= {PW{1'b0}};
            for (int i = 0; i < N; i++) begin
                ip_tab[i]  <= 32'd0;
                mac_tab[i] <= 48'd0;
            end
        end
        else if (wr_en) begin
            if (hit_any) begin
                // 已有表项：只刷新 MAC，保持"最近使用"的表项不被挤掉
                mac_tab[hit_idx] <= wr_mac;
            end
            else begin
                ip_tab[wp]    <= wr_ip;
                mac_tab[wp]   <= wr_mac;
                valid_r[wp]   <= 1'b1;
                wp            <= (wp == PW'(N-1)) ? {PW{1'b0}} : (wp + PW'(1));
            end
        end
    end

    assign entry_valid = valid_r;
    assign wr_ptr      = {{(2-PW){1'b0}}, wp};

endmodule
