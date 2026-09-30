`timescale 1ns / 1ps
`include "../sys_define.svh"

// ============================================================
//  ARPTX — ARP 报文组装
// ------------------------------------------------------------
//  两个请求源（应答优先）：
//      rep_req : ARPRX 判定"有人在问本机 IP"，需要回一个 ARP 应答
//                → oper=2, tha=请求方 MAC, tpa=请求方 IP, 目的 MAC=请求方
//      app_req : APP 查表未命中，需要主动解析某个 IP
//                → oper=1, tha=00..00, tpa=目标 IP, 目的 MAC=广播
//
//  两者都用"一拍脉冲 + 参数"提出请求，内部各自有 pending 锁存，
//  所以即使 ARPTX 正忙也不会丢请求。
//
//  输出直接对接 MACTX 的用户接口：
//      SOP 那一拍给出 tx_dst_mac / tx_ethertype / 第 1 个 payload 字节，
//      之后连续给出 27 个字节，最后一个字节同拍 tx_eop。
//      中间 tx_ready 为低（MAC 在发前导码/帧头）时数据保持不变。
// ============================================================
module ARPTX #(
        parameter [31: 0] LOCAL_IP  = 32'hC0A8_0164,
        parameter [47: 0] LOCAL_MAC = 48'h00_11_22_33_44_55
    )(
        // System Interface
        input  logic         clk,
        input  logic         rst_n,

        // 请求端口 A：APP 主动请求
        input  logic         app_req,
        input  logic [31: 0] app_target_ip,

        // 请求端口 B：ARPRX 应答
        input  logic         rep_req,
        input  logic [47: 0] rep_dst_mac,
        input  logic [31: 0] rep_dst_ip,

        // 下游：MACTX 用户接口
        output logic [7: 0]  tx_data,
        output logic         tx_valid,
        output logic         tx_sop,
        output logic         tx_eop,
        input  logic         tx_ready,
        output logic [47: 0] tx_dst_mac,
        output logic [15: 0] tx_ethertype,

        // 状态
        output logic         busy,
        output logic         done,          // 与最后一个字节被接收同拍之后一拍
        output logic [15: 0] stat_tx_req,
        output logic [15: 0] stat_tx_rep
    );

    // ----------------------------------------
    //  状态
    // ----------------------------------------
    localparam [1:0] S_IDLE = 2'd0,
                     S_RUN  = 2'd1;

    logic [1: 0]  state;
    logic [4: 0]  idx;              // 1..27：当前送给 MAC 的字节序号
    logic         rep_pend, app_pend;
    logic [223: 0] pkt_r;           // 本帧 28 字节载荷
    logic         cur_is_rep;
    logic [47: 0] dst_mac_r;

    // ----------------------------------------
    //  待发请求锁存
    // ----------------------------------------
    wire want_rep = rep_pend;
    wire want_app = !rep_pend && app_pend;

    // SOP 拍：MAC 空闲（tx_ready=1）且有请求
    wire start = (state == S_IDLE) && tx_ready && (want_rep || want_app);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rep_pend <= 1'b0;
            app_pend <= 1'b0;
        end
        else begin
            if (rep_req)          rep_pend <= 1'b1;
            else if (start && want_rep) rep_pend <= 1'b0;

            if (app_req)          app_pend <= 1'b1;
            else if (start && want_app) app_pend <= 1'b0;
        end
    end

    // ----------------------------------------
    //  组合组装本帧载荷（SOP 拍就要用到 byte0，所以必须组合生成）
    // ----------------------------------------
    wire [223: 0] pkt_c;
    assign pkt_c[223: 208] = `ARP_HTYPE_ETH;
    assign pkt_c[207: 192] = `ARP_PTYPE_IP;
    assign pkt_c[191: 184] = `ARP_HLEN;
    assign pkt_c[183: 176] = `ARP_PLEN;
    assign pkt_c[175: 160] = want_rep ? `ARP_OP_REPLY : `ARP_OP_REQUEST;
    assign pkt_c[159: 112] = LOCAL_MAC;
    assign pkt_c[111: 80]  = LOCAL_IP;
    assign pkt_c[79: 32]   = want_rep ? rep_dst_mac : 48'd0;
    assign pkt_c[31: 0]    = want_rep ? rep_dst_ip  : app_target_ip;

    // 取第 idx 个字节：pkt_r[8*idx +: 8] 的等价写法
    wire [4: 0] ridx = 5'd27 - idx;
    wire [7: 0] cur_byte = pkt_r[{ridx, 3'b000} +: 8];

    // ----------------------------------------
    //  输出
    // ----------------------------------------
    assign tx_data      = (state == S_IDLE) ? pkt_c[223: 216] : cur_byte;
    assign tx_valid     = (state == S_RUN) || start;
    assign tx_sop       = (state == S_IDLE) && start;
    assign tx_eop       = (state == S_RUN) && (idx == 5'd27);
    assign tx_dst_mac   = (state == S_IDLE) ? (want_rep ? rep_dst_mac : `BROADCAST_MAC)
                                            : dst_mac_r;
    assign tx_ethertype = `ETHERTYPE_ARP;
    assign busy         = (state != S_IDLE);

    // ----------------------------------------
    //  FSM
    // ----------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            idx        <= 5'd1;
            pkt_r      <= 224'd0;
            cur_is_rep <= 1'b0;
            dst_mac_r  <= 48'd0;
            done       <= 1'b0;
        end
        else begin
            done <= 1'b0;
            case (state)
                S_IDLE: begin
                    if (start) begin
                        state      <= S_RUN;
                        pkt_r      <= pkt_c;
                        cur_is_rep <= want_rep;
                        dst_mac_r  <= want_rep ? rep_dst_mac : `BROADCAST_MAC;
                        idx        <= 5'd1;
                    end
                end
                S_RUN: begin
                    if (tx_ready) begin
                        if (idx == 5'd27) begin
                            state <= S_IDLE;
                            done  <= 1'b1;
                        end
                        else begin
                            idx <= idx + 5'd1;
                        end
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end

    // ----------------------------------------
    //  统计
    // ----------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stat_tx_req <= 16'd0;
            stat_tx_rep <= 16'd0;
        end
        else if (start) begin
            if (want_rep && (stat_tx_rep != 16'hFFFF)) stat_tx_rep <= stat_tx_rep + 16'd1;
            if (want_app && (stat_tx_req != 16'hFFFF)) stat_tx_req <= stat_tx_req + 16'd1;
        end
    end

endmodule
