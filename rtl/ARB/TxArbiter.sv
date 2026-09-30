`timescale 1ns / 1ps
`include "../sys_define.svh"

// ============================================================
//  TxArbiter — 发送方向仲裁：ARPTX / IPTX → MACTX
// ------------------------------------------------------------
//  框图里 IPTX 和 ARPTX 都接到 MACTX，同一时刻只能有一个在发，
//  这里做帧级仲裁：
//      · 帧边界切换：只在 MACTX 空闲（tx_ready=1 且没有帧在发）时
//        选新的 owner，一旦开始就锁住直到该帧 eop 被接收
//      · ARP 优先：ARP 报文短(28B)且是 UDP 发送的前置条件，
//        让它插在 IP 帧之间先走
//
//  选中逻辑在空闲拍是**组合**的（不能等到下一拍，否则 SOP 会丢），
//  进入发送后由 state 锁存 owner。
// ============================================================
module TxArbiter (
        // System Interface
        input  logic        clk,
        input  logic        rst_n,

        // 端口 0：ARP（高优先级）
        input  logic [7: 0] arp_tx_data,
        input  logic        arp_tx_valid,
        input  logic        arp_tx_sop,
        input  logic        arp_tx_eop,
        input  logic [47: 0] arp_tx_dst_mac,
        input  logic [15: 0] arp_tx_ethertype,

        // 端口 1：IP
        input  logic [7: 0] ip_tx_data,
        input  logic        ip_tx_valid,
        input  logic        ip_tx_sop,
        input  logic        ip_tx_eop,
        input  logic [47: 0] ip_tx_dst_mac,
        input  logic [15: 0] ip_tx_ethertype,

        // 下游：MACTX 用户接口
        output logic [7: 0] tx_data,
        output logic        tx_valid,
        output logic        tx_sop,
        output logic        tx_eop,
        output logic [47: 0] tx_dst_mac,
        output logic [15: 0] tx_ethertype,
        input  logic        tx_ready
    );

    localparam [1: 0] S_IDLE = 2'd0,
                     S_ARP  = 2'd1,
                     S_IP   = 2'd2;

    logic [1: 0] state;

    wire arp_wants = arp_tx_valid && arp_tx_sop;
    wire ip_wants  = ip_tx_valid  && ip_tx_sop;

    // 空闲拍组合选出 owner，开始后按 state 锁定
    wire sel_arp = (state == S_ARP) || ((state == S_IDLE) && arp_wants);
    wire sel_ip  = (state == S_IP)  || ((state == S_IDLE) && !arp_wants && ip_wants);

    assign tx_data      = sel_arp ? arp_tx_data      : ip_tx_data;
    assign tx_valid     = sel_arp ? arp_tx_valid     : (sel_ip ? ip_tx_valid : 1'b0);
    assign tx_sop       = sel_arp ? arp_tx_sop       : (sel_ip ? ip_tx_sop   : 1'b0);
    assign tx_eop       = sel_arp ? arp_tx_eop       : ip_tx_eop;
    assign tx_dst_mac   = sel_arp ? arp_tx_dst_mac   : ip_tx_dst_mac;
    assign tx_ethertype = sel_arp ? arp_tx_ethertype : ip_tx_ethertype;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
        end
        else begin
            case (state)
                S_IDLE: begin
                    if (arp_wants && tx_ready)      state <= S_ARP;
                    else if (ip_wants && tx_ready)  state <= S_IP;
                end
                S_ARP: begin
                    if (tx_valid && tx_eop && tx_ready) state <= S_IDLE;
                end
                S_IP: begin
                    if (tx_valid && tx_eop && tx_ready) state <= S_IDLE;
                end
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
