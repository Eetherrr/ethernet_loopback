`timescale 1ns / 1ps
`include "../sys_define.svh"

// ============================================================
//  MACTX — 以太网 MAC 发送层
// ------------------------------------------------------------
//  时钟域
//      clk_tx 即 RGMII 的 TX 时钟（125MHz），与 RGMII 模块里 ODDR 同域。
//      rst_tx 需已同步到 clk_tx。
//
//  数据流
//      用户 payload 字节流
//        → 前导码 0x55 × 7 + SFD 0xD5
//        → 目的 MAC（用户给出）+ 源 MAC（LOCAL_MAC）+ EtherType（用户给出）
//        → payload
//        → 不足 60 字节自动补 0（padding）
//        → FCS（CRC32 取反，小端先发）
//        → 最小帧间隔 IFG 12 字节
//
//  用户接口握手（valid/ready）
//      1) 空闲时 tx_ready=1。用户拉高 tx_valid + tx_sop 即被接受，
//         同拍 tx_dst_mac / tx_ethertype / tx_data(第 1 个 payload 字节)
//         被打入 MAC。
//      2) MAC 接着发 22 拍帧头（前导码+SFD+DST+SRC+TYPE），此间 tx_ready=0。
//      3) 进入 payload 阶段后 tx_ready 一直为 1，用户必须**每个时钟**都
//         给出有效字节（tx_valid=1），最后一个字节同拍拉高 tx_eop。
//         中间断流判为 underflow：本帧立即用已发内容的 FCS 收尾，
//         长度不足 64 字节，对端会按 runt 丢弃，同时 tx_err 置位。
//      4) payload 超过 1500 字节判为 oversize：截断并置 tx_err。
//
//      tx_busy 在整帧（含 IFG）期间为 1，供上层仲裁器判断 MAC 是否空闲。
// ============================================================
module MACTX #(
        parameter [47:0] LOCAL_MAC = 48'h00_11_22_33_44_55
    )(
        // System Interface（RGMII TX 时钟域）
        input  logic         clk_tx,
        input  logic         rst_tx,

        // User Interface（← IPTX / ARPTX 仲裁）
        input  logic [7: 0]  tx_data,
        input  logic         tx_valid,
        input  logic         tx_sop,
        input  logic         tx_eop,
        output logic         tx_ready,
        input  logic [47: 0] tx_dst_mac,
        input  logic [15: 0] tx_ethertype,
        output logic         tx_busy,
        output logic         tx_err,       // underflow / oversize，帧结束后保持到下一帧

        // RGMII Interface
        output logic [7: 0]  rgmii_txd,
        output logic         rgmii_txdv
    );

    // ----------------------------------------
    //  状态编码（独热）
    // ----------------------------------------
    localparam [9:0] S_IDLE    = 10'b00_0000_0001,
                     S_PRE     = 10'b00_0000_0010,
                     S_SFD     = 10'b00_0000_0100,
                     S_DST     = 10'b00_0000_1000,
                     S_SRC     = 10'b00_0001_0000,
                     S_TYPE    = 10'b00_0010_0000,
                     S_PAYLOAD = 10'b00_0100_0000,
                     S_PAD     = 10'b00_1000_0000,
                     S_FCS     = 10'b01_0000_0000,
                     S_IFG     = 10'b10_0000_0000;

    // 帧体长度限制（DST..payload，不含 FCS）
    localparam [15:0] MIN_BODY_LAST = `MIN_FRAME_LEN - `FCS_LEN - 1;   // 59
    localparam [15:0] MAX_BODY      = `MAC_HEAD_LEN + `MAX_PAYLOAD;    // 1514

    // ----------------------------------------
    //  内部寄存器 / 线网声明（必须在组合逻辑引用之前）
    // ----------------------------------------
    logic [9: 0]  state;

    logic [2: 0]  cnt_pre;        // 前导码计数 0..6
    logic [2: 0]  cnt_byte;       // DST/SRC/TYPE/FCS 段内字节计数
    logic [15:0]  cnt_body;       // 帧体已发字节数（DST..payload/pad）
    logic [3: 0]  cnt_ifg;        // 帧间隔计数 0..11

    logic [47: 0] dst_mac_r;
    logic [15: 0] type_r;
    logic [7: 0]  pay_byte;       // 当前要发的 payload 字节
    logic         pay_eop;        // pay_byte 是本帧最后一个字节
    logic [31: 0] crc_r;
    logic [31: 0] fcs_r;

    logic [2: 0]  idx_dst;
    logic [2: 0]  idx_typ;
    logic [7: 0]  tx_byte;

    // ----------------------------------------
    //  状态转移条件
    // ----------------------------------------
    wire idle2pre = (state == S_IDLE) && tx_valid && tx_sop;
    wire pre_done = (state == S_PRE) && (cnt_pre == 3'd6);

    wire dst2src  = (state == S_DST) && (cnt_byte == 3'd5);
    wire src2type = (state == S_SRC) && (cnt_byte == 3'd5);
    wire type2payload = (state == S_TYPE) && (cnt_byte == 3'd1);

    wire pay_last      = (state == S_PAYLOAD) && pay_eop;
    wire pay_oversize  = (state == S_PAYLOAD) && !pay_eop && (cnt_body >= MAX_BODY);
    wire pay_underflow = (state == S_PAYLOAD) && !pay_eop && !pay_oversize && !tx_valid;

    wire pad_done = (state == S_PAD) && (cnt_body == MIN_BODY_LAST);
    wire fcs_done = (state == S_FCS) && (cnt_byte == 3'd3);
    wire ifg_done = (state == S_IFG) && (cnt_ifg == `IFG_LEN - 1);

    wire body_cycle = (state == S_DST) || (state == S_SRC) || (state == S_TYPE)
                      || (state == S_PAYLOAD) || (state == S_PAD);

    // ----------------------------------------
    //  发送字节选择
    // ----------------------------------------
    assign idx_dst = 3'd5 - cnt_byte;     // 0->bit47, 5->bit0（高字节先发）
    assign idx_typ = 3'd1 - cnt_byte;     // 0->bit15, 1->bit0

    always_comb begin
        case (state)
            S_PRE:     tx_byte = `PRE_CODE;
            S_SFD:     tx_byte = `SFD_CODE;
            S_DST:     tx_byte = dst_mac_r[{idx_dst, 3'b000} +: 8];
            S_SRC:     tx_byte = LOCAL_MAC[{idx_dst, 3'b000} +: 8];
            S_TYPE:    tx_byte = type_r[{idx_typ, 3'b000} +: 8];
            S_PAYLOAD: tx_byte = pay_byte;
            S_PAD:     tx_byte = 8'h00;
            // FCS 小端先发：fcs[7:0] → fcs[31:24]
            S_FCS:     tx_byte = fcs_r[{cnt_byte, 3'b000} +: 8];
            default:   tx_byte = 8'h00;
        endcase
    end

    assign rgmii_txd  = tx_byte;
    assign rgmii_txdv = (state != S_IDLE) && (state != S_IFG);

    assign tx_ready = (state == S_IDLE)
                      || ((state == S_PAYLOAD) && !pay_eop && !pay_oversize);
    assign tx_busy  = (state != S_IDLE);

    // ----------------------------------------
    //  FSM
    // ----------------------------------------
    always_ff @(posedge clk_tx or negedge rst_tx) begin
        if (!rst_tx) begin
            state <= S_IDLE;
        end
        else begin
            case (state)
                S_IDLE: begin
                    if (idle2pre) state <= S_PRE;
                end
                S_PRE: begin
                    if (pre_done) state <= S_SFD;
                end
                S_SFD: begin
                    state <= S_DST;
                end
                S_DST: begin
                    if (dst2src) state <= S_SRC;
                end
                S_SRC: begin
                    if (src2type) state <= S_TYPE;
                end
                S_TYPE: begin
                    if (type2payload) state <= S_PAYLOAD;
                end
                S_PAYLOAD: begin
                    if (pay_last)           state <= (cnt_body < MIN_BODY_LAST) ? S_PAD : S_FCS;
                    else if (pay_oversize)  state <= S_FCS;
                    else if (pay_underflow) state <= S_FCS;
                end
                S_PAD: begin
                    if (pad_done) state <= S_FCS;
                end
                S_FCS: begin
                    if (fcs_done) state <= S_IFG;
                end
                S_IFG: begin
                    if (ifg_done) state <= S_IDLE;
                end
                default: state <= S_IDLE;
            endcase
        end
    end

    // ----------------------------------------
    //  帧头信息锁存（SOP 同拍采样）
    // ----------------------------------------
    always_ff @(posedge clk_tx or negedge rst_tx) begin
        if (!rst_tx) begin
            dst_mac_r <= 48'd0;
            type_r    <= 16'd0;
        end
        else if (idle2pre) begin
            dst_mac_r <= tx_dst_mac;
            type_r    <= tx_ethertype;
        end
    end

    // ----------------------------------------
    //  计数器
    // ----------------------------------------
    always_ff @(posedge clk_tx or negedge rst_tx) begin
        if (!rst_tx) begin
            cnt_pre <= 3'd0;
        end
        else if (idle2pre) begin
            cnt_pre <= 3'd0;
        end
        else if (state == S_PRE) begin
            cnt_pre <= cnt_pre + 3'd1;
        end
    end

    always_ff @(posedge clk_tx or negedge rst_tx) begin
        if (!rst_tx) begin
            cnt_byte <= 3'd0;
        end
        else begin
            case (state)
                S_DST:   cnt_byte <= (cnt_byte == 3'd5) ? 3'd0 : cnt_byte + 3'd1;
                S_SRC:   cnt_byte <= (cnt_byte == 3'd5) ? 3'd0 : cnt_byte + 3'd1;
                S_TYPE:  cnt_byte <= (cnt_byte == 3'd1) ? 3'd0 : cnt_byte + 3'd1;
                S_FCS:   cnt_byte <= (cnt_byte == 3'd3) ? 3'd0 : cnt_byte + 3'd1;
                default: cnt_byte <= 3'd0;
            endcase
        end
    end

    always_ff @(posedge clk_tx or negedge rst_tx) begin
        if (!rst_tx) begin
            cnt_body <= 16'd0;
        end
        else if (idle2pre) begin
            cnt_body <= 16'd0;
        end
        else if (body_cycle) begin
            cnt_body <= cnt_body + 16'd1;
        end
    end

    always_ff @(posedge clk_tx or negedge rst_tx) begin
        if (!rst_tx) begin
            cnt_ifg <= 4'd0;
        end
        else if (state == S_IFG) begin
            cnt_ifg <= cnt_ifg + 4'd1;
        end
        else begin
            cnt_ifg <= 4'd0;
        end
    end

    // ----------------------------------------
    //  payload 数据通路
    // ----------------------------------------
    always_ff @(posedge clk_tx or negedge rst_tx) begin
        if (!rst_tx) begin
            pay_byte <= 8'h00;
            pay_eop  <= 1'b0;
        end
        else if (idle2pre) begin
            pay_byte <= tx_data;
            pay_eop  <= tx_eop;
        end
        else if ((state == S_PAYLOAD) && !pay_eop && !pay_oversize && tx_valid) begin
            pay_byte <= tx_data;
            pay_eop  <= tx_eop;
        end
    end

    // ----------------------------------------
    //  CRC32 / FCS
    // ----------------------------------------
    function [31: 0] crc32_byte;
        input [31: 0] crc_in;
        input [7: 0]  data;
        reg [31: 0] c;
        integer i;
        begin
            c = crc_in ^ {24'h0, data};
            for (i = 0; i < 8; i = i + 1)
                c = c[0] ? ((c >> 1) ^ `CRC_POLY) : (c >> 1);
            crc32_byte = c;
        end
    endfunction

    wire [31: 0] crc_next = crc32_byte(crc_r, tx_byte);

    always_ff @(posedge clk_tx or negedge rst_tx) begin
        if (!rst_tx) begin
            crc_r <= 32'hFFFF_FFFF;
        end
        else if (idle2pre) begin
            crc_r <= 32'hFFFF_FFFF;
        end
        else if (body_cycle) begin
            crc_r <= crc_next;
        end
    end

    // 进入 FCS 段那一拍，把当前 CRC 取反锁存
    wire to_fcs = pay_last || pay_oversize || pay_underflow || pad_done;

    always_ff @(posedge clk_tx or negedge rst_tx) begin
        if (!rst_tx) begin
            fcs_r <= 32'd0;
        end
        else if (to_fcs) begin
            fcs_r <= ~crc_next;
        end
    end

    // ----------------------------------------
    //  异常记录
    // ----------------------------------------
    always_ff @(posedge clk_tx or negedge rst_tx) begin
        if (!rst_tx) begin
            tx_err <= 1'b0;
        end
        else if (idle2pre) begin
            tx_err <= 1'b0;
        end
        else if (pay_underflow || pay_oversize) begin
            tx_err <= 1'b1;
        end
    end

endmodule
