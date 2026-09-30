// ============================================================
//  sys_define.svh — 全局常量定义
// ============================================================
`ifndef SYS_DEFINE_SVH
`define SYS_DEFINE_SVH

// ---------------- MAC 帧结构 ----------------
`define PRE_LEN        7               // 前导码长度（0x55 的个数）
`define SFD_LEN        1               // SFD 长度
`define DST_MAC_LEN    6
`define SRC_MAC_LEN    6
`define TYPE_LEN       2
`define FCS_LEN        4
`define MAC_HEAD_LEN   14              // DST + SRC + TYPE

`define PRE_CODE       8'h55
`define SFD_CODE       8'hD5

// ---------------- CRC32 ----------------
// 采用"反射"实现：poly = 0xEDB88320，初值 0xFFFFFFFF，逐字节右移。
// 接收端把 DST..PAYLOAD..FCS 全部喂进同一个算法，结果为固定残差。
// 注意：残差 0xDEBB20E3 是反射域的值；非反射（MSB first）实现的残差
//       0xC704DD7B 恰好是它的 32 位比特翻转，两者不要混用。
`define CRC_POLY       32'hEDB8_8320
`define CRC_RESIDUE    32'hDEBB_20E3

// ---------------- 帧长限制（均含 FCS） ----------------
`define MIN_FRAME_LEN  64
`define MAX_FRAME_LEN  1518            // 不含 VLAN tag；带 tag 为 1522
`define MAX_PAYLOAD    1500
`define MIN_PAYLOAD    46              // 不足部分由 MAC 自动补 0（padding）

`define IFG_LEN        12              // 最小帧间隔，单位：字节时钟

`define BROADCAST_MAC  48'hFF_FF_FF_FF_FF_FF

// ---------------- EtherType ----------------
`define ETHERTYPE_IP   16'h0800
`define ETHERTYPE_ARP  16'h0806
`define ETHERTYPE_IPV6 16'h86DD
`define ETHERTYPE_VLAN 16'h8100

// ---------------- ARP ----------------
`define ARP_HTYPE_ETH  16'h0001
`define ARP_PTYPE_IP   16'h0800
`define ARP_HLEN       8'd6
`define ARP_PLEN       8'd4
`define ARP_OP_REQUEST 16'h0001
`define ARP_OP_REPLY   16'h0002
`define ARP_PKT_LEN    28              // ARP 载荷固定 28 字节（以太网/IPv4）

// ---------------- IPv4 ----------------
`define IP_VER_IHL     8'h45           // version=4, IHL=5（无选项）
`define IP_HDR_LEN     20
`define IP_PROTO_UDP   8'd17

// ---------------- UDP ----------------
`define UDP_HDR_LEN    8

`endif // SYS_DEFINE_SVH
