# 由 scripts/sim.tcl 自动生成 —— xsim 批处理脚本（导出 VCD）
open_vcd {/home/ether/edev/fpga/prj/ethernet_loopback/sim/waveform_tb_mac.vcd}
set objs [get_objects -r /tb_mac/*]
if {[llength $objs] > 0} {
  log_vcd $objs
} else {
  puts "警告：没有找到可记录的信号"
}
run all
close_vcd
quit
