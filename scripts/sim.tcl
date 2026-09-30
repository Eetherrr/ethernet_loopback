# ============================================================
#  sim.tcl — 仿真
#  WAVE=1 → 打开波形查看器 (Vivado 以 -mode gui 启动)
#  WAVE=0 → 批处理运行并输出 VCD 到 sim/
#
#  说明：xsim 没有 "-vcd" 命令行开关，导出 VCD 必须用 xsim 的
#        Tcl 命令 open_vcd / log_vcd / close_vcd。批处理模式下
#        这里生成一个自定义 xsim 批处理脚本（xsim.simulate.custom_tcl），
#        它替代 Vivado 自动生成的命令文件，负责 dump + run + 退出。
# ============================================================

set root_dir $::env(VIVADOPRJ_ROOT)
set prj_name $::env(VIVADOPRJ_NAME)
set tb_top   $::env(VIVADOPRJ_TB)
set wave     $::env(VIVADOPRJ_WAVE)
set sim_run  $::env(VIVADOPRJ_SIMRUN)

set sim_dir  $root_dir/sim
# VCD / 脚本按 TB 顶层命名，多个 TB 的波形不会互相覆盖
set vcd_path $sim_dir/waveform_$tb_top.vcd
set dump_tcl $sim_dir/dump_vcd_$tb_top.tcl

open_project $root_dir/prj/$prj_name.xpr

set_property top     $tb_top         [get_filesets sim_1]
set_property top_lib xil_defaultlib  [get_filesets sim_1]
set_property -name {xsim.simulate.runtime} -value $sim_run -objects [get_filesets sim_1]

if {$wave} {
    # ---------- GUI 模式 ----------
    puts "==> 打开波形查看器（Vivado GUI 模式）"
    # 清掉批处理模式留下的自定义脚本，恢复 Vivado 默认行为
    set_property -name {xsim.simulate.custom_tcl} -value {} -objects [get_filesets sim_1]
    launch_simulation
    # 不关闭工程，用户交互查看波形
} else {
    # ---------- 批处理模式：输出 VCD ----------
    puts "==> 运行仿真，VCD 输出到 $vcd_path"
    if {![file isdirectory $sim_dir]} { file mkdir $sim_dir }

    set fh [open $dump_tcl w]
    puts $fh "# 由 scripts/sim.tcl 自动生成 —— xsim 批处理脚本（导出 VCD）"
    puts $fh "open_vcd {$vcd_path}"
    puts $fh "set objs \[get_objects -r /$tb_top/*\]"
    puts $fh "if {\[llength \$objs\] > 0} \{"
    puts $fh "  log_vcd \$objs"
    puts $fh "\} else \{"
    puts $fh "  puts \"警告：没有找到可记录的信号\""
    puts $fh "\}"
    puts $fh "run $sim_run"
    puts $fh "close_vcd"
    puts $fh "quit"
    close $fh

    set_property -name {xsim.simulate.custom_tcl} -value $dump_tcl -objects [get_filesets sim_1]
    launch_simulation
    catch {close_sim}
    puts "==> VCD 文件：$vcd_path"
    close_project
}
