######################################################################################
# This file is part of the Coyote <https://github.com/fpgasystems/Coyote>
# 
# MIT Licence
# Copyright (c) 2026, Systems Group, ETH Zurich
# All rights reserved.
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
######################################################################################

# DDR design through the Versal NoC Memory Controller

# Please note that Versal devices may contain multiple DDR components.
# Here, we assume V80 and the on-board 4GB module. The constraint file
# is stored at hw/bd/constraints/v80/dynamic/impl/v80_shell_ddr_0.xdc.
# The AXI_NOC MC settings might need to be changed accrodingly for other
# Versal devices and different DDR modules.
proc cr_bd_design_ddr { parentCell } {
   upvar #0 cfg cnfg

   set design_name design_ddr

   common::send_msg_id "BD_TCL-003" "INFO" "Currently there is no design <$design_name> in project, so creating one..."

   create_bd_design $design_name

########################################################################################################
# Check IPs
########################################################################################################
   set bCheckIPs 1
   set bCheckIPsPassed 1

   if { $bCheckIPs == 1 } {
      set list_check_ips "\ 
         xilinx.com:ip:axi_noc:1.1\
      "

      set list_ips_missing ""
      common::send_msg_id "BD_TCL-006" "INFO" "Checking if the following IPs exist in the project's IP catalog: $list_check_ips ."

      foreach ip_vlnv $list_check_ips {
         set ip_obj [get_ipdefs -all $ip_vlnv]
         if { $ip_obj eq "" } {
            lappend list_ips_missing $ip_vlnv
         }
      }

      if { $list_ips_missing ne "" } {
         catch {common::send_msg_id "BD_TCL-115" "ERROR" "The following IPs are not found in the IP Catalog:\n  $list_ips_missing\n\nResolution: Please add the repository containing the IP(s) to the project." }
         set bCheckIPsPassed 0
      }
   }

   if { $bCheckIPsPassed != 1 } {
      common::send_msg_id "BD_TCL-1003" "WARNING" "Will not continue with creation of design due to the error(s) above."
      return 3
   }

   if { $parentCell eq "" } {
      set parentCell [get_bd_cells /]
   }

   # Get object for parentCell
   set parentObj [get_bd_cells $parentCell]
   if { $parentObj == "" } {
      catch {common::send_msg_id "BD_TCL-100" "ERROR" "Unable to find parent cell <$parentCell>!"}
      return
   }

   # Make sure parentObj is hier blk
   set parentType [get_property TYPE $parentObj]
   if { $parentType ne "hier" } {
      catch {common::send_msg_id "BD_TCL-101" "ERROR" "Parent <$parentObj> has TYPE = <$parentType>. Expected to be <hier>."}
      return
   }

   # Save current instance; Restore later
   set oldCurInst [current_bd_instance .]

   # Set parent object as current
   current_bd_instance $parentObj

########################################################################################################
########################################################################################################
# DDR MAIN
########################################################################################################
########################################################################################################

########################################################################################################
# Create interface ports
########################################################################################################
   set n_outsanding [expr {$cnfg(n_outs) * $cnfg(pmtu) / $cnfg(stripe_frag_size)}] 
   # Check cr_hbm.tcl for explanation

   # AXI-MM ports
   for {set i 0}  {$i < $cnfg(n_mem_chan)} {incr i} {   
      set cmd "set axi_ddr_in_$i \[ create_bd_intf_port -mode Slave -vlnv xilinx.com:interface:aximm_rtl:1.0 axi_ddr_in_$i ]
               set_property -dict \[ list \
                  CONFIG.ADDR_WIDTH {64} \
                  CONFIG.ARUSER_WIDTH {0} \
                  CONFIG.AWUSER_WIDTH {0} \
                  CONFIG.BUSER_WIDTH {0} \
                  CONFIG.DATA_WIDTH {512} \
                  CONFIG.HAS_BRESP {1} \
                  CONFIG.HAS_BURST {1} \
                  CONFIG.HAS_CACHE {1} \
                  CONFIG.HAS_LOCK {1} \
                  CONFIG.HAS_PROT {1} \
                  CONFIG.HAS_QOS {1} \
                  CONFIG.HAS_REGION {0} \
                  CONFIG.HAS_RRESP {1} \
                  CONFIG.HAS_WSTRB {1} \
                  CONFIG.ID_WIDTH {6} \
                  CONFIG.MAX_BURST_LENGTH {64} \
                  CONFIG.NUM_READ_OUTSTANDING {$n_outsanding} \
                  CONFIG.NUM_READ_THREADS {8} \
                  CONFIG.NUM_WRITE_OUTSTANDING {$n_outsanding} \
                  CONFIG.NUM_WRITE_THREADS {8} \
                  CONFIG.PROTOCOL {AXI4} \
                  CONFIG.READ_WRITE_MODE {READ_WRITE} \
                  CONFIG.RUSER_BITS_PER_BYTE {0} \
                  CONFIG.RUSER_WIDTH {0} \
                  CONFIG.SUPPORTS_NARROW_BURST {0} \
                  CONFIG.WUSER_BITS_PER_BYTE {0} \
                  CONFIG.WUSER_WIDTH {0} \
               ] \$axi_ddr_in_$i"
      eval $cmd
   }

   # DDR Interface
   set CH0_DDR4_0_0 [ create_bd_intf_port -mode Master -vlnv xilinx.com:interface:ddr4_rtl:1.0 CH0_DDR4_0_0 ]

   # DDR Diff Clock
   set sys_clk0_0 [ create_bd_intf_port -mode Slave -vlnv xilinx.com:interface:diff_clock_rtl:1.0 sys_clk0_0 ]
   set_property -dict [ list \
       CONFIG.FREQ_HZ {200000000} \
   ] $sys_clk0_0

########################################################################################################
# Create ports
########################################################################################################
   # Clock associated with the AXI-MM interfaces
   set cmd "set aclk \[ create_bd_port -dir I -type clk aclk ]
         set_property -dict \[ list \
            CONFIG.ASSOCIATED_BUSIF {" 
            for {set i 0}  {$i < $cnfg(n_mem_chan)} {incr i} {
               append cmd "axi_ddr_in_$i"
               if {$i != $cnfg(n_mem_chan) - 1} {
                  append cmd ":"
               }
            }
            append cmd "} \
            CONFIG.FREQ_HZ {$cnfg(aclk_f)000000} \
         ] \$aclk"
   eval $cmd

########################################################################################################
# Create components
########################################################################################################

   set avg_burst [expr {$cnfg(stripe_frag_size) / 512 * 8}] 
   # Check cr_hbm.tcl for explanation

   if {$cnfg(fdev) eq "v80"} {

        # AXI NoC IP
        # Address Offset 0x0500_0000_0000 (DDR_CH1)
        set inst_ddrmc_noc [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_noc inst_ddrmc_noc ]
        set cmd [format "set_property -dict \[list \
            CONFIG.CONTROLLERTYPE {DDR4_SDRAM} \
            CONFIG.MC_CHAN_REGION0 {DDR_CH1} \
            CONFIG.MC_COMPONENT_WIDTH {x16} \
            CONFIG.MC_DATAWIDTH {72} \
            CONFIG.MC_DM_WIDTH {9} \
            CONFIG.MC_DQS_WIDTH {9} \
            CONFIG.MC_DQ_WIDTH {72} \
            CONFIG.MC_INIT_MEM_USING_ECC_SCRUB {true} \
            CONFIG.MC_INPUTCLK0_PERIOD {5000} \
            CONFIG.MC_MEMORY_DEVICETYPE {Components} \
            CONFIG.MC_MEMORY_SPEEDGRADE {DDR4-3200AA(22-22-22)} \
            CONFIG.MC_NO_CHANNELS {Single} \
            CONFIG.MC_RANK {1} \
            CONFIG.MC_ROWADDRESSWIDTH {16} \
            CONFIG.MC_STACKHEIGHT {1} \
            CONFIG.MC_SYSTEM_CLOCK {Differential} \
            CONFIG.NUM_CLKS {1} \
            CONFIG.NUM_MC {1} \
            CONFIG.NUM_MCP {4} \
            CONFIG.NUM_MI {0} \
            CONFIG.NUM_NMI {0} \
            CONFIG.NUM_NSI {0} \
            CONFIG.NUM_SI {$cnfg(n_mem_chan)} \
        ] \$inst_ddrmc_noc "]
        eval $cmd

        # Internal Connectivity (TODO: Fair Sharing?)
        for {set i 0}  {$i < $cnfg(n_mem_chan)} {incr i} {
            set si_idx [expr {$i}]
            set port_idx [expr {$i % 4}]

            set cmd [format "set_property -dict \[ list \
                CONFIG.CONNECTIONS {MC_%d {read_bw {250} write_bw {250} read_avg_burst {$avg_burst} write_avg_burst {$avg_burst} } } \
                CONFIG.NOC_PARAMS {} \
            ] \[get_bd_intf_pins inst_ddrmc_noc/S%02d_AXI]" $port_idx $si_idx]
            eval $cmd
        }

        # Set associated clock
        set cmd "set_property -dict \[ list \
            CONFIG.ASSOCIATED_BUSIF {"
            for {set i 0}  {$i < $cnfg(n_mem_chan)} {incr i} {
                append cmd [format "S%02d_AXI" $i]
                if {$i != $cnfg(n_mem_chan) - 2} {
                    append cmd ":"
                }
            }
            append cmd "} \
        ] \[get_bd_pins inst_ddrmc_noc/aclk0]"
        eval $cmd

   } else {
      puts "ERROR: Requested unsupported Versal device."
      exit 1
   }


########################################################################################################
# Create interface connections
########################################################################################################
   # Connect mem channels to NoC
   for {set i 0}  {$i < $cnfg(n_mem_chan)} {incr i} {       
      set cmd "[format "connect_bd_intf_net \[get_bd_intf_port axi_ddr_in_%d] \[get_bd_intf_pins inst_ddrmc_noc/S%02d_AXI]" $i $i ]"
      eval $cmd
   }

   # Connect DDR interface
   connect_bd_intf_net [get_bd_intf_ports CH0_DDR4_0_0] [get_bd_intf_pins inst_ddrmc_noc/CH0_DDR4_0] 
   connect_bd_intf_net [get_bd_intf_ports sys_clk0_0] [get_bd_intf_pins inst_ddrmc_noc/sys_clk0]

########################################################################################################
# Create port connections
########################################################################################################
   connect_bd_net [get_bd_ports aclk] [get_bd_pins inst_ddrmc_noc/aclk0]

########################################################################################################
# Create address segments
########################################################################################################
   assign_bd_address

   # Restore current instance
   current_bd_instance $oldCurInst

   # Validate and save
   validate_bd_design
   save_bd_design
   close_bd_design $design_name 

   return 0
}