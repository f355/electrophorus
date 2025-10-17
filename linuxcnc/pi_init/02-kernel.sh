#!/usr/bin/env bash

set -e

# pin kernel to 6.12.34, later versions are unstable as of 2025-10-17
apt install -y linux-image-6.12.34+rpt-rpi-v8-rt

# switch to realtime kernel and enable UART
cat >>/boot/firmware/config.txt <<EOF
kernel=kernel8_rt.img
dtparam=uart0=on
EOF

# set kernel command line to isolate the 4th core for linuxcnc's servo thread
sed -i 's/console=serial0,115200/processor.max_cstate=1 isolcpus=3 irqaffinity=0-2 skew_tick=1 kthread_cpus=0-2 rcu_nocb_poll rcu_nocbs=3 nohz=on nohz_full=3/' /boot/firmware/cmdline.txt



