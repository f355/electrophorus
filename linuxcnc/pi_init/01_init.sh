#!/usr/bin/env bash

set -e

# various APT repositories
mkdir ~/.gnupg
gpg --no-default-keyring --keyring gnupg-ring:/etc/apt/trusted.gpg.d/linuxcnc.gpg --keyserver hkp://keyserver.ubuntu.com --recv-keys 3cb9fd148f374fef
chmod go+r /etc/apt/trusted.gpg.d/linuxcnc.gpg
curl -sS https://repository.qtpyvcp.com/repo/kcjengr.key | gpg --dearmor >/etc/apt/trusted.gpg.d/kcjengr.gpg
curl --silent --show-error https://gnipsel.com/flexgui/apt-repo/pgp-key.public -o /etc/apt/trusted.gpg.d/flexgui.asc

cat >/etc/apt/sources.list.d/linuxcnc.list <<EOF
deb [arch=arm64] https://www.linuxcnc.org/ bookworm base 2.9-uspace
deb-src [arch=arm64] https://www.linuxcnc.org/ bookworm base 2.9-uspace
deb [arch=arm64] https://repository.qtpyvcp.com/apt develop main
deb [arch=arm64] https://gnipsel.com/flexgui/apt-repo stable main
EOF

apt update
DEBIAN_NONINTERACTIVE=1 apt -y upgrade

rpi-update

apt install -y linux-image-rpi-v8-rt xfce4 xfce4-terminal linuxcnc-uspace linuxcnc-uspace-dev python3-qtpyvcp \
    python3-probe-basic flexgui cmake ninja-build gcc-arm-none-eabi openocd cpufrequtils

# switch to realtime kernel and set kernel command line to isolate cores 2 and 3
# (LinuxCNC will take the first isolated core, CPU2, for the servo; CPU3 reserved for comms)
# also enable UART
cat >>/boot/firmware/config.txt <<EOF
kernel=kernel8_rt.img
dtparam=uart0=on
EOF

sed -i 's/console=serial0,115200/processor.max_cstate=1 isolcpus=2,3 irqaffinity=0-1 skew_tick=1 kthread_cpus=0-1 rcu_nocb_poll rcu_nocbs=2,3 nohz=on nohz_full=2,3/' /boot/firmware/cmdline.txt

# free up UART from being used as a login shell
SERIAL=$(readlink /dev/serial0)
systemctl mask serial-getty@$SERIAL.service

# xfce4 as the desktop environment
echo "xfce4-session" >/home/$USERNAME/.xsession

# allow shutdown etc. from xfce4
cat >/etc/polkit-1/rules.d/02-allow-freedesktop.rules <<EOF
polkit.addRule(function(action, subject) {
    if (action.id.startsWith("org.freedesktop."))
    {
        return polkit.Result.YES;
    }
});
EOF

# linuxcnc 2.9.4 is checking /sys/kernel/realtime, RPi kernel doesn't have that, so we need to use force
echo "export LINUXCNC_FORCE_REALTIME=1" >/etc/profile.d/linuxcnc-force-realtime.sh


# set FT232R latency_timer to 1ms via udev (for low-latency LinuxCNC comms)
cat >/etc/udev/rules.d/99-ftdi-latency.rules <<'EOF'
# FTDI FT232R: vendor 0x0403, product 0x6001
ACTION=="add", SUBSYSTEM=="usb-serial", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTR{latency_timer}="1"
EOF


# Keep USB and power management from perturbing the RT core
# - Disable irqbalance (it can repin IRQs onto the isolated CPU)
systemctl disable --now irqbalance || true

# - Force CPU governor to performance now and persist
#   cpufrequtils provides a simple persistent config on Debian/PI OS
if ! systemctl is-enabled --quiet cpufrequtils 2>/dev/null; then
  echo 'GOVERNOR="performance"' >/etc/default/cpufrequtils
  systemctl enable --now cpufrequtils || true
fi
for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
  [ -w "$g" ] && echo performance >"$g" || true
done

# - Disable USB autosuspend for the FT232R (ttyUSB*)
cat >/etc/udev/rules.d/99-ftdi-power.rules <<'EOF'
ACTION=="add", SUBSYSTEM=="tty", KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTR{power/control}="on"
EOF

# - Pin USB host IRQs (xhci/dwc) to CPU3 (mask 0x8), same core as comms thread, separate from servo on CPU2
cat >/usr/local/sbin/pin-usb-irqs.sh <<'EOF'
#!/bin/sh
mask=8
awk '/xhci|dwc|usb/ {print $1}' /proc/interrupts | tr -d ':' | while read -r irq; do
  [ -w "/proc/irq/$irq/smp_affinity" ] && echo $mask > "/proc/irq/$irq/smp_affinity"
done
EOF
chmod +x /usr/local/sbin/pin-usb-irqs.sh

cat >/etc/systemd/system/pin-usb-irqs.service <<'EOF'
[Unit]
Description=Pin USB IRQs to CPU3 (comms core)
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/usr/local/sbin/pin-usb-irqs.sh

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable pin-usb-irqs.service || true
