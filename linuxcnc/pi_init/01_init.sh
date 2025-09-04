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
    python3-probe-basic flexgui cmake ninja-build gcc-arm-none-eabi openocd

# switch to realtime kernel and set kernel command line to isolate the 4th core for the servo thread,
# also enable UART
cat >>/boot/firmware/config.txt <<EOF
kernel=kernel8_rt.img
dtparam=uart0=on
EOF

sed -i 's/console=serial0,115200/processor.max_cstate=1 isolcpus=3 irqaffinity=0-2 skew_tick=1 kthread_cpus=0-2 rcu_nocb_poll rcu_nocbs=3 nohz=on nohz_full=3/' /boot/firmware/cmdline.txt

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
