#!/usr/bin/env bash

set -e

apt install -y xfce4 xfce4-terminal

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