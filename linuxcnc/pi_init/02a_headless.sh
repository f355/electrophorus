#!/usr/bin/env bash

apt install -y tigervnc-standalone-server novnc

# allow passwordless VNC
cat >>/etc/tigervnc/vncserver-config-defaults <<EOF
\$SecurityTypes = "None";
EOF

# our user is on display :1
echo ":1=$USERNAME" >>/etc/tigervnc/vncserver.users

# start VNC server on boot
cat >/lib/systemd/system/tigervncserver@.service <<EOF
[Unit]
Description=Remote desktop service (VNC)
After=network.target

[Service]
Type=forking
ExecStart=/usr/libexec/tigervncsession-start %i
PIDFile=/run/tigervncsession-%i.pid
SELinuxContext=system_u:system_r:vnc_session_t:s0

[Install]
WantedBy=multi-user.target
EOF

systemctl enable tigervncserver@:1.service

# start noVNC web UI on boot
cat >/lib/systemd/system/novnc.service <<EOF
[Unit]
Description=NoVNC WebSocket Proxy
After=network.target
Wants=network.target

[Service]
Type=simple
User=root
Group=root
ExecStart=/usr/bin/websockify --web /usr/share/novnc 80 localhost:5901
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

systemctl enable novnc.service

