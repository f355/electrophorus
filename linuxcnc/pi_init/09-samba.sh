#!/usr/bin/env bash

set -e

apt install -y samba

cat >>/etc/samba/smb.conf <<EOF

# electrophorus: guest share for linuxcnc
map to guest = Bad User

[linuxcnc]
   path = /home/$USERNAME/linuxcnc
   browseable = yes
   writable = yes
   guest ok = yes
   guest only = yes
   force user = $USERNAME
   force group = $USERNAME
   create mask = 0664
   directory mask = 0775
EOF

systemctl enable smbd
systemctl restart smbd

