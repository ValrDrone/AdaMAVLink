#!/bin/sh
# Installs Debian trixie packages into an image, pinned to one
# snapshot.debian.org snapshot so that every build, on every architecture,
# gets the same package versions. Usage: apt-install.sh SNAPSHOT PACKAGE...
#
# SNAPSHOT is a snapshot.debian.org timestamp (20261025T000000Z), or "live"
# for deb.debian.org: only for when snapshot.debian.org is down, as the
# image is then not reproducible. The snapshot's Release files have expired,
# so Valid-Until is not checked; their signatures still are, which is also
# why plain HTTP is enough.
set -eu

snapshot=$1
shift

if [ "$snapshot" = live ]; then
    main=http://deb.debian.org/debian
    security=http://deb.debian.org/debian-security
else
    main=http://snapshot.debian.org/archive/debian/$snapshot
    security=http://snapshot.debian.org/archive/debian-security/$snapshot
fi

cat > /etc/apt/sources.list.d/debian.sources <<EOF
Types: deb
URIs: $main
Suites: trixie trixie-updates
Components: main
Signed-By: /usr/share/keyrings/debian-archive-keyring.gpg
Check-Valid-Until: no

Types: deb
URIs: $security
Suites: trixie-security
Components: main
Signed-By: /usr/share/keyrings/debian-archive-keyring.gpg
Check-Valid-Until: no
EOF

# snapshot.debian.org is slow and sometimes briefly unavailable.
apt="apt-get -o Acquire::Retries=5 -o Acquire::http::Timeout=120"
$apt update
DEBIAN_FRONTEND=noninteractive $apt install -y --no-install-recommends "$@"
rm -rf /var/lib/apt/lists/*
