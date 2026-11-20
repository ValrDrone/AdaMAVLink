#!/bin/sh
# Installs the pinned Alire release for the image's architecture into
# /opt/alire, verified against its SHA-256. Alire then fetches the pinned
# toolchain (alire.toml) for that architecture itself.
set -eu

version=2.1.1
case $(dpkg --print-architecture) in
    amd64)
        arch=x86_64
        sha256=09c66bcd8c35dd4b97b72c3d9b76e44caa6964a2db35aba069f396f00f1f64c7 ;;
    arm64)
        arch=aarch64
        sha256=d76c93ad3dc631826144e10bdabc6b3bf98783805bebfd5e4a0e852dd524d812 ;;
    *)
        echo "install-alr.sh: unsupported architecture $(dpkg --print-architecture)" >&2
        exit 1 ;;
esac

curl -fL --retry 3 -o /tmp/alr.zip \
    "https://github.com/alire-project/alire/releases/download/v$version/alr-$version-bin-$arch-linux.zip"
echo "$sha256  /tmp/alr.zip" | sha256sum -c -
unzip -q /tmp/alr.zip -d /opt/alire
ln -s /opt/alire/bin/alr /usr/local/bin/alr
rm /tmp/alr.zip
