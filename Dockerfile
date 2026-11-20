# Development image for tests and proof (make setup). Multi-architecture
# (linux/amd64, linux/arm64). Debian packages come from one
# snapshot.debian.org snapshot (scripts/apt-install.sh); Alire is pinned by
# version and hash (scripts/install-alr.sh) and fetches the toolchain pinned
# in tests/alire.toml.
FROM docker.io/library/debian:trixie-slim@sha256:a72b4433de1cf7d6bae0c4cee74d0547f58601a643b3375f070c51600d63b6b2

ARG DEBIAN_SNAPSHOT=20261025T000000Z
COPY scripts/apt-install.sh scripts/install-alr.sh /tmp/
RUN sh /tmp/apt-install.sh ${DEBIAN_SNAPSHOT} \
    ca-certificates curl unzip git make build-essential libgmp10 libmpfr6 \
    libmpc3 && \
    sh /tmp/install-alr.sh

RUN useradd --create-home --uid 1000 --shell /bin/bash dev && \
    install -d -o dev -g dev /home/dev/.local/share/alire \
    /home/dev/.config/alire /home/dev/.alire

USER dev
ENV PATH="/home/dev/.alire/bin:${PATH}"
WORKDIR /work
