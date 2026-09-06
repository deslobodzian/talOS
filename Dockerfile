# Multi-OS Linux testing image for talOS
# Usage:
#   docker build -t talos-test:ubuntu-24.04 .
#   docker build --build-arg BASE_IMAGE=ubuntu:22.04 -t talos-test:ubuntu-22.04 .
#   docker build --build-arg BASE_IMAGE=debian:bookworm -t talos-test:debian-12 .
ARG BASE_IMAGE=ubuntu:24.04
FROM ${BASE_IMAGE}

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install build essentials, compilers, git, python, and dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    clang \
    git \
    curl \
    wget \
    ca-certificates \
    python3 \
    python3-dev \
    python3-venv \
    pkg-config \
    libssl-dev \
    zip \
    unzip \
    && (apt-get install -y --no-install-recommends libwebkit2gtk-4.1-dev libgtk-3-dev libayatana-appindicator3-dev librsvg2-dev || \
        apt-get install -y --no-install-recommends libwebkit2gtk-4.0-dev libgtk-3-dev libayatana-appindicator3-dev librsvg2-dev || true) \
    && rm -rf /var/lib/apt/lists/*

# Install Bazelisk as /usr/local/bin/bazel
ARG TARGETARCH
RUN ARCH="${TARGETARCH:-$(dpkg --print-architecture)}" && \
    curl -fsSL "https://github.com/bazelbuild/bazelisk/releases/download/v1.25.0/bazelisk-linux-${ARCH}" \
         -o /usr/local/bin/bazel && \
    chmod +x /usr/local/bin/bazel

WORKDIR /workspace

ENTRYPOINT ["bazel"]
CMD ["test", "//...", "--test_output=errors"]
