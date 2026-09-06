#!/usr/bin/env bash
set -euo pipefail

# Default configuration
DEFAULT_OS="ubuntu:24.04"
OS_IMAGE="${DEFAULT_OS}"
FORCE_BUILD=false
INTERACTIVE_SHELL=false
BAZEL_ARGS=()

# Usage information
print_usage() {
    cat << 'USAGE'
talOS Linux Docker Test Runner

Usage:
  ./docker_test.sh [options] [--] [bazel args...]

Options:
  --os <image>     Base Linux image to test (e.g., ubuntu:24.04, ubuntu:22.04, debian:bookworm)
                   Default: ubuntu:24.04
  --build          Rebuild the Docker image before running tests
  --shell          Open an interactive bash shell inside the container
  -h, --help       Show this help message

Examples:
  # Run full test suite on Ubuntu 24.04
  ./docker_test.sh

  # Run full test suite on Ubuntu 22.04
  ./docker_test.sh --os ubuntu:22.04

  # Run a specific test target
  ./docker_test.sh test //talOS/bridge:node_test

  # Run with custom bazel flags
  ./docker_test.sh test //... --test_output=streamed

  # Interactive shell for debugging
  ./docker_test.sh --shell
USAGE
}

# Parse command-line flags
while [[ $# -gt 0 ]]; do
    case "$1" in
        --os)
            OS_IMAGE="$2"
            shift 2
            ;;
        --build)
            FORCE_BUILD=true
            shift
            ;;
        --shell)
            INTERACTIVE_SHELL=true
            shift
            ;;
        -h|--help)
            print_usage
            exit 0
            ;;
        --)
            shift
            BAZEL_ARGS+=("$@")
            break
            ;;
        *)
            BAZEL_ARGS+=("$1")
            shift
            ;;
    esac
done

# Sanitize tag for docker image & cache volume name
SAFE_TAG=$(echo "${OS_IMAGE}" | tr ':/' '-')
IMAGE_TAG="talos-test:${SAFE_TAG}"
CACHE_VOLUME="talos-bazel-cache-${SAFE_TAG}"
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Build Docker image if not found or forced
if [ "${FORCE_BUILD}" = true ] || ! docker image inspect "${IMAGE_TAG}" >/dev/null 2>&1; then
    echo "==> Building Docker image for ${OS_IMAGE} (${IMAGE_TAG})..."
    docker build \
        --build-arg BASE_IMAGE="${OS_IMAGE}" \
        -t "${IMAGE_TAG}" \
        -f "${PROJECT_ROOT}/Dockerfile" \
        "${PROJECT_ROOT}"
fi

# Run interactive shell or bazel tests
if [ "${INTERACTIVE_SHELL}" = true ]; then
    echo "==> Starting interactive bash shell in ${IMAGE_TAG}..."
    docker run --rm -it \
        --shm-size=2g \
        -v "${PROJECT_ROOT}:/workspace" \
        -v "${CACHE_VOLUME}:/root/.cache" \
        --entrypoint /bin/bash \
        "${IMAGE_TAG}"
else
    if [ ${#BAZEL_ARGS[@]} -eq 0 ]; then
        BAZEL_ARGS=("test" "//..." "--test_output=errors")
    fi

    echo "==> Running on ${OS_IMAGE}: bazel ${BAZEL_ARGS[*]}"
    docker run --rm \
        --shm-size=2g \
        -v "${PROJECT_ROOT}:/workspace" \
        -v "${CACHE_VOLUME}:/root/.cache" \
        "${IMAGE_TAG}" \
        "${BAZEL_ARGS[@]}"
fi
