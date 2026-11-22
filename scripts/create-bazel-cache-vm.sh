#!/usr/bin/env bash
# Creates the persistent Bazel cache VM: an e2-medium Container-Optimized OS
# instance running bazel-remote, backed by the bucket setup-gcp-cicd.sh made.
#
#   scripts/create-bazel-cache-vm.sh            # print the command
#   PROJECT_ID=... scripts/create-bazel-cache-vm.sh --apply
#
# Costs money from the moment it is applied: the VM and its disk run until
# deleted (`gcloud compute instances delete bazel-remote-cache`).
#
# Access: the VM has no external address. Port 9092 (gRPC) is open only to
# VMs tagged ci-build in the same network; bazel-remote's HTTP port listens
# on loopback only. To look at it:
#   gcloud compute ssh bazel-remote-cache --zone=us-east4-a --tunnel-through-iap \
#     --command='sudo journalctl -u bazel-remote -n 50; curl -s localhost:8080/status'
set -euo pipefail
cd "$(dirname "$0")"
# shellcheck source=scripts/gcp-cicd.env.sh
. ./gcp-cicd.env.sh

user_data="$(mktemp)"
trap 'rm -f "$user_data"' EXIT
sed -e "s|__BUCKET__|$CACHE_BUCKET|g" \
    -e "s|__IMAGE__|$BAZEL_REMOTE_IMAGE|g" \
    -e "s|__MAX_SIZE_GIB__|$CACHE_MAX_SIZE_GIB|g" \
    cache-vm-cloud-init.yaml >"$user_data"

case "$BAZEL_REMOTE_IMAGE" in
  *:latest) echo "# warning: BAZEL_REMOTE_IMAGE is $BAZEL_REMOTE_IMAGE; pin a release tag or digest" >&2 ;;
esac

ensure gcloud compute instances describe "$CACHE_VM" --project="$PROJECT_ID" --zone="$ZONE" -- \
  gcloud compute instances create "$CACHE_VM" \
    --project="$PROJECT_ID" --zone="$ZONE" \
    --machine-type="$CACHE_MACHINE_TYPE" \
    --image-family=cos-stable --image-project=cos-cloud \
    --boot-disk-size="${CACHE_DISK_GB}GB" --boot-disk-type=pd-balanced \
    --network-interface="subnet=$SUBNET,private-network-ip=$CACHE_IP,no-address" \
    --tags="$CACHE_TAG" \
    --service-account="$(sa_email "$CACHE_SA")" --scopes=cloud-platform \
    --shielded-secure-boot --shielded-vtpm --shielded-integrity-monitoring \
    --metadata=enable-oslogin=TRUE,google-logging-enabled=true \
    --metadata-from-file=user-data="$user_data"

echo "# Builds reach it with: bazel build --config=gcp //...   (grpc://$CACHE_IP:9092, .bazelrc)"
