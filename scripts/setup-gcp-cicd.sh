#!/usr/bin/env bash
# One-time Google Cloud setup for the Bazel remote cache and the Spot build
# VMs: APIs, a private network with Cloud NAT, firewall rules, the cache
# bucket with its lifecycle rule, the Artifact Registry Docker repository and
# the two VM service accounts.
#
#   scripts/setup-gcp-cicd.sh            # print the commands
#   PROJECT_ID=... scripts/setup-gcp-cicd.sh --apply
#
# Safe to re-run: with --apply an existing resource is left as it is.
# What costs money once applied: Cloud NAT (per VM-hour and per GB), the
# bucket's storage, and Artifact Registry storage. Nothing here starts a VM.
set -euo pipefail
cd "$(dirname "$0")"
# shellcheck source=scripts/gcp-cicd.env.sh
. ./gcp-cicd.env.sh

P=(--project="$PROJECT_ID")

echo "# --- APIs"
run gcloud services enable "${P[@]}" \
  compute.googleapis.com storage.googleapis.com artifactregistry.googleapis.com \
  iam.googleapis.com iamcredentials.googleapis.com sts.googleapis.com \
  iap.googleapis.com oslogin.googleapis.com

echo "# --- Network: private subnet, outbound through Cloud NAT"
ensure gcloud compute networks describe "$NETWORK" "${P[@]}" -- \
  gcloud compute networks create "$NETWORK" "${P[@]}" --subnet-mode=custom
ensure gcloud compute networks subnets describe "$SUBNET" "${P[@]}" --region="$REGION" -- \
  gcloud compute networks subnets create "$SUBNET" "${P[@]}" --region="$REGION" \
    --network="$NETWORK" --range="$SUBNET_RANGE" --enable-private-ip-google-access
ensure gcloud compute routers describe "$ROUTER" "${P[@]}" --region="$REGION" -- \
  gcloud compute routers create "$ROUTER" "${P[@]}" --region="$REGION" --network="$NETWORK"
ensure gcloud compute routers nats describe "$NAT" "${P[@]}" --region="$REGION" --router="$ROUTER" -- \
  gcloud compute routers nats create "$NAT" "${P[@]}" --region="$REGION" --router="$ROUTER" \
    --auto-allocate-nat-external-ips --nat-all-subnet-ip-ranges
ensure gcloud compute addresses describe "$CACHE_IP_NAME" "${P[@]}" --region="$REGION" -- \
  gcloud compute addresses create "$CACHE_IP_NAME" "${P[@]}" --region="$REGION" \
    --subnet="$SUBNET" --addresses="$CACHE_IP"

echo "# --- Firewall: the cache's gRPC port from build VMs only; SSH from IAP only"
ensure gcloud compute firewall-rules describe "$NETWORK-cache-grpc" "${P[@]}" -- \
  gcloud compute firewall-rules create "$NETWORK-cache-grpc" "${P[@]}" --network="$NETWORK" \
    --direction=INGRESS --action=ALLOW --rules=tcp:9092 \
    --source-tags="$BUILD_TAG" --target-tags="$CACHE_TAG"
# 35.235.240.0/20 is Identity-Aware Proxy's TCP forwarding range.
ensure gcloud compute firewall-rules describe "$NETWORK-iap-ssh" "${P[@]}" -- \
  gcloud compute firewall-rules create "$NETWORK-iap-ssh" "${P[@]}" --network="$NETWORK" \
    --direction=INGRESS --action=ALLOW --rules=tcp:22 \
    --source-ranges=35.235.240.0/20 --target-tags="$BUILD_TAG,$CACHE_TAG"

echo "# --- Cache bucket: private, objects deleted $CACHE_OBJECT_AGE_DAYS days after upload"
ensure gcloud storage buckets describe "gs://$CACHE_BUCKET" "${P[@]}" -- \
  gcloud storage buckets create "gs://$CACHE_BUCKET" "${P[@]}" --location="$REGION" \
    --uniform-bucket-level-access --public-access-prevention
lifecycle="$(mktemp)"
trap 'rm -f "$lifecycle"' EXIT
cat >"$lifecycle" <<EOF
{"rule": [{"action": {"type": "Delete"}, "condition": {"age": $CACHE_OBJECT_AGE_DAYS}}]}
EOF
echo "# lifecycle file: $(cat "$lifecycle")"
run gcloud storage buckets update "gs://$CACHE_BUCKET" "${P[@]}" --lifecycle-file="$lifecycle"

echo "# --- Artifact Registry Docker repository"
ensure gcloud artifacts repositories describe "$ARTIFACT_REPO" "${P[@]}" --location="$REGION" -- \
  gcloud artifacts repositories create "$ARTIFACT_REPO" "${P[@]}" --location="$REGION" \
    --repository-format=docker --description="talOS container images"

echo "# --- Service accounts for the two kinds of VM"
ensure gcloud iam service-accounts describe "$(sa_email "$CACHE_SA")" "${P[@]}" -- \
  gcloud iam service-accounts create "$CACHE_SA" "${P[@]}" \
    --display-name="bazel-remote cache VM"
ensure gcloud iam service-accounts describe "$(sa_email "$BUILD_SA")" "${P[@]}" -- \
  gcloud iam service-accounts create "$BUILD_SA" "${P[@]}" \
    --display-name="Spot build VM"
# Only the cache VM touches the bucket. A build VM reaches cached results
# through bazel-remote, never the bucket itself.
run gcloud storage buckets add-iam-policy-binding "gs://$CACHE_BUCKET" "${P[@]}" \
  --member="serviceAccount:$(sa_email "$CACHE_SA")" --role=roles/storage.objectAdmin
for sa in "$CACHE_SA" "$BUILD_SA"; do
  run gcloud projects add-iam-policy-binding "$PROJECT_ID" --condition=None \
    --member="serviceAccount:$(sa_email "$sa")" --role=roles/logging.logWriter
done

echo "# Next: scripts/create-bazel-cache-vm.sh, then scripts/setup-gcp-wif.sh"
