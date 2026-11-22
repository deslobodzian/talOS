#!/usr/bin/env bash
# Workload Identity Federation for GitHub Actions: lets workflows of
# GITHUB_REPO, and no other repository, act as the github-ci service account
# with no stored key.
#
#   scripts/setup-gcp-wif.sh            # print the commands
#   PROJECT_ID=... scripts/setup-gcp-wif.sh --apply
#
# Run scripts/setup-gcp-cicd.sh first (it creates the build VM's service
# account). Free of charge; it only changes IAM.
set -euo pipefail
cd "$(dirname "$0")"
# shellcheck source=scripts/gcp-cicd.env.sh
. ./gcp-cicd.env.sh

P=(--project="$PROJECT_ID")
if [ "$APPLY" = 1 ]; then
  PROJECT_NUMBER="$(gcloud projects describe "$PROJECT_ID" --format='value(projectNumber)')"
else
  PROJECT_NUMBER="<PROJECT_NUMBER>"
fi
POOL_PATH="projects/$PROJECT_NUMBER/locations/global/workloadIdentityPools/$WIF_POOL"

echo "# --- Pool and OIDC provider, restricted to $GITHUB_REPO"
ensure gcloud iam workload-identity-pools describe "$WIF_POOL" "${P[@]}" --location=global -- \
  gcloud iam workload-identity-pools create "$WIF_POOL" "${P[@]}" --location=global \
    --display-name="GitHub Actions"
# The condition is what keeps every other repository on GitHub out: without
# it any repository's token is accepted by the provider.
ensure gcloud iam workload-identity-pools providers describe "$WIF_PROVIDER" "${P[@]}" \
    --location=global --workload-identity-pool="$WIF_POOL" -- \
  gcloud iam workload-identity-pools providers create-oidc "$WIF_PROVIDER" "${P[@]}" \
    --location=global --workload-identity-pool="$WIF_POOL" \
    --issuer-uri=https://token.actions.githubusercontent.com \
    --attribute-mapping="google.subject=assertion.sub,attribute.repository=assertion.repository,attribute.ref=assertion.ref" \
    --attribute-condition="assertion.repository == '$GITHUB_REPO'"

echo "# --- The service account GitHub Actions impersonates"
ensure gcloud iam service-accounts describe "$(sa_email "$GITHUB_SA")" "${P[@]}" -- \
  gcloud iam service-accounts create "$GITHUB_SA" "${P[@]}" \
    --display-name="GitHub Actions CI"
run gcloud iam service-accounts add-iam-policy-binding "$(sa_email "$GITHUB_SA")" "${P[@]}" \
  --role=roles/iam.workloadIdentityUser \
  --member="principalSet://iam.googleapis.com/$POOL_PATH/attribute.repository/$GITHUB_REPO"

echo "# --- What it may do: create and delete build VMs, and SSH to them through IAP"
# Project-wide Compute roles: it could also stop or delete the cache VM. A
# custom role limited to instances create/delete/get and setMetadata is the
# tighter alternative.
for role in roles/compute.instanceAdmin.v1 roles/iap.tunnelResourceAccessor \
            roles/compute.osAdminLogin; do
  run gcloud projects add-iam-policy-binding "$PROJECT_ID" --condition=None \
    --member="serviceAccount:$(sa_email "$GITHUB_SA")" --role="$role"
done
# Needed to start a VM that runs as the build service account, and only that one.
run gcloud iam service-accounts add-iam-policy-binding "$(sa_email "$BUILD_SA")" "${P[@]}" \
  --role=roles/iam.serviceAccountUser \
  --member="serviceAccount:$(sa_email "$GITHUB_SA")"

cat <<EOF
# --- Repository variables for .github/workflows/ci-cd.yml (not secrets:
# none of them grants access by itself)
#   gh variable set GCP_PROJECT_ID --repo $GITHUB_REPO --body '$PROJECT_ID'
#   gh variable set GCP_WIF_PROVIDER --repo $GITHUB_REPO --body '$POOL_PATH/providers/$WIF_PROVIDER'
#   gh variable set GCP_CI_SERVICE_ACCOUNT --repo $GITHUB_REPO --body '$(sa_email "$GITHUB_SA")'
EOF
