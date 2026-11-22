# Shared settings for the Google Cloud CI scripts in this directory. Sourced,
# not run. Every value can be overridden from the environment.
#
# The scripts print the gcloud commands they would run and change nothing
# unless they are given --apply.

: "${PROJECT_ID:=<YOUR_TALOS_PROJECT_ID>}"
: "${REGION:=us-east4}"
: "${ZONE:=us-east4-a}"
: "${GITHUB_REPO:=deslobodzian/talOS}"

# Bazel cache bucket (globally unique name) and the Docker repository.
: "${CACHE_BUCKET:=${PROJECT_ID}-bazel-cache}"
: "${CACHE_OBJECT_AGE_DAYS:=14}"
: "${ARTIFACT_REPO:=talos}"

# A network of its own: no VM in it has an external address. Outbound traffic
# (apt, Bazel's downloads, Docker Hub) leaves through Cloud NAT.
: "${NETWORK:=talos-ci}"
: "${SUBNET:=talos-ci-${REGION}}"
: "${SUBNET_RANGE:=10.10.0.0/24}"
: "${ROUTER:=talos-ci-router}"
: "${NAT:=talos-ci-nat}"

# The cache VM keeps one reserved internal address, the one .bazelrc's
# build:gcp names. Change both together.
: "${CACHE_VM:=bazel-remote-cache}"
: "${CACHE_IP_NAME:=bazel-remote-cache-ip}"
: "${CACHE_IP:=10.10.0.10}"
: "${CACHE_MACHINE_TYPE:=e2-medium}"
: "${CACHE_DISK_GB:=60}"
# bazel-remote's local LRU size in GiB; keep it below CACHE_DISK_GB.
: "${CACHE_MAX_SIZE_GIB:=40}"
# Pin this to a release tag or a digest before --apply: "latest" changes
# under a persistent VM whenever it restarts.
: "${BAZEL_REMOTE_IMAGE:=buchgr/bazel-remote-cache:latest}"

# Network tags the firewall rules are written against.
: "${CACHE_TAG:=bazel-cache}"
: "${BUILD_TAG:=ci-build}"

# Service accounts: the cache VM (writes the bucket), the Spot build VM (no
# roles beyond logging) and the one GitHub Actions impersonates.
: "${CACHE_SA:=bazel-cache-vm}"
: "${BUILD_SA:=ci-build-vm}"
: "${GITHUB_SA:=github-ci}"
: "${WIF_POOL:=github}"
: "${WIF_PROVIDER:=talos-repo}"

sa_email() { printf '%s@%s.iam.gserviceaccount.com' "$1" "$PROJECT_ID"; }

APPLY=0
for arg in "$@"; do
  case "$arg" in
    --apply) APPLY=1 ;;
    *) echo "usage: $0 [--apply]" >&2; exit 2 ;;
  esac
done

if [ "$APPLY" = 1 ]; then
  case "$PROJECT_ID" in
    *"<"*|"") echo "PROJECT_ID is not set; export PROJECT_ID=<id> first" >&2; exit 2 ;;
  esac
  command -v gcloud >/dev/null || { echo "gcloud is not installed" >&2; exit 2; }
else
  echo "# Dry run: printing commands only. Re-run with --apply to execute."
fi

# Prints the command; with --apply also runs it.
run() {
  printf '+'
  printf ' %q' "$@"
  printf '\n'
  if [ "$APPLY" = 1 ]; then "$@"; fi
}

# run, unless the resource already exists: `ensure <describe command> -- <create command>`.
# A dry run never asks gcloud and prints the create command.
ensure() {
  local probe=()
  while [ "$1" != "--" ]; do probe+=("$1"); shift; done
  shift
  if [ "$APPLY" = 1 ] && "${probe[@]}" >/dev/null 2>&1; then
    echo "# exists: ${probe[*]}"
    return 0
  fi
  run "$@"
}
