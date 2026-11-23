# Google Cloud CI scripts

A Bazel remote cache and on-demand build VMs in Google Cloud (`us-east4`),
used by the manually started workflow `.github/workflows/ci-cd.yml`. The
workflows in [docs/TESTING.md §9](../docs/TESTING.md#9-lanes-and-ci) remain
what gates a push; this one adds nothing to them.

Every script prints the `gcloud` commands it would run and changes nothing
until it is given `--apply`. Settings (project, region, names, the cache's
address) are in `gcp-cicd.env.sh` and can be overridden from the environment.

| Step | Script | Creates | Costs money |
|---|---|---|---|
| 1 | `setup-gcp-cicd.sh` | APIs, the `talos-ci` network with Cloud NAT, firewall rules, the cache bucket (objects deleted 14 days after upload), the Artifact Registry Docker repository, two VM service accounts | Cloud NAT, storage |
| 2 | `create-bazel-cache-vm.sh` | the persistent `e2-medium` Container-Optimized OS VM running `bazel-remote` (`cache-vm-cloud-init.yaml`) | the VM and its disk, until deleted |
| 3 | `setup-gcp-wif.sh` | the Workload Identity pool and provider for the GitHub repository, and the `github-ci` service account | no |

```sh
scripts/setup-gcp-cicd.sh                       # read the commands
PROJECT_ID=my-project scripts/setup-gcp-cicd.sh --apply
```

After step 3, set the three repository variables it prints, then start the
workflow from the Actions tab.

## Access

- No VM has an external address. Outbound traffic goes through Cloud NAT.
- The cache's gRPC port (9092) accepts connections only from VMs tagged
  `ci-build` in the same network. Its HTTP port listens on loopback.
- SSH to either kind of VM goes through Identity-Aware Proxy with OS Login.
- Only the cache VM's service account can read or write the bucket.
- The Workload Identity provider accepts tokens of the one repository named
  in `GITHUB_REPO`. The workflow is started by hand because it writes the
  shared cache: a trigger that runs code from a fork would let that code
  write cache entries later builds trust.

## Using the cache

`bazel build --config=gcp //...` (`.bazelrc`) on a VM in the `talos-ci`
network. The workflow does this for a branch, and then runs one lane:

```sh
gh workflow run "Cloud Build" --ref my-branch -f lane=fast   # replay (default), fast, full, asan, tsan
```

Three more lanes check what the per-push workflows do not, and take a count:

```sh
gh workflow run "Cloud Build" --ref main -f lane=soak   -f count=8  -f max_minutes=40   # the bench soak, seeds 0 to 7
gh workflow run "Cloud Build" --ref main -f lane=stress -f count=20 -f max_minutes=60   # every test 20 times, cores loaded
gh workflow run "Cloud Build" --ref main -f lane=rtprio                                 # //talOS/process under a realtime grant
```

One lane runs at a time: the project's quota is 16 N2D CPUs in the region,
which one build VM uses, so a lane started while another runs fails at
"Create the Spot VM" with the quota error. `soak` takes about four minutes a
seed. `stress` reports a test that fails
even once, which is a flake to fix ([docs/TESTING.md §8](../docs/TESTING.md#8-flakes)).
`rtprio` raises the realtime limit before Bazel starts and fails unless
`ProcessTest.BackgroundThreadLeavesAnInheritedRealtimePolicy`, which skips
on every host without that grant, ran.

`-f max_minutes=40` or `60` gives a lane that does not fit in the default 20
minutes (a sanitizer build with a cold cache, several soak seeds) a
longer-lived VM. When a lane fails, the run's artifact holds every test's
log, and each failed soak seed's. The address in `build:gcp` is `CACHE_IP` in `gcp-cicd.env.sh`;
change both together. It is not reachable from a laptop, whose builds keep
using the local disk cache.

The bucket's lifecycle rule deletes an object 14 days after it was uploaded,
whether or not it is still used; `bazel-remote` uploads it again on the next
build that produces it.
