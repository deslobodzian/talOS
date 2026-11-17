@AGENTS.md

## Verification practice

Adds to [docs/DEVELOPMENT.md §5](docs/DEVELOPMENT.md#5-verification-ladder)
and [docs/TESTING.md](docs/TESTING.md); where they or RULES.md state a
rule, they win.

1. **Run the change, not only its tests.** Before a change to what a
   running node, the launcher or the bridge does is called done, the main
   session runs it end to end (`./sim.sh --sim-gateway --no-studio`, or the
   launcher on the affected config) and reads what it did: console,
   `/hw/status`, the session's logs. A unit test can pass while the launch
   is broken. Subagents run only the target their brief names.
2. **Pick the test that is hard to fool.** The tier is the lowest that can
   observe the behaviour (docs/TESTING.md §2). Within it, prefer a property
   test for anything with an inverse or an invariant (a frame or payload
   codec's encode and decode, a parser, an operation that must be
   idempotent), and a differential test when behaviour must stay the same:
   old against new implementation, C++ against its Python wrapper, the sim
   gateway against the RoboRIO gateway, a replay against its live log, on
   generated inputs, with the intended differences listed.
3. **Three rounds, then a person.** A check still failing after three fix
   attempts is reported with its output and what was tried, not given a
   fourth variation. Unattended, the main session records it in TODO.md and
   continues other work; red CI on a pushed commit is still fixed or
   reverted by the next commit (docs/DEVELOPMENT.md §6).
4. **A change cannot loosen what judges it.** An edit that makes a judge
   weaker (a widened allowlist or baseline, a removed or weakened
   assertion, a disabled lint or tidy check, a relaxed
   `.claude/agents/reviewer.md`) goes in its own commit with the reason in
   `Why:`, and the rest of the change is judged by the base branch's
   version. Edits that tighten or track (a lowered allowlist count, an
   assertion the requirement's new behaviour forces) stay in the change.
5. **A result belongs to one commit.** A review, suite, Mac pass or prepush
   holds for the commit it ran on and nothing later. After a fold, rebase
   or follow-up, run again what the change reaches, and prepush on the head
   being pushed; a report names the hash each result is for. A commit
   nothing was run on says `Tests: none run` and why.
6. **One reviewer, rounds that build.** Improve the reviewer's brief before
   adding a reviewer. A later round gets the earlier findings and the
   follow-ups, confirms each, and attacks what changed; it does not start
   over. A second reviewer is for safety paths only.
7. **The review brief names what the diff triggers.** Add the lines whose
   trigger the diff matches:
   - a lint or check tool: try to get a violating change past it (merge
     commits, attributes, quoted or renamed paths); it must fail closed;
   - a commit folded or reworded: its message is true to the diff it now
     holds;
   - a doc or requirement saying "never", "every" or "no": name the case
     that breaks it, or its conditions;
   - a test said to close a proof gap: name the wrong implementation the
     old test passed, and mutate to check both;
   - a lint, graph or naming change: `//bench/...` and `//examples/...`
     run as well;
   - a cherry-picked slice: `python3 tools/lint_test_time.py` (its
     allowlist holds exact counts).
8. **Turn a miss into a check, and prune.** When review, CI or the bench
   finds what an earlier check should have caught, the same group adds the
   smallest check that catches it: a test, a lint, a trigger line above.
   The drift pass
   ([docs/DEVELOPMENT.md §7](docs/DEVELOPMENT.md#7-drift-pass-main-session))
   deletes trigger lines that no longer catch anything. RULES.md changes
   only by the owner's decision.
9. **A new gate starts by observing.** A new lint first reports (a warning,
   or a baseline of its existing hits) and fails the build once its false
   positives have been read. A check guarding RULES.md 1 to 4 refuses from
   the start.
