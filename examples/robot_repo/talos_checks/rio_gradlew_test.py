"""@talos//controller_processor/rio:gradlew runs from a dependent robot repo.

REQ-DEV-30 (a robot repository uses talOS only through @talos// labels),
REQ-OPS-25 (controller addressing and revision stamping),
REQ-CONTRACT-30 (RoboRIO build revision stamp).
"""

import json
import os
import subprocess
import tempfile
import unittest

from python.runfiles import runfiles


def _make_fake_jdk(root):
    jdk = os.path.join(root, "fake_jdk")
    bin_dir = os.path.join(jdk, "bin")
    os.makedirs(bin_dir, exist_ok=True)
    java = os.path.join(bin_dir, "java")
    with open(java, "w", encoding="utf-8") as f:
        f.write(
            "#!/usr/bin/env python3\n"
            "import json, os, sys\n"
            "args = sys.argv[1:]\n"
            "marker = 'org.gradle.wrapper.GradleWrapperMain'\n"
            "idx = args.index(marker) + 1 if marker in args else 0\n"
            "payload = {'cwd': os.getcwd(), 'gradle_args': args[idx:]}\n"
            "with open(os.environ['FAKE_JAVA_OUT'], 'w', encoding='utf-8') as out:\n"
            "    json.dump(payload, out)\n"
        )
    os.chmod(java, 0o755)
    return jdk


class RioGradlewFromDependentRepoTest(unittest.TestCase):
    def test_stages_from_external_talos_and_forwards_robot_preferences(self):
        gradlew = runfiles.Create().Rlocation(os.environ["RIO_GRADLEW"])
        self.assertTrue(gradlew and os.path.isfile(gradlew), gradlew)
        with tempfile.TemporaryDirectory(
                dir=os.environ.get("TEST_TMPDIR")) as tmp:
            jdk = _make_fake_jdk(tmp)
            cache = os.path.join(tmp, "cache")
            out_json = os.path.join(tmp, "out.json")
            caller = os.path.join(tmp, "robot")
            os.makedirs(os.path.join(caller, ".wpilib"))
            with open(
                os.path.join(caller, ".wpilib", "wpilib_preferences.json"),
                "w",
                encoding="utf-8",
            ) as f:
                json.dump({"projectYear": "2026", "teamNumber": 4321}, f)

            env = {
                k: v
                for k, v in os.environ.items()
                if k
                not in (
                    "ORG_GRADLE_PROJECT_talosRevision",
                    "ORG_GRADLE_PROJECT_talosDirty",
                    "GRADLE_OPTS",
                    "JAVA_OPTS",
                )
            }
            env.update(
                JAVA_HOME=jdk,
                FAKE_JAVA_OUT=out_json,
                TALOS_RIO_CACHE_DIR=cache,
                BUILD_WORKSPACE_DIRECTORY=caller,
            )
            proc = subprocess.run(
                [gradlew, "deploy", "-Pcompanion=10.43.21.11"],
                env=env,
                capture_output=True,
                text=True,
                check=False,
                timeout=60,
            )
            self.assertEqual(proc.returncode, 0, proc.stdout + proc.stderr)
            with open(out_json, encoding="utf-8") as f:
                result = json.load(f)

            rio_dir = os.path.realpath(
                os.path.join(cache, "controller_processor", "rio")
            )
            self.assertEqual(os.path.realpath(result["cwd"]), rio_dir)
            self.assertTrue(
                os.path.isfile(
                    os.path.join(cache, "talOS", "hardware", "gateway.cc")
                )
            )
            self.assertTrue(
                os.path.isfile(
                    os.path.join(cache, "talOS", "process", "durable_sync.h")
                )
            )
            with open(
                os.path.join(rio_dir, ".wpilib", "wpilib_preferences.json"),
                encoding="utf-8",
            ) as f:
                self.assertEqual(json.load(f)["teamNumber"], 4321)
            gargs = result["gradle_args"]
            self.assertIn("deploy", gargs)
            self.assertIn("-Pcompanion=10.43.21.11", gargs)
            rev_args = [a for a in gargs if a.startswith("-PtalosRevision=")]
            dirty_args = [a for a in gargs if a.startswith("-PtalosDirty=")]
            self.assertEqual(len(rev_args), len(dirty_args), gargs)
            if rev_args:
                self.assertRegex(
                    rev_args[0], r"^-PtalosRevision=[0-9a-f]{16}$"
                )
                self.assertIn(
                    dirty_args[0], ("-PtalosDirty=true", "-PtalosDirty=false")
                )


if __name__ == "__main__":
    unittest.main()
