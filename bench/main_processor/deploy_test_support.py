"""Shared by bundle_test, install_check_test and warm_up_test: the
tools/deploy.sh they install with, from the runfiles."""

from python.runfiles import runfiles


def deploy_script():
    return runfiles.Create().Rlocation("_main/tools/deploy.sh")
