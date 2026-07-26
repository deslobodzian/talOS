#!/bin/bash

echo Running IPC Test
./bazel-bin/talOS/ipc/pub_process -d 10 & ./bazel-bin/talOS/ipc/sub_process -d 5
