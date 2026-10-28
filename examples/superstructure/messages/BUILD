load("//talOS/build:contract.bzl", "talos_messages")

# The example's messages (namespace demo), shared by the C++ and Python nodes:
#   C++:    #include "demo/intake_generated.h"   demo::IntakeGoal
#   Python: from demo.IntakeGoal import IntakeGoal
talos_messages(
    name = "messages",
    srcs = [
        "elevator.fbs",
        "intake.fbs",
        "superstructure.fbs",
    ],
    hdrs = ["topics.h"],
    include_prefix = "demo",
)
