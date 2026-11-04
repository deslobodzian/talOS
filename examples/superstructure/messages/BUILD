load("//talOS/build:contract.bzl", "talos_messages")

# The example's messages and topics (namespace demo), shared by the C++ and
# Python nodes:
#   C++:    #include "demo/intake_generated.h"   demo::IntakeGoal
#           #include "demo/topics.h"            demo::topics::kIntakeStatus
#   Python: from demo.IntakeGoal import IntakeGoal
#           from demo import topics             topics.INTAKE_STATUS
# Each node publishes its own /<name>/state. Goals travel on target lanes,
# /<name>/target/<source> (talos::Target / talos::TargetLane), named by the
# subsystem and source rather than listed here: the superstructure writes
# /intake/target/superstructure and /elevator/target/superstructure, and a
# driver or autonomous node writes /superstructure/target/{teleop,auto}.
talos_messages(
    name = "messages",
    srcs = [
        "elevator.fbs",
        "intake.fbs",
        "superstructure.fbs",
    ],
    include_prefix = "demo",
    topics = {
        "intake_status": "/intake/state",
        "elevator_status": "/elevator/state",
        "superstructure_status": "/superstructure/state",
    },
)
