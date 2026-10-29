load("//talOS/build:contract.bzl", "talos_messages")

# The example's messages and topics (namespace demo), shared by the C++ and
# Python nodes:
#   C++:    #include "demo/intake_generated.h"   demo::IntakeGoal
#           #include "demo/topics.h"            demo::topics::kIntakeGoal
#   Python: from demo.IntakeGoal import IntakeGoal
#           from demo import topics             topics.INTAKE_GOAL
# Each mechanism publishes its own /<name>/state and reads /<name>/target;
# the superstructure is the only writer of the mechanism targets.
talos_messages(
    name = "messages",
    srcs = [
        "elevator.fbs",
        "intake.fbs",
        "superstructure.fbs",
    ],
    include_prefix = "demo",
    topics = {
        "intake_goal": "/intake/target",
        "intake_status": "/intake/state",
        "elevator_goal": "/elevator/target",
        "elevator_status": "/elevator/state",
        "superstructure_goal": "/superstructure/target",
        "superstructure_status": "/superstructure/state",
    },
)
