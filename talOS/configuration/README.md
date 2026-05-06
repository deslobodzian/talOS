# Robot Configuration:

## Subsystem
The idea of subsystems should still exist, each subsystem should outline thier
motors, sensors, etc.

### Subsystem config
Example config idea
```
[subsystem]
name = "flywheel"

[motors]
[motors.left_leader]
can_id = 1;
canbus = "canivore"
supply_current = 30
stator_current = 120
# other configuration, should I keep pid here?
```

## Config Message
Each subsystem will have its own toml configuration, we need to send a message
to the gateway controller that will configure each motor, sensor, etc.

### Config Questions
- Should I send multiple messages for the configs?
- Do I aggregate all configs into one large message to the gateway?
- How do I have configs communicate to each other (prewent motor overlap)?

