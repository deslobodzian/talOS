## Event Loop

# Purpose:
In our robotics program things do not always occure in sequence, for example, we may always be reading the output of a motor, but not necessarily asking the motor to do something.
As such if we always are looking (polling) if a condition to do something is meet, we are wasting cpu cycles. An event loop tries to take care of this.
We can have events that trigger some action (sending a command to run the motor), or a sensor reads some error, causing a safety mechanims event to trigger.

We can also think of events as some periodic function, where we call to run something (like a motor) every timestep (1kHz, 10kHz, 20Hz, etc.).

# Types of events:

For now we will have a couple tyeps of events:

* PERIODIC: Executes a callback every interval at some period.
* NOTIFICATION:
