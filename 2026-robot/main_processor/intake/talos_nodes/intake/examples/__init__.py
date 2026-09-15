"""Teaching examples: three complete mechanisms, sim-only.

Each example pairs one mechanism with one control mode and runs without
hardware, shared memory, or the native loop -- pure stdlib plus the
student API (``student_policy``, ``motor_commands``, ``units``,
``control_config``, ``behaviors``):

* ``roller_intake`` -- velocity roller with beam-break capture;
* ``flywheel`` -- velocity flywheel with at-speed detection;
* ``position_mechanism`` -- position servo with gearing.

``plant.py`` (this module's sibling) is the shared sim-only motor: it is
a teaching stand-in, never robot code.
"""
