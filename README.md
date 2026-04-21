# talOS
Named after the first automaton, talOS is a robotics control architecture aimed to work with the NI RobotRIO or SystemCore acting as a HAL layer with a seperate main computer handling logic

## Building
Build used bazel for all projects in the repo to build all. </b> 

Build all: `bazle build //...`</b> 

Example for building a specific module:

`bazel build //.talos`


To generate `compile_comands.json` for clangd or other LSPs in c++: </b>

`bazel run @hedron_compile_commands//:refresh_all`