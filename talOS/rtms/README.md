# Real Time Messaging System (RTMS)

This will be a single producer, mutlti consumer architecture pub-sub system.

Utilizing a Ring Buffer in shared memory for each topic. Goal is to be lock free.

## Nameing scheme

All topics will be broadcasted under `"/dev/rtms/topic_name"`

## Messages

All messages will use flatbuffers for serialization.
