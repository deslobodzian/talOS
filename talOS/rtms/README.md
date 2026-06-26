# Real Time Messaging System (RTMS)

This will be a single producer, mutlti consumer architecture pub-sub system.

Utilizing a Ring Buffer in shared memory for each topic. Goal is to be lock free.

## Nameing scheme

All topics will be broadcasted under `"/dev/rtms/topic_name"`

## Messages

All messages will use flatbuffers for serialization.

## Memory
Each ring buffer with be stored with a corresponding header, the shared memory
layout will look like this</b>

`[rmts_header][data...]`


## Performance:
Read this for performance increase [ideas](https://en.algorithmica.org/hpc/cpu-cache/cache-lines/)
