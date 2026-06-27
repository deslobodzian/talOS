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

### RTMS Header:
This defines the state of the shared memory reader and writers</b>
- `capacity`: This is the entire availble size of the ring buffer, for now its defined by `MAX_BUFFER_SIZE` but in the future can be
identified by the size of the data * number of slots/messages I want in the buffer
- `message_size`: Self explanitory, size of the message type for each slot.
- `writer`: Writer struct that keeps track of the position of the sequence, this is an atomic and seperate cache-lines.
- `Reader`: Array of reader structs that keep track of the state of the reader, `ReaderStatus` enum and `sequence` the potition of the reader in the ring buffer

## Performance:
Read this for performance increase [ideas](https://en.algorithmica.org/hpc/cpu-cache/cache-lines/)
