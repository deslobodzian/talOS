import assert from 'node:assert/strict';
import {test} from 'node:test';

import {dispatch, handleRpc, RpcFault} from '../agent/server';
import {decode, Timeline} from '../src/core';
import {demoPacket} from '../src/demo';

function history() {
  const timeline = new Timeline();
  [0, 4, 5, 0, 7].forEach((current, i) => {
    const frame = decode(demoPacket(BigInt(i)));
    frame.channels['motor.current'] = current;
    timeline.append(frame);
  });
  return timeline;
}
test(
    'RPC exact lookup rejects in-between samples and supports explicit preceding state',
    () => {
      const timeline = history();
      assert.throws(
          () => dispatch(
              timeline, 'query_state_at', {timestamp_ns: '1020000001'}),
          (e: unknown) => e instanceof RpcFault && e.code === -32001);
      const response = dispatch(timeline, 'query_state_at', {
                         timestamp_ns: '1020000001',
                         mode: 'at_or_before',
                         topics: ['motor.current', 'chassis.x']
                       }) as any;
      assert.equal(response.timestamp_ns, '1020000000');
      assert.equal(response.topics['motor.current'], 4);
      assert.equal(typeof response.topics['chassis.x'], 'number');
      assert.throws(
          () => dispatch(
              timeline, 'query_state_at',
              {timestamp_ns: '18446744073709551616'}),
          /uint64/);
    });
test(
    'threshold scans group samples and edge scans use the preceding sample outside bounds',
    () => {
      const timeline = history();
      const result =
          dispatch(
              timeline, 'scan_channel_events',
              {topic: 'motor.current', condition: {op: 'gt', value: 3}}) as any;
      assert.deepEqual(result.ranges, [
        {start_ns: '1020000000', end_ns: '1040000000', samples: 2},
        {start_ns: '1080000000', end_ns: '1080000000', samples: 1}
      ]);
      const edge = dispatch(timeline, 'scan_channel_events', {
                     topic: 'motor.current',
                     condition: {op: 'falling'},
                     start_ns: '1060000000'
                   }) as any;
      assert.deepEqual(
          edge.ranges,
          [{start_ns: '1060000000', end_ns: '1060000000', samples: 1}]);
    });
test(
    'JSON-RPC preserves IDs, suppresses notifications, and validates requests',
    () => {
      const timeline = history();
      assert.equal(
          handleRpc(timeline, {jsonrpc: '2.0', method: 'get_schema_tree'}),
          undefined);
      assert.equal(
          (handleRpc(
               timeline, {jsonrpc: '2.0', id: 0, method: 'get_schema_tree'}) as
           any)
              .id,
          0);
      assert.equal(
          (handleRpc(timeline, {jsonrpc: '2.0', id: 'a', method: 'missing'}) as
           any)
              .error.code,
          -32601);
      assert.equal(
          (handleRpc(timeline, {
             jsonrpc: '2.0',
             id: 1,
             method: 'query_state_at',
             params: {timestamp_ns: 1020000000}
           }) as any)
              .error.code,
          -32602);
      assert.equal(
          (handleRpc(timeline, {jsonrpc: '1.0', method: 'get_schema_tree'}) as
           any)
              .error.code,
          -32600);
      assert.throws(
          () => dispatch(
              timeline, 'query_state_at',
              {timestamp_ns: '1020000000', topics: ['__proto__.polluted']}),
          /Unknown topic/);
    });
