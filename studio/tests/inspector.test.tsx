// Inspector: the per-channel staleness derivation.

import assert from 'node:assert/strict';
import {test} from 'node:test';

import {channelStalenessNs} from '../src/panels/inspector/staleness';

test('channelStalenessNs returns the age of the newest reading', () => {
  const series = {
    t0_ns: '0',
    times: new Float64Array([0, 0.02, 0.04]),
    values: {
      a: new Float64Array([1, 2, NaN]),   // last reading at 0.02 s
      b: new Float64Array([NaN, NaN, NaN])
    }
  };
  // cursor at 60 ms: age of channel a's 20 ms reading is 40 ms.
  assert.equal(channelStalenessNs(series, 'a', '60000000'), '40000000');
  // cursor at the reading itself is fresh.
  assert.equal(channelStalenessNs(series, 'a', '20000000'), '0');
  // a cursor before the reading clamps to fresh rather than a negative age.
  assert.equal(channelStalenessNs(series, 'a', '10000000'), '0');
  // a channel that never had a value, an unknown channel, and a null cursor.
  assert.equal(channelStalenessNs(series, 'b', '60000000'), null);
  assert.equal(channelStalenessNs(series, 'missing', '60000000'), null);
  assert.equal(channelStalenessNs(series, 'a', null), null);
});
