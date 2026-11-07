// Unit tests for the shared formatters. The key contract: every duration
// carries a unit, including zero -- "not known yet" is "—", but a zero reading
// is a real zero, not a unitless bare digit.

import assert from 'node:assert/strict';
import {test} from 'node:test';

import {duration, hertz, integer, number, timestamp} from '../src/ui/formatters';

test('duration always carries a unit, including exactly zero', () => {
  // Zero (the bug this guards): a unit, not a bare '0'.
  assert.equal(duration(0), '0 µs');
  assert.equal(duration('0'), '0 µs');
  assert.equal(duration(-0), '0 µs');

  // Non-zero values pick a unit by magnitude.
  assert.equal(duration(500), '500 ns');
  assert.equal(duration(5000), '5.0 µs');
  assert.equal(duration(5e6), '5.00 ms');
  assert.equal(duration(1e9), '1.00 s');

  // Non-finite is "not known", not a number.
  assert.equal(duration(NaN), '—');
  assert.equal(duration('not-a-number'), '—');
});

test('integer formats decimal-string uint64 with thousands separators', () => {
  assert.equal(integer('1204338'), '1,204,338');
  assert.equal(integer(1204338), '1,204,338');
});

test('hertz formats rates at the right precision', () => {
  assert.equal(hertz(undefined), '—');
  assert.equal(hertz(200), '200 Hz');
  assert.equal(hertz(12.5), '12.5 Hz');
  assert.equal(hertz(0.25), '0.25 Hz');
});

test('number formats finite values and marks absent with an em dash', () => {
  assert.equal(number(1.234567), '1.2346');
  assert.equal(number(NaN), '—');
});

test('timestamp carries the nanoseconds unit', () => {
  assert.equal(timestamp(null), '—');
  assert.equal(timestamp('1204338'), '1,204,338 ns');
});
