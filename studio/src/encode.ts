import {Builder} from 'flatbuffers';

import {ChannelInfo} from './generated/talos/telemetry/channel-info';
import {ChannelKind} from './generated/talos/telemetry/channel-kind';
import {Frame} from './generated/talos/telemetry/frame';
import {Pose} from './generated/talos/telemetry/pose';

// TLM3 page encoding for Studio's own frames: the demo stream and log export
// (talOS/telemetry/README.md). The robot and the bridge encode with
// talOS/telemetry/frames.h; these pages decode the same way.

export {ChannelKind};

export type ChannelSpec = {
  name: string; unit: string; labels?: string[]; kind?: ChannelKind
};
export type PageHeader = {
  streamId: bigint; sequenceId: bigint; timestampNs: bigint;
  dictionaryId: number; channelCount: number
};
export type WirePose = {
  x: number; y: number; z: number; roll: number; pitch: number; yaw: number
};

const scratch = new DataView(new ArrayBuffer(8));
export function rawFromDouble(value: number): bigint {
  scratch.setFloat64(0, value, true);
  return scratch.getBigUint64(0, true);
}
export function doubleFromRaw(raw: bigint): number {
  scratch.setBigUint64(0, raw, true);
  return scratch.getFloat64(0, true);
}

// The producer's dictionary id: FNV-1a 32 over names, units, kinds and
// labels with the same separators (talOS/telemetry/frames.h, DictionaryId).
export function dictionaryId(channels: ChannelSpec[]): number {
  let hash = 2166136261;
  const byte = (b: number) => {
    hash = Math.imul(hash ^ b, 16777619) >>> 0;
  };
  const text = (s: string) => {
    for (const b of new TextEncoder().encode(s)) byte(b);
  };
  for (const channel of channels) {
    text(channel.name);
    byte(0);
    text(channel.unit);
    byte(0);
    byte(channel.kind ?? ChannelKind.Double);
    byte(0);
    for (const label of channel.labels ?? []) {
      text(label);
      byte(0);
    }
    byte(0xff);
  }
  return hash;
}

// Upper bound on one entry's bytes in a dictionary page (frames.h,
// DictionaryEntryBytes), so pages stay under the 16384-byte slot.
const entryBytes = (c: ChannelSpec) => 64 + c.name.length * 3 +
    c.unit.length * 3 +
    (c.labels ?? []).reduce((n, label) => n + 12 + label.length * 3, 0);
const DICTIONARY_PAGE_ROOM = 16000;

export function encodeDictionaryPages(header: PageHeader,
                                      channels: ChannelSpec[]): Uint8Array[] {
  const groups: {first: number; entries: ChannelSpec[]}[] = [];
  let bytes = Infinity;
  channels.forEach((channel, i) => {
    const size = entryBytes(channel);
    if (bytes + size > DICTIONARY_PAGE_ROOM) {
      groups.push({first: i, entries: []});
      bytes = 0;
    }
    groups.at(-1)!.entries.push(channel);
    bytes += size;
  });
  if (!groups.length) groups.push({first: 0, entries: []});
  return groups.map(({first, entries}, page) => {
    const b = new Builder(1024);
    const offsets = entries.map(channel => {
      // Strings and vectors first: nothing may be built while a table is open.
      const name = b.createString(channel.name);
      const unit = channel.unit ? b.createString(channel.unit) : 0;
      const labels = channel.labels?.length ?
          ChannelInfo.createLabelsVector(
              b, channel.labels.map(label => b.createString(label))) :
          0;
      return ChannelInfo.createChannelInfo(
          b, name, unit, labels, channel.kind ?? ChannelKind.Double);
    });
    const dictionary = Frame.createDictionaryVector(b, offsets);
    Frame.startFrame(b);
    Frame.addStreamId(b, header.streamId);
    Frame.addSequenceId(b, header.sequenceId);
    Frame.addTimestampNs(b, header.timestampNs);
    Frame.addDictionaryId(b, header.dictionaryId);
    Frame.addPage(b, page);
    Frame.addPageCount(b, groups.length);
    Frame.addChannelCount(b, header.channelCount);
    Frame.addFirstChannel(b, first);
    Frame.addDictionary(b, dictionary);
    Frame.finishSizePrefixedFrameBuffer(b, Frame.endFrame(b));
    return b.asUint8Array().slice();
  });
}

// Bytes a value page may spend on samples (12 B each) and absent channels
// (4 B each): the 1200-byte page cap less an upper bound on everything else,
// the same bound talOS/telemetry/wire.h sizes the producer's pages with.
const MAX_VALUE_PAGE_BYTES = 1200;
const VALUE_PAGE_OVERHEAD_BOUND = 200;
const PAGE_ROOM = MAX_VALUE_PAGE_BYTES - VALUE_PAGE_OVERHEAD_BOUND;

// One update as value pages: `samples` are [channel, raw] pairs; on a
// keyframe every channel not carried is absent, otherwise `absent` lists the
// channels that lost their sample. Pages fill with samples first, then
// absent channels, as the producer's do (frames.h EncodeValuePages), so no
// page outgrows the cap.
export function encodeValuePages(header: PageHeader, chassis: WirePose,
                                 keyframe: boolean,
                                 samples: [number, bigint][],
                                 absent: number[] = []): Uint8Array[] {
  const spans: {samples: [number, bigint][]; gone: number[]}[] = [];
  let sample = 0, gone = 0;
  do {
    const carried = Math.min(samples.length - sample, Math.floor(PAGE_ROOM / 12));
    const left = PAGE_ROOM - carried * 12;
    const dropped = Math.min(absent.length - gone, Math.floor(left / 4));
    spans.push({samples: samples.slice(sample, sample + carried),
                gone: absent.slice(gone, gone + dropped)});
    sample += carried;
    gone += dropped;
  } while (sample < samples.length || gone < absent.length);
  return spans.map(({samples: carried, gone: lost}, page) => {
    const b = new Builder(1024);
    const channels = carried.length ?
        Frame.createChannelsVector(b, carried.map(([channel]) => channel)) : 0;
    const values = carried.length ?
        Frame.createValuesVector(b, carried.map(([, raw]) => raw)) : 0;
    const absentVector = lost.length ? Frame.createAbsentVector(b, lost) : 0;
    Frame.startFrame(b);
    Frame.addStreamId(b, header.streamId);
    Frame.addSequenceId(b, header.sequenceId);
    Frame.addTimestampNs(b, header.timestampNs);
    Frame.addDictionaryId(b, header.dictionaryId);
    Frame.addChassis(b, Pose.createPose(b, chassis.x, chassis.y, chassis.z,
                                        chassis.roll, chassis.pitch,
                                        chassis.yaw));
    Frame.addPage(b, page);
    Frame.addPageCount(b, spans.length);
    Frame.addChannelCount(b, header.channelCount);
    Frame.addKeyframe(b, keyframe);
    if (channels) Frame.addChannels(b, channels);
    if (values) Frame.addValues(b, values);
    if (absentVector) Frame.addAbsent(b, absentVector);
    Frame.finishSizePrefixedFrameBuffer(b, Frame.endFrame(b));
    return b.asUint8Array().slice();
  });
}

export function concatPages(pages: Uint8Array[]): Uint8Array {
  const out = new Uint8Array(pages.reduce((n, page) => n + page.length, 0));
  let p = 0;
  for (const page of pages) {
    out.set(page, p);
    p += page.length;
  }
  return out;
}
