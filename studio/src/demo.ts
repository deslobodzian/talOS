import {Builder} from 'flatbuffers';

import {Channel} from './generated/talos/telemetry/channel';
import {Frame} from './generated/talos/telemetry/frame';
import {Ghost} from './generated/talos/telemetry/ghost';
import {Pose} from './generated/talos/telemetry/pose';
import {Target} from './generated/talos/telemetry/target';

export function demoPacket(sequence: bigint): Uint8Array {
  const b = new Builder(2048);
  const t = Number(sequence % 100000n) * .02;
  const x = 8 + 5 * Math.cos(t * .35), y = 4 + 2.5 * Math.sin(t * .35),
        yaw = t * .35 + Math.PI / 2;
  const ghosts =
      ['RawOdometry', 'VisionEstimate', 'FusedPose'].map((name, i) => {
        const s = b.createString(name);
        Ghost.startGhost(b);
        Ghost.addName(b, s);
        Ghost.addPose(
            b,
            Pose.createPose(
                b,
                x +
                    (i === 0     ? .15 * Math.sin(t) :
                         i === 1 ? .05 :
                                   0),
                y + (i === 0 ? .12 : 0), 0, 0, 0, yaw));
        return Ghost.endGhost(b);
      });
  const gv = Frame.createGhostsVector(b, ghosts);
  const channels =
      [
        ['velocity', 1.75, 'm/s'], ['heading', yaw, 'rad'],
        ['voltage', 12.4 - .8 * Math.sin(t * .7), 'V'], ['setpoint', 2, 'm/s'],
        ['error', .2 * Math.sin(t * 2), 'm/s'],
        ['vx', -1.75 * Math.sin(t * .35), 'm/s'],
        ['vy', .875 * Math.cos(t * .35), 'm/s']
      ]
          .map(
              ([name, value, unit]) => Channel.createChannel(
                  b, b.createString(String(name)), Number(value),
                  b.createString(String(unit))));
  const cv = Frame.createChannelsVector(b, channels);
  Target.startTarget(b);
  Target.addId(b, 1);
  Target.addPose(b, Pose.createPose(b, 1, 1, 1.4, 0, 0, 0));
  const target = Target.endTarget(b);
  const tv = Frame.createTargetsVector(b, [target]);
  Frame.startFrame(b);
  Frame.addTimestampNs(b, 1000000000n + sequence * 20000000n);
  Frame.addSequenceId(b, sequence);
  Frame.addGhosts(b, gv);
  Frame.addChannels(b, cv);
  Frame.addTargets(b, tv);
  Frame.addElevatorM(b, .6 + .4 * Math.sin(t));
  Frame.addArmRad(b, .5 * Math.sin(t));
  Frame.addCamera(b, Pose.createPose(b, x, y, .7, 0, 0, yaw));
  Frame.addChassis(b, Pose.createPose(b, x, y, .15, 0, 0, yaw));
  Frame.finishSizePrefixedFrameBuffer(b, Frame.endFrame(b));
  return b.asUint8Array();
}
