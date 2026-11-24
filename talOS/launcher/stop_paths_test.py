import os
import struct
import subprocess
import tempfile
import unittest
import zlib

from python.runfiles import runfiles


def line(ms, process, topic, text):
    """One stop_paths line, in the columns BENCH.md's trials read."""
    return f"{ms:12.3f} {process:<14} {topic:<26} {text}"


_FILE_HDR = struct.Struct("<8sIIQqqII64sQ")
_MANIFEST_ENTRY = struct.Struct("<HHIIIqq64sQII")
_RECORD_HDR = struct.Struct("<IHHQqqqQIIII")
_EVENT_KIND_MESSAGE = 2
_SOURCE_KIND_WATCHER = 2


def _move_ds_sends_to_subscribers(directory, subscribers):
    """Removes /hw/state/driver_station SEND records from hardware_node.tlog
    (matching real hardware_node, which publishes DS via ds_pub_->write without
    logging SEND) and writes subscriber .tlog files holding only MESSAGE
    records for /hw/state/driver_station."""
    bridge_path = os.path.join(directory, "hardware_node.tlog")
    with open(bridge_path, "rb") as f:
        data = f.read()

    (magic, version, hdr_bytes, session_id, start_mono, start_wall,
     manifest_count, _crc, _proc, schema_bytes) = _FILE_HDR.unpack_from(data, 0)
    assert magic == b"TALOSLOG" and hdr_bytes == _FILE_HDR.size

    entries_end = hdr_bytes + manifest_count * _MANIFEST_ENTRY.size
    schema_end = entries_end + schema_bytes

    ds_id = None
    ds_entry = None
    ds_schema = b""
    schema_pos = entries_end
    for i in range(manifest_count):
        off = hdr_bytes + i * _MANIFEST_ENTRY.size
        entry = _MANIFEST_ENTRY.unpack_from(data, off)
        name = entry[7].split(b"\x00", 1)[0]
        slen = entry[9] + entry[10]
        if name == b"/hw/state/driver_station":
            ds_id = entry[0]
            ds_entry = entry
            ds_schema = data[schema_pos:schema_pos + slen]
        schema_pos += slen
    assert ds_id is not None

    kept_records = bytearray()
    ds_records = []
    pos = schema_end
    while pos < len(data):
        hdr = _RECORD_HDR.unpack_from(data, pos)
        stored = hdr[11]
        rec_end = pos + _RECORD_HDR.size + stored
        payload = data[pos + _RECORD_HDR.size:rec_end]
        if hdr[2] == ds_id:
            ds_records.append((hdr, payload))
        else:
            kept_records += data[pos:rec_end]
        pos = rec_end
    assert ds_records

    with open(bridge_path, "wb") as f:
        f.write(data[:schema_end] + kept_records)

    sub_entry = _MANIFEST_ENTRY.pack(
        0, _SOURCE_KIND_WATCHER, *ds_entry[2:])
    sub_crc = zlib.crc32(sub_entry + ds_schema) & 0xFFFFFFFF
    for process_name, delay_ns in subscribers:
        proc_bytes = process_name.encode("ascii").ljust(64, b"\x00")
        sub_hdr = _FILE_HDR.pack(
            magic, version, hdr_bytes, session_id, start_mono, start_wall,
            1, sub_crc, proc_bytes, len(ds_schema))
        out = bytearray(sub_hdr + sub_entry + ds_schema)
        for idx, (hdr, payload) in enumerate(ds_records, start=1):
            pub_ns = hdr[6]
            recv_ns = pub_ns + delay_ns
            rec_zero = _RECORD_HDR.pack(
                hdr[0], _EVENT_KIND_MESSAGE, 0, idx, recv_ns, recv_ns,
                pub_ns, hdr[7], hdr[8], hdr[9], 0, hdr[11])
            rec_crc = zlib.crc32(payload, zlib.crc32(rec_zero)) & 0xFFFFFFFF
            rec_hdr = _RECORD_HDR.pack(
                hdr[0], _EVENT_KIND_MESSAGE, 0, idx, recv_ns, recv_ns,
                pub_ns, hdr[7], hdr[8], hdr[9], rec_crc, hdr[11])
            out += rec_hdr + payload
        with open(os.path.join(directory, f"{process_name}.tlog"), "wb") as f:
            f.write(out)


class StopPathsTest(unittest.TestCase):
    def run_tool(self, mode=None):
        files = runfiles.Create()
        fixture = files.Rlocation("_main/talOS/launcher/stop_paths_fixture")
        binary = files.Rlocation("_main/talOS/launcher/stop_paths")
        with tempfile.TemporaryDirectory() as directory:
            subprocess.run([fixture, directory] + ([mode] if mode else []), check=True)
            return subprocess.run([binary, directory], capture_output=True, text=True)

    # REQ-SAFE-07, REQ-SAFE-02 (bench evidence, docs/BENCH.md "Reading a
    # trial"): a recorded session of stop path 2 (teleop stopped 300 ms,
    # stick held) and stop path 1 (a Driver Station disable) reads as the
    # lines each trial's Pass column measures between: the lane's PAUSE and
    # the first neutral request, the disabling packet and State, the first
    # State at 0 V; and a log's lost records read as a LOG GAP, with the
    # PAUSE it causes, not as the robot's silence.
    def test_recorded_stop_paths(self):
        result = self.run_tool()
        self.assertEqual(result.returncode, 0, result.stderr)
        lines = result.stdout.splitlines()
        lane = "/motor/target/teleop"
        request = "/hw/request/motor"
        expected = [
            # Stop path 2: 150 - 50 = 100 ms from the PAUSE's start (the
            # lane's last message) to the first neutral request.
            line(155, "motor", request,
                 "observed_us=1155000 kraken=neutral 0 (previous value last at 150.000)"),
            line(156, "hardware_node", "/hw/command",
                 "observed_us=1155000 kraken=neutral 0 (previous value last at 151.000)"),
            line(350, "teleop", lane, "PAUSE 300.0 ms after 50.000"),
            line(350, "teleop", lane,
                 "{mode_since_us: 900000, active: false, message: "
                 "{run_index: 1, sequence_count: 2}} (previous value last at 50.000)"),
            # Stop path 1: the disabling packet, the first State with
            # enabled=False (already inactive), the first at 0 V.
            line(400, "hardware_node", "/hw/state/driver_station",
                 "enabled=False estop=False mode=teleop (previous value last at 0.000)"),
            line(400, "hardware_node", "/hw/state",
                 "sample_us=1400000 enabled=False active=False epoch=1 since_us=1400000 "
                 "kraken_v=1 kraken_rps=10 (previous value last at 395.000)"),
            line(410, "hardware_node", "/hw/state",
                 "sample_us=1410000 enabled=False active=False epoch=1 since_us=1410000 "
                 "kraken_v=0 kraken_rps=0 (previous value last at 405.000)"),
            # The motor log's lost requests, then the 45 ms they leave.
            f"{200:12.3f} {'motor':<14} LOG GAP to 240.000: 8 records lost (pool_full)",
            line(240, "motor", request, "PAUSE 45.0 ms after 195.000"),
        ]
        for want in expected:
            with self.subTest(line=want):
                self.assertIn(want, lines)
        # Only the gap and the stopped lane pause: no stream ticking every
        # 5 ms reads a PAUSE, and nothing is UNDECODED.
        self.assertEqual(sum("PAUSE" in text for text in lines), 2, result.stdout)
        self.assertNotIn("UNDECODED", result.stdout)

    # REQ-OPS-51: a session of another hardware protocol is refused, not read
    # with this build's layout.
    def test_refuses_undecoded_hardware_packets(self):
        result = self.run_tool("foreign-protocol")
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertEqual(result.stdout, "")
        self.assertIn("stop_paths: hardware packets not decoded", result.stderr)

    # REQ-SAFE-02, REQ-SAFE-07, REQ-OPS-21: hardware_node publishes
    # /hw/state/driver_station via ds_pub_->write(pkt) with zero SEND records
    # logged; real sessions contain only subscriber MESSAGE records. stop_paths
    # must select one subscriber's MESSAGE stream (never pooling duplicate
    # readers) so the Driver Station disable event appears in Stop Path 1.
    def test_reads_driver_station_from_one_subscriber_when_no_send_logged(self):
        files = runfiles.Create()
        fixture = files.Rlocation("_main/talOS/launcher/stop_paths_fixture")
        binary = files.Rlocation("_main/talOS/launcher/stop_paths")
        with tempfile.TemporaryDirectory() as directory:
            subprocess.run([fixture, directory], check=True)
            _move_ds_sends_to_subscribers(
                directory,
                [("teleop_sub", 2_000_000), ("motor_sub", 3_000_000)],
            )
            result = subprocess.run([binary, directory], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        lines = result.stdout.splitlines()
        # Both subscribers received the same 2 DS packets; session_report.stream
        # tie-breaks on process name ("teleop_sub" > "motor_sub") and emits only
        # that one reader's stream (never pooling duplicate readers).
        ds_lines = [text for text in lines if "/hw/state/driver_station" in text]
        self.assertEqual(
            ds_lines,
            [
                line(0, "teleop_sub", "/hw/state/driver_station",
                     "enabled=True estop=False mode=teleop"),
                line(400, "teleop_sub", "/hw/state/driver_station",
                     "enabled=False estop=False mode=teleop (previous value last at 0.000)"),
            ],
            result.stdout,
        )


if __name__ == "__main__":
    unittest.main()
