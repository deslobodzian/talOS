"""Tests for tools/run_rio_clang_tidy.py."""

import json
import os
import pathlib
import stat
import tempfile
import unittest

from tools import run_rio_clang_tidy as rio_tidy


def _write_fake_tidy(directory, body):
    path = directory / 'fake-clang-tidy'
    path.write_text('#!/bin/sh\n' + body, encoding='utf-8')
    path.chmod(path.stat().st_mode | stat.S_IXUSR)
    return str(path)


class SanitizeArgumentsTest(unittest.TestCase):
    def test_strips_nostdinc_and_host_gcc_isystems_only(self):
        raw = [
            '/usr/bin/g++',
            '-std=c++20',
            '-nostdinc',
            '-nostdinc++',
            '-I', '/repo/controller_processor/rio/src/main/include',
            '-I', '/repo/talOS',
            '-isystem', '/home/user/.gradle/caches/wpilib/include',
            '-isystem', '/usr/include/c++/16',
            '-isystem', '/usr/include/x86_64-linux-gnu',
            '-isystem', '/usr/lib/gcc/x86_64-pc-linux-gnu/16/include',
            '-isystem', '/usr/include',
            '-isystem/usr/local/include',
            '-isystem/home/user/.gradle/caches/phoenix/include',
            'Robot.cpp',
        ]
        got = rio_tidy.sanitize_arguments(raw, gcc_install_dir='/opt/gcc14')
        self.assertEqual(
            got,
            [
                '/usr/bin/g++',
                '-std=c++20',
                '-I', '/repo/controller_processor/rio/src/main/include',
                '-I', '/repo/talOS',
                '-isystem', '/home/user/.gradle/caches/wpilib/include',
                '-isystem/home/user/.gradle/caches/phoenix/include',
                'Robot.cpp',
                '--gcc-install-dir=/opt/gcc14',
            ],
        )

    def test_sanitize_database_converts_command_string(self):
        entries = [{
            'directory': '/repo/controller_processor/rio',
            'file': '/repo/controller_processor/rio/src/main/cpp/Robot.cpp',
            'command': '/usr/bin/g++ -nostdinc -isystem /usr/include -I /repo/talOS Robot.cpp',
        }]
        cleaned = rio_tidy.sanitize_database(entries)
        self.assertEqual(len(cleaned), 1)
        self.assertNotIn('command', cleaned[0])
        self.assertEqual(
            cleaned[0]['arguments'],
            ['/usr/bin/g++', '-I', '/repo/talOS', 'Robot.cpp'],
        )


class MainFailClosedAndObserveTest(unittest.TestCase):
    def _make_fixture(self, tmp_path, sources=('A.cpp', 'B.cpp'), db_sources=None):
        src_dir = tmp_path / 'src'
        src_dir.mkdir()
        for name in sources:
            (src_dir / name).write_text('int x = 0;\n', encoding='utf-8')
        if db_sources is None:
            db_sources = sources
        db = tmp_path / 'compile_commands.json'
        entries = [
            {
                'directory': str(tmp_path),
                'file': str(src_dir / name),
                'arguments': ['/usr/bin/g++', '-nostdinc', '-isystem', '/usr/include', str(src_dir / name)],
            }
            for name in db_sources
        ]
        db.write_text(json.dumps(entries), encoding='utf-8')
        return src_dir, db

    def test_missing_database_fails_closed_even_under_observe(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = pathlib.Path(tmp)
            src_dir = tmp_path / 'src'
            src_dir.mkdir()
            (src_dir / 'A.cpp').write_text('', encoding='utf-8')
            fake = _write_fake_tidy(tmp_path, 'exit 0\n')
            rc = rio_tidy.main([
                '--db', str(tmp_path / 'missing.json'),
                '--src-dir', str(src_dir),
                '--tidy-bin', fake,
                '--observe',
            ])
            self.assertEqual(rc, 2)

    def test_missing_source_in_db_fails_closed_even_under_observe(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = pathlib.Path(tmp)
            src_dir, db = self._make_fixture(
                tmp_path, sources=('A.cpp', 'B.cpp'), db_sources=('A.cpp',)
            )
            fake = _write_fake_tidy(tmp_path, 'exit 0\n')
            rc = rio_tidy.main([
                '--db', str(db),
                '--src-dir', str(src_dir),
                '--tidy-bin', fake,
                '--observe',
            ])
            self.assertEqual(rc, 2)

    def test_findings_fail_strict_and_pass_observe(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = pathlib.Path(tmp)
            src_dir, db = self._make_fixture(tmp_path, sources=('A.cpp',))
            fake = _write_fake_tidy(
                tmp_path,
                'echo "A.cpp:1:1: error: sample [bugprone-empty-catch]"\nexit 1\n',
            )
            self.assertEqual(
                rio_tidy.main([
                    '--db', str(db), '--src-dir', str(src_dir),
                    '--tidy-bin', fake,
                ]),
                1,
            )
            self.assertEqual(
                rio_tidy.main([
                    '--db', str(db), '--src-dir', str(src_dir),
                    '--tidy-bin', fake, '--observe',
                ]),
                0,
            )

    def test_compiler_parse_error_fails_closed_even_under_observe(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = pathlib.Path(tmp)
            src_dir, db = self._make_fixture(tmp_path, sources=('A.cpp',))
            fake = _write_fake_tidy(
                tmp_path,
                'echo "xmmintrin.h:54:1: error: bad [clang-diagnostic-error]"\n'
                'echo "Found compiler error(s)."\nexit 1\n',
            )
            self.assertEqual(
                rio_tidy.main([
                    '--db', str(db), '--src-dir', str(src_dir),
                    '--tidy-bin', fake, '--observe',
                ]),
                2,
            )

    def test_clean_run_passes_sanitized_db_and_exits_zero(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = pathlib.Path(tmp)
            src_dir, db = self._make_fixture(tmp_path, sources=('A.cpp',))
            seen = tmp_path / 'seen.json'
            fake = _write_fake_tidy(
                tmp_path,
                f'cp "$2/compile_commands.json" "{seen}"\nexit 0\n',
            )
            rc = rio_tidy.main([
                '--db', str(db), '--src-dir', str(src_dir),
                '--tidy-bin', fake,
            ])
            self.assertEqual(rc, 0)
            sanitized = json.loads(seen.read_text(encoding='utf-8'))
            self.assertNotIn('-nostdinc', sanitized[0]['arguments'])
            self.assertNotIn('/usr/include', sanitized[0]['arguments'])


if __name__ == '__main__':
    unittest.main()
