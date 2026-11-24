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
    def test_strips_nostdinc_and_host_gcc_isystems_across_arch_and_ubuntu(self):
        raw = [
            '/usr/bin/g++',
            '-std=c++20',
            '-nostdinc',
            '-nostdinc++',
            '-I', '/repo/controller_processor/rio/src/main/include',
            '-I', '/repo/talOS',
            '-isystem', '/home/user/.gradle/caches/wpilib/include',
            '-isystem', '/usr/include/c++/13',
            '-isystem', '/usr/include/x86_64-linux-gnu/c++/13',
            '-isystem', '/usr/include/c++/16',
            '-isystem', '/usr/include/x86_64-linux-gnu',
            '-isystem', '/usr/lib/gcc/x86_64-linux-gnu/13/include',
            '-isystem', '/usr/lib/gcc/x86_64-linux-gnu/13/include-fixed',
            '-isystem', '/usr/lib/gcc/x86_64-pc-linux-gnu/16/include',
            '-isystem', '/usr/include',
            '-isystem/usr/local/include',
            '-isystem/usr/include/x86_64-linux-gnu/c++/14',
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

    def test_sanitize_database_converts_command_and_resolves_symlinks(self):
        with tempfile.TemporaryDirectory() as tmp:
            real_dir = pathlib.Path(tmp) / 'real'
            real_dir.mkdir()
            src = real_dir / 'Robot.cpp'
            src.write_text('int main() { return 0; }\n', encoding='utf-8')
            link_dir = pathlib.Path(tmp) / 'link'
            link_dir.symlink_to(real_dir)
            entries = [{
                'directory': str(link_dir),
                'file': 'Robot.cpp',
                'command': '/usr/bin/g++ -nostdinc -isystem /usr/include -I /repo/talOS Robot.cpp',
            }]
            cleaned = rio_tidy.sanitize_database(entries)
            self.assertEqual(len(cleaned), 1)
            self.assertNotIn('command', cleaned[0])
            self.assertEqual(cleaned[0]['file'], str(src.resolve()))
            self.assertEqual(
                cleaned[0]['arguments'],
                ['/usr/bin/g++', '-I', '/repo/talOS', 'Robot.cpp'],
            )


class MainFailClosedAndObserveTest(unittest.TestCase):
    def _make_fixture(self, tmp_path, sources=('A.cpp', 'B.cpp'), db_sources=None):
        src_dir = tmp_path / 'src'
        src_dir.mkdir()
        for rel in sources:
            path = src_dir / rel
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('int x = 0;\n', encoding='utf-8')
        if db_sources is None:
            db_sources = sources
        db = tmp_path / 'compile_commands.json'
        entries = [
            {
                'directory': str(tmp_path),
                'file': str(src_dir / rel),
                'arguments': ['/usr/bin/g++', '-nostdinc', '-isystem', '/usr/include', str(src_dir / rel)],
            }
            for rel in db_sources
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

    def test_nested_source_missing_from_db_fails_closed_even_under_observe(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = pathlib.Path(tmp)
            src_dir, db = self._make_fixture(
                tmp_path,
                sources=('A.cpp', 'sub/Nested.cpp'),
                db_sources=('A.cpp',),
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

    def test_tool_and_parse_errors_fail_closed_even_under_observe(self):
        cases = {
            'compiler_error': (
                'echo "xmmintrin.h:54:1: error: bad [clang-diagnostic-error]"\n'
                'echo "Found compiler error(s)."\nexit 1\n'
            ),
            'yaml_config_error': (
                'echo "Error parsing .clang-tidy: YAML:1:1: error"\nexit 1\n'
            ),
            'exit_code_2': (
                'echo "A.cpp:1:1: error: sample [bugprone-empty-catch]"\nexit 2\n'
            ),
            'empty_crash_output': 'exit 1\n',
            'signal_kill': 'kill -9 $$\n',
            'fatal_error_with_lint_warning': (
                'echo "A.cpp:1:1: warning: sample [bugprone-empty-catch]"\n'
                'echo "A.cpp:2:10: fatal error: too many errors emitted"\n'
                'exit 1\n'
            ),
            'driver_error_with_lint_warning': (
                'echo "error: invalid value in -std=c++99"\n'
                'echo "A.cpp:1:1: warning: sample [bugprone-empty-catch]"\n'
                'exit 1\n'
            ),
        }
        for label, body in cases.items():
            with self.subTest(label=label), tempfile.TemporaryDirectory() as tmp:
                tmp_path = pathlib.Path(tmp)
                src_dir, db = self._make_fixture(tmp_path, sources=('A.cpp',))
                fake = _write_fake_tidy(tmp_path, body)
                rc = rio_tidy.main([
                    '--db', str(db), '--src-dir', str(src_dir),
                    '--tidy-bin', fake, '--observe',
                ])
                self.assertEqual(rc, 2)

    def test_clean_run_passes_header_filter_and_sanitized_db(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = pathlib.Path(tmp)
            src_dir, db = self._make_fixture(
                tmp_path, sources=('A.cpp', 'sub/Nested.cpp')
            )
            seen_db = tmp_path / 'seen.json'
            seen_argv = tmp_path / 'argv.txt'
            fake = _write_fake_tidy(
                tmp_path,
                f'printf "%s\\n" "$@" >> "{seen_argv}"\n'
                f'cp "$2/compile_commands.json" "{seen_db}"\n'
                'exit 0\n',
            )
            rc = rio_tidy.main([
                '--db', str(db), '--src-dir', str(src_dir),
                '--tidy-bin', fake,
            ])
            self.assertEqual(rc, 0)
            sanitized = json.loads(seen_db.read_text(encoding='utf-8'))
            self.assertNotIn('-nostdinc', sanitized[0]['arguments'])
            self.assertNotIn('/usr/include', sanitized[0]['arguments'])
            argv_text = seen_argv.read_text(encoding='utf-8')
            self.assertIn(
                f'--header-filter={rio_tidy.DEFAULT_HEADER_FILTER}', argv_text
            )
            self.assertIn('Nested.cpp', argv_text)

    # REQ-DEV-25
    def test_exit_zero_tool_errors_and_warnings_are_not_ignored(self):
        # Broken .clang-tidy YAML or driver errors often exit 0 after falling
        # back to defaults; they must still fail closed (exit 2) even under
        # --observe.
        for label, body in (
            (
                'yaml_error_exit_0',
                'echo "Error parsing .clang-tidy: YAML:1:1: error: bad"\nexit 0\n',
            ),
            (
                'driver_error_exit_0',
                'echo "error: invalid value in -std=c++99"\nexit 0\n',
            ),
            (
                'yaml_error_plus_warning_exit_0',
                'echo "Error parsing .clang-tidy: YAML:1:1: error: bad"\n'
                'echo "A.cpp:1:1: warning: empty catch [bugprone-empty-catch]"\n'
                'exit 0\n',
            ),
        ):
            with self.subTest(label=label), tempfile.TemporaryDirectory() as tmp:
                tmp_path = pathlib.Path(tmp)
                src_dir, db = self._make_fixture(tmp_path, sources=('A.cpp',))
                fake = _write_fake_tidy(tmp_path, body)
                self.assertEqual(
                    rio_tidy.main([
                        '--db', str(db), '--src-dir', str(src_dir),
                        '--tidy-bin', fake,
                    ]),
                    2,
                )
                self.assertEqual(
                    rio_tidy.main([
                        '--db', str(db), '--src-dir', str(src_dir),
                        '--tidy-bin', fake, '--observe',
                    ]),
                    2,
                )

        # A lint warning printed while exiting 0 fails strict (exit 1) and
        # passes --observe (exit 0).
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = pathlib.Path(tmp)
            src_dir, db = self._make_fixture(tmp_path, sources=('A.cpp',))
            fake = _write_fake_tidy(
                tmp_path,
                'echo "A.cpp:1:1: warning: empty catch [bugprone-empty-catch]"\n'
                'exit 0\n',
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

        # Benign suppressed-warnings summary with exit 0 stays clean.
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = pathlib.Path(tmp)
            src_dir, db = self._make_fixture(tmp_path, sources=('A.cpp',))
            fake = _write_fake_tidy(
                tmp_path,
                'echo "12 warnings generated."\n'
                'echo "Suppressed 12 warnings (12 in non-user code)."\n'
                'exit 0\n',
            )
            self.assertEqual(
                rio_tidy.main([
                    '--db', str(db), '--src-dir', str(src_dir),
                    '--tidy-bin', fake,
                ]),
                0,
            )


if __name__ == '__main__':
    unittest.main()
