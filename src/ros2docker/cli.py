"""Command line interface for ros2docker."""

from __future__ import annotations

import argparse
import sys

from . import __version__
from .api import build, build_run, exec_shell, run, stop


def main(argv: list[str] | None = None) -> int:
    parser = _make_parser()
    args = parser.parse_args(argv)

    if not hasattr(args, "func"):
        parser.print_help()
        return 0

    try:
        args.func(args)
    except Exception as exc:  # noqa: BLE001 - CLI should report concise user errors.
        print(f"ros2docker: error: {exc}", file=sys.stderr)
        return 1
    return 0


def _make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="ros2docker")
    parser.add_argument("--version", action="version", version=f"ros2docker {__version__}")
    subparsers = parser.add_subparsers(dest="command_name")

    build_parser = subparsers.add_parser("build", help="Build the configured Docker image.")
    _add_config_options(build_parser)
    _add_dry_run(build_parser)
    build_parser.set_defaults(func=_build)

    run_parser = subparsers.add_parser("run", help="Build and run the configured Docker container.")
    _add_config_options(run_parser)
    _add_dry_run(run_parser)
    run_parser.add_argument("-m", "--mount", metavar="PATH", help="Mount PATH into /ws.")
    run_parser.add_argument("--no-build", action="store_true", help="Run without building first.")
    run_parser.add_argument("extra_run_args", nargs=argparse.REMAINDER, help="Extra docker run args after --.")
    run_parser.set_defaults(func=_run)

    stop_parser = subparsers.add_parser("stop", help="Stop the configured Docker container.")
    _add_config_options(stop_parser)
    _add_dry_run(stop_parser)
    stop_parser.set_defaults(func=_stop)

    exec_parser = subparsers.add_parser("exec", help="Execute a command in the configured Docker container.")
    _add_config_options(exec_parser)
    _add_dry_run(exec_parser)
    exec_parser.add_argument("command", nargs=argparse.REMAINDER, help="Command after --, defaults to bash.")
    exec_parser.set_defaults(func=_exec)

    return parser


def _add_config_options(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("-f", "--config", required=True, metavar="CONFIG", help="Path to ros2docker config.")
    parser.add_argument("-o", "--override", metavar="JSON", help="JSON object overriding config values.")


def _add_dry_run(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--dry-run", action="store_true", help="Print Docker argv without running Docker.")


def _strip_separator(args: list[str]) -> list[str]:
    if args and args[0] == "--":
        return args[1:]
    return args


def _build(args: argparse.Namespace) -> None:
    build(args.config, args.override, dry_run=args.dry_run)


def _run(args: argparse.Namespace) -> None:
    extra_run_args = _strip_separator(args.extra_run_args)
    if args.no_build:
        run(
            args.config,
            args.override,
            mount=args.mount,
            extra_run_args=extra_run_args,
            dry_run=args.dry_run,
        )
    else:
        build_run(
            args.config,
            args.override,
            mount=args.mount,
            extra_run_args=extra_run_args,
            dry_run=args.dry_run,
        )


def _stop(args: argparse.Namespace) -> None:
    stop(args.config, args.override, dry_run=args.dry_run)


def _exec(args: argparse.Namespace) -> None:
    command = _strip_separator(args.command)
    exec_shell(
        args.config,
        args.override,
        command=command or None,
        interactive=not command,
        dry_run=args.dry_run,
    )


if __name__ == "__main__":
    raise SystemExit(main())

