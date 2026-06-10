#!/usr/bin/env python3
"""Run the miniMUAS v2 NDNSF ServiceController."""

from __future__ import annotations

import argparse

from ndnsf_runtime import (
    DEFAULT_POLICY,
    add_common_arguments,
    add_ndnsf_path,
    controller_kwargs,
    optional_local_nfd,
    print_json,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the miniMUAS v2 controller")
    add_common_arguments(parser)
    parser.add_argument("--policy", default=DEFAULT_POLICY)
    parser.add_argument(
        "--bootstrap-identity",
        action="append",
        default=["/muas/v2/gcs", "/muas/v2/wuas-01", "/muas/v2/iuas-01"],
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.dry_run:
        print_json(
            "ndnsf.controller.dry_run",
            controller=args.controller,
            policy=str(args.policy),
            trust_schema=str(args.trust_schema),
            bootstrap_identities=args.bootstrap_identity,
        )
        return 0

    add_ndnsf_path(args.ndnsf_root)
    from ndnsf import ServiceController

    with optional_local_nfd(args.start_local_nfd):
        controller = ServiceController(**controller_kwargs(args))
        print_json("ndnsf.controller.starting", controller=args.controller)
        return controller.run()


if __name__ == "__main__":
    raise SystemExit(main())
