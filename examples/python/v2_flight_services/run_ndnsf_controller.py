#!/usr/bin/env python3
"""Run the miniMUAS v2 NDNSF ServiceController."""

from __future__ import annotations

import argparse

from ndnsf_runtime import (
    DEFAULT_POLICY,
    add_common_arguments,
    add_ndnsf_path,
    controller_kwargs,
    flush_json_log,
    optional_local_nfd,
    print_json,
    start_nfd_counter_scrape,
    start_role_journal,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the miniMUAS v2 controller")
    add_common_arguments(parser)
    parser.add_argument("--policy", default=DEFAULT_POLICY)
    parser.add_argument(
        "--bootstrap-identity",
        action="append",
        # Full 4-node fleet: gcs + wuas-01 + iuas-01 (camera) + iuas-02 (mic).
        # The controller must bootstrap EVERY vehicle identity or an omitted
        # node's provider never gets its ABE/policy set up (silent decrypt
        # failures). iuas-02 was added with the mic airframe; keep this in
        # sync with the deployment's fleetIds.
        default=[
            "/muas/v2/gcs",
            "/muas/v2/wuas-01",
            "/muas/v2/iuas-01",
            "/muas/v2/iuas-02",
        ],
    )
    parser.add_argument(
        "--log-dir",
        default="/var/lib/minimuas/log",
        help="Directory for the fsync-per-line metrics/event journal "
        "(empty string disables).",
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

    start_role_journal("ndnsf-controller", args.log_dir)
    start_nfd_counter_scrape(args.nfd_metrics_interval, enabled=args.nfd_metrics)

    add_ndnsf_path(args.ndnsf_root)
    from ndnsf import ServiceController

    try:
        with optional_local_nfd(args.start_local_nfd):
            controller = ServiceController(**controller_kwargs(args))
            print_json("ndnsf.controller.starting", controller=args.controller)
            return controller.run()
    finally:
        flush_json_log()


if __name__ == "__main__":
    raise SystemExit(main())
