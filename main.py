"""V2 entrypoint (compat wrapper for app runtime)."""

from __future__ import annotations

import asyncio

from .app.runtime import run_async


def main() -> None:
    asyncio.run(run_async())


if __name__ == "__main__":
    main()

