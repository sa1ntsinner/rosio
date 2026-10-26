"""`python -m rosio.stamps`, the same entry point as `rosio stamps`."""

import sys

from .cli import main

sys.exit(main())
