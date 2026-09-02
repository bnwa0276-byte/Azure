from __future__ import annotations

"""Export analytics results to CSV and JSON formats."""

import json
import csv
from typing import Mapping, Any


def export_stats_csv(data: Mapping[str, Any], fp) -> None:
    close_when_done = False
    if isinstance(fp, str):
        f = open(fp, "w", newline="")
        close_when_done = True
    else:
        f = fp

    writer = csv.writer(f)
    # header
    writer.writerow(["metric", "value"])
    for k, v in data.items():
        writer.writerow([k, v])

    if close_when_done:
        f.close()


def export_stats_json(data: Mapping[str, Any], fp) -> None:
    close_when_done = False
    if isinstance(fp, str):
        f = open(fp, "w")
        close_when_done = True
    else:
        f = fp

    json.dump(data, f)

    if close_when_done:
        f.close()
