"""Read the ExtremeDrive keyframe-aligned CAN export without third-party packages."""
import argparse
import collections
import contextlib
import io
import json
from pathlib import Path
import zipfile


def iter_can(archive, episode=None):
    path = Path(archive)
    with contextlib.ExitStack() as stack:
        if path.is_dir():
            stream = stack.enter_context((path / "frames.jsonl").open(encoding="utf-8"))
        elif path.suffix == ".zip":
            bundle = stack.enter_context(zipfile.ZipFile(path))
            stream = stack.enter_context(io.TextIOWrapper(bundle.open("frames.jsonl"), encoding="utf-8"))
        else:
            stream = stack.enter_context(path.open(encoding="utf-8"))
        for line in stream:
            if line.strip():
                row = json.loads(line)
                if episode is None or row["episode"] == episode:
                    yield row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", default=str(Path(__file__).resolve().parents[1] / "assets/can/ExtremeDrive_CAN.zip"))
    parser.add_argument("--episode")
    parser.add_argument("--summary", action="store_true")
    parser.add_argument("--limit", type=int, default=3)
    args = parser.parse_args()
    counts = collections.Counter()
    matched = 0
    for i, row in enumerate(iter_can(args.archive, args.episode)):
        counts[row["episode"]] += 1
        matched += int(row["matched"])
        if not args.summary and i < args.limit:
            print(json.dumps(row, ensure_ascii=False))
    if not counts:
        parser.error("No matching CAN records")
    if args.summary:
        print(json.dumps({"episodes": len(counts), "frames": sum(counts.values()), "matched_frames": matched}, indent=2))


if __name__ == "__main__":
    main()
