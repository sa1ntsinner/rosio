"""Download the corpus entries listed in manifest.yaml. Standard library only."""

import argparse
import sys
import urllib.error
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
MANIFEST = HERE / "manifest.yaml"
DEFAULT_DEST = HERE / "data"
HF_PATTERNS = ["*.bag", "*.mcap", "*.db3", "*.yaml"]
CHUNK = 1 << 16
PROGRESS_STEP = 8 << 20
TIERS = ("control", "buggy", "noise", "synthetic")


def load_manifest(path=MANIFEST):
    text = Path(path).read_text()
    try:
        import yaml

        return yaml.safe_load(text)["entries"]
    except ImportError:
        pass
    try:
        from ruamel.yaml import YAML

        return YAML(typ="safe").load(text)["entries"]
    except ImportError:
        return _mini_yaml(text)["entries"]


def _mini_yaml(text):
    lines = []
    for raw in text.splitlines():
        body = raw.split(" #")[0].rstrip()
        stripped = body.strip()
        if not stripped or stripped.startswith("#"):
            continue
        lines.append((len(body) - len(body.lstrip()), stripped))
    value, _ = _parse_map(lines, 0, 0)
    return value


def _parse_map(lines, i, indent):
    out = {}
    while i < len(lines) and lines[i][0] >= indent:
        key, _, rest = lines[i][1].partition(":")
        rest = rest.strip()
        i += 1
        if rest in (">", "|"):
            out[key.strip()], i = _parse_block_scalar(lines, i, indent, rest)
        elif rest:
            out[key.strip()] = _scalar(rest)
        elif i < len(lines) and lines[i][0] > indent:
            out[key.strip()], i = _parse_value(lines, i, lines[i][0])
        else:
            out[key.strip()] = None
    return out, i


def _parse_value(lines, i, indent):
    if lines[i][1].startswith("- "):
        return _parse_list(lines, i, indent)
    return _parse_map(lines, i, indent)


def _parse_list(lines, i, indent):
    items = []
    while i < len(lines) and lines[i][0] == indent and lines[i][1].startswith("- "):
        rest = lines[i][1][2:].strip()
        if ":" in rest:
            inner = [(indent + 2, rest), *lines[i + 1 :]]
            value, consumed = _parse_map(inner, 0, indent + 2)
            items.append(value)
            i += consumed
        else:
            items.append(_scalar(rest))
            i += 1
    return items, i


def _parse_block_scalar(lines, i, indent, style):
    parts = []
    while i < len(lines) and lines[i][0] > indent:
        parts.append(lines[i][1])
        i += 1
    return ("\n" if style == "|" else " ").join(parts) + "\n", i


def _scalar(text):
    if len(text) > 1 and text[0] == text[-1] and text[0] in "\"'":
        return text[1:-1]
    low = text.lower()
    if low in ("true", "false"):
        return low == "true"
    if low in ("null", "~"):
        return None
    for cast in (int, float):
        try:
            return cast(text)
        except ValueError:
            pass
    return text


def select(entries, names, tier):
    chosen = entries
    if tier:
        chosen = [e for e in chosen if e.get("tier") == tier]
    if names:
        wanted = set(names)
        chosen = [e for e in chosen if e["name"] in wanted]
        missing = wanted - {e["name"] for e in entries}
        if missing:
            raise SystemExit("unknown entries: {}".format(", ".join(sorted(missing))))
    return chosen


def size_hint(entry):
    if entry.get("size_gb"):
        return "~{} GB".format(entry["size_gb"])
    if entry.get("size_mb"):
        return "~{} MB".format(entry["size_mb"])
    return "size unknown"


def print_list(entries):
    for entry in entries:
        print(
            "{:<28} {:<8} {:<8} {:<12} {}".format(
                entry["name"],
                entry.get("tier", ""),
                entry.get("format", ""),
                entry.get("fetch", ""),
                size_hint(entry),
            )
        )


def target_url(entry):
    url = entry.get("url")
    if url and entry.get("fetch") == "dropbox":
        return url.replace("?dl=0", "?dl=1")
    return url


def fetch_http(entry, dest):
    dest.mkdir(parents=True, exist_ok=True)
    url = target_url(entry)
    name = url.split("?")[0].rstrip("/").rsplit("/", 1)[-1] or entry["name"]
    final = dest / name
    if final.exists():
        print("{}: already have {}".format(entry["name"], final))
        return final
    part = final.parent / (final.name + ".part")
    have = part.stat().st_size if part.exists() else 0
    request = urllib.request.Request(url)
    if have:
        request.add_header("Range", f"bytes={have}-")
    try:
        response = urllib.request.urlopen(request)
    except urllib.error.URLError as exc:
        print("{}: download failed, {}".format(entry["name"], exc))
        return None
    resuming = response.status == 206
    if have and not resuming:
        have = 0
    total = _content_length(response, have)
    mode = "ab" if resuming and have else "wb"
    with response, part.open(mode) as handle:
        done = have if resuming else 0
        shown = 0
        while True:
            chunk = response.read(CHUNK)
            if not chunk:
                break
            handle.write(chunk)
            done += len(chunk)
            if done - shown >= PROGRESS_STEP:
                shown = done
                _progress(entry["name"], done, total)
    _progress(entry["name"], done, total)
    print("")
    part.replace(final)
    return final


def _content_length(response, have):
    length = response.headers.get("Content-Length")
    if length is None:
        return None
    return int(length) + (have if response.status == 206 else 0)


def _progress(name, done, total):
    mb = done / 1e6
    if total:
        sys.stdout.write(f"\r{name}: {mb:.1f} MB of {total / 1e6:.1f} MB")
    else:
        sys.stdout.write(f"\r{name}: {mb:.1f} MB")
    sys.stdout.flush()


def fetch_huggingface(entry, dest):
    try:
        from huggingface_hub import snapshot_download
    except ImportError:
        print(
            "{}: needs the huggingface_hub package, "
            "install it with: pip install huggingface_hub".format(entry["name"])
        )
        return None
    dest.mkdir(parents=True, exist_ok=True)
    return Path(
        snapshot_download(
            repo_id=entry["hf_repo"],
            repo_type="dataset",
            local_dir=str(dest),
            allow_patterns=entry.get("hf_patterns") or HF_PATTERNS,
        )
    )


def skip_unresolved(entry):
    print("{}: no direct link in the manifest, resolve it by hand".format(entry["name"]))
    if entry.get("page"):
        print("  page: {}".format(entry["page"]))
    if entry.get("fetch_note"):
        print("  note: {}".format(entry["fetch_note"]))


def fetch_entry(entry, root):
    kind = entry.get("fetch")
    dest = root / entry["name"]
    if kind == "generate":
        print("{}: generated locally, run run.py".format(entry["name"]))
        return None
    if kind == "huggingface" and entry.get("hf_repo"):
        return fetch_huggingface(entry, dest)
    if kind in ("http", "dropbox") and entry.get("url"):
        return fetch_http(entry, dest)
    skip_unresolved(entry)
    return None


def main(argv=None):
    parser = argparse.ArgumentParser(description="fetch the public bag corpus")
    parser.add_argument("--manifest", default=str(MANIFEST))
    parser.add_argument("--dest", default=str(DEFAULT_DEST))
    parser.add_argument("--names", nargs="+", metavar="NAME")
    parser.add_argument("--tier", choices=TIERS)
    parser.add_argument("--list", action="store_true", help="print the selection and exit")
    args = parser.parse_args(argv)

    entries = select(load_manifest(args.manifest), args.names, args.tier)
    if args.list:
        print_list(entries)
        return 0

    root = Path(args.dest).expanduser()
    root.mkdir(parents=True, exist_ok=True)
    for entry in entries:
        fetch_entry(entry, root)
    return 0


if __name__ == "__main__":
    sys.exit(main())
