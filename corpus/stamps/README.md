# Public bag corpus

A list of publicly available robot recordings that `rosio stamps` is validated
against, plus a fetcher and a runner. The recordings themselves are never
committed. `fetch.py` pulls them from the original hosts into `corpus/stamps/data/`,
which is git ignored.

## Tiers

- `control` clean recordings. `rosio stamps` must stay quiet on them, so they catch
  false positives.
- `buggy` recordings with a publicly documented timestamp defect. The manifest
  records what `rosio stamps` has to find, so they catch regressions in the checks.
- `noise` unlabelled real world data. No expectations, only used to make sure
  the reader survives odd bags.
- `synthetic` positive controls built offline. Nothing is downloaded. The entry
  carries a `generate` command list instead of a url, `run.py` executes it into
  `corpus/stamps/data/` the first time the entry is missing. `synthesize.py` writes a
  clean three topic bag, `/scan` at 10 Hz, `/imu` at 50 Hz and `/gps` at 5 Hz on
  one shared clock, and `bagmutate` injects the pathology, so these entries also
  cover the mutation tool. Three topics rather than two because the inter topic
  skew check needs at least three to have a reference. They need no network and
  are the fastest way to see the runner work.

## Licences

Every entry keeps its own licence and terms, listed in `manifest.yaml`. Some
are non commercial, some need an account on the host. Read the terms of an
entry before you use it. This repository redistributes nothing, not even
excerpts, it only stores the urls and the expected findings.

## Usage

List the corpus:

    python corpus/stamps/fetch.py --list

Fetch a single entry, or a whole tier:

    python corpus/stamps/fetch.py --names foxglove-nuscenes-mini
    python corpus/stamps/fetch.py --tier control

Entries whose link cannot be fetched without a human, such as the expiring
Hilti S3 urls or a Dropbox link that lives in a readme, print their landing
page and are skipped. Resolve the link, put it in the manifest as `url` or
`hf_repo`, and fetch again. Completed downloads are never overwritten, an
interrupted download resumes from its `.part` file when the host allows it.

Hugging Face entries need `huggingface_hub` installed. Nothing else in this
directory needs a dependency beyond the standard library and rosio itself.

Run the checks over whatever has been fetched:

    python corpus/stamps/run.py
    python corpus/stamps/run.py --tier synthetic
    python corpus/stamps/run.py --entry euroc-v2-03 --limit 20000

`run.py` reads at most `--limit` messages per topic, prints every finding, then
prints PASS or FAIL per entry against the `expect` and `forbid` rules in the
manifest. Entries with no data on disk are skipped. An entry that reads but
yields no topic with a header is a FAIL, not a pass, because nothing was
analyzed. Exit status is `0` when at least one entry passed and none failed,
`1` if any entry failed, and `3` when nothing was validated at all, which is
what an all skipped run or an empty selection gives.

## Verified results

Measured 2026-08-11, limit 3000 per topic unless noted.

- foxglove-nuscenes-scene0061: no errors. inter_topic_skew warnings of about 0.25 s on /imu, /odom and /diagnostics, clock_drift warnings of 0.37 and 0.55 ms/s on /imu and /odom.
- hf-nuway: /bond stamps in 1970 (epoch_offset error), /global_costmap/costmap 1.03 s behind the rest of the bag.
- hilti-trimble-2026-sample: the documented 10000 s shift is applied to both stamps and record times, so every relative check passes and the absolute plausibility gate is what catches it. The bag needed the sqlite fallback under rosbags 0.11.3 (empty type description hashes); on 0.11.4 it opens on the plain reader, rechecked 2026-08-14, same three epoch_offset errors.
- hilti-2021-lab-survey-2 at limit 5000: the dropouts reported in the challenge issue tracker did not reproduce, no errors, p95 jitter at or below 1 ms. Kept as a clean control.
- hf-tesla3: under rosbags 0.11.3 it read with one topic dropped (a binary integer literal in an embedded type definition that release could not parse); on 0.11.4 every definition parses and nothing is dropped, rechecked 2026-08-14. Two stamp_gaps warnings on bursty RTCM correction streams either way.
