//! The triangles of one mesh file, for views that draw a robot model.
use crate::atlas::{Atlas, Evidence};
use crate::paths::Paths;
use serde_json::{json, Value};
use std::collections::{HashMap, HashSet};
use std::path::{Path, PathBuf};
use std::time::SystemTime;

/// Most triangles one answer carries.
const LIMIT: usize = 20_000;
/// Grid cells along the longest side of the bounds in the first decimation pass.
const FINEST: u32 = 1024;

type Triangle = [f32; 9];

/// The local path of a file id, through the root labels the atlas states.
pub(crate) fn locate(atlas: &Atlas, paths: &Paths, file: &str) -> Option<PathBuf> {
    let labels = atlas.generated.roots.iter().map(|r| r.label.as_str());
    labels.chain([""]).find_map(|root| {
        paths.resolve(&Evidence {
            file: file.into(),
            root: root.into(),
            ..Default::default()
        })
    })
}

/// The answer of `rosio/mesh` for a file id and the path it resolved to.
pub(crate) fn mesh(file: &str, path: Option<&Path>) -> Value {
    let format = file
        .rsplit_once('.')
        .map(|(_, ext)| ext.to_ascii_lowercase())
        .filter(|ext| !ext.contains(['/', '|']))
        .unwrap_or_default();
    let refused = |code: &str| json!({"file": file, "format": format, "refused": code});
    let Some(path) = path else {
        return refused("file_missing");
    };
    let parse = match format.as_str() {
        "stl" => stl,
        "obj" => obj,
        _ => return refused("format_not_read"),
    };
    let Some(triangles) = std::fs::read(path).ok().and_then(|bytes| parse(&bytes)) else {
        return refused("mesh_unreadable");
    };
    let count = triangles.len();
    let mut min = [f32::INFINITY; 3];
    let mut max = [f32::NEG_INFINITY; 3];
    for vertex in triangles.iter().flat_map(|t| t.chunks(3)) {
        for axis in 0..3 {
            min[axis] = min[axis].min(vertex[axis]);
            max[axis] = max[axis].max(vertex[axis]);
        }
    }
    let truncated = count > LIMIT;
    let kept = if truncated {
        decimate(&triangles, min, max).0
    } else {
        triangles
    };
    let kept: Vec<f32> = kept.iter().flatten().copied().collect();
    json!({
        "file": file,
        "format": format,
        "triangles": kept,
        "truncated": truncated,
        "triangleCount": count,
        "bounds": (count > 0).then(|| json!({"min": min, "max": max})),
    })
}

/// Answers kept, newest last.
const CACHED: usize = 64;

/// The last `CACHED` answers, each keyed by file id, path, size and modification time, so an
/// unchanged file is neither read nor decimated again.
#[derive(Default)]
pub(crate) struct Cache {
    entries: Vec<(MeshKey, Value)>,
}

type MeshKey = (String, PathBuf, u64, SystemTime);

impl Cache {
    pub(crate) fn answer(&mut self, file: &str, path: Option<&Path>) -> Value {
        let key = path.and_then(|path| {
            let meta = std::fs::metadata(path).ok()?;
            Some((
                file.to_owned(),
                path.to_owned(),
                meta.len(),
                meta.modified().ok()?,
            ))
        });
        let Some(key) = key else {
            return mesh(file, path);
        };
        if let Some(at) = self.entries.iter().position(|(k, _)| *k == key) {
            let entry = self.entries.remove(at);
            let answer = entry.1.clone();
            self.entries.push(entry);
            return answer;
        }
        let answer = mesh(file, path);
        if self.entries.len() == CACHED {
            self.entries.remove(0);
        }
        self.entries.push((key, answer.clone()));
        answer
    }
}

/// Vertex clustering on the coarsest grid tried that keeps at most `LIMIT` triangles, and the
/// number of cells along the longest side of that grid. Grids go from `FINEST` down by a
/// fifth per pass, and a single cell keeps nothing, so the loop ends.
fn decimate(triangles: &[Triangle], min: [f32; 3], max: [f32; 3]) -> (Vec<Triangle>, u32) {
    let extent = (0..3)
        .map(|a| f64::from(max[a] - min[a]))
        .fold(0.0, f64::max);
    let mut cells = FINEST;
    loop {
        let kept = cluster(triangles, min, extent / f64::from(cells), cells);
        if kept.len() <= LIMIT || cells == 1 {
            return (kept, cells);
        }
        cells = (cells * 4 / 5).max(1);
    }
}

/// Every corner moves to the mean of the corners in its grid cell. A triangle with two corners
/// in one cell is dropped, and so is a repeat of a kept triangle in the same orientation.
/// Output follows file order and corner order, so no map order reaches it.
fn cluster(triangles: &[Triangle], min: [f32; 3], size: f64, cells: u32) -> Vec<Triangle> {
    let mut index: HashMap<[u32; 3], u32> = HashMap::new();
    let mut sums: Vec<[f64; 4]> = Vec::new();
    let corners: Vec<[u32; 3]> = triangles
        .iter()
        .map(|t| {
            let mut ids = [0; 3];
            for (id, v) in ids.iter_mut().zip(t.chunks(3)) {
                let key = [0, 1, 2].map(|a| {
                    let at = (f64::from(v[a]) - f64::from(min[a])) / size;
                    (at as u32).min(cells - 1)
                });
                *id = *index.entry(key).or_insert_with(|| {
                    sums.push([0.0; 4]);
                    (sums.len() - 1) as u32
                });
                let sum = &mut sums[*id as usize];
                for a in 0..3 {
                    sum[a] += f64::from(v[a]);
                }
                sum[3] += 1.0;
            }
            ids
        })
        .collect();
    let mut seen = HashSet::new();
    corners
        .into_iter()
        .filter(|&[a, b, c]| a != b && b != c && a != c)
        .filter(|&ids| {
            let first = (0..3).min_by_key(|&i| ids[i]).unwrap_or(0);
            seen.insert([0, 1, 2].map(|i| ids[(first + i) % 3]))
        })
        .map(|ids| {
            let mut t = [0.0; 9];
            for (slot, id) in t.chunks_mut(3).zip(ids) {
                let [x, y, z, n] = sums[id as usize];
                slot.copy_from_slice(&[(x / n) as f32, (y / n) as f32, (z / n) as f32]);
            }
            t
        })
        .collect()
}

fn finite(triangles: Vec<Triangle>) -> Option<Vec<Triangle>> {
    triangles
        .iter()
        .all(|t| t.iter().all(|v| v.is_finite()))
        .then_some(triangles)
}

/// Binary when the header's count matches the length exactly, else ASCII, else binary when
/// the file holds at least the triangles its header counts, ignoring the bytes after them.
fn stl(bytes: &[u8]) -> Option<Vec<Triangle>> {
    let end = bytes
        .get(80..84)
        .map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]) as usize)
        .and_then(|n| n.checked_mul(50))
        .and_then(|n| n.checked_add(84))
        .filter(|&end| end <= bytes.len());
    match end {
        Some(end) if end == bytes.len() => binary_stl(&bytes[84..end]),
        Some(end) => ascii_stl(bytes).or_else(|| binary_stl(&bytes[84..end])),
        None => ascii_stl(bytes),
    }
}

fn binary_stl(facets: &[u8]) -> Option<Vec<Triangle>> {
    let triangles = facets
        .as_chunks::<50>()
        .0
        .iter()
        .map(|facet| {
            let mut t = [0.0; 9];
            for (v, word) in t.iter_mut().zip(facet[12..48].as_chunks::<4>().0) {
                *v = f32::from_le_bytes(*word);
            }
            t
        })
        .collect();
    finite(triangles)
}

fn ascii_stl(bytes: &[u8]) -> Option<Vec<Triangle>> {
    let text = std::str::from_utf8(bytes).ok()?;
    let mut words = text.split_ascii_whitespace();
    if words.next() != Some("solid") || !text.split_ascii_whitespace().any(|w| w == "endsolid") {
        return None;
    }
    let mut vertices = Vec::new();
    while let Some(word) = words.next() {
        if word == "vertex" {
            for _ in 0..3 {
                vertices.push(words.next()?.parse::<f32>().ok()?);
            }
        }
    }
    let (triangles, rest) = vertices.as_chunks::<9>();
    if !rest.is_empty() {
        return None;
    }
    finite(triangles.to_vec())
}

/// Wavefront OBJ, every face triangulated as a fan from its first corner.
fn obj(bytes: &[u8]) -> Option<Vec<Triangle>> {
    let text = std::str::from_utf8(bytes).ok()?;
    let mut vertices: Vec<[f32; 3]> = Vec::new();
    let mut triangles = Vec::new();
    for line in text.lines() {
        let mut words = line.split_ascii_whitespace();
        match words.next() {
            Some("v") => {
                let mut v = [0.0; 3];
                for axis in &mut v {
                    *axis = words.next()?.parse().ok()?;
                }
                vertices.push(v);
            }
            Some("f") => {
                let corners = words
                    .map(|w| {
                        let i: i64 = w.split('/').next()?.parse().ok()?;
                        let i = if i < 0 {
                            vertices.len() as i64 + i
                        } else {
                            i - 1
                        };
                        vertices.get(usize::try_from(i).ok()?).copied()
                    })
                    .collect::<Option<Vec<_>>>()?;
                if corners.len() < 3 {
                    return None;
                }
                for pair in corners[1..].windows(2) {
                    let mut t = [0.0; 9];
                    for (slot, corner) in t.chunks_mut(3).zip([corners[0], pair[0], pair[1]]) {
                        slot.copy_from_slice(&corner);
                    }
                    triangles.push(t);
                }
            }
            _ => {}
        }
    }
    finite(triangles)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;

    fn scratch(name: &str, bytes: &[u8]) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("rosio-mesh-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join(name);
        std::fs::write(&path, bytes).unwrap();
        path
    }

    fn binary(triangles: &[Triangle]) -> Vec<u8> {
        let mut bytes = vec![b' '; 80];
        bytes.extend((triangles.len() as u32).to_le_bytes());
        for t in triangles {
            bytes.extend([0u8; 12]);
            t.iter().for_each(|v| bytes.extend(v.to_le_bytes()));
            bytes.extend([0u8; 2]);
        }
        bytes
    }

    const TWO: [Triangle; 2] = [
        [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0, 1.0, 1.0, 0.5, 0.0, 1.0, -0.25],
    ];

    fn answer(name: &str, bytes: &[u8]) -> Value {
        let file = format!("file:bot|meshes/{name}");
        mesh(&file, Some(&scratch(name, bytes)))
    }

    fn expect_two(result: &Value, format: &str) {
        let flat: Vec<f32> = TWO.iter().flatten().copied().collect();
        assert_eq!(result["format"], format);
        assert_eq!(result["triangles"], json!(flat));
        assert_eq!(result["triangleCount"], 2);
        assert_eq!(result["truncated"], false);
        assert_eq!(
            result["bounds"],
            json!({"min": [0.0, 0.0, -0.25], "max": [1.0, 1.0, 0.5]})
        );
        assert!(result.get("refused").is_none());
    }

    #[test]
    fn binary_and_ascii_stl_and_obj_give_the_same_triangles() {
        expect_two(&answer("b.stl", &binary(&TWO)), "stl");
        let ascii = "solid two\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\n\
            facet normal 0 0 1\nouter loop\nvertex 1 0 0\nvertex 1 1 0.5\nvertex 0 1 -0.25\nendloop\nendfacet\nendsolid two\n";
        expect_two(&answer("a.STL", ascii.as_bytes()), "stl");
        let quad = "# two\nv 0 0 0\nv 1 0 0\nv 0 1 0\nv 1 1 0.5\nv 0 1 -0.25\nvn 0 0 1\nf 1//1 2//1 -3//1\nf 2/1/1 4/1/1 5/1/1\n";
        expect_two(&answer("o.obj", quad.as_bytes()), "obj");
        let fan = answer("f.obj", b"v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nf 1 2 3 4\n");
        assert_eq!(fan["triangleCount"], 2);
        assert_eq!(
            &fan["triangles"].as_array().unwrap()[9..12],
            [0.0, 0.0, 0.0]
        );
    }

    /// A UV sphere of radius one, corners counter clockwise seen from outside.
    fn sphere(rings: usize, segments: usize) -> Vec<Triangle> {
        let at = |r: usize, s: usize| {
            let lat = std::f32::consts::PI * r as f32 / rings as f32;
            let lon = std::f32::consts::TAU * s as f32 / segments as f32;
            [lat.sin() * lon.cos(), lat.sin() * lon.sin(), lat.cos()]
        };
        let mut out = Vec::new();
        for r in 0..rings {
            for s in 0..segments {
                let (a, b, c, d) = (at(r, s), at(r + 1, s), at(r + 1, s + 1), at(r, s + 1));
                for corners in [[a, b, c], [a, c, d]] {
                    let mut t = [0.0; 9];
                    for (slot, corner) in t.chunks_mut(3).zip(corners) {
                        slot.copy_from_slice(&corner);
                    }
                    out.push(t);
                }
            }
        }
        out
    }

    fn bounds(triangles: &[Triangle]) -> ([f32; 3], [f32; 3]) {
        let mut min = [f32::INFINITY; 3];
        let mut max = [f32::NEG_INFINITY; 3];
        for v in triangles.iter().flat_map(|t| t.chunks(3)) {
            for a in 0..3 {
                min[a] = min[a].min(v[a]);
                max[a] = max[a].max(v[a]);
            }
        }
        (min, max)
    }

    #[test]
    fn a_large_mesh_is_decimated_under_the_cap_the_same_way_every_time() {
        let ball = sphere(200, 160);
        let bytes = binary(&ball);
        let result = answer("ball.stl", &bytes);
        assert_eq!(result["truncated"], true);
        assert_eq!(result["triangleCount"], ball.len());
        let kept = result["triangles"].as_array().unwrap();
        assert!(!kept.is_empty() && kept.len() <= LIMIT * 9);
        let (min, max) = bounds(&ball);
        assert_eq!(result["bounds"], json!({"min": min, "max": max}));
        let again = answer("ball2.stl", &bytes);
        assert_eq!(
            serde_json::to_vec(&result["triangles"]).unwrap(),
            serde_json::to_vec(&again["triangles"]).unwrap()
        );
    }

    #[test]
    fn a_mesh_at_the_cap_is_returned_as_read() {
        let flat: Vec<Triangle> = (0..LIMIT)
            .map(|i| [i as f32, 0.0, 0.0, i as f32, 1.0, 0.0, i as f32, 0.0, 1.0])
            .collect();
        let result = answer("cap.stl", &binary(&flat));
        assert_eq!(result["truncated"], false);
        assert_eq!(result["triangles"], json!(flat.concat()));
    }

    #[test]
    fn a_closed_mesh_keeps_its_extent_and_its_orientation() {
        let ball = sphere(200, 160);
        let (min, max) = bounds(&ball);
        let (kept, cells) = decimate(&ball, min, max);
        assert!(
            kept.len() <= LIMIT && kept.len() > LIMIT / 4,
            "{}",
            kept.len()
        );
        let cell = 2.0 / cells as f32;
        let (kmin, kmax) = bounds(&kept);
        for a in 0..3 {
            assert!((kmin[a] - min[a]).abs() <= cell, "{kmin:?} {min:?}");
            assert!((kmax[a] - max[a]).abs() <= cell, "{kmax:?} {max:?}");
        }
        let outward = kept
            .iter()
            .filter(|t| {
                let e1 = [t[3] - t[0], t[4] - t[1], t[5] - t[2]];
                let e2 = [t[6] - t[0], t[7] - t[1], t[8] - t[2]];
                let n = [
                    e1[1] * e2[2] - e1[2] * e2[1],
                    e1[2] * e2[0] - e1[0] * e2[2],
                    e1[0] * e2[1] - e1[1] * e2[0],
                ];
                n[0] * t[0] + n[1] * t[1] + n[2] * t[2] > 0.0
            })
            .count();
        assert!(
            outward * 100 >= kept.len() * 99,
            "{outward} of {}",
            kept.len()
        );
    }

    #[test]
    fn an_unchanged_file_is_answered_from_the_cache_until_it_is_evicted() {
        let mut cache = Cache::default();
        let file = "file:bot|meshes/cached.stl";
        let path = scratch("cached.stl", &binary(&TWO));
        let first = cache.answer(file, Some(&path));
        expect_two(&first, "stl");
        let stamp = std::fs::metadata(&path).unwrap().modified().unwrap();
        let mut moved = TWO;
        moved[0][0] = 0.5;
        std::fs::write(&path, binary(&moved)).unwrap();
        let handle = std::fs::File::options().write(true).open(&path).unwrap();
        handle.set_modified(stamp).unwrap();
        drop(handle);
        assert_eq!(cache.answer(file, Some(&path)), first, "same size and time");
        for i in 0..CACHED {
            let other = scratch(&format!("other{i}.stl"), &binary(&TWO));
            cache.answer(&format!("file:bot|meshes/other{i}.stl"), Some(&other));
        }
        assert_ne!(
            cache.answer(file, Some(&path)),
            first,
            "evicted, so read again"
        );
        std::fs::write(&path, binary(&TWO)).unwrap();
        expect_two(&cache.answer(file, Some(&path)), "stl");
    }

    #[test]
    fn a_binary_stl_with_trailing_bytes_reads_its_declared_triangles() {
        let mut padded = binary(&TWO);
        padded.extend([0u8; 13]);
        expect_two(&answer("padded.stl", &padded), "stl");
        let mut solid = binary(&TWO);
        solid[..5].copy_from_slice(b"solid");
        solid.push(b'\n');
        expect_two(&answer("solid.stl", &solid), "stl");
    }

    #[test]
    fn each_refusal_names_its_reason() {
        let missing = mesh("file:bot|meshes/gone.stl", None);
        assert_eq!(
            missing,
            json!({"file":"file:bot|meshes/gone.stl","format":"stl","refused":"file_missing"})
        );
        assert_eq!(answer("m.dae", b"<COLLADA/>")["refused"], "format_not_read");
        let mut cut = binary(&TWO);
        cut.truncate(cut.len() - 7);
        assert_eq!(answer("cut.stl", &cut)["refused"], "mesh_unreadable");
        let open = "solid x\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\n";
        assert_eq!(
            answer("open.stl", open.as_bytes())["refused"],
            "mesh_unreadable"
        );
        assert_eq!(
            answer("bad.obj", b"v 0 0 0\nf 1 2 3\n")["refused"],
            "mesh_unreadable"
        );
    }
}
