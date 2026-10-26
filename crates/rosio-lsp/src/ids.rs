//! Parsing of the canonical, percent escaped id grammar.

/// A decoded id, retaining the synthetic identity marker separately.
#[derive(Debug, PartialEq, Eq)]
pub struct Id {
    pub kind: String,
    pub segments: Vec<String>,
    pub synthetic: bool,
}

/// Decode an id and validate known segment counts.
pub fn parse(text: &str) -> Result<Id, String> {
    let (kind, body) = text.split_once(':').ok_or("Id needs kind:body")?;
    if !kind.as_bytes().first().is_some_and(u8::is_ascii_lowercase)
        || !kind
            .bytes()
            .all(|b| b.is_ascii_lowercase() || b.is_ascii_digit() || b == b'_')
    {
        return Err("Invalid id kind".into());
    }
    if body.is_empty() {
        return Err("Id body must not be empty".into());
    }
    let synthetic = body.starts_with('~');
    let body = if synthetic { &body[1..] } else { body };
    let segments: Vec<String> = body.split('|').map(decode).collect::<Result<_, _>>()?;
    let count = match (synthetic, kind) {
        (true, "node") => Some(4),
        (true, "param" | "pinst") => Some(5),
        (
            true,
            "pkg" | "file" | "sym" | "exe" | "launchfile" | "launchent" | "profile" | "topic"
            | "service" | "action" | "pparam" | "frame" | "type" | "plugin" | "run" | "bag"
            | "bagtopic" | "edge" | "find" | "unres",
        ) => return Err("Invalid synthetic id kind".into()),
        (false, "file" | "exe" | "launchfile" | "param" | "plugin" | "pinst" | "bagtopic") => {
            Some(2)
        }
        (false, "sym" | "pparam" | "type" | "run" | "find") => Some(3),
        (false, "edge") => Some(4),
        (
            false,
            "pkg" | "profile" | "node" | "topic" | "service" | "action" | "frame" | "bag" | "unres",
        ) => Some(1),
        (false, "launchent") if segments.len() < 3 => {
            return Err("Launch entity needs at least three segments".into())
        }
        _ => None,
    };
    if count.is_some_and(|n| n != segments.len()) {
        return Err("Invalid id segment count".into());
    }
    Ok(Id {
        kind: kind.into(),
        segments,
        synthetic,
    })
}

/// Build an id from its kind and segments, escaping as `rosio.ids.build` does.
pub fn build(kind: &str, segments: &[&str]) -> String {
    let body: Vec<String> = segments.iter().map(|s| escape(s)).collect();
    format!("{kind}:{}", body.join("|"))
}

fn escape(text: &str) -> String {
    let mut out = String::new();
    for byte in text.bytes() {
        if (0x21..0x7f).contains(&byte) && !matches!(byte, b'%' | b'|' | b'~') {
            out.push(byte as char);
        } else {
            out.push_str(&format!("%{byte:02X}"));
        }
    }
    out
}

fn decode(text: &str) -> Result<String, String> {
    let mut bytes = Vec::new();
    let mut input = text.bytes();
    while let Some(byte) = input.next() {
        if byte == b'%' {
            let high = input.next().and_then(|b| (b as char).to_digit(16));
            let low = input.next().and_then(|b| (b as char).to_digit(16));
            match (high, low) {
                (Some(h), Some(l)) => bytes.push((h * 16 + l) as u8),
                _ => return Err("Invalid percent escape".into()),
            }
        } else if !(0x20..=0x7e).contains(&byte) || byte == b'~' {
            return Err("Id segment must be escaped ASCII".into());
        } else {
            bytes.push(byte);
        }
    }
    String::from_utf8(bytes).map_err(|e| format!("Invalid UTF-8 in id: {e}"))
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;
    #[test]
    fn escaped_segments() {
        let id = parse("file:@repo|a%7Cb%25%7E%C3%A9").unwrap();
        assert_eq!(id.segments, ["@repo", "a|b%~é"]);
        assert!(parse("file:p|%").is_err());
        assert!(parse("file:p|%GG").is_err());
        assert!(parse("file:p").is_err());
        assert!(parse("node:").is_err());
        assert!(parse("future:~p|e").is_ok());
        assert!(parse("1bad:x").is_err());
        assert!(parse("node:~p|e|/|abc").unwrap().synthetic);
        assert_eq!(
            build("file", &["@repo", "a|b%~é"]),
            "file:@repo|a%7Cb%25%7E%C3%A9"
        );
    }
}
