//! One extraction worker with a coalesced pending request.
use crate::server::{log, notify};
use crossbeam_channel::Sender;
use lsp_server::{Message, Request};
use serde_json::json;
use std::ffi::OsStr;
use std::path::Path;
use std::process::Command;
use std::time::{Duration, Instant};

const INSTALL: &str = "install it with `pipx install rosio` or `pip install rosio`";

/// What to tell a person whose engine command does not exist, in one sentence.
pub(crate) fn engine_missing(program: &str) -> String {
    format!(
        "The rosio engine `{program}` was not found: {INSTALL}, then restart the language \
         server, or point the extract command at an installed rosio."
    )
}

pub(crate) fn is_engine_missing(message: &str) -> bool {
    message.contains(INSTALL)
}

/// The line a failed command ends on, which is where python and the cli state the cause.
fn last_line(output: &str) -> &str {
    output
        .lines()
        .rev()
        .map(str::trim)
        .find(|line| !line.is_empty())
        .unwrap_or("no output")
}

/// The spawn error as the install sentence when the program or the python module is missing.
pub(crate) fn spawn_error(program: &str, error: &std::io::Error) -> String {
    if error.kind() == std::io::ErrorKind::NotFound {
        engine_missing(program)
    } else {
        format!("Cannot start {program}: {error}")
    }
}

/// Whether `program` names a file, directly or through PATH.
pub(crate) fn engine_found(program: &str) -> bool {
    found_in(program, std::env::var_os("PATH").as_deref())
}

fn found_in(program: &str, path: Option<&OsStr>) -> bool {
    let candidate = Path::new(program);
    if candidate.components().count() > 1 {
        return candidate.is_file();
    }
    let suffixes: &[&str] = if cfg!(windows) {
        &["", ".exe", ".cmd", ".bat", ".com"]
    } else {
        &[""]
    };
    path.into_iter().flat_map(std::env::split_paths).any(|dir| {
        suffixes
            .iter()
            .any(|suffix| dir.join(format!("{program}{suffix}")).is_file())
    })
}

#[derive(Default)]
pub(crate) struct Extraction {
    running: bool,
    due: Option<Instant>,
    serial: u64,
}
impl Extraction {
    pub(crate) fn request(&mut self, delay: Duration) {
        self.due = Some(Instant::now() + delay);
    }

    pub(crate) fn start(
        &mut self,
        workspace: &std::path::Path,
        command: &[String],
        sender: &Sender<Message>,
        completed: &Sender<Result<String, String>>,
        progress: bool,
    ) {
        if self.running || self.due.is_none_or(|due| due > Instant::now()) {
            return;
        }
        self.due = None;
        self.running = true;
        self.serial += 1;
        let token = format!("rosio-extract-{}", self.serial);
        if progress {
            let _ = sender.send(Message::Request(Request::new(
                format!("progress-{}", self.serial).into(),
                "window/workDoneProgress/create".into(),
                json!({"token":token}),
            )));
        }
        notify(
            sender,
            "$/progress",
            json!({"token":token,"value":{"kind":"begin","title":"rosio extract"}}),
        );
        let command = command.to_owned();
        let workspace = workspace.to_owned();
        let completed = completed.clone();
        std::thread::spawn(move || {
            let result = match command.split_first() {
                Some((program, arguments)) => Command::new(program)
                    .args(arguments)
                    .current_dir(workspace)
                    .output()
                    .map_err(|e| spawn_error(program, &e))
                    .and_then(|output| {
                        let stderr = String::from_utf8_lossy(&output.stderr).into_owned();
                        if output.status.success() {
                            Ok(stderr)
                        } else if stderr.contains("No module named rosio") {
                            Err(engine_missing("rosio"))
                        } else {
                            Err(format!(
                                "Extraction exited with {}\n{}",
                                output.status, stderr
                            ))
                        }
                    }),
                None => Err("extractCommand must not be empty".into()),
            };
            let _ = completed.send(result);
        });
    }

    pub(crate) fn finish(&mut self, sender: &Sender<Message>, result: Result<String, String>) {
        self.running = false;
        match result {
            Ok(stderr) if !stderr.is_empty() => log(sender, 3, &stderr),
            Err(error) => {
                log(sender, 1, &error);
                let message = if is_engine_missing(&error) {
                    error
                } else {
                    format!("rosio extract failed: {}", last_line(&error))
                };
                notify(
                    sender,
                    "window/showMessage",
                    json!({"type":1,"message":message}),
                );
            }
            _ => {}
        }
        notify(
            sender,
            "$/progress",
            json!({"token":format!("rosio-extract-{}",self.serial),"value":{"kind":"end"}}),
        );
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;
    #[test]
    fn extraction_coalesces_requests() {
        let mut extraction = Extraction {
            running: true,
            ..Default::default()
        };
        extraction.request(Duration::ZERO);
        extraction.request(Duration::ZERO);
        assert!(extraction.running);
        assert!(extraction.due.is_some());
        assert_eq!(extraction.serial, 0);
    }

    #[test]
    fn a_missing_engine_is_one_sentence_with_the_install_command() {
        let message = engine_missing("rosio");
        assert!(message.contains("`pipx install rosio`"));
        assert!(message.contains("`pip install rosio`"));
        assert_eq!(message.matches(". ").count(), 0);
        assert!(!message.contains('\n'));
        let error = std::io::Error::from(std::io::ErrorKind::NotFound);
        assert_eq!(spawn_error("rosio", &error), message);
        assert!(is_engine_missing(&message));
    }

    #[test]
    fn engine_lookup_reads_path() {
        let dir = std::env::temp_dir().join(format!("rosio-path-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let name = if cfg!(windows) { "fake.exe" } else { "fake" };
        std::fs::write(dir.join(name), "").unwrap();
        let path = std::env::join_paths([&dir]).unwrap();
        assert!(found_in("fake", Some(&path)));
        assert!(!found_in("absent", Some(&path)));
        assert!(!found_in("fake", None));
        assert!(found_in(dir.join(name).to_str().unwrap(), None));
        std::fs::remove_dir_all(dir).unwrap();
    }
}
