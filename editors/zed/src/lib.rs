//! Starts the rosio language server and MCP server for Zed.

use std::collections::BTreeMap;
use zed_extension_api::{
    self as zed,
    settings::{CommandSettings, ContextServerSettings, LspSettings},
    ContextServerId, LanguageServerId, Result,
};

const REPOSITORY: &str = "sa1ntsinner/rosio";

/// Connects Zed to the rosio binaries, downloading the language server when none is installed.
struct RosioExtension {
    cached_server: Option<String>,
}

/// The release asset name, the same target names the VS Code packages use.
fn asset_name(os: zed::Os, arch: zed::Architecture) -> Result<String> {
    let target = match (os, arch) {
        (zed::Os::Linux, zed::Architecture::X8664) => "linux-x64",
        (zed::Os::Linux, zed::Architecture::Aarch64) => "linux-arm64",
        (zed::Os::Mac, zed::Architecture::X8664) => "darwin-x64",
        (zed::Os::Mac, zed::Architecture::Aarch64) => "darwin-arm64",
        (zed::Os::Windows, zed::Architecture::X8664) => "win32-x64",
        _ => return Err("rosio-lsp has no release build for this platform".into()),
    };
    let suffix = if os == zed::Os::Windows { ".exe" } else { "" };
    Ok(format!("rosio-lsp-{target}{suffix}"))
}

impl RosioExtension {
    /// The latest released rosio-lsp, downloaded once per version into the extension directory.
    fn downloaded_server(&mut self, id: &LanguageServerId) -> Result<String> {
        if let Some(path) = &self.cached_server {
            if std::fs::metadata(path).is_ok_and(|stat| stat.is_file()) {
                return Ok(path.clone());
            }
        }
        zed::set_language_server_installation_status(
            id,
            &zed::LanguageServerInstallationStatus::CheckingForUpdate,
        );
        let release = zed::latest_github_release(
            REPOSITORY,
            zed::GithubReleaseOptions {
                require_assets: true,
                pre_release: false,
            },
        )?;
        let (os, arch) = zed::current_platform();
        let name = asset_name(os, arch)?;
        let asset = release
            .assets
            .iter()
            .find(|asset| asset.name == name)
            .ok_or_else(|| format!("release {} has no {name}", release.version))?;
        let directory = format!("rosio-lsp-{}", release.version);
        let path = format!("{directory}/{name}");
        if !std::fs::metadata(&path).is_ok_and(|stat| stat.is_file()) {
            zed::set_language_server_installation_status(
                id,
                &zed::LanguageServerInstallationStatus::Downloading,
            );
            std::fs::create_dir_all(&directory).map_err(|e| e.to_string())?;
            zed::download_file(
                &asset.download_url,
                &path,
                zed::DownloadedFileType::Uncompressed,
            )?;
            zed::make_file_executable(&path)?;
            for entry in std::fs::read_dir(".").map_err(|e| e.to_string())?.flatten() {
                let old = entry.file_name().to_string_lossy().into_owned();
                if old.starts_with("rosio-lsp-") && old != directory {
                    let _ = std::fs::remove_dir_all(entry.path());
                }
            }
        }
        zed::set_language_server_installation_status(
            id,
            &zed::LanguageServerInstallationStatus::None,
        );
        self.cached_server = Some(path.clone());
        Ok(path)
    }
}

/// Applies configured environment variables over the inherited environment.
fn command_env(env: zed::EnvVars, settings: Option<&CommandSettings>) -> zed::EnvVars {
    let mut env: BTreeMap<_, _> = env.into_iter().collect();
    if let Some(extra) = settings.and_then(|settings| settings.env.as_ref()) {
        env.extend(
            extra
                .iter()
                .map(|(key, value)| (key.clone(), value.clone())),
        );
    }
    env.into_iter().collect()
}

impl zed::Extension for RosioExtension {
    /// Creates the extension; the language server is found or downloaded on first start.
    fn new() -> Self {
        Self {
            cached_server: None,
        }
    }

    /// Starts rosio-lsp from settings, the worktree PATH, or the latest GitHub release.
    fn language_server_command(
        &mut self,
        id: &LanguageServerId,
        worktree: &zed::Worktree,
    ) -> Result<zed::Command> {
        let settings = LspSettings::for_worktree(id.as_ref(), worktree)?;
        let binary = settings.binary.as_ref();
        let command = match binary
            .and_then(|binary| binary.path.clone())
            .or_else(|| worktree.which("rosio-lsp"))
        {
            Some(command) => command,
            None => self.downloaded_server(id).map_err(|error| {
                zed::set_language_server_installation_status(
                    id,
                    &zed::LanguageServerInstallationStatus::Failed(error.clone()),
                );
                format!(
                    "rosio-lsp is not on PATH and could not be downloaded ({error}). Set \
                     lsp.rosio.binary.path in Zed settings."
                )
            })?,
        };
        Ok(zed::Command {
            command,
            args: binary
                .and_then(|binary| binary.arguments.clone())
                .unwrap_or_default(),
            env: command_env(worktree.shell_env(), binary),
        })
    }

    /// Passes through initialization options, with the standard atlas path as a default.
    fn language_server_initialization_options(
        &mut self,
        id: &LanguageServerId,
        worktree: &zed::Worktree,
    ) -> Result<Option<zed::serde_json::Value>> {
        let settings = LspSettings::for_worktree(id.as_ref(), worktree)?;
        Ok(Some(settings.initialization_options.unwrap_or_else(
            || zed::serde_json::json!({ "atlasPath": "rosio-out/atlas.json" }),
        )))
    }

    /// Starts the configured MCP command or rosio serve over stdio.
    fn context_server_command(
        &mut self,
        id: &ContextServerId,
        project: &zed::Project,
    ) -> Result<zed::Command> {
        if id.as_ref() != "rosio" {
            return Err(format!("Unknown context server: {}", id.as_ref()));
        }
        let settings = ContextServerSettings::for_project(id.as_ref(), project)?;
        let binary = settings.command.as_ref();
        let env = command_env(Vec::new(), binary);
        let command = if let Some(path) = binary.and_then(|binary| binary.path.clone()) {
            path
        } else {
            let output = zed::process::Command::new("rosio")
                .arg("--version")
                .envs(env.clone())
                .output()
                .map_err(|error| {
                    format!(
                        "Could not run rosio on PATH ({error}). Install it with \
                         `pip install 'rosio[mcp]'`, or set context_servers.rosio.command.path \
                         in Zed settings."
                    )
                })?;
            if output.status != Some(0) {
                return Err(
                    "rosio --version failed. Check your rosio installation, or set \
                     context_servers.rosio.command.path in Zed settings."
                        .to_string(),
                );
            }
            "rosio".into()
        };
        Ok(zed::Command {
            command,
            args: binary
                .and_then(|binary| binary.arguments.clone())
                .unwrap_or_else(|| vec!["serve".into()]),
            env,
        })
    }
}

zed::register_extension!(RosioExtension);
