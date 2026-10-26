//! A language server reading the rosio atlas.
pub mod actions;
pub mod atlas;
pub mod diagnostics;
pub mod hover;
pub mod ids;
pub mod index;
pub mod navigation;
pub mod paths;
pub mod server;

mod extraction;
mod inlay;
mod lens;
mod mesh;
mod migration;
mod queries;
