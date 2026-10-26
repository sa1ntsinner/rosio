# Working on the extension

- `npm test` runs the unit tests, `npm run typecheck` checks the types and `npm run build`
  bundles the extension.
- `npm run test:e2e` drives a real VS Code. `VSCODE_EXECUTABLE_PATH` selects an installed one,
  `ROSIO_LSP` selects the language server binary. The test workspace and profile stay under
  `.vscode-test/`.
- The release migration commands run the configured engine: apply runs without `--force`, so
  the engine refuses files with uncommitted changes, and the source release comes from the
  engine environment's `ROS_DISTRO` or from the current plan.
