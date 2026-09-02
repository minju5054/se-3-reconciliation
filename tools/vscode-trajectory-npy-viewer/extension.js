"use strict";

const crypto = require("crypto");
const fs = require("fs");
const os = require("os");
const path = require("path");
const { execFile } = require("child_process");
const vscode = require("vscode");

const VIEW_TYPE = "reconciliation.trajectoryNpyGraph";
const REFERENCE_NAME = "reference_trajectory.npy";
const ACTUAL_NAME = "actual_trajectory.npy";

class TrajectoryDocument {
  constructor(uri) {
    this.uri = uri;
  }

  dispose() {}
}

class TrajectoryGraphProvider {
  async openCustomDocument(uri) {
    return new TrajectoryDocument(uri);
  }

  async resolveCustomEditor(document, panel) {
    panel.webview.options = { enableScripts: false };
    panel.webview.html = loadingHtml();

    let renderSequence = 0;
    let debounceTimer;
    const render = () => {
      renderSequence += 1;
      const sequence = renderSequence;
      panel.webview.html = loadingHtml();
      renderGraph(document.uri)
        .then((html) => {
          if (sequence === renderSequence) {
            panel.webview.html = html;
          }
        })
        .catch((error) => {
          if (sequence === renderSequence) {
            panel.webview.html = errorHtml(error);
          }
        });
    };

    const watchers = plotInputs(document.uri).map((input) => {
      const watcher = vscode.workspace.createFileSystemWatcher(
        new vscode.RelativePattern(path.dirname(input), path.basename(input)),
      );
      watcher.onDidChange(() => {
        clearTimeout(debounceTimer);
        debounceTimer = setTimeout(render, 150);
      });
      return watcher;
    });
    panel.onDidDispose(() => {
      clearTimeout(debounceTimer);
      watchers.forEach((watcher) => watcher.dispose());
    });
    render();
  }
}

function repositoryRoot(uri) {
  const workspace = vscode.workspace.getWorkspaceFolder(uri);
  if (workspace && isRepositoryRoot(workspace.uri.fsPath)) {
    return workspace.uri.fsPath;
  }

  let candidate = path.dirname(uri.fsPath);
  while (candidate !== path.dirname(candidate)) {
    if (isRepositoryRoot(candidate)) {
      return candidate;
    }
    candidate = path.dirname(candidate);
  }
  throw new Error("Open the .npy file from the se-3-reconciliation workspace.");
}

function isRepositoryRoot(candidate) {
  return (
    fs.existsSync(path.join(candidate, "scripts", "view_trajectory_npy.py")) &&
    fs.existsSync(path.join(candidate, ".venv", "bin", "python"))
  );
}

function plotInputs(uri) {
  const directory = path.dirname(uri.fsPath);
  const reference = path.join(directory, REFERENCE_NAME);
  const actual = path.join(directory, ACTUAL_NAME);
  if (
    [REFERENCE_NAME, ACTUAL_NAME].includes(path.basename(uri.fsPath)) &&
    fs.existsSync(reference) &&
    fs.existsSync(actual)
  ) {
    return [reference, actual];
  }
  return [uri.fsPath];
}

async function renderGraph(uri) {
  if (!vscode.workspace.isTrusted) {
    throw new Error("Trust this workspace before rendering trajectory files.");
  }

  const root = repositoryRoot(uri);
  const python = path.join(root, ".venv", "bin", "python");
  const viewer = path.join(root, "scripts", "view_trajectory_npy.py");
  if (!fs.existsSync(python)) {
    throw new Error(`Research Python was not found: ${python}`);
  }
  if (!fs.existsSync(viewer)) {
    throw new Error(`Trajectory viewer was not found: ${viewer}`);
  }

  const output = path.join(
    os.tmpdir(),
    `se2-trajectory-vscode-${crypto.randomUUID()}.png`,
  );
  const args = [
    viewer,
    ...plotInputs(uri),
    "--rows",
    "0",
    "--no-show",
    "--save",
    output,
    "--force",
  ];

  try {
    await execute(python, args, root);
    const encoded = await fs.promises.readFile(output, { encoding: "base64" });
    return imageHtml(encoded);
  } finally {
    await fs.promises.rm(output, { force: true });
  }
}

function execute(executable, args, cwd) {
  return new Promise((resolve, reject) => {
    execFile(executable, args, { cwd, maxBuffer: 4 * 1024 * 1024 }, (error, stdout, stderr) => {
      if (error) {
        const detail = (stderr || stdout || error.message).trim();
        reject(new Error(detail));
        return;
      }
      resolve();
    });
  });
}

function imageHtml(encoded) {
  const nonce = crypto.randomBytes(16).toString("hex");
  return `<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta http-equiv="Content-Security-Policy" content="default-src 'none'; img-src data:; style-src 'nonce-${nonce}';">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <style nonce="${nonce}">
    html, body { width: 100%; height: 100%; margin: 0; overflow: auto; background: var(--vscode-editor-background); }
    main { box-sizing: border-box; width: 100%; min-height: 100%; display: flex; align-items: center; justify-content: center; padding: 12px; }
    img { display: block; max-width: 100%; height: auto; object-fit: contain; }
  </style>
</head>
<body><main><img src="data:image/png;base64,${encoded}" alt="SE(2) trajectory XY and yaw graph"></main></body>
</html>`;
}

function loadingHtml() {
  return messageHtml("Rendering SE(2) trajectory graph…");
}

function errorHtml(error) {
  return messageHtml(`Unable to render trajectory: ${error.message}`, true);
}

function messageHtml(message, isError = false) {
  const escaped = String(message)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#039;");
  const color = isError ? "var(--vscode-errorForeground)" : "var(--vscode-descriptionForeground)";
  return `<!DOCTYPE html><html><body style="font-family:var(--vscode-font-family);color:${color};padding:20px">${escaped}</body></html>`;
}

function activate(context) {
  context.subscriptions.push(
    vscode.window.registerCustomEditorProvider(
      VIEW_TYPE,
      new TrajectoryGraphProvider(),
      {
        webviewOptions: { retainContextWhenHidden: true },
        supportsMultipleEditorsPerDocument: false,
      },
    ),
  );
}

function deactivate() {}

module.exports = { activate, deactivate };
