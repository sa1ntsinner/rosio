// Writes index.html from index.src.html, the recorded command output and the picture credits.
import { readFileSync, writeFileSync } from "node:fs";

const here = new URL(".", import.meta.url);
const read = (name) => readFileSync(new URL(name, here), "utf8");
const escape = (text) => text.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");

function terminal(text) {
  return escape(text.trimEnd())
    .split("\n")
    .map((line) => line
      .replace(/^(\$ .*)$/, '<span class="prompt">$1</span>')
      .replace(/^(\s*)(error)(\s)/, '$1<span class="sev-error">$2</span>$3')
      .replace(/^(\s*)(warn)(\s)/, '$1<span class="sev-warn">$2</span>$3')
      .replace(/^(\s*)(info)(\s)/, '$1<span class="note">$2</span>$3'))
    .join("\n");
}

const credits = JSON.parse(read("assets/credits.json"))
  .map((item) => `<a href="${item.page}">${escape(item.title.replace(/, sh$/, ""))}</a>${item.creator ? `, ${escape(item.creator)}` : ""}, ${item.license}`)
  .join(" · ");

const outputs = {
  "rover-drift": read("output/rover-drift.txt"),
  "rover-explain": read("output/rover-explain.txt"),
  "nav2-param": read("output/nav2-param.txt"),
  migrate: read("output/migrate.txt"),
  agent: read("output/agent.txt"),
};

let page = read("index.src.html");
for (const [name, text] of Object.entries(outputs)) page = page.replaceAll(`{{${name}}}`, terminal(text));
page = page.replace("{{credits}}", `Pictures: ${credits}`);
writeFileSync(new URL("index.html", here), page);
