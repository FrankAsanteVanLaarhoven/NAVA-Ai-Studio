#!/usr/bin/env node
import { readFileSync, readdirSync, statSync, existsSync } from "node:fs";
import { resolve, extname } from "node:path";

function diffSets(a, b) {
  const A = new Set(a), B = new Set(b);
  return [...A].filter(x => !B.has(x));
}

function match(pattern, topic) {
  if (pattern === topic) return true;
  if (pattern.endsWith("/*")) return topic.startsWith(pattern.slice(0,-1));
  return false;
}

function isSubset(patterns, allowed) {
  return patterns.every(p => allowed.some(a => match(a, p)));
}

function collectManifestCaps(path) {
  const raw = JSON.parse(readFileSync(path, "utf8"));
  const m = raw.signature ? raw : raw; // ignore signature for PR checks
  const caps = m.capabilities || [];
  return { id: m.id || path, caps };
}

function deriveRules(caps) {
  const CapToTopics = {
    "cap:vision.pub": { publish: ["vision/*"] },
    "cap:vision.sub": { subscribe: ["vision/*"] },
    "cap:assistant.pub": { publish: ["assistant/*"] },
    "cap:assistant.sub": { subscribe: ["assistant/*"] },
    "cap:fs.read": {},
    "cap:fs.write": {},
    "cap:shell.exec": {},
    "cap:rtc": { subscribe: ["vision/*"] },
    "cap:mic": {},
    "cap:camera": { publish: ["vision/*"] },
    "cap:llm.invoke": { publish: ["assistant/output"] }
  };
  const pub = new Set(), sub = new Set();
  for (const c of caps) {
    const m = CapToTopics[c] || {};
    (m.publish||[]).forEach(x => pub.add(x));
    (m.subscribe||[]).forEach(x => sub.add(x));
  }
  return { publish: [...pub], subscribe: [...sub] };
}

function walkManifests(dir) {
  const files = [];
  for (const name of readdirSync(dir)) {
    const p = resolve(dir, name);
    const st = statSync(p);
    if (st.isDirectory()) files.push(...walkManifests(p));
    else if (extname(p) === ".json") files.push(p);
  }
  return files;
}

const POLICY_PATH = ".nava/policy.json";
if (!existsSync(POLICY_PATH)) {
  console.error(`Policy file not found: ${POLICY_PATH}`);
  process.exit(1);
}

const policy = JSON.parse(readFileSync(POLICY_PATH, "utf8"));
const changed = process.env.GITHUB_WORKSPACE ? process.env.CHANGED || "" : "";
let files = [];
if (changed) {
  files = changed.split("\n").filter(x => x && x.endsWith(".json"));
} else {
  files = walkManifests("manifests");
}

let errors = [];
for (const f of files) {
  try {
    const { id, caps } = collectManifestCaps(f);
    const unapproved = diffSets(caps, policy.allowedCapabilities || []);
    if (unapproved.length) errors.push(`❌ ${id}: unapproved capabilities: ${unapproved.join(", ")}`);
    const rules = deriveRules(caps);
    if (policy.allowedPublishPatterns && !isSubset(rules.publish, policy.allowedPublishPatterns)) {
      errors.push(`❌ ${id}: publish patterns exceed policy: ${rules.publish.join(", ")}`);
    }
    if (policy.allowedSubscribePatterns && !isSubset(rules.subscribe, policy.allowedSubscribePatterns)) {
      errors.push(`❌ ${id}: subscribe patterns exceed policy: ${rules.subscribe.join(", ")}`);
    }
  } catch (e) {
    errors.push(`❌ ${f}: ${e.message}`);
  }
}

if (errors.length) {
  console.log(errors.join("\n"));
  process.exit(1);
} else {
  console.log("✅ Policy check passed");
}
