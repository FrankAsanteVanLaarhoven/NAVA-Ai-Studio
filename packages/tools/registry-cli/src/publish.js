#!/usr/bin/env node
import { readFileSync, readdirSync, statSync, existsSync, writeFileSync, mkdirSync } from "node:fs";
import { resolve, extname, join } from "node:path";
import { execSync } from "node:child_process";
import { createServer } from "node:http";
import { webcrypto as crypto } from "node:crypto";

function b64u(arr) {
  const b64 = Buffer.from(arr).toString("base64").replace(/\+/g,'-').replace(/\//g,'_').replace(/=+$/,'');
  return b64;
}

function canonicalize(input) {
  if (input === null || typeof input !== "object") return JSON.stringify(input);
  if (Array.isArray(input)) return '[' + input.map(canonicalize).join(',') + ']';
  const keys = Object.keys(input).sort();
  const out = keys.map(k => JSON.stringify(k) + ':' + canonicalize(input[k])).join(',');
  return '{' + out + '}';
}

async function signManifest(unsignedPath, jwkPath, issuer) {
  const unsigned = JSON.parse(readFileSync(unsignedPath, "utf8"));
  const { subtle } = crypto;
  const jwk = JSON.parse(readFileSync(jwkPath, "utf8"));
  const key = await subtle.importKey("jwk", jwk, { name: "ECDSA", namedCurve: "P-256" }, true, ["sign"]);
  const withMeta = { ...unsigned, issuer, keyId: jwk.kid || "org-key" };
  const payload = canonicalize(withMeta);
  const sig = await subtle.sign({ name:"ECDSA", hash:"SHA-256" }, key, Buffer.from(payload, "utf8"));
  const signature = b64u(sig);
  const signed = { ...withMeta, signature };
  const out = unsignedPath.replace(/\.json$/, ".signed.json");
  writeFileSync(out, JSON.stringify(signed, null, 2));
  return out;
}

async function signTrust(trustPath, jwkPath) {
  const trust = JSON.parse(readFileSync(trustPath, "utf8"));
  const { subtle } = crypto;
  const jwk = JSON.parse(readFileSync(jwkPath, "utf8"));
  const key = await subtle.importKey("jwk", jwk, { name: "ECDSA", namedCurve: "P-256" }, true, ["sign"]);
  const withMeta = { ...trust, issuer: jwk.kid || "org-key", keyId: jwk.kid || "org-key" };
  const payload = canonicalize(withMeta);
  const sig = await subtle.sign({ name:"ECDSA", hash:"SHA-256" }, key, Buffer.from(payload, "utf8"));
  const signature = b64u(sig);
  const signed = { ...withMeta, signature };
  const out = trustPath.replace(/\.json$/, ".signed.json");
  writeFileSync(out, JSON.stringify(signed, null, 2));
  return out;
}

function walkManifests(dir) {
  const files = [];
  if (!existsSync(dir)) return files;
  for (const name of readdirSync(dir)) {
    const p = resolve(dir, name);
    const st = statSync(p);
    if (st.isDirectory()) files.push(...walkManifests(p));
    else if (extname(p) === ".json" && !name.includes("signed") && !name.includes("private") && !name.includes("jwks")) {
      files.push(p);
    }
  }
  return files;
}

function parseArgs() {
  const args = process.argv.slice(2);
  const opts = { target: "repo", ref: "main", issuer: null };
  for (let i = 0; i < args.length; i++) {
    if (args[i] === "--target" && args[i+1]) {
      opts.target = args[i+1];
      i++;
    } else if (args[i] === "--ref" && args[i+1]) {
      opts.ref = args[i+1];
      i++;
    } else if (args[i] === "--issuer" && args[i+1]) {
      opts.issuer = args[i+1];
      i++;
    }
  }
  return opts;
}

async function publishToRepo(issuer, ref) {
  const repo = process.env.PUBLISH_REPO || process.env.GITHUB_REPOSITORY;
  const token = process.env.GITHUB_TOKEN;
  
  if (!repo) {
    throw new Error("PUBLISH_REPO or GITHUB_REPOSITORY must be set");
  }
  if (!token) {
    throw new Error("GITHUB_TOKEN must be set");
  }

  const [owner, repoName] = repo.split("/");
  const registryIssuer = issuer || `https://raw.githubusercontent.com/${owner}/${repoName}/${ref}`;
  
  console.log(`[publish] Publishing to GitHub repo: ${repo}`);
  console.log(`[publish] Issuer: ${registryIssuer}`);
  console.log(`[publish] Ref: ${ref}`);

  // Generate keys if not exist
  if (!existsSync("./registry/private.jwk.json")) {
    execSync("node packages/tools/registry-cli/src/index.js gen-key ./registry org-key", { stdio: "inherit" });
  }

  // Sign manifests
  const manifests = walkManifests("./manifests");
  for (const manifest of manifests) {
    await signManifest(manifest, "./registry/private.jwk.json", registryIssuer);
    console.log(`[publish] Signed: ${manifest}`);
  }

  // Create and sign trust.json with JWKS path: /jwks.json
  const trustJson = { issuers: [{ issuer: registryIssuer, jwksPath: "/jwks.json" }] };
  writeFileSync("./trust.json", JSON.stringify(trustJson, null, 2));
  await signTrust("./trust.json", "./registry/private.jwk.json");

  // Prepare files
  mkdirSync("./public/.well-known", { recursive: true });
  mkdirSync("./public/trust", { recursive: true });
  writeFileSync("./public/jwks.json", readFileSync("./registry/jwks.json", "utf8"));
  writeFileSync("./public/trust/trust.signed.json", readFileSync("./trust.signed.json", "utf8"));

  // Git operations
  execSync("git config user.name 'registry-publisher'", { stdio: "inherit" });
  execSync("git config user.email 'registry@nava.studio'", { stdio: "inherit" });
  
  execSync(`git remote set-url origin https://x-access-token:${token}@github.com/${repo}.git`, { stdio: "inherit" });
  execSync(`git checkout -b ${ref} || git checkout ${ref}`, { stdio: "inherit" });
  
  execSync("git add registry/jwks.json manifests/*.signed.json trust.signed.json public/jwks.json public/trust/trust.signed.json", { stdio: "inherit" });
  
  try {
    execSync(`git commit -m "Publish registry: ${new Date().toISOString()}"`, { stdio: "inherit" });
    execSync(`git push origin ${ref}`, { stdio: "inherit" });
    console.log(`[publish] ✅ Published to ${repo} (${ref})`);
    console.log(`[publish] 📦 Manifest URLs:`);
    const manifests = walkManifests("./manifests");
    for (const manifest of manifests) {
      const signed = manifest.replace(/\.json$/, ".signed.json");
      if (existsSync(signed)) {
        const name = signed.split("/").pop();
        console.log(`[publish]   ${registryIssuer}/${name}`);
      }
    }
  } catch (e) {
    console.log("[publish] No changes to commit or push failed");
  }
}

async function publishToS3(issuer) {
  const bucket = process.env.REGISTRY_BUCKET || process.env.S3_BUCKET;
  const region = process.env.AWS_REGION || "us-east-1";
  
  if (!bucket) {
    throw new Error("REGISTRY_BUCKET or S3_BUCKET must be set");
  }
  if (!process.env.AWS_ACCESS_KEY_ID || !process.env.AWS_SECRET_ACCESS_KEY) {
    throw new Error("AWS_ACCESS_KEY_ID and AWS_SECRET_ACCESS_KEY must be set");
  }

  // S3 issuer format: https://<bucket>.s3.<region>.amazonaws.com
  const registryIssuer = issuer || `https://${bucket}.s3.${region}.amazonaws.com`;
  
  console.log(`[publish] Publishing to S3: ${bucket}`);
  console.log(`[publish] Region: ${region}`);
  console.log(`[publish] Issuer: ${registryIssuer}`);
  console.log(`[publish] JWKS path: /jwks.json`);

  // Generate keys if not exist
  if (!existsSync("./registry/private.jwk.json")) {
    execSync("node packages/tools/registry-cli/src/index.js gen-key ./registry org-key", { stdio: "inherit" });
  }

  // Sign manifests
  const manifests = walkManifests("./manifests");
  for (const manifest of manifests) {
    await signManifest(manifest, "./registry/private.jwk.json", registryIssuer);
    console.log(`[publish] Signed: ${manifest}`);
  }

  // Create and sign trust.json with JWKS path: /jwks.json
  const trustJson = { issuers: [{ issuer: registryIssuer, jwksPath: "/jwks.json" }] };
  writeFileSync("./trust.json", JSON.stringify(trustJson, null, 2));
  await signTrust("./trust.json", "./registry/private.jwk.json");

  // Upload to S3 using AWS CLI
  // JWKS at root: /jwks.json
  execSync(`aws s3 cp ./registry/jwks.json s3://${bucket}/jwks.json --region ${region}`, { stdio: "inherit" });
  execSync(`aws s3 cp ./trust.signed.json s3://${bucket}/trust.signed.json --region ${region}`, { stdio: "inherit" });
  
  // Manifests at root: /vision-viewer.signed.json, etc.
  for (const manifest of manifests) {
    const signed = manifest.replace(/\.json$/, ".signed.json");
    if (existsSync(signed)) {
      const name = signed.split("/").pop();
      // Upload to root: https://bucket.s3.region.amazonaws.com/vision-viewer.signed.json
      execSync(`aws s3 cp ${signed} s3://${bucket}/${name} --region ${region}`, { stdio: "inherit" });
      console.log(`[publish] Published: ${registryIssuer}/${name}`);
    }
  }

  console.log(`[publish] ✅ Published to S3: ${bucket}`);
  console.log(`[publish] 📦 Manifest URLs:`);
  for (const manifest of manifests) {
    const signed = manifest.replace(/\.json$/, ".signed.json");
    if (existsSync(signed)) {
      const name = signed.split("/").pop();
      console.log(`[publish]   ${registryIssuer}/${name}`);
    }
  }
  
  console.log(`[publish] ⚠️  IMPORTANT: Configure S3 bucket for public access:`);
  console.log(`[publish]   1. Disable Block Public Access`);
  console.log(`[publish]   2. Apply bucket policy (see docs/S3_BUCKET_SETUP.md)`);
  console.log(`[publish]   3. Configure CORS for browser access`);
  console.log(`[publish]   4. Consider using CloudFront for HTTPS`);
}

async function publishToR2(issuer) {
  const accountId = process.env.CLOUDFLARE_ACCOUNT_ID;
  const bucket = process.env.R2_BUCKET;
  const accessKeyId = process.env.R2_ACCESS_KEY_ID;
  const secretAccessKey = process.env.R2_SECRET_ACCESS_KEY;
  
  if (!accountId || !bucket || !accessKeyId || !secretAccessKey) {
    throw new Error("CLOUDFLARE_ACCOUNT_ID, R2_BUCKET, R2_ACCESS_KEY_ID, and R2_SECRET_ACCESS_KEY must be set");
  }

  const registryIssuer = issuer || `https://${bucket}.${accountId}.r2.cloudflarestorage.com`;
  
  console.log(`[publish] Publishing to Cloudflare R2: ${bucket}`);
  console.log(`[publish] Issuer: ${registryIssuer}`);

  // Generate keys if not exist
  if (!existsSync("./registry/private.jwk.json")) {
    execSync("node packages/tools/registry-cli/src/index.js gen-key ./registry org-key", { stdio: "inherit" });
  }

  // Sign manifests
  const manifests = walkManifests("./manifests");
  for (const manifest of manifests) {
    await signManifest(manifest, "./registry/private.jwk.json", registryIssuer);
    console.log(`[publish] Signed: ${manifest}`);
  }

  // Create and sign trust.json with JWKS path: /jwks.json
  const trustJson = { issuers: [{ issuer: registryIssuer, jwksPath: "/jwks.json" }] };
  writeFileSync("./trust.json", JSON.stringify(trustJson, null, 2));
  await signTrust("./trust.json", "./registry/private.jwk.json");

  // Upload to R2 using AWS CLI (R2 is S3-compatible)
  const endpoint = `https://${accountId}.r2.cloudflarestorage.com`;
  
  // JWKS at root: /jwks.json
  execSync(`aws s3 cp ./registry/jwks.json s3://${bucket}/jwks.json --endpoint-url ${endpoint} --region auto`, { 
    stdio: "inherit",
    env: { ...process.env, AWS_ACCESS_KEY_ID: accessKeyId, AWS_SECRET_ACCESS_KEY: secretAccessKey }
  });
  execSync(`aws s3 cp ./trust.signed.json s3://${bucket}/trust.signed.json --endpoint-url ${endpoint} --region auto`, { 
    stdio: "inherit",
    env: { ...process.env, AWS_ACCESS_KEY_ID: accessKeyId, AWS_SECRET_ACCESS_KEY: secretAccessKey }
  });
  
  // Manifests at root: /vision-viewer.signed.json, etc.
  for (const manifest of manifests) {
    const signed = manifest.replace(/\.json$/, ".signed.json");
    if (existsSync(signed)) {
      const name = signed.split("/").pop();
      // Upload to root: https://bucket.r2.cloudflarestorage.com/vision-viewer.signed.json
      execSync(`aws s3 cp ${signed} s3://${bucket}/${name} --endpoint-url ${endpoint} --region auto`, { 
        stdio: "inherit",
        env: { ...process.env, AWS_ACCESS_KEY_ID: accessKeyId, AWS_SECRET_ACCESS_KEY: secretAccessKey }
      });
      console.log(`[publish] Published: ${registryIssuer}/${name}`);
    }
  }

  console.log(`[publish] ✅ Published to R2: ${bucket}`);
  console.log(`[publish] 📦 Manifest URLs:`);
  for (const manifest of manifests) {
    const signed = manifest.replace(/\.json$/, ".signed.json");
    if (existsSync(signed)) {
      const name = signed.split("/").pop();
      console.log(`[publish]   ${registryIssuer}/${name}`);
    }
  }
  
  console.log(`[publish] ⚠️  IMPORTANT: Configure R2 bucket for public access:`);
  console.log(`[publish]   1. Set bucket to public in Cloudflare dashboard`);
  console.log(`[publish]   2. Configure CORS if browsers will fetch module URLs`);
  console.log(`[publish]   3. Consider using Cloudflare Workers for custom domain`);
}

function validateTarget(target, allowedTargets) {
  if (!allowedTargets || allowedTargets.length === 0) {
    return true; // No restrictions
  }
  if (!allowedTargets.includes(target)) {
    throw new Error(`Target '${target}' not allowed. Allowed targets: ${allowedTargets.join(', ')}`);
  }
  return true;
}

async function main() {
  const opts = parseArgs();
  let { target, ref, issuer } = opts;

  // Support REGISTRY_TARGET environment variable
  if (!target) {
    target = process.env.REGISTRY_TARGET || "repo";
  }

  // Validate target against allowed targets
  const allowedTargetsStr = process.env.CI_ALLOWED_TARGETS || process.env.REGISTRY_TARGET || "";
  const allowedTargets = allowedTargetsStr ? allowedTargetsStr.split(",").map(t => t.trim()) : [];
  if (allowedTargets.length > 0) {
    validateTarget(target, allowedTargets);
    console.log(`[publish] 🔒 Target validation: ${target} is allowed`);
  }

  console.log(`[publish] Target: ${target}, Ref: ${ref}`);
  if (issuer) console.log(`[publish] Custom issuer: ${issuer}`);

  try {
    if (target === "r2") {
      await publishToR2(issuer);
    } else if (target === "s3") {
      await publishToS3(issuer);
    } else {
      await publishToRepo(issuer, ref);
    }
  } catch (error) {
    console.error(`[publish] ❌ Error: ${error.message}`);
    process.exit(1);
  }
}

main();

