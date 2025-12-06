import React, { useState } from "react";
import { Input } from "../../components/UI";
import { Button } from "../../components/UI";
import { useKernel } from "../../kernel/kernel";
import { register } from "../../kernel/registry";
import { loadRemoteManifest, materializeManifest, verifyManifest } from "../../kernel/remote";

// Load registry keys (in production, fetch from trusted origin)
async function loadRegistryKeys(): Promise<Record<string, JsonWebKey>> {
  try {
    const res = await fetch("/registry/jwks.json");
    if (res.ok) {
      const data = await res.json();
      const keys: Record<string, JsonWebKey> = {};
      for (const key of data.keys || []) {
        // Remove key_ops and ext for verification (not part of JWK standard for import)
        const { key_ops, ext, ...jwk } = key;
        keys[key.kid] = jwk as JsonWebKey;
      }
      return keys;
    }
  } catch (e) {
    console.warn("Failed to load registry keys:", e);
  }
  // Fallback to demo key
  return {
    "org-key": {
      "kty":"EC","crv":"P-256",
      "x":"AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA",
      "y":"BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB"
    }
  };
}

export function InstallFromUrl() {
  const { } = useKernel();
  const [url, setUrl] = useState("");
  const [status, setStatus] = useState<string>("");

  async function install() {
    try {
      setStatus("Fetching manifest…");
      const m = await loadRemoteManifest(url);
      setStatus("Verifying signature…");
      const keys = await loadRegistryKeys();
      await verifyManifest(m, keys).catch(e => { throw e; });
      const manifest = await materializeManifest(m);
      register(manifest);
      setStatus("Installed " + manifest.title);
    } catch (error: any) {
      setStatus("Error: " + (error.message || String(error)));
    }
  }

  return (
    <div className="flex gap-2 items-center">
      <Input 
        value={url} 
        onChange={(e) => setUrl(e.target.value)} 
        placeholder="https://registry/app.json" 
        className="flex-1"
      />
      <Button onClick={install}>Install</Button>
      {status && <div className="text-xs opacity-70 self-center">{status}</div>}
    </div>
  );
}

