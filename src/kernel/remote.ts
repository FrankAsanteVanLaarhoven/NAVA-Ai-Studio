import { lt as semverLt } from "semver";
import type { AppManifest } from "./types";
import { canonicalize } from "./canonical";
import { importP256Key, verifyP256, utf8 } from "./crypto";
import type { SignedManifest } from "../protocols";
import { SignedManifest as SignedManifestSchema } from "../protocols";

// Registry key ring (demo). In production, fetch this from a trusted origin.
export type KeyRing = Record<string, JsonWebKey>;

export async function loadRemoteManifest(url: string): Promise<SignedManifest> {
  const res = await fetch(url, { cache: "no-cache" });
  if (!res.ok) throw new Error(`Failed to fetch manifest: ${res.status}`);
  const json = await res.json();
  const m = SignedManifestSchema.parse(json);
  return m;
}

export async function verifyManifest(m: SignedManifest, keys: KeyRing) {
  const jwk = keys[m.keyId];
  if (!jwk) throw new Error(`Unknown keyId: ${m.keyId}`);
  const key = await importP256Key(jwk);
  const { signature, ...unsigned } = m;
  const payload = canonicalize(unsigned);
  const ok = await verifyP256(m.signature, utf8(payload), key);
  if (!ok) throw new Error("Manifest signature invalid");
  return true;
}

export async function materializeManifest(m: SignedManifest): Promise<AppManifest> {
  return {
    id: m.id,
    title: m.title,
    version: m.version,
    icon: m.icon,
    capabilities: m.capabilities as any,
    entry: async () => {
      // Import a remote ESM module. Requires CORS and proper headers.
      // eslint-disable-next-line @typescript-eslint/ban-ts-comment
      // @ts-ignore
      const mod = await import(/* webpackIgnore: true */ m.module.url);
      const App = mod[m.module.export || "default"];
      return App;
    }
  };
}

export function isNewer(a: string, b: string) {
  // true if a > b
  try { return semverLt(b, a); } catch { return false; }
}

