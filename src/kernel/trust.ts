export type Issuer = { issuer: string; jwksPath?: string };
const KEY = "nava.trust.issuers";

export class TrustStore {
  getIssuers(): Issuer[] {
    try { return JSON.parse(localStorage.getItem(KEY)||"[]"); } catch { return []; }
  }
  setIssuers(list: Issuer[]) {
    localStorage.setItem(KEY, JSON.stringify(list));
  }
  add(issuer: Issuer) { const list = this.getIssuers(); list.push(issuer); this.setIssuers(list); }
  isTrusted(url: string) { return this.getIssuers().some(i => url.startsWith(i.issuer)); }
  jwksUrl(url: string) {
    const i = this.getIssuers().find(i => url.startsWith(i.issuer));
    return i ? new URL(i.jwksPath || "/.well-known/jwks.json", i.issuer).toString() : null;
  }
}

export type Jwks = { keys: Array<any> };
export async function fetchJWKS(jwksUrl: string): Promise<Jwks> {
  const res = await fetch(jwksUrl, { cache: "no-cache" });
  if (!res.ok) throw new Error("Failed to fetch JWKS");
  return await res.json();
}

