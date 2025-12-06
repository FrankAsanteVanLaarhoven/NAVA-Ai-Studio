import React, { createContext, useContext, useMemo, useState, useEffect } from "react";
import type { AppManifest, Capability, Message } from "./types";
import { Bus } from "./bus";
import { list as listApps } from "./registry";
import { PermissionStore } from "./permissions";
import { initializeBootstrapTrust } from "./trust-config";
import { PolicyEngine } from "./policy";
import { AuditChain } from "./audit";
import { RevocationStore } from "./revocations";
import { ACLStore } from "./acl";
import { TrustStore } from "./trust";

export type RunningApp = {
  instanceId: string;
  manifest: AppManifest;
  capabilities: Capability[]; // granted at launch
};

type KernelCtx = {
  bus: Bus;
  perms: PermissionStore;
  policy: PolicyEngine;
  audit: AuditChain;
  revocations: RevocationStore;
  running: RunningApp[];
  apps: AppManifest[];
  launch: (id: string, caps?: Capability[]) => Promise<RunningApp | null>;
  kill: (instanceId: string) => void;
  send: (instanceId: string, msg: Message) => void;
};

const KernelContext = createContext<KernelCtx | null>(null);

export function useKernel() {
  const k = useContext(KernelContext);
  if (!k) throw new Error("KernelProvider missing");
  return k;
}

export function KernelProvider({ children }: { children: React.ReactNode }) {
  const bus = useMemo(() => new Bus(), []);
  const perms = useMemo(() => new PermissionStore(bus), [bus]);
  const [running, setRunning] = useState<RunningApp[]>([]);
  const apps = useMemo(() => listApps(), []);

  // Initialize bootstrap trust on mount
  useEffect(() => {
    initializeBootstrapTrust();
  }, []);

  async function launch(id: string, caps: Capability[] = []) {
    const manifest = apps.find((a) => a.id === id);
    if (!manifest) return null;
    const requested = caps.length ? caps : (manifest.capabilities as Capability[]);
    const granted = await perms.ensure(manifest.id, manifest.title, requested);
    const instanceId = `${id}-${Math.random().toString(36).slice(2, 8)}`;
    const r: RunningApp = { instanceId, manifest, capabilities: granted };
    setRunning((s) => [...s, r]);
    return r;
  }

  function kill(instanceId: string) {
    setRunning((s) => s.filter((r) => r.instanceId != instanceId));
  }

  function send(instanceId: string, msg: Message) {
    bus.emit(`app:${instanceId}`, msg);
  }

  const value: KernelCtx = { bus, perms, running, apps, launch, kill, send };
  return <KernelContext.Provider value={value}>{children}</KernelContext.Provider>;
}

