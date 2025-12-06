'use client'
import React, { useEffect, useMemo, useRef, useState } from "react";
import { FocusScope } from "../../widgets";
import { resolveAllCommands, bumpUsage, decayStats, allowedByCapabilities, fuzzySearch, Command } from "./registry";
import { QuickActionsProvider, RecentFilesProvider, ROSGraphProvider, RecentWorkspacesProvider } from "./providers";
import { TopicBus } from "../../kernel";
import { useKernel } from "../../kernel";

type Props = { open: boolean; onClose: () => void };

export default function CommandPalette({ open, onClose }: Props) {
  const ref = useRef<HTMLDivElement|null>(null);
  const [q, setQ] = useState("");
  const [idx, setIdx] = useState(0);
  const [all, setAll] = useState<Command[]>([]);
  const { apps, perms, bus } = useKernel();

  useEffect(() => {
    // register providers (idempotent if duplicate ids)
    import('./registry').then(m => m.registerProvider && m.registerProvider(QuickActionsProvider));
    import('./registry').then(m => m.registerProvider && m.registerProvider(RecentFilesProvider));
    import('./registry').then(m => m.registerProvider && m.registerProvider(ROSGraphProvider));
    import('./registry').then(m => m.registerProvider && m.registerProvider(RecentWorkspacesProvider));
    // apply usage decay occasionally
    try { decayStats(); } catch {}
    const ctx = { apps, perms, bus: new TopicBus(bus) } as any;
    resolveAllCommands(ctx).then(setAll);
  }, [open, apps, perms, bus]);

  const filtered = useMemo(() => {
    // capability-aware: hide cmds with capability unmet
    const granted = new Set<string>(apps.flatMap(a => perms.get(a.id) || []));
    const items = all.filter(c => (!c.capability || granted.has(c.capability)) && allowedByCapabilities(granted, c.topicAccess));
    return fuzzySearch(q, items);
  }, [q, all, apps, perms]);

  useEffect(() => { setIdx(0); }, [q, open]);

  useEffect(() => {
    function onKey(ev: KeyboardEvent) {
      if (!open) return;
      if (ev.key === "ArrowDown") { ev.preventDefault(); setIdx(i => Math.min(i+1, Math.max(0, filtered.length-1))); }
      if (ev.key === "ArrowUp") { ev.preventDefault(); setIdx(i => Math.max(i-1, 0)); }
      if (ev.key === "Enter") {
        ev.preventDefault();
        const it = filtered[idx];
        if (it) { try { bumpUsage(it.id); } catch {} Promise.resolve(it.run()).finally(onClose); }
      }
    }
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [open, filtered, idx, onClose]);

  if (!open) return null;
  return (
    <div className="fixed inset-0 z-[2000] grid place-items-start pt-24" onClick={onClose}>
      <div className="rounded-2xl border border-white/10 bg-[#0b0f15] text-white p-3 w-[760px]" onClick={e=>e.stopPropagation()}>
        <FocusScope initialFocus="input[type='text']" onEscape={onClose}>
          <div className="flex items-center gap-2 px-2 pb-2">
            <span className="opacity-60 text-sm">⌘K</span>
            <input
              type="text"
              className="w-full rounded bg-white/10 p-2 text-white placeholder-white/50 outline-none focus:ring-1 focus:ring-white/20"
              placeholder="Type a command…"
              value={q}
              onChange={(e)=>setQ(e.target.value)}
              onKeyDown={(e)=>{ if (e.key==='Escape') onClose(); }}
            />
          </div>
          <div className="max-h-[420px] overflow-auto">
            {filtered.length === 0 && (
              <div className="px-3 py-8 text-sm opacity-70">No matches.</div>
            )}
            {filtered.map((it, i) => (
              <button key={it.id} onClick={()=>{ try { bumpUsage(it.id); } catch {} Promise.resolve(it.run()).finally(onClose); }}
                className={`w-full text-left px-3 py-2 rounded-xl ${i===idx ? 'bg-white/10' : 'hover:bg-white/5'}`}>
                <div className="flex items-center justify-between">
                  <div className="flex items-center gap-2">
                    <span className="w-6 text-center">{it.icon || '›'}</span>
                    <div>
                      <div className="text-sm">{it.title}</div>
                      {it.subtitle && <div className="text-xs opacity-70">{it.subtitle}</div>}
                    </div>
                  </div>
                  {it.group && <div className="text-[10px] opacity-60">{it.group}</div>}
                </div>
              </button>
            ))}
          </div>
        </FocusScope>
      </div>
    </div>
  );
}

