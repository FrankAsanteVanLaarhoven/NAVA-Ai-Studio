'use client'
import React, { useEffect, useMemo, useRef, useState } from "react";
import { listWidgets, getModel, setModel, removeFromDock, addSeparator, addFolder, addToFolder, removeItem, moveLeft, moveRight, renameFolder, extractFromFolder, removeFromFolder, rememberFocusAnchor } from "./registry";
import { Popover, PopoverTrigger, PopoverContent } from "../components/UI/Popover";
import { Input } from "../components/UI/Input";
import { FocusTrap } from "./focus-trap";
import type { DockItem, WidgetDef } from "./types";

function useDockModel() {
  const [tick, setTick] = useState(0);
  useEffect(() => {
    const id = setInterval(()=>setTick(t=>t+1), 800);
    return () => clearInterval(id);
  }, []);
  const model = getModel();
  const defs = new Map(listWidgets().map(w => [w.id, w]));
  return { model, defs, refresh: () => setTick(t=>t+1) };
}

function ContextMenu({ x, y, onClose, items }: { x:number, y:number, onClose:()=>void, items: Array<{label:string, onClick:()=>void}> }) {
  useEffect(() => {
    const h = () => onClose();
    document.addEventListener("click", h, true);
    return () => document.removeEventListener("click", h, true);
  }, [onClose]);
  return (
    <div className="fixed z-[10000] rounded-xl border border-white/10 bg-[#0b0f15] text-white text-sm shadow-lg"
         style={{ left: x, top: y }}>
      {items.map((it,i)=>(
        <div key={i} className="px-3 py-2 hover:bg-white/10 cursor-pointer" onClick={()=>{ it.onClick(); onClose(); }}>{it.label}</div>
      ))}
    </div>
  );
}

export default function DockHost() {
  const { model, defs, refresh } = useDockModel();
  const [menu, setMenu] = useState<{x:number,y:number,items:any[]} | null>(null);
  const [focus, setFocus] = useState(false);
  const [hoveredFolder, setHoveredFolder] = useState<string | null>(null);
  const [renameOpen, setRenameOpen] = useState<string | null>(null);
  const [renameVal, setRenameVal] = useState<string>("");
  const [trayIndex, setTrayIndex] = useState<number>(0);
  const trayRef = React.useRef<HTMLDivElement | null>(null);

  useEffect(() => {
    const h = (e: any) => setFocus(!!e?.detail?.on);
    window.addEventListener("nava:widgets:focus", h as any);
    const f = (e: any) => {
      const id = e?.detail?.id as string | undefined;
      let el: HTMLElement | null = null;
      if (id) {
        el = document.querySelector(`[data-dock-id="${id}"]`) as HTMLElement | null;
        if (!el) el = document.querySelector(`[data-folder-item-id="${id}"]`) as HTMLElement | null;
      }
      if (!el) el = document.querySelector('[data-dock-id]') as HTMLElement | null;
      el?.focus();
    };
    window.addEventListener("nava:widgets:return-focus", f as any);
    return () => {
      window.removeEventListener("nava:widgets:focus", h as any);
      window.removeEventListener("nava:widgets:return-focus", f as any);
    };
  }, []);

  function onDragStart(e: React.DragEvent, id: string) {
    e.dataTransfer.setData("text/plain", id);
    e.dataTransfer.effectAllowed = "move";
  }
  function onDragOver(e: React.DragEvent) { e.preventDefault(); e.dataTransfer.dropEffect = "move"; }
  function onDropOn(e: React.DragEvent, targetId: string) {
    e.preventDefault();
    const srcId = e.dataTransfer.getData("text/plain");
    if (!srcId || srcId === targetId) return;

    const m = getModel();
    let srcIdx = m.findIndex(x => x.id === srcId);
    const tgtIdx = m.findIndex(x => x.id === targetId);
    if (tgtIdx < 0) return;

    let src = srcIdx >= 0 ? m[srcIdx] : null;

    // If src not top-level, try extracting from a folder and create a widget item
    if (!src) {
      const extracted = extractFromFolder(srcId);
      if (extracted) {
        src = { type: 'widget', id: srcId } as any;
      } else {
        return;
      }
    } else {
      // if top-level, remove before reinsert
      m.splice(srcIdx,1);
    }

    const tgt = m[tgtIdx];

    // drop into folder
    if (tgt.type === "folder" && src.type === "widget") {
      addToFolder(tgt.id, (src as any).id);
      setModel(m);
      refresh();
      return;
    }

    const newIdx = m.findIndex(x => x.id === targetId);
    m.splice(newIdx, 0, src);
    setModel(m);
    refresh();
  }

  function openMenu(e: React.MouseEvent, it: DockItem) {
    e.preventDefault();
    const items: any[] = [];
    if (it.type === "widget") {
      items.push({ label: "Open", onClick: () => {
        const el = document.querySelector(`[data-dock-id="${it.id}"]`) as HTMLElement | null; el?.click();
      }});
      // list folders to add
      const folders = model.filter(x => x.type === "folder") as any[];
      for (const f of folders) {
        items.push({ label: `Add to Folder: ${f.title || f.id}`, onClick: () => { addToFolder(f.id, it.id); refresh(); } });
      }
      items.push({ label: "New Folder (put here)", onClick: () => {
        const id = addFolder("Stack");
        addToFolder(id, it.id); refresh();
      }});
      items.push({ label: "Add Separator After", onClick: () => { addSeparator(); refresh(); } });
      items.push({ label: "Move Left", onClick: () => { moveLeft(it.id); refresh(); } });
      items.push({ label: "Move Right", onClick: () => { moveRight(it.id); refresh(); } });
      items.push({ label: "Remove from Dock", onClick: () => { removeFromDock(it.id); refresh(); } });
    } else if (it.type === "separator") {
      items.push({ label: "Remove Separator", onClick: () => { removeItem(it.id); refresh(); } });
    } else if (it.type === "folder") {
      items.push({ label: "Open", onClick: () => {
        const el = document.querySelector(`[data-dock-id="${it.id}"]`) as HTMLElement | null; el?.click();
      }});
      items.push({ label: "Rename Folder", onClick: () => {
        setRenameOpen(it.id); setRenameVal((it as any).title || 'Stack');
      }});
      items.push({ label: "Remove Folder (keep items)", onClick: () => { removeItem(it.id); refresh(); } });
      items.push({ label: "Move Left", onClick: () => { moveLeft(it.id); refresh(); } });
      items.push({ label: "Move Right", onClick: () => { moveRight(it.id); refresh(); } });
    } else {
      items.push({ label: "Add Separator", onClick: () => { addSeparator(); refresh(); } });
    }
    setMenu({ x: e.clientX, y: e.clientY, items });
  }

  // Folder popover state
  const [openFolder, setOpenFolder] = useState<string | null>(null);

  if (!model.length) return null;

  return (
    <>
      {/* Focus overlay */}
      <div className={`fixed inset-0 pointer-events-none transition-opacity duration-200 ${focus ? "opacity-100" : "opacity-0"}`} style={{ background: "rgba(0,0,0,0.35)" }}></div>

      {/* Dock bar */}
      <div className="fixed bottom-3 left-1/2 -translate-x-1/2 z-[999] bg-white/5 backdrop-blur border border-white/10 rounded-2xl px-2 py-1 flex gap-1 shadow-lg">
        {model.map((it) => {
          if (it.type === "separator") {
            return <div key={it.id} onContextMenu={(e)=>openMenu(e,it)} className="w-[1px] h-8 bg-white/20 mx-1" />;
          }
          if (it.type === "folder") {
            const children = it.children.map(id => defs.get(id)).filter(Boolean) as WidgetDef[];
            return (
              <div key={it.id} className="relative"
                   onMouseEnter={()=>setHoveredFolder(it.id)} onMouseLeave={()=>setHoveredFolder(prev => prev===it.id ? null : prev)}>
                <div className="relative">
                  <div
                    draggable
                    onDragStart={(e)=>onDragStart(e, it.id)}
                    onDragOver={onDragOver}
                    onDrop={(e)=>onDropOn(e,it.id)}
                    onContextMenu={(e)=>openMenu(e,it)}
                    data-dock-id={it.id}
                    data-popover-trigger
                    className="rounded-xl hover:bg-white/10 transition-colors"
                    title={it.title || "Stack"}
                    onClick={()=>setOpenFolder(openFolder===it.id?null:it.id)}
                  >
                    <div className="px-2 py-1 w-8 h-8 flex items-center justify-center">📁</div>
                  </div>
                  {/* folder label */}
                  <div className="text-[10px] text-center mt-0.5 opacity-80 hover:opacity-100 cursor-text select-none"
                       onClick={(e)=>{ e.stopPropagation(); setRenameOpen(it.id); setRenameVal((it as any).title || 'Stack'); }}>
                    {(it as any).title || 'Stack'}
                  </div>
                  <Popover open={renameOpen===it.id} onOpenChange={(v)=>{ setRenameOpen(v?it.id:null); }}>
                    <PopoverTrigger asChild>
                      <button className="hidden" aria-hidden="true" />
                    </PopoverTrigger>
                    <PopoverContent className="rounded-2xl border border-white/10 bg-[#0b0f15] text-white p-3 w-64">
                      <div className="text-xs mb-2 opacity-80">Rename folder</div>
                      <FocusTrap initialFocus={`#rename-input-${it.id}`}>
                        <Input
                          id={`rename-input-${it.id}`}
                          autoFocus
                          value={renameVal}
                          onChange={(e)=>setRenameVal(e.target.value)}
                          onKeyDown={(e)=>{
                            if (e.key==='Enter') { renameFolder(it.id, renameVal.trim()||'Stack'); setRenameOpen(null); refresh(); }
                            if (e.key==='Escape') { setRenameOpen(null); }
                          }}
                        />
                        <div className="flex justify-end gap-2 mt-2">
                          <button className="rounded-xl px-3 py-1 bg-white/10 hover:bg-white/20 text-xs" onClick={()=>setRenameOpen(null)}>Cancel</button>
                          <button className="rounded-xl px-3 py-1 bg-white/10 hover:bg-white/20 text-xs" onClick={()=>{ renameFolder(it.id, renameVal.trim()||'Stack'); setRenameOpen(null); refresh(); }}>Save</button>
                        </div>
                      </FocusTrap>
                    </PopoverContent>
                  </Popover>
                </div>
                {/* Stack preview on hover */}
                {hoveredFolder === it.id && children.length > 0 && openFolder !== it.id && (
                  <div className="absolute bottom-12 left-1/2 -translate-x-1/2 bg-[#0b0f15] border border-white/10 rounded-2xl p-2 shadow-lg flex gap-2 max-w-[200px] overflow-x-auto">
                    {children.slice(0, 5).map(def => (
                      <div key={def.id} className="rounded-xl hover:bg-white/10 flex-shrink-0" title={def.title}>
                        <div className="w-8 h-8 flex items-center justify-center">
                          {def.component() as any}
                        </div>
                      </div>
                    ))}
                    {children.length > 5 && <div className="text-xs opacity-70 px-2 flex items-center">+{children.length - 5}</div>}
                  </div>
                )}
                {openFolder===it.id && (
                  <div ref={trayRef} className="absolute bottom-12 left-1/2 -translate-x-1/2 bg-[#0b0f15] border border-white/10 rounded-2xl p-2 shadow-lg flex gap-2 outline-none" role="toolbar" aria-label="Folder tray"
                       tabIndex={0}
                       onKeyDown={(e)=>{
                         const items = trayRef.current?.querySelectorAll('[data-tray-item="true"]');
                         if (!items || items.length===0) return;
                         if (e.key==='ArrowRight') { e.preventDefault(); const ni = (trayIndex+1)%items.length; setTrayIndex(ni); (items[ni] as HTMLElement).focus(); }
                         if (e.key==='ArrowLeft') { e.preventDefault(); const ni = (trayIndex-1+items.length)%items.length; setTrayIndex(ni); (items[ni] as HTMLElement).focus(); }
                         if (e.key==='Enter') { e.preventDefault(); const el = document.activeElement as HTMLElement | null; el?.click(); }
                       }}
                       onFocus={()=>{
                         // focus first item when the tray itself gets focus
                         const items = trayRef.current?.querySelectorAll('[data-tray-item="true"]');
                         if (items && items.length>0) { setTrayIndex(0); (items[0] as HTMLElement).focus(); }
                       }}
                  >
                    {children.length === 0 && <div className="text-xs opacity-70 px-2">Empty</div>}
                    {children.map((def, idx) => (
                      <div key={def.id} className="relative rounded-xl hover:bg-white/10 group focus-within:ring-2 focus-within:ring-white/30 focus-within:outline-none" title={def.title}
                           draggable
                           tabIndex={0}
                           data-tray-item="true"
                           data-folder-item-id={def.id}
                           onDragStart={(e)=>{ e.dataTransfer.setData('text/plain', def.id); e.dataTransfer.effectAllowed='move'; }}
                           onMouseDown={()=>rememberFocusAnchor(`[data-folder-item-id="${def.id}"]`)}
                           onClick={()=>{
                             const el = document.querySelector(`[data-dock-id="${def.id}"]`) as HTMLElement | null; el?.click();
                           }}
                           onKeyDown={(e)=>{
                             if (e.key==='Enter') {
                               e.preventDefault();
                               const el = document.querySelector(`[data-dock-id="${def.id}"]`) as HTMLElement | null; el?.click();
                             }
                           }}
                      >
                        <div className="w-8 h-8 flex items-center justify-center">
                          {def.component() as any}
                        </div>
                        <button
                          className="absolute -top-1 -right-1 w-4 h-4 rounded-full bg-red-500/80 hover:bg-red-500 text-white text-xs flex items-center justify-center opacity-0 group-hover:opacity-100 transition-opacity"
                          onClick={(e) => {
                            e.stopPropagation();
                            removeFromFolder(it.id, def.id);
                            refresh();
                          }}
                          title="Remove from folder"
                        >
                          ×
                        </button>
                      </div>
                    ))}
                  </div>
                )}
              </div>
            );
          }
          // widget
          const def = defs.get(it.id);
          return (
            <div key={it.id}
              draggable
              onDragStart={(e)=>onDragStart(e,it.id)}
              onDragOver={onDragOver}
              onDrop={(e)=>onDropOn(e,it.id)}
              onContextMenu={(e)=>openMenu(e,it)}
              className="rounded-xl hover:bg-white/10 transition-colors"
              title={def?.title || it.id}
            >
              <div className="px-2 py-1">
                <div data-dock-id={it.id} tabIndex={0} className="w-8 h-8 flex items-center justify-center focus:ring-1 focus:ring-white/20 outline-none"
                     onMouseDown={()=>rememberFocusAnchor(`[data-dock-id="${it.id}"]`)}>
                  {def ? (def.component() as any) : <div className="opacity-40 text-xs">{it.id[0]?.toUpperCase()}</div>}
                </div>
              </div>
            </div>
          );
        })}
        {/* end-drop zone */}
        <div onDragOver={(e)=>{e.preventDefault();}}
             onDrop={(e)=>{ e.preventDefault(); const id = e.dataTransfer.getData('text/plain'); if (!id) return; const m = getModel(); const idx = m.findIndex(x=>x.id===id); if (idx>=0) { const it=m.splice(idx,1)[0]; m.push(it); setModel(m); refresh(); } else { if (extractFromFolder(id)) { m.push({ type:'widget', id } as any); setModel(m); refresh(); } } }}
             className="w-4 h-8 ml-1 rounded bg-white/5 border border-white/10 opacity-30 hover:opacity-80" title="Drop to end" />
      </div>

      {menu && <ContextMenu x={menu.x} y={menu.y} items={menu.items} onClose={()=>setMenu(null)} />}
    </>
  );
}
