'use client'
import React, { useEffect, useState } from "react";
import { createPortal } from "react-dom";

type Status = { 
  status?: string; 
  conclusion?: string; 
  html_url?: string; 
  error?: string; 
  updated_at?: string;
  id?: number;
};

export default function CIIndicatorWidget() {
  const [s, setS] = useState<Status | null>(null);
  
  async function poll() {
    try {
      // Use CI service instead of API route
      const { ciService } = await import("../../services/ci-service");
      const status = await ciService.getStatus();
      setS(status);
    } catch (e) {
      // Ignore errors
    }
  }
  
  useEffect(() => {
    poll();
    const id = setInterval(poll, 30000); // Poll every 30 seconds
    return () => clearInterval(id);
  }, []);

  const label = s?.error ? "error" : s?.conclusion || s?.status || "unknown";
  const ts = s?.updated_at ? new Date(s?.updated_at).toLocaleTimeString() : "";
  const cls = s?.error ? "bg-red-500/20 text-red-300" :
              (s?.status === "completed" && s?.conclusion === "success") ? "bg-green-500/20 text-green-300" :
              (s?.status === "in_progress" || s?.status === "queued") ? "bg-yellow-500/20 text-yellow-300" :
              "bg-white/10 text-white/70";

  const node = (
    <a 
      href={s?.html_url || "#"} 
      target={s?.html_url ? "_blank" : "_self"}
      className={`fixed top-3 right-3 z-[1000] rounded-xl px-2 py-1 text-xs ${cls}`}
      title={ts ? `Last update: ${ts}` : "CI status"}
      style={{ 
        position: 'fixed',
        top: '12px',
        right: '12px',
        zIndex: 1000,
        borderRadius: '12px',
        padding: '4px 8px',
        fontSize: '12px',
        textDecoration: 'none',
        backdropFilter: 'blur(10px)',
        border: '1px solid rgba(255, 255, 255, 0.1)'
      }}
    >
      {label}{ts ? ` • ${ts}` : ""}
    </a>
  );
  
  if (typeof document === "undefined") return null;
  return createPortal(node, document.body);
}

