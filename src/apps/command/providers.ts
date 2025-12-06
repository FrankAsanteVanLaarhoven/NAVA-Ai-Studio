'use client'
import type { CommandProvider } from "./registry";

export const QuickActionsProvider: CommandProvider = {
  id: "quick-actions",
  async get(ctx) {
    try {
      const res = await fetch("/api/command/quick", { cache: "no-store" });
      if (!res.ok) return [];
      const actions = await res.json() as Array<{ id?: string; title: string; subtitle?: string; icon?: string; topic: string; payload?: any; capability?: string; group?: string }>;
      return actions.map(a => ({
        id: a.id || `qa:${a.topic}`,
        title: a.title,
        subtitle: a.subtitle,
        icon: a.icon || "⚡",
        capability: a.capability,
        group: a.group || "Quick",
        run: () => ctx.bus?.publish(a.topic, a.payload)
      }));
    } catch { return []; }
  }
};

export const RecentFilesProvider: CommandProvider = {
  id: "recent-files",
  async get(ctx) {
    try {
      const raw = localStorage.getItem("nava.recentFiles");
      if (!raw) return [];
      const items = JSON.parse(raw) as Array<{ path: string; title?: string; openedAt?: number; count?: number }>;
      return items.slice(0, 20).map((f, i) => ({
        id: `recent:${f.path}`,
        title: f.title || f.path.split(/[\\/]/).pop() || f.path,
        subtitle: f.path,
        icon: "📄",
        group: "Recent",
        run: () => ctx.bus?.publish("fs/open", { path: f.path })
      }));
    } catch { return []; }
  }
};

export const ROSGraphProvider: CommandProvider = {
  id: "ros-graph",
  async get(ctx) {
    try {
      const res = await fetch("/api/ros/graph", { cache: "no-store" });
      if (!res.ok) return [];
      const { topics = [], services = [] } = await res.json();
      const cmds: any[] = [];
      for (const t of topics.slice(0, 100)) {
        cmds.push({
          id: `ros:echo:${t.name}`,
          title: `Echo ${t.name}`,
          subtitle: t.type || "topic",
          icon: "🛰️",
          group: "ROS Topics",
          topicAccess: [{ pattern: "ros", mode: "sub" }],
          run: () => ctx.bus?.publish("ros2/topic/echo", { topic: t.name })
        });
        cmds.push({
          id: `ros:pub:${t.name}`,
          title: `Publish to ${t.name}`,
          subtitle: t.type || "topic",
          icon: "📡",
          group: "ROS Topics",
          topicAccess: [{ pattern: "ros", mode: "pub" }],
          run: () => ctx.bus?.publish("ros2/topic/publish", { topic: t.name })
        });
      }
      for (const s of services.slice(0, 100)) {
        cmds.push({
          id: `ros:call:${s.name}`,
          title: `Call ${s.name}`,
          subtitle: s.type || "service",
          icon: "🛠️",
          group: "ROS Services",
          topicAccess: [{ pattern: "ros", mode: "call" }],
          run: () => ctx.bus?.publish("ros2/service/call", { service: s.name })
        });
      }
      // Refresh graph
      cmds.push({
        id: "ros:graph:refresh",
        title: "Refresh ROS Graph",
        icon: "🔄",
        group: "ROS",
        topicAccess: [{ pattern: "ros", mode: "sub" }],
        run: () => ctx.bus?.publish("ros2/graph/refresh", {})
      });
      return cmds;
    } catch { return []; }
  }
};

export const RecentWorkspacesProvider: CommandProvider = {
  id: "recent-workspaces",
  async get(ctx) {
    try {
      const res = await fetch("/api/workspaces/recent", { cache: "no-store" });
      if (!res.ok) return [];
      const items = await res.json() as Array<{ path: string; title?: string; lastOpened?: number }>;
      return items.slice(0, 20).map(w => ({
        id: `ws:${w.path}`,
        title: w.title || w.path.split(/[\\/]/).pop() || w.path,
        subtitle: w.path,
        icon: "🗂️",
        group: "Workspaces",
        run: () => ctx.bus?.publish("workspace/open", { path: w.path })
      }));
    } catch { return []; }
  }
};

