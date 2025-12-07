/**
 * NAVΛ Studio Kernel Types
 */

export interface AppManifest {
  id: string;
  name: string;
  version: string;
  description?: string;
  icon?: string;
  category?: string;
  author?: string;
  keywords?: string[];
  capabilities?: string[];
  routes?: Array<{ path: string; component: string }>;
  actions?: Array<{
    id: string;
    title: string;
    prefix?: string;
    description?: string;
    shortcut?: string;
  }>;
  dock?: {
    enabled: boolean;
    icon: string;
    name: string;
    description?: string;
  };
  [key: string]: any;
}

export type Capability = string;

export type Message = {
  type: string;
  payload?: any;
  sender?: string;
  target?: string;
  timestamp?: number;
};

export type Permission = {
  id: string;
  name: string;
  description: string;
  granted: boolean;
};

export type TrustLevel = 'untrusted' | 'basic' | 'elevated' | 'full';

export type RunningApp = {
  instanceId: string;
  manifest: AppManifest;
  capabilities: Capability[];
};

