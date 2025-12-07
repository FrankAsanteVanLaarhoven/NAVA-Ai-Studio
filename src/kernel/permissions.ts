/**
 * NAVΛ Studio Permission Store
 */

import type { Capability } from './types';

export class PermissionStore {
  private granted: Set<string> = new Set();

  grant(capability: Capability): void {
    this.granted.add(capability);
  }

  revoke(capability: Capability): void {
    this.granted.delete(capability);
  }

  has(capability: Capability): boolean {
    return this.granted.has(capability);
  }

  getAll(): Capability[] {
    return Array.from(this.granted);
  }

  clear(): void {
    this.granted.clear();
  }
}

