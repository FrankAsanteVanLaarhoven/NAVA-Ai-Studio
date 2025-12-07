/**
 * NAVΛ Studio Policy Engine
 */

export class PolicyEngine {
  check(capability: string): boolean {
    // Default: allow all for now
    return true;
  }
}

