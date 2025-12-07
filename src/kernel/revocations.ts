/**
 * NAVΛ Studio Revocation Store
 */

export class RevocationStore {
  private revoked: Set<string> = new Set();

  revoke(id: string): void {
    this.revoked.add(id);
  }

  isRevoked(id: string): boolean {
    return this.revoked.has(id);
  }
}

