/**
 * NAVΛ Studio ACL Store
 */

export class ACLStore {
  private acls: Map<string, string[]> = new Map();

  setACL(resource: string, permissions: string[]): void {
    this.acls.set(resource, permissions);
  }

  getACL(resource: string): string[] {
    return this.acls.get(resource) || [];
  }

  check(resource: string, permission: string): boolean {
    const acl = this.getACL(resource);
    return acl.includes(permission) || acl.includes('*');
  }
}

