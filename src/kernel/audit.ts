/**
 * NAVΛ Studio Audit Chain
 */

export class AuditChain {
  private entries: any[] = [];

  log(entry: any): void {
    this.entries.push({
      ...entry,
      timestamp: Date.now(),
    });
  }

  getEntries(): any[] {
    return [...this.entries];
  }
}

