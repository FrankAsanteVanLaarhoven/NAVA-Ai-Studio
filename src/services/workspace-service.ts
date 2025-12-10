export interface Workspace {
  id: string;
  name: string;
  folders: WorkspaceFolder[];
  settings?: Record<string, any>;
  createdAt: Date;
  lastModified: Date;
}

export interface WorkspaceFolder {
  path: string;
  name: string;
}

export interface RecentWorkspace {
  workspace: Workspace;
  lastOpened: Date;
  openCount: number;
}

class WorkspaceService {
  private currentWorkspace: Workspace | null = null;
  private recentWorkspaces: RecentWorkspace[] = [];
  private readonly MAX_RECENT = 10;

  constructor() {
    this.loadCurrentWorkspace();
    this.loadRecentWorkspaces();
  }

  // Workspace Management
  createWorkspace(name: string, folders: WorkspaceFolder[]): Workspace {
    const workspace: Workspace = {
      id: `workspace-${Date.now()}`,
      name,
      folders,
      createdAt: new Date(),
      lastModified: new Date(),
    };

    this.setCurrentWorkspace(workspace);
    this.addToRecent(workspace);
    return workspace;
  }

  setCurrentWorkspace(workspace: Workspace): void {
    this.currentWorkspace = workspace;
    workspace.lastModified = new Date();
    this.saveCurrentWorkspace();
    this.addToRecent(workspace);
    this.notifyWorkspaceChanged();
  }

  getCurrentWorkspace(): Workspace | null {
    return this.currentWorkspace;
  }

  addFolderToWorkspace(folderPath: string): void {
    if (!this.currentWorkspace) {
      // Create default workspace
      this.currentWorkspace = this.createWorkspace('Untitled Workspace', []);
    }

    const folderName = folderPath.split('/').pop() || folderPath;
    const folder: WorkspaceFolder = {
      path: folderPath,
      name: folderName,
    };

    // Check if folder already exists
    if (!this.currentWorkspace.folders.some(f => f.path === folderPath)) {
      this.currentWorkspace.folders.push(folder);
      this.currentWorkspace.lastModified = new Date();
      this.saveCurrentWorkspace();
      this.notifyWorkspaceChanged();
    }
  }

  removeFolderFromWorkspace(folderPath: string): void {
    if (this.currentWorkspace) {
      this.currentWorkspace.folders = this.currentWorkspace.folders.filter(
        f => f.path !== folderPath
      );
      this.currentWorkspace.lastModified = new Date();
      this.saveCurrentWorkspace();
      this.notifyWorkspaceChanged();
    }
  }

  saveWorkspaceAs(name: string): Workspace {
    if (!this.currentWorkspace) {
      throw new Error('No current workspace to save');
    }

    const newWorkspace: Workspace = {
      ...this.currentWorkspace,
      id: `workspace-${Date.now()}`,
      name,
      createdAt: new Date(),
      lastModified: new Date(),
    };

    this.setCurrentWorkspace(newWorkspace);
    return newWorkspace;
  }

  duplicateWorkspace(): Workspace {
    if (!this.currentWorkspace) {
      throw new Error('No current workspace to duplicate');
    }

    const duplicated: Workspace = {
      ...this.currentWorkspace,
      id: `workspace-${Date.now()}`,
      name: `${this.currentWorkspace.name} (Copy)`,
      createdAt: new Date(),
      lastModified: new Date(),
    };

    this.setCurrentWorkspace(duplicated);
    return duplicated;
  }

  // Recent Workspaces
  getRecentWorkspaces(): RecentWorkspace[] {
    return this.recentWorkspaces.sort((a, b) => 
      b.lastOpened.getTime() - a.lastOpened.getTime()
    );
  }

  private addToRecent(workspace: Workspace): void {
    const existing = this.recentWorkspaces.findIndex(r => r.workspace.id === workspace.id);
    
    if (existing >= 0) {
      this.recentWorkspaces[existing].lastOpened = new Date();
      this.recentWorkspaces[existing].openCount += 1;
    } else {
      this.recentWorkspaces.push({
        workspace,
        lastOpened: new Date(),
        openCount: 1,
      });
    }

    // Keep only recent MAX_RECENT workspaces
    this.recentWorkspaces = this.recentWorkspaces
      .sort((a, b) => b.lastOpened.getTime() - a.lastOpened.getTime())
      .slice(0, this.MAX_RECENT);

    this.saveRecentWorkspaces();
  }

  openRecentWorkspace(workspaceId: string): Workspace | null {
    const recent = this.recentWorkspaces.find(r => r.workspace.id === workspaceId);
    if (recent) {
      this.setCurrentWorkspace(recent.workspace);
      return recent.workspace;
    }
    return null;
  }

  // Workspace Sharing (simplified - would use actual sharing service in production)
  async shareWorkspace(workspaceId: string): Promise<string> {
    const workspace = this.currentWorkspace;
    if (!workspace || workspace.id !== workspaceId) {
      throw new Error('Workspace not found');
    }

    // In production, this would upload to a sharing service
    const shareData = {
      workspace,
      shareId: `share-${Date.now()}`,
      createdAt: new Date(),
    };

    // Store share data (in production, this would be on a server)
    localStorage.setItem(`nava-share-${shareData.shareId}`, JSON.stringify(shareData));

    return shareData.shareId;
  }

  async importSharedWorkspace(shareId: string): Promise<Workspace | null> {
    const shareData = localStorage.getItem(`nava-share-${shareId}`);
    if (shareData) {
      try {
        const data = JSON.parse(shareData);
        const workspace: Workspace = {
          ...data.workspace,
          id: `workspace-${Date.now()}`,
          name: `${data.workspace.name} (Shared)`,
          createdAt: new Date(),
          lastModified: new Date(),
        };
        this.setCurrentWorkspace(workspace);
        return workspace;
      } catch (error) {
        console.error('[WorkspaceService] Failed to import shared workspace:', error);
      }
    }
    return null;
  }

  // Persistence
  private saveCurrentWorkspace(): void {
    if (this.currentWorkspace) {
      localStorage.setItem('nava-current-workspace', JSON.stringify({
        ...this.currentWorkspace,
        createdAt: this.currentWorkspace.createdAt.toISOString(),
        lastModified: this.currentWorkspace.lastModified.toISOString(),
      }));
    }
  }

  private loadCurrentWorkspace(): void {
    const saved = localStorage.getItem('nava-current-workspace');
    if (saved) {
      try {
        const data = JSON.parse(saved);
        this.currentWorkspace = {
          ...data,
          createdAt: new Date(data.createdAt),
          lastModified: new Date(data.lastModified),
        };
      } catch (error) {
        console.error('[WorkspaceService] Failed to load current workspace:', error);
      }
    }
  }

  private saveRecentWorkspaces(): void {
    const data = this.recentWorkspaces.map(r => ({
      workspace: {
        ...r.workspace,
        createdAt: r.workspace.createdAt.toISOString(),
        lastModified: r.workspace.lastModified.toISOString(),
      },
      lastOpened: r.lastOpened.toISOString(),
      openCount: r.openCount,
    }));
    localStorage.setItem('nava-recent-workspaces', JSON.stringify(data));
  }

  private loadRecentWorkspaces(): void {
    const saved = localStorage.getItem('nava-recent-workspaces');
    if (saved) {
      try {
        const data = JSON.parse(saved);
        this.recentWorkspaces = data.map((r: any) => ({
          workspace: {
            ...r.workspace,
            createdAt: new Date(r.workspace.createdAt),
            lastModified: new Date(r.workspace.lastModified),
          },
          lastOpened: new Date(r.lastOpened),
          openCount: r.openCount,
        }));
      } catch (error) {
        console.error('[WorkspaceService] Failed to load recent workspaces:', error);
        this.recentWorkspaces = [];
      }
    }
  }

  private notifyWorkspaceChanged(): void {
    const event = new CustomEvent('nava:workspace-changed', {
      detail: { workspace: this.currentWorkspace },
    });
    window.dispatchEvent(event);
  }
}

export const workspaceService = new WorkspaceService();

