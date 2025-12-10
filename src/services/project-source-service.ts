/**
 * Project Source Service
 * Handles loading projects from various sources:
 * - Local filesystem (Desktop)
 * - GitHub
 * - Slack
 * - Kaggle
 * - HuggingFace
 * - Cloud Storage (Drive, Dropbox, OneDrive)
 * - AirDrop
 */

export type ProjectSource = 
  | 'local'
  | 'github'
  | 'slack'
  | 'kaggle'
  | 'huggingface'
  | 'drive'
  | 'dropbox'
  | 'onedrive'
  | 'airdrop';

export interface ProjectSourceConfig {
  source: ProjectSource;
  name: string;
  icon: string;
  description: string;
  requiresAuth: boolean;
}

export const projectSources: ProjectSourceConfig[] = [
  {
    source: 'local',
    name: 'Local Folder',
    icon: '📁',
    description: 'Open from your computer',
    requiresAuth: false,
  },
  {
    source: 'github',
    name: 'GitHub',
    icon: '🐙',
    description: 'Clone from GitHub repository',
    requiresAuth: true,
  },
  {
    source: 'slack',
    name: 'Slack',
    icon: '💬',
    description: 'Import from Slack workspace',
    requiresAuth: true,
  },
  {
    source: 'kaggle',
    name: 'Kaggle',
    icon: '🏆',
    description: 'Import Kaggle dataset/project',
    requiresAuth: true,
  },
  {
    source: 'huggingface',
    name: 'HuggingFace',
    icon: '🤗',
    description: 'Import from HuggingFace Hub',
    requiresAuth: true,
  },
  {
    source: 'drive',
    name: 'Google Drive',
    icon: '☁️',
    description: 'Open from Google Drive',
    requiresAuth: true,
  },
  {
    source: 'dropbox',
    name: 'Dropbox',
    icon: '📦',
    description: 'Open from Dropbox',
    requiresAuth: true,
  },
  {
    source: 'onedrive',
    name: 'OneDrive',
    icon: '☁️',
    description: 'Open from OneDrive',
    requiresAuth: true,
  },
  {
    source: 'airdrop',
    name: 'AirDrop',
    icon: '📡',
    description: 'Receive via AirDrop',
    requiresAuth: false,
  },
];

class ProjectSourceService {
  /**
   * Open project from local filesystem
   */
  async openFromLocal(): Promise<{ path: string; files: any[] } | null> {
    return new Promise((resolve) => {
      // Use HTML5 File System Access API if available
      if ('showDirectoryPicker' in window) {
        (window as any).showDirectoryPicker().then(async (dirHandle: any) => {
          try {
            const files = await this.readDirectoryHandle(dirHandle);
            resolve({
              path: dirHandle.name,
              files,
            });
          } catch (error) {
            console.error('Error reading directory:', error);
            resolve(null);
          }
        }).catch(() => {
          // User cancelled
          resolve(null);
        });
      } else {
        // Fallback: use file input
        const input = document.createElement('input');
        input.type = 'file';
        input.webkitdirectory = true;
        input.multiple = true;
        input.onchange = async (e: any) => {
          const files = Array.from(e.target.files || []);
          const fileTree = await this.buildFileTree(files);
          resolve({
            path: 'Imported Project',
            files: fileTree,
          });
        };
        input.click();
      }
    });
  }

  /**
   * Clone from GitHub
   */
  async cloneFromGitHub(repoUrl: string): Promise<{ path: string; files: any[] } | null> {
    try {
      // Extract owner and repo from URL
      const match = repoUrl.match(/github\.com\/([^\/]+)\/([^\/]+)/);
      if (!match) {
        throw new Error('Invalid GitHub URL');
      }
      const [, owner, repo] = match;
      
      // Use GitHub API to fetch repository
      const response = await fetch(`https://api.github.com/repos/${owner}/${repo}/contents`, {
        headers: {
          'Accept': 'application/vnd.github.v3+json',
        },
      });
      
      if (!response.ok) {
        throw new Error('Failed to fetch repository');
      }
      
      const contents = await response.json();
      const files = await this.processGitHubContents(contents, owner, repo);
      
      return {
        path: repo,
        files,
      };
    } catch (error) {
      console.error('Error cloning from GitHub:', error);
      return null;
    }
  }

  /**
   * Import from HuggingFace
   */
  async importFromHuggingFace(repoId: string): Promise<{ path: string; files: any[] } | null> {
    try {
      const response = await fetch(`https://huggingface.co/api/datasets/${repoId}`);
      if (!response.ok) {
        throw new Error('Failed to fetch from HuggingFace');
      }
      const data = await response.json();
      // Process HuggingFace dataset structure
      return {
        path: repoId,
        files: [],
      };
    } catch (error) {
      console.error('Error importing from HuggingFace:', error);
      return null;
    }
  }

  /**
   * Read directory handle recursively
   */
  private async readDirectoryHandle(dirHandle: any, path: string = ''): Promise<any[]> {
    const entries: any[] = [];
    
    for await (const entry of dirHandle.values()) {
      if (entry.kind === 'file') {
        const file = await entry.getFile();
        const fileContent = await file.text();
        const filePath = path ? `${path}/${entry.name}` : entry.name;
        
        entries.push({
          path: filePath,
          name: entry.name,
          isDirectory: false,
          content: fileContent, // Store content for immediate access
        });
        
        // Also store in file service immediately
        try {
          const { fileService } = await import('./file-service');
          await fileService.createFile(filePath, fileContent);
        } catch (error) {
          console.warn('Could not store file in file service:', error);
        }
      } else if (entry.kind === 'directory') {
        const subDir = await entry.getDirectory();
        const dirPath = path ? `${path}/${entry.name}` : entry.name;
        entries.push({
          path: dirPath,
          name: entry.name,
          isDirectory: true,
          children: await this.readDirectoryHandle(subDir, dirPath),
        });
      }
    }
    
    return entries;
  }

  /**
   * Build file tree from FileList
   */
  private async buildFileTree(files: File[]): Promise<any[]> {
    const tree: any = {};
    
    // Process all files and store their content
    const filePromises = files.map(async (file) => {
      const parts = file.webkitRelativePath.split('/');
      let current = tree;
      
      for (let i = 0; i < parts.length - 1; i++) {
        const part = parts[i];
        if (!current[part]) {
          current[part] = {};
        }
        current = current[part];
      }
      
      const fileName = parts[parts.length - 1];
      const content = await file.text();
      current[fileName] = { file, content };
      
      // Store in file service immediately
      try {
        const { fileService } = await import('./file-service');
        await fileService.createFile(file.webkitRelativePath, content);
      } catch (error) {
        console.warn('Could not store file in file service:', error);
      }
    });
    
    await Promise.all(filePromises);
    return this.convertTreeToFileEntries(tree, '');
  }

  private convertTreeToFileEntries(tree: any, path: string): any[] {
    const entries: any[] = [];
    
    for (const [name, value] of Object.entries(tree)) {
      const currentPath = path ? `${path}/${name}` : name;
      
      if (value && typeof value === 'object' && 'file' in value && 'content' in value) {
        // File with content
        entries.push({
          path: currentPath,
          name,
          isDirectory: false,
          content: value.content, // Include content for immediate access
        });
      } else if (value instanceof File) {
        // Legacy File object
        entries.push({
          path: currentPath,
          name,
          isDirectory: false,
        });
      } else {
        // Directory
        entries.push({
          path: currentPath,
          name,
          isDirectory: true,
          children: this.convertTreeToFileEntries(value, currentPath),
        });
      }
    }
    
    return entries;
  }

  private async processGitHubContents(contents: any[], owner: string, repo: string, path: string = ''): Promise<any[]> {
    const entries: any[] = [];
    
    for (const item of contents) {
      if (item.type === 'file') {
        try {
          const fileResponse = await fetch(item.download_url);
          const content = await fileResponse.text();
          entries.push({
            path: item.path,
            name: item.name,
            isDirectory: false,
            content,
          });
        } catch (error) {
          console.error(`Error fetching file ${item.path}:`, error);
        }
      } else if (item.type === 'dir') {
        const dirResponse = await fetch(`https://api.github.com/repos/${owner}/${repo}/contents/${item.path}`);
        const dirContents = await dirResponse.json();
        entries.push({
          path: item.path,
          name: item.name,
          isDirectory: true,
          children: await this.processGitHubContents(dirContents, owner, repo, item.path),
        });
      }
    }
    
    return entries;
  }
}

export const projectSourceService = new ProjectSourceService();

