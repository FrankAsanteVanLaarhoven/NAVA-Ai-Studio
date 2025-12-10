import { invoke } from '@tauri-apps/api/tauri';

export interface FileEntry {
  path: string;
  name: string;
  isDirectory: boolean;
  children?: FileEntry[];
}

export interface Project {
  name: string;
  path: string;
  files: FileEntry[];
  createdAt: Date;
  lastModified: Date;
}

class FileService {
  private recentFiles: string[] = [];
  private maxRecentFiles = 10;
  private currentProject: Project | null = null;
  private readonly CURRENT_PROJECT_KEY = 'navlambda-current-project';

  constructor() {
    this.loadRecentFiles();
    this.loadCurrentProject();
  }

  getCurrentProject(): Project | null {
    if (!this.currentProject) {
      this.loadCurrentProject();
    }
    return this.currentProject;
  }

  setCurrentProject(project: Project | null) {
    this.currentProject = project;
    if (project) {
      // Persist current project to localStorage
      try {
        localStorage.setItem(this.CURRENT_PROJECT_KEY, JSON.stringify({
          ...project,
          createdAt: project.createdAt.toISOString(),
          lastModified: project.lastModified.toISOString(),
        }));
        console.log('[FileService] Saved current project to localStorage');
      } catch (error) {
        console.error('[FileService] Failed to save current project:', error);
      }
    } else {
      localStorage.removeItem(this.CURRENT_PROJECT_KEY);
    }
  }

  private async loadCurrentProject(): Promise<void> {
    try {
      const saved = localStorage.getItem(this.CURRENT_PROJECT_KEY);
      if (saved) {
        const project = JSON.parse(saved) as Project;
        project.createdAt = new Date(project.createdAt);
        project.lastModified = new Date(project.lastModified);
        // Reload files from storage
        await this.reloadProjectFiles(project);
        this.currentProject = project;
        console.log('[FileService] Loaded current project from localStorage:', project.name, 'Files:', project.files.length);
      }
    } catch (error) {
      console.error('[FileService] Failed to load current project:', error);
    }
  }

  private async reloadProjectFiles(project: Project): Promise<void> {
    try {
      // Rebuild file tree from localStorage
      const files = await this.readDirectory(project.path);
      project.files = files;
      console.log('[FileService] Reloaded project files:', files.length);
    } catch (error) {
      console.error('[FileService] Failed to reload project files:', error);
      // Keep existing files if reload fails
    }
  }

  private loadRecentFiles() {
    try {
      const stored = localStorage.getItem('navlambda-recent-files');
      if (stored) {
        this.recentFiles = JSON.parse(stored);
      }
    } catch (error) {
      console.error('Error loading recent files:', error);
    }
  }

  private saveRecentFiles() {
    try {
      localStorage.setItem('navlambda-recent-files', JSON.stringify(this.recentFiles));
    } catch (error) {
      console.error('Error saving recent files:', error);
    }
  }

  addRecentFile(path: string) {
    this.recentFiles = this.recentFiles.filter((f) => f !== path);
    this.recentFiles.unshift(path);
    if (this.recentFiles.length > this.maxRecentFiles) {
      this.recentFiles = this.recentFiles.slice(0, this.maxRecentFiles);
    }
    this.saveRecentFiles();
  }

  getRecentFiles(): string[] {
    return [...this.recentFiles];
  }

  clearRecentFiles() {
    this.recentFiles = [];
    this.saveRecentFiles();
  }

  async readFile(path: string): Promise<string> {
    try {
      if (window.__TAURI__) {
        const content = await invoke<string>('read_file', { path });
        this.addRecentFile(path);
        return content;
      }
      
      // Try normalized path first
      const normalizedPath = this.normalizePath(path);
      let content = localStorage.getItem(`file:${normalizedPath}`);
      
      // Try original path if normalized doesn't work
      if (!content) {
        content = localStorage.getItem(`file:${path}`);
      }
      
      // Try with leading slash
      if (!content && !path.startsWith('/')) {
        content = localStorage.getItem(`file:/${path}`);
      }
      
      // Try without leading slash
      if (!content && path.startsWith('/')) {
        content = localStorage.getItem(`file:${path.substring(1)}`);
      }
      
      if (content !== null) {
        this.addRecentFile(normalizedPath);
        return content;
      }
      
      throw new Error(`File not found: ${path} (tried: ${normalizedPath}, ${path}, /${path}, ${path.substring(1)})`);
    } catch (error) {
      console.error('Error reading file:', error);
      throw error;
    }
  }

  async saveFile(path: string, content: string): Promise<void> {
    try {
      if (window.__TAURI__) {
        await invoke('write_file', { path, content });
        this.addRecentFile(path);
        return;
      }
      // Normalize path for consistent storage
      const normalizedPath = this.normalizePath(path);
      localStorage.setItem(`file:${normalizedPath}`, content);
      this.addRecentFile(normalizedPath);
      
      // Update current project if file belongs to it
      const project = this.getCurrentProject();
      if (project && normalizedPath.startsWith(project.path)) {
        project.files = await this.readDirectory(project.path);
        await this.saveProject(project);
        this.setCurrentProject(project);
      }
    } catch (error) {
      console.error('Error saving file:', error);
      throw error;
    }
  }

  async createFile(path: string, content: string = ''): Promise<void> {
    await this.saveFile(path, content);
  }

  async deleteFile(path: string): Promise<void> {
    try {
      if (window.__TAURI__) {
        await invoke('delete_file', { path });
        return;
      }
      localStorage.removeItem(`file:${path}`);
    } catch (error) {
      console.error('Error deleting file:', error);
      throw error;
    }
  }

  async readDirectory(path: string): Promise<FileEntry[]> {
    try {
      if (window.__TAURI__) {
        const entries = await invoke<FileEntry[]>('read_directory', { path });
        return entries;
      }
      
      // Normalize the path
      const normalizedPath = this.normalizePath(path);
      const pathPrefix = normalizedPath === '/' ? '' : `${normalizedPath}/`;
      
      // Collect all files and directories
      const fileMap = new Map<string, FileEntry>();
      const dirSet = new Set<string>();
      
      // Scan localStorage for all files
      for (let i = 0; i < localStorage.length; i++) {
        const key = localStorage.key(i);
        if (!key || !key.startsWith('file:')) continue;
        
        const filePath = key.substring(5);
        const normalizedFilePath = this.normalizePath(filePath);
        
        // Check if this file belongs to the requested directory
        if (normalizedPath === '/') {
          // Root directory - get top-level items
          const parts = normalizedFilePath.split('/');
          if (parts.length === 1) {
            // Top-level file
            fileMap.set(normalizedFilePath, {
              path: normalizedFilePath,
              name: parts[0],
              isDirectory: false,
            });
          } else if (parts.length > 1) {
            // Top-level directory
            const dirName = parts[0];
            if (!dirSet.has(dirName)) {
              dirSet.add(dirName);
            }
          }
        } else {
          // Check if file is in this directory or a subdirectory
          if (normalizedFilePath.startsWith(pathPrefix)) {
            const relativePath = normalizedFilePath.substring(pathPrefix.length);
            const parts = relativePath.split('/');
            
            if (parts.length === 1) {
              // Direct child file
              fileMap.set(normalizedFilePath, {
                path: normalizedFilePath,
                name: parts[0],
                isDirectory: false,
              });
            } else if (parts.length > 1) {
              // File in subdirectory
              const dirName = parts[0];
              const dirPath = `${normalizedPath}/${dirName}`;
              if (!dirSet.has(dirPath)) {
                dirSet.add(dirPath);
              }
            }
          }
        }
      }
      
      // Build result array
      const result: FileEntry[] = [];
      
      // Add direct files
      fileMap.forEach((file) => {
        const fileDir = this.getDirectoryName(file.path);
        if (fileDir === normalizedPath || (normalizedPath === '/' && !file.path.includes('/'))) {
          result.push(file);
        }
      });
      
      // Add directories with their children
      dirSet.forEach((dirPath) => {
        const dirName = this.getFileName(dirPath);
        const children = this.buildDirectoryChildren(dirPath, localStorage);
        result.push({
          path: dirPath,
          name: dirName,
          isDirectory: true,
          children: children.length > 0 ? children : undefined,
        });
      });
      
      // Sort: directories first, then files, both alphabetically
      result.sort((a, b) => {
        if (a.isDirectory !== b.isDirectory) {
          return a.isDirectory ? -1 : 1;
        }
        return a.name.localeCompare(b.name);
      });
      
      console.log(`[FileService] readDirectory(${path}) found ${result.length} items`);
      return result;
    } catch (error) {
      console.error('Error reading directory:', error);
      throw error;
    }
  }

  private buildDirectoryChildren(dirPath: string, storage: Storage): FileEntry[] {
    const children: FileEntry[] = [];
    const normalizedDirPath = this.normalizePath(dirPath);
    const dirPrefix = normalizedDirPath === '/' ? '' : `${normalizedDirPath}/`;
    const childMap = new Map<string, FileEntry>();
    const childDirs = new Set<string>();
    
    for (let i = 0; i < storage.length; i++) {
      const key = storage.key(i);
      if (!key || !key.startsWith('file:')) continue;
      
      const filePath = key.substring(5);
      const normalizedFilePath = this.normalizePath(filePath);
      
      if (normalizedFilePath.startsWith(dirPrefix) && normalizedFilePath !== normalizedDirPath) {
        const relativePath = normalizedFilePath.substring(dirPrefix.length);
        const parts = relativePath.split('/');
        
        if (parts.length === 1) {
          // Direct child file
          childMap.set(normalizedFilePath, {
            path: normalizedFilePath,
            name: parts[0],
            isDirectory: false,
          });
        } else if (parts.length > 1) {
          // File in subdirectory
          const subDirName = parts[0];
          const subDirPath = `${normalizedDirPath}/${subDirName}`;
          childDirs.add(subDirPath);
        }
      }
    }
    
    // Add files
    childMap.forEach((file) => {
      children.push(file);
    });
    
    // Add directories recursively
    childDirs.forEach((subDirPath) => {
      const dirName = this.getFileName(subDirPath);
      const subChildren = this.buildDirectoryChildren(subDirPath, storage);
      children.push({
        path: subDirPath,
        name: dirName,
        isDirectory: true,
        children: subChildren.length > 0 ? subChildren : undefined,
      });
    });
    
    // Sort
    children.sort((a, b) => {
      if (a.isDirectory !== b.isDirectory) {
        return a.isDirectory ? -1 : 1;
      }
      return a.name.localeCompare(b.name);
    });
    
    return children;
  }

  async createDirectory(path: string): Promise<void> {
    try {
      if (window.__TAURI__) {
        await invoke('create_directory', { path });
        return;
      }
      localStorage.setItem(`dir:${path}`, 'true');
    } catch (error) {
      console.error('Error creating directory:', error);
      throw error;
    }
  }

  async deleteDirectory(path: string): Promise<void> {
    try {
      if (window.__TAURI__) {
        await invoke('delete_directory', { path });
        return;
      }
      const keysToRemove: string[] = [];
      for (let i = 0; i < localStorage.length; i++) {
        const key = localStorage.key(i);
        if (key?.startsWith(`file:${path}/`) || key === `dir:${path}`) {
          keysToRemove.push(key);
        }
      }
      keysToRemove.forEach((key) => localStorage.removeItem(key));
    } catch (error) {
      console.error('Error deleting directory:', error);
      throw error;
    }
  }

  async createProject(name: string, path: string): Promise<Project> {
    const project: Project = {
      name,
      path,
      files: [],
      createdAt: new Date(),
      lastModified: new Date(),
    };
    await this.createDirectory(path);
    await this.createDirectory(`${path}/src`);
    await this.createDirectory(`${path}/tests`);
    await this.createDirectory(`${path}/docs`);
    await this.createFile(
      `${path}/src/main.navλ`,
      '// Welcome to NAVΛ Studio\n\nnav main() {\n  // Your code here\n}\n'
    );
    await this.createFile(
      `${path}/README.md`,
      `# ${name}\n\nA NAVΛ project for Van Laarhoven Navigation Calculus.\n`
    );
    await this.saveProject(project);
    return project;
  }

  async openProject(path: string): Promise<Project | null> {
    try {
      // Try to read project metadata first
      try {
        const metadataPath = `${path}/project.json`;
        const metadata = await this.readFile(metadataPath);
        const project = JSON.parse(metadata) as Project;
        project.files = await this.readDirectory(path);
        return project;
      } catch {
        // If no project.json, create a project from the directory structure
        const files = await this.readDirectory(path);
        const project: Project = {
          name: this.getFileName(path) || 'Imported Project',
          path: path,
          files: files,
          createdAt: new Date(),
          lastModified: new Date(),
        };
        await this.saveProject(project);
        return project;
      }
    } catch (error) {
      console.error('Error opening project:', error);
      return null;
    }
  }

  async importFiles(files: Array<{ path: string; name: string; content?: string; isDirectory: boolean; children?: any[] }>): Promise<void> {
    console.log(`[FileService] Starting import of ${files.length} items...`);
    let importedCount = 0;
    let errorCount = 0;
    
    // Normalize and store imported files in localStorage or file system
    for (const file of files) {
      // Normalize path (remove leading/trailing slashes except root)
      const normalizedPath = this.normalizePath(file.path);
      
      if (file.isDirectory && file.children) {
        try {
          await this.createDirectory(normalizedPath);
          if (file.children && file.children.length > 0) {
            await this.importFiles(file.children);
          }
        } catch (error) {
          console.error(`[FileService] Failed to import directory ${normalizedPath}:`, error);
          errorCount++;
        }
      } else if (!file.isDirectory) {
        const content = file.content || '';
        try {
          // Ensure path is normalized and content is stored
          await this.createFile(normalizedPath, content);
          importedCount++;
          
          // Verify it was stored
          const stored = localStorage.getItem(`file:${normalizedPath}`);
          if (!stored) {
            console.warn(`[FileService] File ${normalizedPath} was not stored properly!`);
            // Try storing again with explicit path
            localStorage.setItem(`file:${normalizedPath}`, content);
          }
          
          console.log(`[FileService] ✓ Imported file: ${normalizedPath} (${content.length} bytes)`);
        } catch (error) {
          console.error(`[FileService] ✗ Failed to import file ${normalizedPath}:`, error);
          errorCount++;
        }
      }
    }
    
    console.log(`[FileService] Import complete: ${importedCount} files imported, ${errorCount} errors`);
    
    // Refresh current project if it exists
    const project = this.getCurrentProject();
    if (project) {
      try {
        project.files = await this.readDirectory(project.path);
        await this.saveProject(project);
        this.setCurrentProject(project);
        console.log(`[FileService] Updated project with ${project.files.length} files`);
      } catch (error) {
        console.error('[FileService] Failed to refresh project:', error);
      }
    }
  }

  private normalizePath(path: string): string {
    // Remove leading slash except for root
    let normalized = path.startsWith('/') && path.length > 1 ? path.substring(1) : path;
    // Remove trailing slash
    normalized = normalized.endsWith('/') ? normalized.slice(0, -1) : normalized;
    return normalized || '/';
  }

  async saveProject(project: Project): Promise<void> {
    const metadataPath = `${project.path}/project.json`;
    await this.saveFile(metadataPath, JSON.stringify(project, null, 2));
  }

  async closeProject(_project: Project): Promise<void> {
    // Cleanup if needed
  }

  getFileExtension(path: string): string {
    const parts = path.split('.');
    return parts.length > 1 ? parts[parts.length - 1] : '';
  }

  getFileName(path: string): string {
    const parts = path.split('/');
    return parts[parts.length - 1];
  }

  getDirectoryName(path: string): string {
    const parts = path.split('/');
    parts.pop();
    return parts.join('/');
  }
}

// Export fileService instance
export const fileService = new FileService();

// Also export as default for compatibility
export default fileService;

