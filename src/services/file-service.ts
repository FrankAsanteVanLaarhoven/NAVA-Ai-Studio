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

  constructor() {
    this.loadRecentFiles();
  }

  getCurrentProject(): Project | null {
    return this.currentProject;
  }

  setCurrentProject(project: Project | null) {
    this.currentProject = project;
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
      const content = localStorage.getItem(`file:${path}`);
      if (content !== null) {
        this.addRecentFile(path);
        return content;
      }
      throw new Error('File not found');
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
      localStorage.setItem(`file:${path}`, content);
      this.addRecentFile(path);
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
      const files: FileEntry[] = [];
      for (let i = 0; i < localStorage.length; i++) {
        const key = localStorage.key(i);
        if (key?.startsWith(`file:${path}/`)) {
          const filePath = key.substring(5);
          const fileName = this.getFileName(filePath);
          files.push({
            path: filePath,
            name: fileName,
            isDirectory: false,
          });
        }
      }
      return files;
    } catch (error) {
      console.error('Error reading directory:', error);
      throw error;
    }
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
      const metadataPath = `${path}/project.json`;
      const metadata = await this.readFile(metadataPath);
      const project = JSON.parse(metadata) as Project;
      project.files = await this.readDirectory(path);
      return project;
    } catch (error) {
      console.error('Error opening project:', error);
      return null;
    }
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

export const fileService = new FileService();
