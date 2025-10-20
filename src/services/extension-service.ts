/**
 * Extension Service
 * 
 * Manages installation, activation, and lifecycle of IDE extensions
 */

export interface Extension {
  id: string;
  name: string;
  displayName: string;
  description: string;
  version: string;
  author: string;
  publisher: string;
  icon?: string;
  category: string;
  downloads: string;
  rating: number;
  installed: boolean;
  enabled: boolean;
  main?: string; // Entry point
  contributes?: ExtensionContributions;
  dependencies?: string[];
  repository?: string;
  license?: string;
}

export interface ExtensionContributions {
  commands?: Array<{
    id: string;
    title: string;
    category?: string;
  }>;
  languages?: Array<{
    id: string;
    extensions: string[];
    aliases?: string[];
  }>;
  themes?: Array<{
    id: string;
    label: string;
    path: string;
  }>;
  snippets?: Array<{
    language: string;
    path: string;
  }>;
  keybindings?: Array<{
    command: string;
    key: string;
    when?: string;
  }>;
}

export interface ExtensionAPI {
  registerCommand: (id: string, callback: () => void) => void;
  registerLanguage: (config: any) => void;
  registerTheme: (theme: any) => void;
  getWorkspaceConfig: () => any;
  showMessage: (message: string, type?: 'info' | 'warning' | 'error') => void;
}

class ExtensionService {
  private extensions: Map<string, Extension> = new Map();
  private installedExtensions: Set<string> = new Set();
  private enabledExtensions: Set<string> = new Set();
  private extensionAPIs: Map<string, ExtensionAPI> = new Map();
  private readonly STORAGE_KEY = 'navlambda_installed_extensions';
  private readonly ENABLED_KEY = 'navlambda_enabled_extensions';

  constructor() {
    this.loadFromStorage();
    this.initializeBuiltInExtensions();
  }

  /**
   * Initialize built-in extensions (already installed)
   */
  private initializeBuiltInExtensions(): void {
    const builtIn: Extension[] = [
      {
        id: 'nava-syntax',
        name: 'nava-syntax-highlighter',
        displayName: 'NAVΛ Syntax Highlighter',
        description: 'Advanced syntax highlighting for VNC',
        version: '1.2.0',
        author: 'NAVΛ Studio',
        publisher: 'NAVΛ Studio',
        category: 'Programming Languages',
        downloads: '10.5K',
        rating: 4.9,
        installed: true,
        enabled: true,
        contributes: {
          languages: [
            {
              id: 'nava',
              extensions: ['.nava', '.vnc'],
              aliases: ['NAVA', 'VNC']
            }
          ]
        }
      },
      {
        id: 'vnc-intellisense',
        name: 'vnc-intellisense',
        displayName: 'VNC IntelliSense',
        description: 'Smart code completion for Navigation Calculus',
        version: '2.0.1',
        author: 'Van Laarhoven',
        publisher: 'Van Laarhoven',
        category: 'Programming Languages',
        downloads: '8.2K',
        rating: 4.8,
        installed: true,
        enabled: true,
        contributes: {
          commands: [
            { id: 'vnc.complete', title: 'VNC: Trigger Completion' },
            { id: 'vnc.docs', title: 'VNC: Show Documentation' }
          ]
        }
      }
    ];

    builtIn.forEach(ext => {
      this.extensions.set(ext.id, ext);
      if (ext.installed) {
        this.installedExtensions.add(ext.id);
      }
      if (ext.enabled) {
        this.enabledExtensions.add(ext.id);
      }
    });
  }

  /**
   * Load installed extensions from localStorage
   */
  private loadFromStorage(): void {
    try {
      const installed = localStorage.getItem(this.STORAGE_KEY);
      if (installed) {
        const extensionIds = JSON.parse(installed) as string[];
        extensionIds.forEach(id => this.installedExtensions.add(id));
      }

      const enabled = localStorage.getItem(this.ENABLED_KEY);
      if (enabled) {
        const extensionIds = JSON.parse(enabled) as string[];
        extensionIds.forEach(id => this.enabledExtensions.add(id));
      }
    } catch (error) {
      console.error('Failed to load extensions from storage:', error);
    }
  }

  /**
   * Save installed extensions to localStorage
   */
  private saveToStorage(): void {
    try {
      localStorage.setItem(
        this.STORAGE_KEY,
        JSON.stringify(Array.from(this.installedExtensions))
      );
      localStorage.setItem(
        this.ENABLED_KEY,
        JSON.stringify(Array.from(this.enabledExtensions))
      );
    } catch (error) {
      console.error('Failed to save extensions to storage:', error);
    }
  }

  /**
   * Get all available extensions from marketplace
   */
  getMarketplaceExtensions(): Extension[] {
    return [
      {
        id: 'nav-debugger',
        name: 'navigation-debugger',
        displayName: 'Navigation Debugger',
        description: 'Visual debugging tools for navigation paths',
        version: '1.5.0',
        author: 'NAVΛ Team',
        publisher: 'NAVΛ Team',
        category: 'Debuggers',
        downloads: '5.1K',
        rating: 4.7,
        installed: this.installedExtensions.has('nav-debugger'),
        enabled: this.enabledExtensions.has('nav-debugger'),
        contributes: {
          commands: [
            { id: 'nav.debug.start', title: 'Start Navigation Debug Session' },
            { id: 'nav.debug.step', title: 'Step Through Navigation' },
            { id: 'nav.debug.visualize', title: 'Visualize Navigation Path' }
          ]
        }
      },
      {
        id: 'wasm-preview',
        name: 'webassembly-preview',
        displayName: 'WebAssembly Preview',
        description: 'Live preview for WASM compilation',
        version: '3.1.2',
        author: 'WASM Tools',
        publisher: 'WASM Tools',
        category: 'Programming Languages',
        downloads: '12.3K',
        rating: 4.9,
        installed: this.installedExtensions.has('wasm-preview'),
        enabled: this.enabledExtensions.has('wasm-preview'),
        contributes: {
          commands: [
            { id: 'wasm.compile', title: 'Compile to WebAssembly' },
            { id: 'wasm.preview', title: 'Preview WASM Module' }
          ],
          languages: [
            {
              id: 'wat',
              extensions: ['.wat', '.wast'],
              aliases: ['WebAssembly Text']
            }
          ]
        }
      },
      {
        id: 'ros-tools',
        name: 'ros-toolkit',
        displayName: 'ROS Toolkit',
        description: 'Complete ROS/ROS2 development tools',
        version: '2.3.0',
        author: 'Robotics Team',
        publisher: 'NAVΛ Robotics',
        category: 'Robotics',
        downloads: '15.7K',
        rating: 4.9,
        installed: this.installedExtensions.has('ros-tools'),
        enabled: this.enabledExtensions.has('ros-tools'),
        contributes: {
          commands: [
            { id: 'ros.launch', title: 'ROS: Launch Node' },
            { id: 'ros.build', title: 'ROS: Build Package' },
            { id: 'ros.topics', title: 'ROS: Show Topics' }
          ]
        }
      },
      {
        id: 'ai-assistant-plus',
        name: 'ai-assistant-plus',
        displayName: 'AI Assistant Plus',
        description: 'Enhanced AI features with multi-model support',
        version: '1.8.0',
        author: 'AI Labs',
        publisher: 'NAVΛ AI',
        category: 'AI Tools',
        downloads: '22.1K',
        rating: 4.8,
        installed: this.installedExtensions.has('ai-assistant-plus'),
        enabled: this.enabledExtensions.has('ai-assistant-plus'),
        contributes: {
          commands: [
            { id: 'ai.explain', title: 'AI: Explain Code' },
            { id: 'ai.refactor', title: 'AI: Suggest Refactor' },
            { id: 'ai.generate', title: 'AI: Generate Code' }
          ]
        }
      },
      {
        id: 'theme-neon',
        name: 'neon-theme',
        displayName: 'Neon Theme Pack',
        description: 'Cyberpunk-inspired color themes',
        version: '1.0.5',
        author: 'Theme Studio',
        publisher: 'NAVΛ Themes',
        category: 'Themes',
        downloads: '8.9K',
        rating: 4.6,
        installed: this.installedExtensions.has('theme-neon'),
        enabled: this.enabledExtensions.has('theme-neon'),
        contributes: {
          themes: [
            { id: 'neon-green', label: 'Neon Green', path: '/themes/neon-green.json' },
            { id: 'neon-blue', label: 'Neon Blue', path: '/themes/neon-blue.json' },
            { id: 'neon-purple', label: 'Neon Purple', path: '/themes/neon-purple.json' }
          ]
        }
      },
      {
        id: 'git-lens',
        name: 'git-lens-extended',
        displayName: 'Git Lens Extended',
        description: 'Advanced Git history and blame annotations',
        version: '4.2.1',
        author: 'Git Tools',
        publisher: 'NAVΛ Tools',
        category: 'Source Control',
        downloads: '18.3K',
        rating: 4.9,
        installed: this.installedExtensions.has('git-lens'),
        enabled: this.enabledExtensions.has('git-lens'),
        contributes: {
          commands: [
            { id: 'git.history', title: 'Git: Show File History' },
            { id: 'git.blame', title: 'Git: Toggle Blame Annotations' },
            { id: 'git.compare', title: 'Git: Compare Commits' }
          ]
        }
      }
    ];
  }

  /**
   * Get all extensions (marketplace + installed)
   */
  getAllExtensions(): Extension[] {
    const marketplace = this.getMarketplaceExtensions();
    const builtIn = Array.from(this.extensions.values());
    
    // Merge, avoiding duplicates
    const allExtensions = new Map<string, Extension>();
    [...builtIn, ...marketplace].forEach(ext => {
      allExtensions.set(ext.id, ext);
    });
    
    return Array.from(allExtensions.values());
  }

  /**
   * Get only installed extensions
   */
  getInstalledExtensions(): Extension[] {
    return this.getAllExtensions().filter(ext => ext.installed);
  }

  /**
   * Get extension by ID
   */
  getExtension(id: string): Extension | undefined {
    return this.getAllExtensions().find(ext => ext.id === id);
  }

  /**
   * Install an extension
   */
  async install(extensionId: string): Promise<{ success: boolean; message: string }> {
    try {
      const extension = this.getExtension(extensionId);
      if (!extension) {
        return { success: false, message: 'Extension not found' };
      }

      if (extension.installed) {
        return { success: false, message: 'Extension already installed' };
      }

      // Simulate installation process
      await this.simulateInstall(extension);

      // Mark as installed and enabled
      extension.installed = true;
      extension.enabled = true;
      this.extensions.set(extensionId, extension);
      this.installedExtensions.add(extensionId);
      this.enabledExtensions.add(extensionId);

      // Activate the extension
      await this.activateExtension(extension);

      // Save to storage
      this.saveToStorage();

      return {
        success: true,
        message: `${extension.displayName} v${extension.version} installed successfully`
      };
    } catch (error) {
      return {
        success: false,
        message: `Failed to install extension: ${error}`
      };
    }
  }

  /**
   * Uninstall an extension
   */
  async uninstall(extensionId: string): Promise<{ success: boolean; message: string }> {
    try {
      const extension = this.getExtension(extensionId);
      if (!extension) {
        return { success: false, message: 'Extension not found' };
      }

      if (!extension.installed) {
        return { success: false, message: 'Extension not installed' };
      }

      // Deactivate first
      await this.deactivateExtension(extension);

      // Mark as uninstalled
      extension.installed = false;
      extension.enabled = false;
      this.installedExtensions.delete(extensionId);
      this.enabledExtensions.delete(extensionId);

      // Save to storage
      this.saveToStorage();

      return {
        success: true,
        message: `${extension.displayName} uninstalled successfully`
      };
    } catch (error) {
      return {
        success: false,
        message: `Failed to uninstall extension: ${error}`
      };
    }
  }

  /**
   * Enable/disable an extension
   */
  async toggleExtension(extensionId: string): Promise<{ success: boolean; message: string }> {
    try {
      const extension = this.getExtension(extensionId);
      if (!extension || !extension.installed) {
        return { success: false, message: 'Extension not found or not installed' };
      }

      if (extension.enabled) {
        await this.deactivateExtension(extension);
        extension.enabled = false;
        this.enabledExtensions.delete(extensionId);
      } else {
        await this.activateExtension(extension);
        extension.enabled = true;
        this.enabledExtensions.add(extensionId);
      }

      this.saveToStorage();

      return {
        success: true,
        message: `${extension.displayName} ${extension.enabled ? 'enabled' : 'disabled'}`
      };
    } catch (error) {
      return {
        success: false,
        message: `Failed to toggle extension: ${error}`
      };
    }
  }

  /**
   * Simulate installation process (downloading, extracting, etc.)
   */
  private async simulateInstall(extension: Extension): Promise<void> {
    return new Promise((resolve) => {
      // Simulate download and installation time
      setTimeout(() => {
        console.log(`📦 Installing ${extension.displayName}...`);
        console.log(`   Version: ${extension.version}`);
        console.log(`   Publisher: ${extension.publisher}`);
        console.log(`   ✓ Downloaded`);
        console.log(`   ✓ Extracted`);
        console.log(`   ✓ Validated`);
        resolve();
      }, 1000);
    });
  }

  /**
   * Activate an extension
   */
  private async activateExtension(extension: Extension): Promise<void> {
    console.log(`🚀 Activating ${extension.displayName}...`);

    // Create extension API
    const api: ExtensionAPI = {
      registerCommand: (id: string, callback: () => void) => {
        console.log(`   📝 Registered command: ${id}`);
      },
      registerLanguage: (config: any) => {
        console.log(`   🔤 Registered language: ${config.id}`);
      },
      registerTheme: (theme: any) => {
        console.log(`   🎨 Registered theme: ${theme.id}`);
      },
      getWorkspaceConfig: () => ({}),
      showMessage: (message: string, type = 'info') => {
        console.log(`   📢 [${type}] ${message}`);
      }
    };

    this.extensionAPIs.set(extension.id, api);

    // Execute contributions
    if (extension.contributes) {
      if (extension.contributes.commands) {
        extension.contributes.commands.forEach(cmd => {
          api.registerCommand(cmd.id, () => {
            console.log(`Executing command: ${cmd.title}`);
          });
        });
      }

      if (extension.contributes.languages) {
        extension.contributes.languages.forEach(lang => {
          api.registerLanguage(lang);
        });
      }

      if (extension.contributes.themes) {
        extension.contributes.themes.forEach(theme => {
          api.registerTheme(theme);
        });
      }
    }

    console.log(`   ✅ ${extension.displayName} activated`);
  }

  /**
   * Deactivate an extension
   */
  private async deactivateExtension(extension: Extension): Promise<void> {
    console.log(`⏸️  Deactivating ${extension.displayName}...`);
    this.extensionAPIs.delete(extension.id);
    console.log(`   ✓ Deactivated`);
  }

  /**
   * Get extension API for a specific extension
   */
  getExtensionAPI(extensionId: string): ExtensionAPI | undefined {
    return this.extensionAPIs.get(extensionId);
  }

  /**
   * Search extensions
   */
  searchExtensions(query: string): Extension[] {
    const lowerQuery = query.toLowerCase();
    return this.getAllExtensions().filter(ext =>
      ext.displayName.toLowerCase().includes(lowerQuery) ||
      ext.description.toLowerCase().includes(lowerQuery) ||
      ext.category.toLowerCase().includes(lowerQuery)
    );
  }

  /**
   * Get extensions by category
   */
  getExtensionsByCategory(category: string): Extension[] {
    return this.getAllExtensions().filter(ext =>
      ext.category.toLowerCase() === category.toLowerCase()
    );
  }
}

// Singleton instance
export const extensionService = new ExtensionService();

