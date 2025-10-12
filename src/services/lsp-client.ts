import { invoke } from '@tauri-apps/api/tauri';

/**
 * LSP Client Service for NAVΛ Studio
 * 
 * Communicates with the Rust backend LSP server.
 */
export class NavLambdaLspClient {
  async initialize(): Promise<string> {
    return await invoke<string>('initialize_lsp');
  }

  async parseCode(code: string): Promise<string> {
    return await invoke<string>('parse_navlambda_code', { code });
  }

  async getCompletions(code: string, position: number): Promise<string[]> {
    return await invoke<string[]>('get_code_completions', { code, position });
  }

  async getHoverInfo(code: string, position: number): Promise<string> {
    return await invoke<string>('get_hover_info', { code, position });
  }

  async visualizeNavigationPath(code: string): Promise<string> {
    return await invoke<string>('visualize_navigation_path', { code });
  }

  async runLivePreview(code: string): Promise<string> {
    return await invoke<string>('run_live_preview', { code });
  }
}

export const lspClient = new NavLambdaLspClient();

