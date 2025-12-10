/**
 * Notebook Utilities
 * 
 * Helper functions for working with Jupyter notebooks
 */

import { fileService } from '../services/file-service';
import { jupyterNotebookService } from '../services/jupyter-notebook-service';

/**
 * Copy an external notebook file into the workspace
 */
export async function importNotebookToWorkspace(externalPath: string, workspacePath?: string): Promise<string> {
  try {
    // Read the external file
    let content: string;
    
    // Try to read via Tauri if available
    if (typeof window !== 'undefined' && (window as any).__TAURI__) {
      const { invoke } = await import('@tauri-apps/api/core');
      content = await invoke<string>('read_file', { path: externalPath });
    } else {
      // For web, we'd need to use File API or fetch
      // This is a placeholder - in production, you'd handle file uploads
      throw new Error('File reading not available in web mode. Please use the file upload feature.');
    }

    // Parse to validate it's a valid notebook
    jupyterNotebookService.parseNotebook(content);

    // Determine save path
    const fileName = fileService.getFileName(externalPath);
    const savePath = workspacePath || `notebooks/${fileName}`;

    // Save to workspace
    await fileService.saveFile(savePath, content);

    return savePath;
  } catch (error) {
    throw new Error(`Failed to import notebook: ${error instanceof Error ? error.message : 'Unknown error'}`);
  }
}

/**
 * Check if a file path is a notebook file
 */
export function isNotebookFile(path: string): boolean {
  return path.endsWith('.ipynb');
}

/**
 * Get notebook display name from path
 */
export function getNotebookDisplayName(path: string): string {
  const fileName = fileService.getFileName(path);
  return fileName || 'Untitled.ipynb';
}

