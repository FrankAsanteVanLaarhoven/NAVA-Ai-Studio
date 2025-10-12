import { open, save } from '@tauri-apps/api/dialog';
import { readTextFile, writeTextFile } from '@tauri-apps/api/fs';

/**
 * File System Operations Service
 */

class FileService {
  async openFile(): Promise<{ path: string; content: string } | null> {
    try {
      const selected = await open({
        multiple: false,
        filters: [
          {
            name: 'NAVΛ Files',
            extensions: ['vnc', 'navλ'],
          },
        ],
      });

      if (typeof selected === 'string') {
        const content = await readTextFile(selected);
        return { path: selected, content };
      }

      return null;
    } catch (error) {
      console.error('Error opening file:', error);
      return null;
    }
  }

  async saveFile(content: string, path?: string): Promise<string | null> {
    try {
      const filePath =
        path ||
        (await save({
          filters: [
            {
              name: 'NAVΛ Files',
              extensions: ['vnc'],
            },
          ],
        }));

      if (filePath) {
        await writeTextFile(filePath, content);
        return filePath;
      }

      return null;
    } catch (error) {
      console.error('Error saving file:', error);
      return null;
    }
  }

  async readFile(path: string): Promise<string> {
    return await readTextFile(path);
  }

  async writeFile(path: string, content: string): Promise<void> {
    await writeTextFile(path, content);
  }
}

export const fileService = new FileService();

