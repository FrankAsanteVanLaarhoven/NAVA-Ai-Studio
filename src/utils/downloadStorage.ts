/**
 * Download Storage Service
 * Tracks and manages recent downloads similar to macOS download stack
 */

export interface DownloadItem {
  id: string;
  name: string;
  url: string;
  path: string;
  size: number;
  type: string;
  downloadedAt: number;
  icon?: string;
}

const STORAGE_KEY = 'nava-downloads';
const MAX_DOWNLOADS = 50; // Keep last 50 downloads

/**
 * Get all downloads, sorted by most recent first
 */
export function getDownloads(): DownloadItem[] {
  try {
    const stored = localStorage.getItem(STORAGE_KEY);
    if (!stored) return [];
    const downloads = JSON.parse(stored) as DownloadItem[];
    return downloads.sort((a, b) => b.downloadedAt - a.downloadedAt);
  } catch (error) {
    console.error('[DownloadStorage] Error reading downloads:', error);
    return [];
  }
}

/**
 * Add a new download
 */
export function addDownload(download: Omit<DownloadItem, 'id' | 'downloadedAt'>): DownloadItem {
  const downloads = getDownloads();
  const newDownload: DownloadItem = {
    ...download,
    id: `download-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
    downloadedAt: Date.now(),
  };
  
  // Add to beginning and keep only MAX_DOWNLOADS
  downloads.unshift(newDownload);
  const trimmed = downloads.slice(0, MAX_DOWNLOADS);
  
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(trimmed));
  } catch (error) {
    console.error('[DownloadStorage] Error saving downloads:', error);
  }
  
  return newDownload;
}

/**
 * Remove a download
 */
export function removeDownload(downloadId: string): boolean {
  const downloads = getDownloads();
  const index = downloads.findIndex(d => d.id === downloadId);
  if (index === -1) return false;
  
  downloads.splice(index, 1);
  
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(downloads));
    return true;
  } catch (error) {
    console.error('[DownloadStorage] Error removing download:', error);
    return false;
  }
}

/**
 * Clear all downloads
 */
export function clearDownloads(): void {
  try {
    localStorage.removeItem(STORAGE_KEY);
  } catch (error) {
    console.error('[DownloadStorage] Error clearing downloads:', error);
  }
}

/**
 * Get download count
 */
export function getDownloadCount(): number {
  return getDownloads().length;
}

/**
 * Get file icon based on file type
 */
export function getFileIcon(type: string, name: string): string {
  const ext = name.split('.').pop()?.toLowerCase() || '';
  
  // Images
  if (type.startsWith('image/') || ['jpg', 'jpeg', 'png', 'gif', 'svg', 'webp'].includes(ext)) {
    return '🖼️';
  }
  // Videos
  if (type.startsWith('video/') || ['mp4', 'mov', 'avi', 'mkv', 'webm'].includes(ext)) {
    return '🎬';
  }
  // Audio
  if (type.startsWith('audio/') || ['mp3', 'wav', 'flac', 'aac', 'ogg'].includes(ext)) {
    return '🎵';
  }
  // Documents
  if (['pdf', 'doc', 'docx', 'txt', 'rtf'].includes(ext)) {
    return '📄';
  }
  // Archives
  if (['zip', 'rar', '7z', 'tar', 'gz'].includes(ext)) {
    return '📦';
  }
  // Code
  if (['js', 'ts', 'py', 'java', 'cpp', 'c', 'html', 'css', 'json', 'xml'].includes(ext)) {
    return '💻';
  }
  // Default
  return '📎';
}

