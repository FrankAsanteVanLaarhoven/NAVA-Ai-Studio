/**
 * NAVA Trash Bin - Storage & Management System
 * 
 * Production-ready CRUD operations for trash management
 * Similar to Apple's Trash functionality
 */

export interface TrashItem {
  id: string;
  originalId: string;
  name: string;
  icon: string;
  color?: string;
  description?: string;
  route?: string | null;
  category?: string;
  type: 'app' | 'file' | 'media';
  deletedAt: number;
  originalData: any; // Store original app/item data for restoration
}

const TRASH_STORAGE_KEY = 'nava-trash-items';

/**
 * Get all items in trash
 */
export function getTrashItems(): TrashItem[] {
  try {
    const stored = localStorage.getItem(TRASH_STORAGE_KEY);
    if (!stored) return [];
    return JSON.parse(stored);
  } catch (error) {
    console.error('[TrashStorage] Error reading trash:', error);
    return [];
  }
}

/**
 * Check if trash has items
 */
export function hasTrashItems(): boolean {
  return getTrashItems().length > 0;
}

/**
 * Add item to trash
 */
export function addToTrash(item: Omit<TrashItem, 'id' | 'deletedAt'>): TrashItem {
  const trashItems = getTrashItems();
  const newTrashItem: TrashItem = {
    ...item,
    id: `trash-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
    deletedAt: Date.now(),
  };
  trashItems.push(newTrashItem);
  saveTrashItems(trashItems);
  return newTrashItem;
}

/**
 * Restore item from trash
 */
export function restoreFromTrash(trashId: string): TrashItem | null {
  const trashItems = getTrashItems();
  const index = trashItems.findIndex(item => item.id === trashId);
  if (index === -1) return null;
  
  const item = trashItems[index];
  trashItems.splice(index, 1);
  saveTrashItems(trashItems);
  return item;
}

/**
 * Delete item permanently from trash
 */
export function deletePermanently(trashId: string): boolean {
  const trashItems = getTrashItems();
  const index = trashItems.findIndex(item => item.id === trashId);
  if (index === -1) return false;
  
  trashItems.splice(index, 1);
  saveTrashItems(trashItems);
  return true;
}

/**
 * Empty trash (delete all items permanently)
 */
export function emptyTrash(): number {
  const count = getTrashItems().length;
  saveTrashItems([]);
  return count;
}

/**
 * Rename item in trash
 */
export function renameTrashItem(trashId: string, newName: string): boolean {
  const trashItems = getTrashItems();
  const index = trashItems.findIndex(item => item.id === trashId);
  if (index === -1) return false;
  
  trashItems[index].name = newName;
  saveTrashItems(trashItems);
  return true;
}

/**
 * Get trash item by ID
 */
export function getTrashItem(trashId: string): TrashItem | null {
  const trashItems = getTrashItems();
  return trashItems.find(item => item.id === trashId) || null;
}

/**
 * Save trash items to storage
 */
function saveTrashItems(items: TrashItem[]): void {
  try {
    localStorage.setItem(TRASH_STORAGE_KEY, JSON.stringify(items));
    // Dispatch event to notify components
    window.dispatchEvent(new CustomEvent('nava:trash-updated'));
  } catch (error) {
    console.error('[TrashStorage] Error saving trash:', error);
  }
}

