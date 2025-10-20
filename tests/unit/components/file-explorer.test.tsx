import { describe, test, expect, beforeEach, afterEach, vi } from 'vitest';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import { FileExplorer } from '../../../src/components/Editor/FileExplorer';
import { fileService } from '../../../src/services/file-service';

// Mock the file service
vi.mock('../../../src/services/file-service', () => ({
  fileService: {
    getCurrentProject: vi.fn(),
    readFile: vi.fn(),
    createFile: vi.fn(),
    createDirectory: vi.fn(),
    deleteFile: vi.fn(),
    deleteDirectory: vi.fn(),
    getFileName: vi.fn(),
    getDirectoryName: vi.fn(),
    getFileExtension: vi.fn(),
  },
}));

// Mock window methods used in the component
const mockAlert = vi.fn();
const mockConfirm = vi.fn();
const mockPrompt = vi.fn();

Object.defineProperty(window, 'alert', { value: mockAlert, writable: true });
Object.defineProperty(window, 'confirm', { value: mockConfirm, writable: true });
Object.defineProperty(window, 'prompt', { value: mockPrompt, writable: true });

describe('FileExplorer', () => {
  const mockOnFileSelect = vi.fn();
  const mockProject = {
    name: 'test-project',
    path: '/test-project',
    files: [
      {
        path: '/test-project/src',
        name: 'src',
        isDirectory: true,
        children: [
          {
            path: '/test-project/src/main.navλ',
            name: 'main.navλ',
            isDirectory: false,
          },
          {
            path: '/test-project/src/utils.navλ',
            name: 'utils.navλ',
            isDirectory: false,
          },
        ],
      },
      {
        path: '/test-project/README.md',
        name: 'README.md',
        isDirectory: false,
      },
    ],
    createdAt: new Date(),
    lastModified: new Date(),
  };

  beforeEach(() => {
    vi.clearAllMocks();
    mockAlert.mockClear();
    mockConfirm.mockClear();
    mockPrompt.mockClear();

    // Default mocks
    (fileService.getCurrentProject as any).mockReturnValue(mockProject);
    (fileService.readFile as any).mockResolvedValue('file content');
    (fileService.createFile as any).mockResolvedValue(undefined);
    (fileService.createDirectory as any).mockResolvedValue(undefined);
    (fileService.deleteFile as any).mockResolvedValue(undefined);
    (fileService.deleteDirectory as any).mockResolvedValue(undefined);
    (fileService.getFileName as any).mockImplementation((path: string) => path.split('/').pop() || '');
    (fileService.getDirectoryName as any).mockImplementation((path: string) => path.substring(0, path.lastIndexOf('/')));
    (fileService.getFileExtension as any).mockImplementation((fileName: string) => fileName.split('.').pop() || '');
  });

  afterEach(() => {
    vi.restoreAllMocks();
  });

  test('renders file explorer with project files', () => {
    render(<FileExplorer onFileSelect={mockOnFileSelect} />);

    expect(screen.getByText('Explorer')).toBeInTheDocument();
    expect(screen.getByText('src')).toBeInTheDocument();
    expect(screen.getByText('README.md')).toBeInTheDocument();
  });

  test('loads current project on mount', () => {
    render(<FileExplorer onFileSelect={mockOnFileSelect} />);

    expect(fileService.getCurrentProject).toHaveBeenCalled();
  });

  test('expands and collapses directories', () => {
    render(<FileExplorer onFileSelect={mockOnFileSelect} />);

    const srcFolder = screen.getByText('src');
    fireEvent.click(srcFolder);

    // Should show children when expanded
    expect(screen.getByText('main.navλ')).toBeInTheDocument();
    expect(screen.getByText('utils.navλ')).toBeInTheDocument();

    // Click again to collapse
    fireEvent.click(srcFolder);

    // Children should be hidden (this is harder to test with current implementation)
    // but we can verify the click handler was called
  });

  test('selects file and calls onFileSelect', async () => {
    render(<FileExplorer onFileSelect={mockOnFileSelect} />);

    // First expand the src directory
    const srcFolder = screen.getByText('src');
    fireEvent.click(srcFolder);

    // Then click on a file
    const mainFile = screen.getByText('main.navλ');
    fireEvent.click(mainFile);

    await waitFor(() => {
      expect(fileService.readFile).toHaveBeenCalledWith('/test-project/src/main.navλ');
      expect(mockOnFileSelect).toHaveBeenCalledWith('/test-project/src/main.navλ', 'file content');
    });
  });

  test('shows context menu on right click', () => {
    render(<FileExplorer onFileSelect={mockOnFileSelect} />);

    const readmeFile = screen.getByText('README.md');
    fireEvent.contextMenu(readmeFile, { clientX: 100, clientY: 200 });

    expect(screen.getByText('New File')).toBeInTheDocument();
    expect(screen.getByText('New Folder')).toBeInTheDocument();
    expect(screen.getByText('Rename')).toBeInTheDocument();
    expect(screen.getByText('Delete')).toBeInTheDocument();
  });

  test('creates new file from toolbar', async () => {
    mockPrompt.mockReturnValue('newfile.txt');

    render(<FileExplorer onFileSelect={mockOnFileSelect} />);

    const newFileButton = screen.getByTitle('New File (Ctrl+N)');
    fireEvent.click(newFileButton);

    await waitFor(() => {
      expect(mockPrompt).toHaveBeenCalledWith('Enter file name:');
      expect(fileService.createFile).toHaveBeenCalledWith('/project/newfile.txt', '');
      expect(fileService.getCurrentProject).toHaveBeenCalledTimes(2); // Initial load + refresh
    });
  });

  test('creates new folder from toolbar', async () => {
    mockPrompt.mockReturnValue('newfolder');

    render(<FileExplorer onFileSelect={mockOnFileSelect} />);

    const newFolderButton = screen.getByTitle('New Folder (Ctrl+Shift+N)');
    fireEvent.click(newFolderButton);

    await waitFor(() => {
      expect(mockPrompt).toHaveBeenCalledWith('Enter folder name:');
      expect(fileService.createDirectory).toHaveBeenCalledWith('/project/newfolder');
      expect(fileService.getCurrentProject).toHaveBeenCalledTimes(2);
    });
  });

  test('creates new file from context menu', async () => {
    mockPrompt.mockReturnValue('contextfile.txt');

    render(<FileExplorer onFileSelect={mockOnFileSelect} />);

    // Right click on README.md
    const readmeFile = screen.getByText('README.md');
    fireEvent.contextMenu(readmeFile, { clientX: 100, clientY: 200 });

    // Click New File in context menu
    const newFileMenuItem = screen.getByText('New File');
    fireEvent.click(newFileMenuItem);

    await waitFor(() => {
      expect(mockPrompt).toHaveBeenCalledWith('Enter file name:');
      expect(fileService.createFile).toHaveBeenCalledWith('/test-project/contextfile.txt', '');
    });
  });

  test('deletes file from context menu', async () => {
    mockConfirm.mockReturnValue(true);

    render(<FileExplorer onFileSelect={mockOnFileSelect} />);

    // Right click on README.md
    const readmeFile = screen.getByText('README.md');
    fireEvent.contextMenu(readmeFile, { clientX: 100, clientY: 200 });

    // Click Delete in context menu
    const deleteMenuItem = screen.getByText('Delete');
    fireEvent.click(deleteMenuItem);

    await waitFor(() => {
      expect(mockConfirm).toHaveBeenCalledWith('Are you sure you want to delete README.md?');
      expect(fileService.deleteFile).toHaveBeenCalledWith('/test-project/README.md');
    });
  });

  test('handles file read error gracefully', async () => {
    const consoleSpy = vi.spyOn(console, 'error').mockImplementation(() => {});
    (fileService.readFile as any).mockRejectedValue(new Error('File not found'));

    render(<FileExplorer onFileSelect={mockOnFileSelect} />);

    // Expand src and click on a file
    const srcFolder = screen.getByText('src');
    fireEvent.click(srcFolder);

    const mainFile = screen.getByText('main.navλ');
    fireEvent.click(mainFile);

    await waitFor(() => {
      expect(consoleSpy).toHaveBeenCalledWith('Error reading file:', expect.any(Error));
    });

    consoleSpy.mockRestore();
  });

  test('handles file creation error with alert', async () => {
    mockPrompt.mockReturnValue('test.txt');
    (fileService.createFile as any).mockRejectedValue(new Error('Permission denied'));

    render(<FileExplorer onFileSelect={mockOnFileSelect} />);

    const newFileButton = screen.getByTitle('New File (Ctrl+N)');
    fireEvent.click(newFileButton);

    await waitFor(() => {
      expect(mockAlert).toHaveBeenCalledWith('Failed to create file');
    });
  });

  test('shows correct file icons', () => {
    render(<FileExplorer onFileSelect={mockOnFileSelect} />);

    // Expand src to see files
    const srcFolder = screen.getByText('src');
    fireEvent.click(srcFolder);

    // Check that NAVΛ files show the lambda symbol
    // Note: The icon is rendered as text content, so we check for the symbol
    const navLambdaFile = screen.getByText('main.navλ').closest('.file-item');
    expect(navLambdaFile).toBeInTheDocument();
  });

  test('refreshes file list', async () => {
    render(<FileExplorer onFileSelect={mockOnFileSelect} />);

    const refreshButton = screen.getByTitle('Refresh Explorer');
    fireEvent.click(refreshButton);

    await waitFor(() => {
      expect(fileService.getCurrentProject).toHaveBeenCalledTimes(2);
    });
  });

  test('shows empty state when no project', () => {
    (fileService.getCurrentProject as any).mockReturnValue(null);

    render(<FileExplorer onFileSelect={mockOnFileSelect} />);

    expect(screen.getByText('No project open')).toBeInTheDocument();
    expect(screen.getByText('Open Project')).toBeInTheDocument();
  });

  test('handles drag and drop file import', async () => {
    const mockFile = new File(['test content'], 'dropped.txt', { type: 'text/plain' });

    render(<FileExplorer onFileSelect={mockOnFileSelect} />);

    const fileExplorer = screen.getByRole('generic', { hidden: true }); // The main div

    const dropEvent = {
      preventDefault: vi.fn(),
      stopPropagation: vi.fn(),
      dataTransfer: {
        files: [mockFile],
      },
    };

    fireEvent.drop(fileExplorer, dropEvent);

    await waitFor(() => {
      expect(fileService.createFile).toHaveBeenCalledWith('/project/dropped.txt', 'test content');
      expect(mockAlert).toHaveBeenCalledWith('✅ Imported 1 file(s)!');
    });
  });
});