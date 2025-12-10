import React, { useState, useEffect } from 'react';
import { File, Code, X, ExternalLink } from 'lucide-react';
import { fileService, FileEntry } from '../../services/file-service';
import './FileReferenceViewer.css';

interface Reference {
  file: FileEntry;
  line: number;
  content: string;
  type: 'definition' | 'reference' | 'import' | 'usage';
}

interface FileReferenceViewerProps {
  symbol?: string;
  filePath?: string;
  onClose?: () => void;
}

export const FileReferenceViewer: React.FC<FileReferenceViewerProps> = ({
  symbol,
  filePath,
  onClose,
}) => {
  const [references, setReferences] = useState<Reference[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [selectedType, setSelectedType] = useState<string>('all');

  useEffect(() => {
    if (symbol || filePath) {
      findReferences();
    }
  }, [symbol, filePath]);

  const findReferences = async () => {
    setIsLoading(true);
    try {
      const project = fileService.getCurrentProject();
      if (!project) {
        setReferences([]);
        return;
      }

      const refs: Reference[] = [];
      await searchReferences(project.files, symbol || filePath || '', refs);
      setReferences(refs);
    } catch (error) {
      console.error('Error finding references:', error);
    } finally {
      setIsLoading(false);
    }
  };

  const searchReferences = async (
    files: FileEntry[],
    searchTerm: string,
    refs: Reference[]
  ) => {
    for (const file of files) {
      if (file.isDirectory && file.children) {
        await searchReferences(file.children, searchTerm, refs);
      } else if (!file.isDirectory) {
        try {
          const content = await fileService.readFile(file.path);
          const lines = content.split('\n');
          
          lines.forEach((line, index) => {
            if (symbol && line.includes(symbol)) {
              refs.push({
                file,
                line: index + 1,
                content: line.trim(),
                type: determineReferenceType(line, symbol),
              });
            } else if (filePath && line.includes(filePath)) {
              refs.push({
                file,
                line: index + 1,
                content: line.trim(),
                type: 'reference',
              });
            }
          });
        } catch (error) {
          // Skip files that can't be read
        }
      }
    }
  };

  const determineReferenceType = (line: string, symbol: string): Reference['type'] => {
    if (line.includes(`import`) || line.includes(`from`)) {
      return 'import';
    }
    if (line.includes(`def `) || line.includes(`function `) || line.includes(`let `)) {
      return 'definition';
    }
    return 'usage';
  };

  const filteredReferences = selectedType === 'all'
    ? references
    : references.filter(ref => ref.type === selectedType);

  const handleReferenceClick = async (ref: Reference) => {
    try {
      const content = await fileService.readFile(ref.file.path);
      const event = new CustomEvent('nava:open-file', {
        detail: {
          path: ref.file.path,
          content,
          line: ref.line,
        },
      });
      window.dispatchEvent(event);
    } catch (error) {
      console.error('Error opening file:', error);
    }
  };

  const typeCounts = {
    all: references.length,
    definition: references.filter(r => r.type === 'definition').length,
    reference: references.filter(r => r.type === 'reference').length,
    import: references.filter(r => r.type === 'import').length,
    usage: references.filter(r => r.type === 'usage').length,
  };

  return (
    <div className="file-reference-viewer">
      <div className="file-reference-header">
        <div className="file-reference-title">
          <Code size={16} />
          <span>
            {symbol ? `References: ${symbol}` : filePath ? `References: ${filePath}` : 'References'}
          </span>
        </div>
        {onClose && (
          <button className="file-reference-close" onClick={onClose}>
            <X size={14} />
          </button>
        )}
      </div>
      <div className="file-reference-filters">
        {(['all', 'definition', 'reference', 'import', 'usage'] as const).map(type => (
          <button
            key={type}
            className={`file-reference-filter ${selectedType === type ? 'active' : ''}`}
            onClick={() => setSelectedType(type)}
          >
            {type.charAt(0).toUpperCase() + type.slice(1)} ({typeCounts[type]})
          </button>
        ))}
      </div>
      <div className="file-reference-results">
        {isLoading ? (
          <div className="file-reference-loading">Finding references...</div>
        ) : filteredReferences.length === 0 ? (
          <div className="file-reference-empty">No references found</div>
        ) : (
          <div className="file-reference-list">
            {filteredReferences.map((ref, index) => (
              <div
                key={index}
                className="file-reference-item"
                onClick={() => handleReferenceClick(ref)}
              >
                <div className="file-reference-item-header">
                  <File size={12} />
                  <span className="file-reference-path">{ref.file.path}</span>
                  <span className="file-reference-line">:{ref.line}</span>
                  <span className={`file-reference-type file-reference-type-${ref.type}`}>
                    {ref.type}
                  </span>
                </div>
                <div className="file-reference-content">
                  {ref.content}
                </div>
              </div>
            ))}
          </div>
        )}
      </div>
    </div>
  );
};

