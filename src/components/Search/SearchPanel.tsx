import React, { useState, useEffect, useRef } from 'react';
import { Search, File, Code, X } from 'lucide-react';
import { fileService, FileEntry } from '../../services/file-service';
import './SearchPanel.css';

interface SearchResult {
  file: FileEntry;
  matches: {
    line: number;
    content: string;
    matchIndex: number;
  }[];
}

export const SearchPanel: React.FC = () => {
  const [query, setQuery] = useState('');
  const [results, setResults] = useState<SearchResult[]>([]);
  const [isSearching, setIsSearching] = useState(false);
  const [searchOptions, setSearchOptions] = useState({
    caseSensitive: false,
    wholeWord: false,
    regex: false,
  });
  const inputRef = useRef<HTMLInputElement>(null);

  useEffect(() => {
    if (query.trim()) {
      performSearch(query);
    } else {
      setResults([]);
    }
  }, [query, searchOptions]);

  const performSearch = async (searchQuery: string) => {
    setIsSearching(true);
    try {
      const project = fileService.getCurrentProject();
      if (!project) {
        setResults([]);
        return;
      }

      const searchResults: SearchResult[] = [];
      await searchInFiles(project.files, searchQuery, searchResults);
      setResults(searchResults);
    } catch (error) {
      console.error('Search error:', error);
    } finally {
      setIsSearching(false);
    }
  };

  const searchInFiles = async (
    files: FileEntry[],
    searchQuery: string,
    results: SearchResult[]
  ) => {
    for (const file of files) {
      if (file.isDirectory && file.children) {
        await searchInFiles(file.children, searchQuery, results);
      } else if (!file.isDirectory) {
        try {
          const content = await fileService.readFile(file.path);
          const matches = findMatches(content, searchQuery, searchOptions);
          if (matches.length > 0) {
            results.push({ file, matches });
          }
        } catch (error) {
          // Skip files that can't be read
        }
      }
    }
  };

  const findMatches = (
    content: string,
    searchQuery: string,
    options: typeof searchOptions
  ): { line: number; content: string; matchIndex: number }[] => {
    const lines = content.split('\n');
    const matches: { line: number; content: string; matchIndex: number }[] = [];
    let pattern: RegExp;

    if (options.regex) {
      try {
        pattern = new RegExp(
          searchQuery,
          options.caseSensitive ? 'g' : 'gi'
        );
      } catch (error) {
        // Invalid regex, fall back to literal search
        pattern = new RegExp(
          searchQuery.replace(/[.*+?^${}()|[\]\\]/g, '\\$&'),
          options.caseSensitive ? 'g' : 'gi'
        );
      }
    } else {
      const escaped = searchQuery.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
      const wordBoundary = options.wholeWord ? '\\b' : '';
      pattern = new RegExp(
        `${wordBoundary}${escaped}${wordBoundary}`,
        options.caseSensitive ? 'g' : 'gi'
      );
    }

    lines.forEach((line, index) => {
      const match = pattern.exec(line);
      if (match) {
        matches.push({
          line: index + 1,
          content: line,
          matchIndex: match.index,
        });
      }
    });

    return matches;
  };

  const handleResultClick = async (result: SearchResult, match: typeof results[0]['matches'][0]) => {
    // Open file and navigate to line
    try {
      const content = await fileService.readFile(result.file.path);
      const event = new CustomEvent('nava:open-file', {
        detail: {
          path: result.file.path,
          content,
          line: match.line,
        },
      });
      window.dispatchEvent(event);
    } catch (error) {
      console.error('Error opening file:', error);
    }
  };

  return (
    <div className="search-panel">
      <div className="search-header">
        <div className="search-input-container">
          <Search size={16} className="search-icon" />
          <input
            ref={inputRef}
            type="text"
            placeholder="Search (Ctrl+Shift+F)"
            value={query}
            onChange={(e) => setQuery(e.target.value)}
            className="search-input"
            autoFocus
          />
          {query && (
            <button
              className="search-clear-btn"
              onClick={() => setQuery('')}
              title="Clear"
            >
              <X size={14} />
            </button>
          )}
        </div>
        <div className="search-options">
          <label className="search-option">
            <input
              type="checkbox"
              checked={searchOptions.caseSensitive}
              onChange={(e) =>
                setSearchOptions({ ...searchOptions, caseSensitive: e.target.checked })
              }
            />
            <span>Match Case</span>
          </label>
          <label className="search-option">
            <input
              type="checkbox"
              checked={searchOptions.wholeWord}
              onChange={(e) =>
                setSearchOptions({ ...searchOptions, wholeWord: e.target.checked })
              }
            />
            <span>Whole Word</span>
          </label>
          <label className="search-option">
            <input
              type="checkbox"
              checked={searchOptions.regex}
              onChange={(e) =>
                setSearchOptions({ ...searchOptions, regex: e.target.checked })
              }
            />
            <span>Regex</span>
          </label>
        </div>
      </div>
      <div className="search-results">
        {isSearching ? (
          <div className="search-loading">Searching...</div>
        ) : results.length === 0 && query ? (
          <div className="search-empty">No results found</div>
        ) : results.length > 0 ? (
          <div className="search-results-list">
            {results.map((result, resultIndex) => (
              <div key={resultIndex} className="search-result-group">
                <div className="search-result-file">
                  <File size={14} />
                  <span>{result.file.path}</span>
                  <span className="search-result-count">
                    {result.matches.length} {result.matches.length === 1 ? 'match' : 'matches'}
                  </span>
                </div>
                {result.matches.map((match, matchIndex) => (
                  <div
                    key={matchIndex}
                    className="search-result-item"
                    onClick={() => handleResultClick(result, match)}
                  >
                    <span className="search-result-line">{match.line}</span>
                    <span className="search-result-content">
                      {match.content.substring(0, match.matchIndex)}
                      <mark className="search-highlight">
                        {match.content.substring(
                          match.matchIndex,
                          match.matchIndex + query.length
                        )}
                      </mark>
                      {match.content.substring(match.matchIndex + query.length)}
                    </span>
                  </div>
                ))}
              </div>
            ))}
          </div>
        ) : (
          <div className="search-empty">Enter a search query to find files</div>
        )}
      </div>
    </div>
  );
};

