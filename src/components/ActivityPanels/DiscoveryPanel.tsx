/**
 * Discovery Panel
 * 
 * Premium discovery section inspired by Perplexity Discover:
 * - Real-time Reddit news
 * - RSS feeds
 * - Hugging Face daily papers
 * - Enterprise-grade UI
 */

import React, { useState, useEffect, useCallback } from 'react';
import { discoveryService, type DiscoveryItem } from '../../services/discovery-service';
import { ExternalLink, RefreshCw, TrendingUp, BookOpen, MessageSquare, Clock, ArrowUp, Filter, Maximize2 } from 'lucide-react';
import { DiscoveryViewer } from './DiscoveryViewer';
import './DiscoveryPanel.css';

type FilterType = 'all' | 'reddit' | 'rss' | 'huggingface';
type SortType = 'recent' | 'popular' | 'trending';

export const DiscoveryPanel: React.FC = () => {
  const [items, setItems] = useState<DiscoveryItem[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [filter, setFilter] = useState<FilterType>('all');
  const [sort, setSort] = useState<SortType>('trending');
  const [selectedCategory, setSelectedCategory] = useState<string | null>(null);
  const [selectedItem, setSelectedItem] = useState<DiscoveryItem | null>(null);
  const [isViewerOpen, setIsViewerOpen] = useState(false);

  const fetchItems = useCallback(async () => {
    setLoading(true);
    setError(null);
    
    try {
      const { reddit, rss, huggingface } = await discoveryService.fetchAll();
      
      let allItems: DiscoveryItem[] = [];
      
      switch (filter) {
        case 'reddit':
          allItems = reddit;
          break;
        case 'rss':
          allItems = rss;
          break;
        case 'huggingface':
          allItems = huggingface;
          break;
        default:
          allItems = [...reddit, ...rss, ...huggingface];
      }

      // Apply sorting
      switch (sort) {
        case 'recent':
          allItems.sort((a, b) => new Date(b.publishedAt).getTime() - new Date(a.publishedAt).getTime());
          break;
        case 'popular':
          allItems.sort((a, b) => (b.upvotes || 0) - (a.upvotes || 0));
          break;
        case 'trending':
          // Combine recency and popularity
          allItems.sort((a, b) => {
            const scoreA = (a.upvotes || 0) * 0.6 + (new Date().getTime() - new Date(a.publishedAt).getTime()) / 10000000;
            const scoreB = (b.upvotes || 0) * 0.6 + (new Date().getTime() - new Date(b.publishedAt).getTime()) / 10000000;
            return scoreB - scoreA;
          });
          break;
      }

      // Filter by category if selected
      if (selectedCategory) {
        allItems = allItems.filter(item => item.category === selectedCategory);
      }

      setItems(allItems);
    } catch (err) {
      console.error('Failed to fetch discovery items:', err);
      setError('Failed to load discovery items. Please try again.');
    } finally {
      setLoading(false);
    }
  }, [filter, sort, selectedCategory]);

  useEffect(() => {
    fetchItems();
    
    // Auto-refresh every 5 minutes
    const interval = setInterval(fetchItems, 5 * 60 * 1000);
    return () => clearInterval(interval);
  }, [fetchItems]);

  const getSourceIcon = (sourceType: DiscoveryItem['sourceType']) => {
    switch (sourceType) {
      case 'reddit':
        return '🔴';
      case 'rss':
        return '📰';
      case 'huggingface':
        return '🤗';
      default:
        return '📄';
    }
  };

  const getTimeAgo = (dateString: string): string => {
    const date = new Date(dateString);
    const now = new Date();
    const diffMs = now.getTime() - date.getTime();
    const diffMins = Math.floor(diffMs / 60000);
    const diffHours = Math.floor(diffMs / 3600000);
    const diffDays = Math.floor(diffMs / 86400000);

    if (diffMins < 1) return 'Just now';
    if (diffMins < 60) return `${diffMins}m ago`;
    if (diffHours < 24) return `${diffHours}h ago`;
    if (diffDays < 7) return `${diffDays}d ago`;
    return date.toLocaleDateString();
  };

  const categories = Array.from(new Set(items.map(item => item.category).filter(Boolean)));

  return (
    <div className="discovery-panel">
      <div className="discovery-header">
        <div className="discovery-header-top">
          <h2 className="discovery-title">
            <TrendingUp size={20} />
            <span>Discover</span>
          </h2>
          <button 
            className="discovery-refresh-btn"
            onClick={fetchItems}
            disabled={loading}
            title="Refresh"
          >
            <RefreshCw size={16} className={loading ? 'spinning' : ''} />
          </button>
        </div>

        <div className="discovery-filters">
          <div className="filter-group">
            <Filter size={14} />
            <button
              className={`filter-btn ${filter === 'all' ? 'active' : ''}`}
              onClick={() => setFilter('all')}
            >
              All
            </button>
            <button
              className={`filter-btn ${filter === 'reddit' ? 'active' : ''}`}
              onClick={() => setFilter('reddit')}
            >
              🔴 Reddit
            </button>
            <button
              className={`filter-btn ${filter === 'rss' ? 'active' : ''}`}
              onClick={() => setFilter('rss')}
            >
              📰 RSS
            </button>
            <button
              className={`filter-btn ${filter === 'huggingface' ? 'active' : ''}`}
              onClick={() => setFilter('huggingface')}
            >
              🤗 Papers
            </button>
          </div>

          <div className="sort-group">
            <select
              className="sort-select"
              value={sort}
              onChange={(e) => setSort(e.target.value as SortType)}
            >
              <option value="trending">🔥 Trending</option>
              <option value="recent">🕐 Recent</option>
              <option value="popular">⭐ Popular</option>
            </select>
          </div>
        </div>

        {categories.length > 0 && (
          <div className="category-filters">
            <button
              className={`category-chip ${selectedCategory === null ? 'active' : ''}`}
              onClick={() => setSelectedCategory(null)}
            >
              All Categories
            </button>
            {categories.slice(0, 8).map(category => (
              <button
                key={category}
                className={`category-chip ${selectedCategory === category ? 'active' : ''}`}
                onClick={() => setSelectedCategory(category)}
              >
                {category}
              </button>
            ))}
          </div>
        )}
      </div>

      <div className="discovery-content">
        {loading && items.length === 0 ? (
          <div className="discovery-loading">
            <RefreshCw size={24} className="spinning" />
            <p>Loading discovery items...</p>
          </div>
        ) : error ? (
          <div className="discovery-error">
            <p>{error}</p>
            <button onClick={fetchItems} className="retry-btn">
              Retry
            </button>
          </div>
        ) : items.length === 0 ? (
          <div className="discovery-empty">
            <BookOpen size={48} />
            <p>No items found</p>
            <button onClick={fetchItems} className="retry-btn">
              Refresh
            </button>
          </div>
        ) : (
          <div className="discovery-items">
            {items.map((item) => (
              <div
                key={item.id}
                className="discovery-item"
                onClick={() => {
                  setSelectedItem(item);
                  setIsViewerOpen(true);
                }}
              >
                <div className="discovery-item-header">
                  <div className="discovery-item-source">
                    <span className="source-icon">{getSourceIcon(item.sourceType)}</span>
                    <span className="source-name">{item.source}</span>
                    {item.category && (
                      <span className="source-category">{item.category}</span>
                    )}
                  </div>
                  <ExternalLink size={14} className="external-link-icon" />
                </div>

                {item.thumbnail && (
                  <img
                    src={item.thumbnail}
                    alt={item.title}
                    className="discovery-item-thumbnail"
                    loading="lazy"
                    onError={(e) => {
                      // Hide image if it fails to load
                      e.currentTarget.style.display = 'none';
                    }}
                  />
                )}

                <h3 className="discovery-item-title">{item.title}</h3>

                {item.description && (
                  <p className="discovery-item-description">{item.description}</p>
                )}

                <div className="discovery-item-footer">
                  <div className="discovery-item-meta">
                    {item.author && (
                      <span className="meta-item">
                        <span className="meta-label">By</span>
                        {item.author}
                      </span>
                    )}
                    <span className="meta-item">
                      <Clock size={12} />
                      {getTimeAgo(item.publishedAt)}
                    </span>
                    {item.upvotes !== undefined && item.upvotes > 0 && (
                      <span className="meta-item">
                        <ArrowUp size={12} />
                        {item.upvotes}
                      </span>
                    )}
                    {item.comments !== undefined && item.comments > 0 && (
                      <span className="meta-item">
                        <MessageSquare size={12} />
                        {item.comments}
                      </span>
                    )}
                  </div>
                  <button
                    className="discovery-item-expand-btn"
                    onClick={(e) => {
                      e.stopPropagation();
                      setSelectedItem(item);
                      setIsViewerOpen(true);
                    }}
                    title="Open in full screen"
                  >
                    <Maximize2 size={14} />
                  </button>
                </div>
              </div>
            ))}
          </div>
        )}
      </div>

      {/* Full Screen Viewer */}
      <DiscoveryViewer
        item={selectedItem}
        isOpen={isViewerOpen}
        onClose={() => {
          setIsViewerOpen(false);
          setSelectedItem(null);
        }}
      />
    </div>
  );
};

