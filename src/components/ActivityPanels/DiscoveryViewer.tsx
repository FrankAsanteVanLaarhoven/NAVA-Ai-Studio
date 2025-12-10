/**
 * Discovery Viewer
 * 
 * Full-screen discovery viewer with rich content:
 * - Images, videos, and media
 * - Perplexity-style layout
 * - Expandable from sidebar
 */

import React, { useState, useEffect } from 'react';
import { X, ExternalLink, ArrowLeft, RefreshCw, TrendingUp, Clock, ArrowUp, MessageSquare, Image as ImageIcon, Video, FileText } from 'lucide-react';
import { discoveryService, type DiscoveryItem } from '../../services/discovery-service';
import './DiscoveryViewer.css';

interface DiscoveryViewerProps {
  item: DiscoveryItem | null;
  isOpen: boolean;
  onClose: () => void;
  onBack?: () => void;
}

export const DiscoveryViewer: React.FC<DiscoveryViewerProps> = ({ item, isOpen, onClose, onBack }) => {
  const [content, setContent] = useState<any>(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    if (isOpen && item) {
      fetchContent(item);
    }
  }, [isOpen, item]);

  const fetchContent = async (discoveryItem: DiscoveryItem) => {
    setLoading(true);
    setError(null);

    try {
      // For Reddit posts, fetch full content
      if (discoveryItem.sourceType === 'reddit') {
        const redditUrl = discoveryItem.url.replace('reddit.com', 'reddit.com.json');
        const response = await fetch(`https://api.allorigins.win/get?url=${encodeURIComponent(redditUrl)}`);
        const data = await response.json();
        
        if (data.contents) {
          const jsonData = JSON.parse(data.contents);
          const post = jsonData[0]?.data?.children[0]?.data;
          
          if (post) {
            setContent({
              title: post.title,
              author: post.author,
              subreddit: post.subreddit,
              selftext: post.selftext,
              url: post.url,
              thumbnail: post.thumbnail && post.thumbnail !== 'self' ? post.thumbnail : null,
              preview: post.preview,
              media: post.media,
              upvotes: post.ups,
              comments: post.num_comments,
              created: post.created_utc,
              permalink: post.permalink,
            });
          }
        }
      } else if (discoveryItem.sourceType === 'rss') {
        // For RSS, try to fetch the article content
        const response = await fetch(`https://api.allorigins.win/get?url=${encodeURIComponent(discoveryItem.url)}`);
        const data = await response.json();
        
        if (data.contents) {
          const parser = new DOMParser();
          const doc = parser.parseFromString(data.contents, 'text/html');
          
          // Extract images
          const images = Array.from(doc.querySelectorAll('img')).map(img => ({
            src: img.getAttribute('src') || img.getAttribute('data-src'),
            alt: img.getAttribute('alt') || '',
          })).filter(img => img.src);

          // Extract videos
          const videos = Array.from(doc.querySelectorAll('video, iframe[src*="youtube"], iframe[src*="vimeo"]')).map(video => ({
            src: video.getAttribute('src'),
            type: video.tagName.toLowerCase(),
          })).filter(video => video.src);

          // Extract main content
          const article = doc.querySelector('article, .article, .post-content, .entry-content, main');
          const contentText = article?.textContent || doc.body.textContent || '';

          setContent({
            title: discoveryItem.title,
            description: discoveryItem.description,
            url: discoveryItem.url,
            images,
            videos,
            content: contentText.substring(0, 5000), // Limit content
            author: discoveryItem.author,
            publishedAt: discoveryItem.publishedAt,
          });
        }
      } else {
        // For Hugging Face papers, use basic info
        setContent({
          title: discoveryItem.title,
          description: discoveryItem.description,
          url: discoveryItem.url,
          author: discoveryItem.author,
          upvotes: discoveryItem.upvotes,
          comments: discoveryItem.comments,
          publishedAt: discoveryItem.publishedAt,
        });
      }
    } catch (err) {
      console.error('Failed to fetch content:', err);
      setError('Failed to load content. Please try again.');
      // Fallback to basic content
      setContent({
        title: item.title,
        description: item.description,
        url: item.url,
      });
    } finally {
      setLoading(false);
    }
  };

  if (!isOpen || !item) return null;

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

  return (
    <div className="discovery-viewer-overlay" onClick={onClose}>
      <div className="discovery-viewer-container" onClick={(e) => e.stopPropagation()}>
        {/* Header */}
        <div className="discovery-viewer-header">
          <div className="discovery-viewer-header-left">
            {onBack && (
              <button className="discovery-viewer-back-btn" onClick={onBack}>
                <ArrowLeft size={20} />
              </button>
            )}
            <div className="discovery-viewer-source">
              <span className="source-icon-large">{getSourceIcon(item.sourceType)}</span>
              <span className="source-name-large">{item.source}</span>
            </div>
          </div>
          <div className="discovery-viewer-header-right">
            <a
              href={item.url}
              target="_blank"
              rel="noopener noreferrer"
              className="discovery-viewer-external-btn"
              onClick={(e) => e.stopPropagation()}
            >
              <ExternalLink size={18} />
              <span>Open Original</span>
            </a>
            <button className="discovery-viewer-close-btn" onClick={onClose}>
              <X size={20} />
            </button>
          </div>
        </div>

        {/* Content */}
        <div className="discovery-viewer-content">
          {loading ? (
            <div className="discovery-viewer-loading">
              <RefreshCw size={32} className="spinning" />
              <p>Loading content...</p>
            </div>
          ) : error ? (
            <div className="discovery-viewer-error">
              <p>{error}</p>
            </div>
          ) : content ? (
            <>
              {/* Title */}
              <h1 className="discovery-viewer-title">{content.title || item.title}</h1>

              {/* Metadata */}
              <div className="discovery-viewer-meta">
                {content.author && (
                  <span className="meta-item">
                    <span className="meta-label">By</span>
                    {content.author}
                  </span>
                )}
                {content.subreddit && (
                  <span className="meta-item">
                    <span className="meta-label">r/</span>
                    {content.subreddit}
                  </span>
                )}
                <span className="meta-item">
                  <Clock size={14} />
                  {getTimeAgo(content.created ? new Date(content.created * 1000).toISOString() : content.publishedAt || item.publishedAt)}
                </span>
                {content.upvotes !== undefined && (
                  <span className="meta-item">
                    <ArrowUp size={14} />
                    {content.upvotes}
                  </span>
                )}
                {content.comments !== undefined && (
                  <span className="meta-item">
                    <MessageSquare size={14} />
                    {content.comments}
                  </span>
                )}
              </div>

              {/* Media Gallery */}
              {(content.images?.length > 0 || content.thumbnail || content.preview) && (
                <div className="discovery-viewer-media">
                  {content.preview?.images?.[0]?.source?.url && (
                    <img
                      src={content.preview.images[0].source.url.replace(/&amp;/g, '&')}
                      alt={content.title}
                      className="discovery-viewer-main-image"
                    />
                  )}
                  {content.thumbnail && content.thumbnail !== 'self' && !content.preview && (
                    <img
                      src={content.thumbnail}
                      alt={content.title}
                      className="discovery-viewer-main-image"
                    />
                  )}
                  {content.images?.length > 0 && !content.preview && (
                    <div className="discovery-viewer-image-gallery">
                      {content.images.slice(0, 6).map((img: any, index: number) => (
                        <img
                          key={index}
                          src={img.src}
                          alt={img.alt || `Image ${index + 1}`}
                          className="discovery-viewer-gallery-image"
                          loading="lazy"
                        />
                      ))}
                    </div>
                  )}
                </div>
              )}

              {/* Videos */}
              {content.videos?.length > 0 && (
                <div className="discovery-viewer-videos">
                  {content.videos.map((video: any, index: number) => (
                    <div key={index} className="discovery-viewer-video-container">
                      {video.type === 'iframe' ? (
                        <iframe
                          src={video.src}
                          className="discovery-viewer-video"
                          allowFullScreen
                          frameBorder="0"
                        />
                      ) : (
                        <video
                          src={video.src}
                          className="discovery-viewer-video"
                          controls
                        />
                      )}
                    </div>
                  ))}
                </div>
              )}

              {/* Main Content */}
              <div className="discovery-viewer-body">
                {content.selftext && (
                  <div className="discovery-viewer-text-content">
                    {content.selftext.split('\n').map((paragraph: string, index: number) => (
                      paragraph.trim() && (
                        <p key={index} className="discovery-viewer-paragraph">
                          {paragraph}
                        </p>
                      )
                    ))}
                  </div>
                )}
                {content.content && (
                  <div className="discovery-viewer-text-content">
                    <p className="discovery-viewer-paragraph">{content.content}</p>
                  </div>
                )}
                {content.description && !content.selftext && !content.content && (
                  <div className="discovery-viewer-text-content">
                    <p className="discovery-viewer-paragraph">{content.description}</p>
                  </div>
                )}
              </div>

              {/* Footer Actions */}
              <div className="discovery-viewer-footer">
                <a
                  href={item.url}
                  target="_blank"
                  rel="noopener noreferrer"
                  className="discovery-viewer-read-more-btn"
                >
                  <ExternalLink size={16} />
                  <span>Read Full Article</span>
                </a>
              </div>
            </>
          ) : (
            <div className="discovery-viewer-empty">
              <p>No content available</p>
            </div>
          )}
        </div>
      </div>
    </div>
  );
};

