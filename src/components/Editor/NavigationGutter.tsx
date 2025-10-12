import React from 'react';

interface NavigationGutterProps {
  lineCount: number;
  navigationHints: Map<number, string>;
}

export const NavigationGutter: React.FC<NavigationGutterProps> = ({
  lineCount,
  navigationHints,
}) => {
  return (
    <div className="navigation-gutter">
      {Array.from({ length: lineCount }, (_, i) => i + 1).map((lineNum) => (
        <div key={lineNum} className="gutter-line">
          {navigationHints.has(lineNum) && (
            <span className="nav-hint" title={navigationHints.get(lineNum)}>
              ⋋
            </span>
          )}
        </div>
      ))}
    </div>
  );
};

