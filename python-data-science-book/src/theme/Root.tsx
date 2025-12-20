import React from 'react';
import FloatingChatbot from '@site/src/components/FloatingChatbot';

// Default wrapper for the whole Docusaurus site
export default function Root({ children }: { children: React.ReactNode }) {
  return (
    <>
      {children}
      <FloatingChatbot
        backendUrl="http://localhost:5000"
        title="Python Data Science Assistant"
        description="Ask me about the documentation"
      />
    </>
  );
}