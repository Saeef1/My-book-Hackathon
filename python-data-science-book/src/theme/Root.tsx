import React from 'react';
import FloatingChatbot from '@site/src/components/FloatingChatbot';
import { useDocusaurusContext } from '@docusaurus/core';

// Default wrapper for the whole Docusaurus site
export default function Root({ children }: { children: React.ReactNode }) {
  const { siteConfig } = useDocusaurusContext();
  const backendUrl = (siteConfig.customFields as any)?.ragApiUrl || 'https://saeef-backend-space.hf.space';

  return (
    <>
      {children}
      <FloatingChatbot
        backendUrl={backendUrl}
        title="Python Data Science Assistant"
        description="Ask me about the documentation"
      />
    </>
  );
}