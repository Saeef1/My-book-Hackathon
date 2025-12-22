import React, { useState, useEffect, useRef } from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';
import RagApiService from '@site/src/services/rag-api';

type Message = {
  id: string;
  text: string;
  sender: 'user' | 'bot';
  sources?: string[];
  timestamp: Date;
};

type FloatingChatbotProps = {
  backendUrl?: string;
  title?: string;
  description?: string;
};

const FloatingChatbot: React.FC<FloatingChatbotProps> = ({
  backendUrl = process.env.REACT_APP_RAG_API_URL ,
  title = 'Documentation Assistant',
  description = 'Ask me anything about the Python Data Science Book'
}) => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputText, setInputText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedProvider, setSelectedProvider] = useState('gemini');
  const [error, setError] = useState<string | null>(null);
  const messagesEndRef = useRef<null | HTMLDivElement>(null);
  const chatContainerRef = useRef<HTMLDivElement>(null);
  const ragService = new RagApiService(backendUrl);

  // Toggle chat window
  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  // Close chat when clicking outside
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (chatContainerRef.current && !chatContainerRef.current.contains(event.target as Node)) {
        if (isOpen) {
          setIsOpen(false);
        }
      }
    };

    if (isOpen) {
      document.addEventListener('mousedown', handleClickOutside);
    } else {
      document.removeEventListener('mousedown', handleClickOutside);
    }

    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [isOpen]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!inputText.trim() || isLoading) return;

    try {
      // Add user message
      const userMessage: Message = {
        id: `user-${Date.now()}`,
        text: inputText,
        sender: 'user',
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, userMessage]);
      setInputText('');
      setIsLoading(true);
      setError(null);

      // Call backend API using the service
      const request = {
        query: inputText,
        provider: selectedProvider,
      };

      const data = await ragService.query(request);

      // Add bot response
      const botMessage: Message = {
        id: `bot-${Date.now()}`,
        text: data.answer,
        sender: 'bot',
        sources: data.sources || [],
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (err) {
      console.error('Error sending message:', err);
      setError('Failed to send message. Please try again.');

      // Add error message to chat
      const errorMessage: Message = {
        id: `error-${Date.now()}`,
        text: 'Sorry, I encountered an error. Please try again.',
        sender: 'bot',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const formatSources = (sources: string[] = []) => {
    if (!sources || sources.length === 0) return null;

    return (
      <div className={styles.sources}>
        <strong>Sources:</strong>
        <ul>
          {sources.slice(0, 3).map((source, index) => (
            <li key={index}>
              <a href={source} target="_blank" rel="noopener noreferrer">
                {source}
              </a>
            </li>
          ))}
        </ul>
      </div>
    );
  };

  return (
    <div className={styles.floatingChatbot}>
      {/* Floating icon */}
      <button
        className={clsx(styles.chatIcon, { [styles.open]: isOpen })}
        onClick={toggleChat}
        aria-label={isOpen ? "Close chat" : "Open chat"}
      >
        {isOpen ? (
          <svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
            <line x1="18" y1="6" x2="6" y2="18"></line>
            <line x1="6" y1="6" x2="18" y2="18"></line>
          </svg>
        ) : (
          <svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
          </svg>
        )}
      </button>

      {/* Chat window - only render when open */}
      {isOpen && (
        <div
          ref={chatContainerRef}
          className={clsx(styles.chatWindow, { [styles.open]: isOpen })}
        >
          <div className={styles.chatHeader}>
            <div>
              <h3>{title}</h3>
              <p>{description}</p>
            </div>
            <button
              className={styles.closeButton}
              onClick={toggleChat}
              aria-label="Close chat"
            >
              <svg xmlns="http://www.w3.org/2000/svg" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                <line x1="18" y1="6" x2="6" y2="18"></line>
                <line x1="6" y1="6" x2="18" y2="18"></line>
              </svg>
            </button>
          </div>

          <div className={styles.messages}>
            {messages.length === 0 ? (
              <div className={styles.welcomeMessage}>
                <p>Hello! I'm your documentation assistant. Ask me anything about the Python Data Science Book content.</p>
                <p className={styles.exampleQueries}>
                  <strong>Try asking:</strong><br />
                  • "What are Humanoid Robotics?"<br />
                  • "Why Physical AI Matters?"<br />
                  • "What is Cognitive Planning with LLMs?"
                </p>
              </div>
            ) : (
              messages.map((message) => (
                <div
                  key={message.id}
                  className={clsx(
                    styles.message,
                    message.sender === 'user' ? styles.userMessage : styles.botMessage
                  )}
                >
                  <div className={styles.messageContent}>
                    {message.text}
                  </div>
                  {message.sender === 'bot' && message.sources && formatSources(message.sources)}
                  <div className={styles.timestamp}>
                    {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                  </div>
                </div>
              ))
            )}
            {isLoading && (
              <div className={clsx(styles.message, styles.botMessage)}>
                <div className={styles.typingIndicator}>
                  Thinking...
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {error && (
            <div className={styles.error}>
              {error}
            </div>
          )}

          <form onSubmit={handleSubmit} className={styles.inputForm}>
            <div className={styles.inputGroup}>
              <input
                type="text"
                value={inputText}
                onChange={(e) => setInputText(e.target.value)}
                placeholder="Ask a question..."
                className={styles.input}
                disabled={isLoading}
                aria-label="Type your message"
              />
              <select
                value={selectedProvider}
                onChange={(e) => setSelectedProvider(e.target.value)}
                className={styles.providerSelect}
                disabled={isLoading}
                aria-label="Select AI provider"
              >
                <option value="gemini">Gemini</option>
              </select>
            </div>
            <div className={styles.formActions}>
              <div className={styles.providerInfo}>
                Using: <strong>Gemini</strong>
              </div>
              <button
                type="submit"
                className={styles.sendButton}
                disabled={isLoading || !inputText.trim()}
                aria-label="Send message"
              >
                {isLoading ? (
                  <svg className={styles.spinner} xmlns="http://www.w3.org/2000/svg" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                    <path d="M21 12a9 9 0 1 1-6.219-8.56"></path>
                  </svg>
                ) : (
                  <svg xmlns="http://www.w3.org/2000/svg" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                    <line x1="22" y1="2" x2="11" y2="13"></line>
                    <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
                  </svg>
                )}
              </button>
            </div>
          </form>
        </div>
      )}
    </div>
  );
};

export default FloatingChatbot;