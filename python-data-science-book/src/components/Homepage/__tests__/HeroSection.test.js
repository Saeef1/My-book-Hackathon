import React from 'react';
import { render, screen } from '@testing-library/react';
import '@testing-library/jest-dom';
import HeroSection from '../HeroSection';

describe('HeroSection', () => {
  const defaultProps = {
    title: 'Test Book Title',
    subtitle: 'Test subtitle for the book',
    readMorePath: '/read-more'
  };

  test('renders book title and subtitle', () => {
    render(<HeroSection {...defaultProps} />);

    expect(screen.getByText(/Test Book Title/i)).toBeInTheDocument();
    expect(screen.getByText(/Test subtitle for the book/i)).toBeInTheDocument();
  });

  test('renders Read More button with correct link', () => {
    render(<HeroSection {...defaultProps} />);

    const buttonElement = screen.getByRole('link', { name: /Read More/i });
    expect(buttonElement).toBeInTheDocument();
    expect(buttonElement).toHaveAttribute('href', '/read-more');
  });

  test('uses default values when props are not provided', () => {
    render(<HeroSection />);

    expect(screen.getByText(/Python Data Science Book/i)).toBeInTheDocument();
    expect(screen.getByText(/A comprehensive guide to using Python for data science/i)).toBeInTheDocument();
    expect(screen.getByRole('link', { name: /Read More/i })).toBeInTheDocument();
  });
});