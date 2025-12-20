import React from 'react';
import { render, screen } from '@testing-library/react';
import '@testing-library/jest-dom';
import ModuleCard from '../ModuleCard';

describe('ModuleCard', () => {
  const mockModule = {
    id: 1,
    number: 1,
    title: 'Test Module',
    description: 'This is a test module description',
    path: '/test-module',
  };

  test('renders module number, title, and description', () => {
    render(<ModuleCard {...mockModule} />);

    expect(screen.getByText(/Module 1/i)).toBeInTheDocument();
    expect(screen.getByText(/Test Module/i)).toBeInTheDocument();
    expect(screen.getByText(/This is a test module description/i)).toBeInTheDocument();
  });

  test('renders as a link with correct path', () => {
    render(<ModuleCard {...mockModule} />);

    const linkElement = screen.getByRole('link', { name: /Test Module/i });
    expect(linkElement).toBeInTheDocument();
    expect(linkElement).toHaveAttribute('href', '/test-module');
  });

  test('renders with image when imageUrl is provided', () => {
    const moduleWithImage = {
      ...mockModule,
      imageUrl: 'https://example.com/image.jpg'
    };

    render(<ModuleCard {...moduleWithImage} />);

    const imageElement = screen.getByRole('img', { name: /Test Module/i });
    expect(imageElement).toBeInTheDocument();
    expect(imageElement).toHaveAttribute('src', 'https://example.com/image.jpg');
  });

  test('does not render image when imageUrl is not provided', () => {
    render(<ModuleCard {...mockModule} />);

    const imageElement = screen.queryByRole('img');
    expect(imageElement).not.toBeInTheDocument();
  });
});