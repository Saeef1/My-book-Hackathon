import React from 'react';
import { render, screen } from '@testing-library/react';
import '@testing-library/jest-dom';
import ModulesGrid from '../ModulesGrid';

describe('ModulesGrid', () => {
  const mockModules = [
    {
      id: 1,
      number: 1,
      title: 'Module 1',
      description: 'First module description',
      path: '/module1'
    },
    {
      id: 2,
      number: 2,
      title: 'Module 2',
      description: 'Second module description',
      path: '/module2'
    }
  ];

  test('renders section title', () => {
    render(<ModulesGrid modules={mockModules} />);

    expect(screen.getByText(/Book Modules/i)).toBeInTheDocument();
  });

  test('renders module cards when modules are provided', () => {
    render(<ModulesGrid modules={mockModules} />);

    expect(screen.getByText(/Module 1/i)).toBeInTheDocument();
    expect(screen.getByText(/First module description/i)).toBeInTheDocument();
    expect(screen.getByText(/Module 2/i)).toBeInTheDocument();
    expect(screen.getByText(/Second module description/i)).toBeInTheDocument();
  });

  test('renders no modules message when no modules are provided', () => {
    render(<ModulesGrid modules={[]} />);

    expect(screen.getByText(/No modules available at this time/i)).toBeInTheDocument();
  });

  test('renders no modules message when modules prop is not provided', () => {
    render(<ModulesGrid />);

    expect(screen.getByText(/No modules available at this time/i)).toBeInTheDocument();
  });

  test('renders correct number of module cards', () => {
    render(<ModulesGrid modules={mockModules} />);

    const moduleCards = screen.getAllByRole('link'); // All module cards are links
    expect(moduleCards).toHaveLength(2);
  });
});