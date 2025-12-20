# Quickstart: Homepage Redesign

## Development Setup

1. **Prerequisites**:
   - Node.js 18+ installed
   - npm or yarn package manager
   - Docusaurus CLI (if not already installed globally)

2. **Environment Setup**:
   ```bash
   cd python-data-science-book
   npm install
   ```

3. **Local Development**:
   ```bash
   npm start
   ```
   This command starts a local development server and opens the site in your browser. Most changes are reflected live without restarting the server.

## Implementation Overview

The homepage redesign consists of:

1. **Hero Section**: Centered book title, subtitle, and "Read More" button
2. **Modules Grid**: Responsive grid of module cards with hover effects
3. **Navigation**: Cards and "Read More" button link to specific chapters

## Key Files

- `src/pages/index.js`: Main homepage component
- `src/components/Homepage/HeroSection.js`: Hero section with title and subtitle
- `src/components/Homepage/ModulesGrid.js`: Grid of module cards
- `src/components/ModuleCard.js`: Individual module card component
- `src/css/homepage.css`: Custom styles for homepage components

## Customization

To customize the module cards data, modify the modules array in the homepage component with:
- Module number
- Title
- Description
- Path to the specific chapter

## Responsive Behavior

- Desktop: 3-4 columns grid layout
- Tablet: 2 columns grid layout
- Mobile: Single column layout

## Testing

To run tests:
```bash
npm test
```

To build for production:
```bash
npm run build
```

To serve the built site locally:
```bash
npm run serve
```