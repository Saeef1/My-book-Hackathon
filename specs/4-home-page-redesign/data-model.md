# Data Model: Homepage Redesign

## Module Card Entity

**Name**: ModuleCard
- **Fields**:
  - id (string/number): Unique identifier for the module
  - number (number): Module/chapter number
  - title (string): Module/chapter title
  - description (string): Short description of the module content
  - path (string): URL path to the specific chapter/module page
  - imageUrl (string, optional): Image URL for the module (if applicable)

**Relationships**:
- ModuleCard belongs to the Homepage component
- ModuleCard collection forms the ModulesGrid component

**Validation Rules**:
- title: Required, maximum 100 characters
- description: Required, maximum 200 characters
- number: Required, positive integer
- path: Required, valid URL format

## Homepage Content Entity

**Name**: HomepageContent
- **Fields**:
  - title (string): Main book title displayed in hero section
  - subtitle (string): Tagline/subtitle displayed under the main title
  - readMorePath (string): URL path for the "Read More" button
  - modules (array of ModuleCard): Collection of module cards to display

**Validation Rules**:
- title: Required, maximum 150 characters
- subtitle: Required, maximum 100 characters
  - readMorePath: Required, valid URL format
- modules: Required, minimum 1 module, maximum 50 modules

## State Transitions

**Module Card Hover State**:
- Normal → Hover: On mouse enter, apply scale and shadow effects
- Hover → Normal: On mouse leave, revert to original state

**Responsive State Changes**:
- Desktop → Tablet/Mobile: Grid layout adapts to fewer columns
- Tablet → Mobile: Layout adjusts for narrow screens