# Diagram and Visual Asset Template

## Purpose
This document provides templates and guidelines for creating diagrams and visual assets used throughout the Physical AI & Humanoid Robotics Book.

## Diagram Categories and Templates

### 1. System Architecture Diagrams
```
Title: [System Name] Architecture
Elements:
- Components with clear labels
- Data flow arrows with descriptions
- Color coding for different component types
Legend:
- [Color 1]: ROS 2 Components
- [Color 2]: Simulation Components
- [Color 3]: Hardware Components
- [Color 4]: AI/ML Components
```

### 2. Data Flow Diagrams
```
Title: [Process Name] Data Flow
Elements:
- Input sources
- Processing components
- Output destinations
- Data transformations
Flow: Left to right or top to bottom
```

### 3. Component Interaction Diagrams
```
Title: [System] Component Interaction
Elements:
- Component boxes with names
- Communication arrows with protocol/protocol type
- Message types between components
- Timing relationships if relevant
```

### 4. Hardware Setup Diagrams
```
Title: [Hardware Configuration] Setup
Elements:
- Physical components with realistic shapes
- Connection cables with labels
- Interface ports and protocols
- Physical dimensions if important
```

## Visual Asset Standards

### 1. Color Palette
- Primary: #123456 (Physical AI blue)
- Secondary: #234567 (Technology accent)
- Success: #4CAF50 (Green for positive states)
- Warning: #FFC107 (Yellow for warnings)
- Error: #F44336 (Red for errors)
- Neutral: #9E9E9E (Gray for backgrounds)

### 2. Typography
- Headings: Bold, 14-16pt
- Labels: Regular, 12pt
- Captions: Regular, 10pt
- Font: Sans-serif (e.g., Arial, Helvetica)

### 3. Line Styles
- Main components: Solid lines, 2px width
- Data flow: Arrow lines, 1px width
- Connections: Dashed lines, 1px width
- Boundaries: Dotted lines, 1px width

## Diagram Creation Guidelines

### 1. Software Tools
- Preferred: draw.io (free, collaborative)
- Alternative: Lucidchart, Visio, or similar
- Export formats: SVG (preferred), PNG (acceptable)

### 2. Dimensions and Resolution
- SVG: Scalable vector format preferred
- PNG: Minimum 300 DPI for print quality
- Recommended size: 800x600px minimum
- Maximum size: 1200x800px for web display

### 3. File Naming Convention
```
[module]-[topic]-[sequence]-[type].[extension]
Examples:
- module1-ros2-01-architecture.svg
- module2-gazebo-03-components.png
- module4-vla-02-dataflow.svg
```

## Diagram Template Files

### 1. Architecture Diagram Template
```xml
<mxGraphModel dx="1426" dy="769" grid="1" gridSize="10" guides="1" tooltips="1" connect="1" arrows="1" fold="1" page="1" pageScale="1" pageWidth="850" pageHeight="1100" background="#ffffff" math="0" shadow="0">
  <root>
    <mxCell id="0"/>
    <mxCell id="1" parent="0"/>
    <!-- Add your diagram elements here -->
  </root>
</mxGraphModel>
```

### 2. Sequence Flow Template
```
[Start] -> [Action 1] -> [Decision] -> [Action 2] -> [End]
    |           |            |            |
  Input      Process      Condition    Output
```

## Accessibility Standards

### 1. Alt Text Requirements
- Brief description of the diagram's purpose
- Key elements and relationships
- Any important visual information
- Maximum 120 characters for concise understanding

### 2. Color Accessibility
- Ensure sufficient contrast (4.5:1 minimum)
- Don't rely solely on color to convey information
- Provide text labels for color-coded elements
- Test with colorblind simulation tools

### 3. Text Readability
- Minimum font size: 12pt for main text
- Maximum font size: 16pt for headings
- Clear, sans-serif fonts
- Sufficient spacing between elements

## Integration with Docusaurus

### 1. Markdown Integration
```markdown
import figure from './path/to/diagram.svg'

<img src={figure} alt="Description of diagram content" />

<!-- Or directly: -->
![Diagram Description](./path/to/diagram.svg)
```

### 2. Responsive Design
- Diagrams should scale appropriately
- Maintain aspect ratio during scaling
- Provide zoom functionality for complex diagrams
- Consider mobile viewing constraints

## Quality Assurance Checklist

Before finalizing any diagram or visual asset, verify:

- [ ] Uses consistent color palette and styling
- [ ] Includes proper alt text description
- [ ] Follows naming convention
- [ ] Maintains accessibility standards
- [ ] Is clear and readable at various sizes
- [ ] Accurately represents the concept being taught
- [ ] Includes appropriate labels and legends
- [ ] File size is optimized for web delivery
- [ ] Export format is appropriate (SVG preferred)
- [ ] All text is spell-checked and accurate

## Common Diagram Types for Each Module

### Module 1: ROS 2
- ROS 2 architecture overview
- Node-topic communication
- Action-service patterns
- Workspace structure

### Module 2: Gazebo + Unity
- Simulation environment setup
- Digital twin architecture
- Sensor integration
- Physics engine workflow

### Module 3: NVIDIA Isaac
- Isaac Sim architecture
- Perception pipeline
- Robot control systems
- AI integration points

### Module 4: VLA
- LLM-robot interface
- Action sequence flow
- Safety protocol integration
- Voice command processing

### Capstone Project
- Complete system architecture
- Integration points overview
- Data flow across modules
- Deployment architecture