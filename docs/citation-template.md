# APA-Style Citation Template for Physical AI & Humanoid Robotics Book

## Purpose
This document provides a template and guidelines for creating APA-style citations throughout the Physical AI & Humanoid Robotics Book, ensuring consistent and proper attribution of sources.

## APA Citation Format Guidelines

### 1. General APA 7th Edition Rules
- Use hanging indent for reference list entries (first line flush left, subsequent lines indented 0.5 inches)
- Alphabetize entries by first author's last name
- Use sentence case for article and book titles (capitalize only the first word, proper nouns, and first word after a colon)
- Use title case for journal and book names
- Include DOI when available; otherwise include URL
- Include retrieval date for resources likely to change

### 2. Common Source Types

#### A. Books
```
Author, A. A. (Year). *Title of work: Capital letter also for subtitle*. Publisher. DOI or URL
```

Example:
```
Quigley, M., Gerkey, B., & Smart, W. (2022). *Programming robots with ROS: A practical introduction to the robot operating system*. O'Reilly Media.
```

#### B. Journal Articles
```
Author, A. A., & Author, B. B. (Year). Title of article. *Title of Periodical*, volume(issue), pages. DOI or URL
```

Example:
```
Schoellig, A. P., & Beard, R. W. (2021). Adaptive control of quadrotor aerial vehicles: A survey. *IEEE Control Systems Magazine*, 41(2), 30-54. https://doi.org/10.1109/MCS.2021.3052434
```

#### C. Conference Papers
```
Author, A. A., & Author, B. B. (Year, Month). Title of paper. In *Proceedings of Conference Name* (pp. page range). Publisher. DOI or URL
```

Example:
```
Koenig, N., & Howard, A. (2004, September). Design and use paradigms for Gazebo, an open-source multi-robot simulator. In *Proceedings of the 2004 IEEE/RSJ International Conference on Intelligent Robots and Systems* (pp. 2149-2154). IEEE. https://doi.org/10.1109/IROS.2004.1389727
```

#### D. Technical Documentation
```
Organization Name. (Year). *Title of documentation*. URL
```

Example:
```
Open Robotics. (2023). *ROS 2 documentation*. https://docs.ros.org/en/humble/
```

#### E. Software/Code Repositories
```
Author, A. A. (Year). *Title of software* [Computer software]. Publisher/Repository. URL
```

Example:
```
Lütkebohle, I. (2023). *NVIDIA Isaac Sim* [Robotics simulation software]. NVIDIA. https://developer.nvidia.com/isaac-sim
```

## Citation Templates for Robotics Sources

### 1. ROS 2 Documentation
```
Open Robotics. (2023). *ROS 2 documentation: [Specific topic]*. https://docs.ros.org/en/humble/[specific-page.html]
```

### 2. Simulation Platforms
```
NVIDIA Corporation. (2023). *Isaac Sim user guide*. https://docs.omniverse.nvidia.com/isaacsim/latest/isaac-sim/index.html

Robotis. (2023). *Gazebo tutorials*. http://gazebosim.org/tutorials
```

### 3. Hardware Documentation
```
Unitree Robotics. (2023). *Go2 robot user manual*. https://www.unitree.com/go2/

Intel Corporation. (2023). *RealSense SDK documentation*. https://dev.intelrealsense.com/docs

NVIDIA Corporation. (2023). *JetPack SDK documentation*. https://developer.nvidia.com/embedded/jetpack
```

## In-Text Citation Format

### 1. Parenthetical Citations
- Single author: (Author, Year)
- Two authors: (Author & Author, Year)
- Three or more authors: (Author et al., Year)
- Direct quote: (Author, Year, p. #)

### 2. Narrative Citations
- Single author: Author (Year) states...
- Two authors: Author and Author (Year) explain...
- Three or more authors: Author et al. (Year) demonstrate...

## Reference List Template

### 1. Standard Format
```markdown
## References

Author, A. A. (Year). *Title of work: Capital letter also for subtitle*. Publisher.

Author, A. A., & Author, B. B. (Year). Title of article. *Title of Periodical*, volume(issue), pages. DOI or URL

Organization Name. (Year). *Title of documentation*. URL
```

### 2. Robotics-Specific Template
```markdown
## References

Open Robotics. (2023). *ROS 2 documentation*. https://docs.ros.org/en/humble/

NVIDIA Corporation. (2023). *Isaac Sim user guide*. https://docs.omniverse.nvidia.com/isaacsim/latest/isaac-sim/index.html

Quigley, M., Gerkey, B., & Smart, W. (2022). *Programming robots with ROS: A practical introduction to the robot operating system*. O'Reilly Media.

Schoellig, A. P., & Beard, R. W. (2021). Adaptive control of quadrotor aerial vehicles: A survey. *IEEE Control Systems Magazine*, 41(2), 30-54. https://doi.org/10.1109/MCS.2021.3052434
```

## Citation Management Best Practices

### 1. Consistency Rules
- Always use the same format for similar sources
- Maintain consistent capitalization style
- Use consistent punctuation throughout
- Keep citation style uniform across all modules

### 2. Accuracy Requirements
- Verify all URLs are active and accessible
- Confirm all DOIs resolve correctly
- Check that all author names are spelled correctly
- Ensure publication dates are accurate

### 3. Accessibility Considerations
- Prioritize sources with stable URLs
- Include retrieval dates for sources likely to change
- Use DOI when available for permanent access
- Verify sources are accessible to target audience

## Implementation in Docusaurus

### 1. Markdown Integration
```markdown
According to recent research, the robot operating system has evolved significantly in recent years (Open Robotics, 2023).

The ROS 2 framework provides improved security and real-time performance compared to its predecessor (Quigley et al., 2022).
```

### 2. Reference List Placement
- Place reference list at the end of each module
- Use consistent heading format: "## References"
- Ensure proper formatting for web display
- Consider using a separate bibliography page for the entire book

## Quality Assurance for Citations

### 1. Verification Checklist
- [ ] All citations follow APA 7th edition format
- [ ] Reference list is alphabetized correctly
- [ ] Hanging indent is properly formatted
- [ ] URLs are valid and accessible
- [ ] DOIs resolve correctly
- [ ] Author names are spelled correctly
- [ ] Publication dates are accurate
- [ ] Titles use correct capitalization
- [ ] All in-text citations have corresponding references
- [ ] All references are cited in the text

### 2. Review Process
1. Automated citation format checking
2. Manual verification of URLs and DOIs
3. Cross-reference accuracy validation
4. Consistency review across modules
5. Accessibility compliance verification

## Common Citation Scenarios in Robotics Content

### 1. Referencing API Documentation
```markdown
The rclpy library provides Python bindings for ROS 2 client library (Open Robotics, 2023a).
```

### 2. Citing Research Papers
```markdown
Recent work in humanoid locomotion control has shown significant improvements using machine learning approaches (Sugihara, 2022).
```

### 3. Referencing Hardware Specifications
```markdown
The NVIDIA Jetson Orin platform provides sufficient computational power for real-time AI inference on humanoid robots (NVIDIA Corporation, 2023).
```

### 4. Citing Code Examples
```markdown
The publisher-subscriber pattern is fundamental to ROS 2 communication (Open Robotics, 2023b).
```

## Troubleshooting Common Citation Issues

### 1. Missing Information
- If no date available, use (n.d.)
- If no author available, start with title
- If no DOI available, provide URL
- If page numbers unavailable, omit

### 2. Special Cases
- Software: Include version number if relevant
- Conference presentations: Include conference name and location
- Preprints: Include repository name and access information
- Patents: Include patent number and issuing office

## Validation Checklist

Before finalizing citations in any content, verify:

- [ ] All citations follow APA 7th edition format
- [ ] Reference list is properly formatted with hanging indents
- [ ] All in-text citations have corresponding entries in the reference list
- [ ] URLs are valid and accessible
- [ ] DOIs resolve correctly
- [ ] Author names are accurate and complete
- [ ] Publication dates are correct
- [ ] Titles use appropriate capitalization
- [ ] Consistency is maintained across all citations
- [ ] Accessibility requirements are met