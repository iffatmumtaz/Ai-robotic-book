---
sidebar_position: 12
title: "GitHub Pages Performance Optimization"
description: "Optimization strategies for GitHub Pages deployment to ensure fast loading times and optimal performance"
---

# GitHub Pages Performance Optimization

This document outlines optimization strategies for GitHub Pages deployment to ensure fast loading times, optimal performance, and excellent user experience for the Physical AI & Humanoid Robotics Book.

## Performance Optimization Strategy

### 1. Core Web Vitals Targets

Our optimization efforts focus on achieving excellent Core Web Vitals scores:

- **Largest Contentful Paint (LCP)**: < 2.5 seconds (Target: < 1.5 seconds)
- **First Input Delay (FID)**: < 100ms (Target: < 50ms)
- **Cumulative Layout Shift (CLS)**: < 0.1 (Target: < 0.05)

### 2. Performance Budget

We establish a performance budget to maintain consistent loading speeds:

- **Total Bundle Size**: < 1.5 MB (compressed)
- **JavaScript Bundle**: < 300 KB (compressed)
- **CSS Bundle**: < 100 KB (compressed)
- **Images**: < 500 KB per page average
- **Load Time**: < 3 seconds on 3G connections

## Image Optimization

### 1. Modern Image Formats

#### WebP and AVIF Support
```javascript
// docusaurus.config.js
module.exports = {
  presets: [
    [
      '@docusaurus/preset-classic',
      {
        docs: {
          // Enable modern image formats
          showLastUpdateAuthor: true,
          showLastUpdateTime: true,
        },
      },
    ],
  ],
  themes: [
    [
      '@docusaurus/theme-classic',
      {
        customCss: require.resolve('./src/css/custom.css'),
      },
    ],
    '@docusaurus/theme-search-algolia', // For search optimization
  ],
  plugins: [
    // Image optimization plugin
    [
      '@docusaurus/plugin-client-redirects',
      {
        createRedirects(existingPath) {
          // Create SEO-friendly redirects
          if (existingPath.includes('/old-section/')) {
            return [existingPath.replace('/old-section/', '/new-section/')];
          }
          return undefined;
        },
      },
    ],
  ],
};
```

#### Image Compression and Lazy Loading
```css
/* src/css/custom.css */
/* Optimize image loading */
article img {
  max-width: 100%;
  height: auto;
  border-radius: 8px;
  box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
  transition: box-shadow 0.3s ease;
}

/* Lazy loading for images */
article img[loading="lazy"] {
  opacity: 0;
  transition: opacity 0.3s ease;
}

article img[loading="lazy"][data-loaded="true"] {
  opacity: 1;
}

/* Optimize for different screen sizes */
@media (max-width: 768px) {
  article img {
    max-width: calc(100% + 2rem);
    margin-left: -1rem;
    margin-right: -1rem;
  }
}

/* Code block images optimization */
.code-block-image {
  max-width: 100%;
  height: auto;
  margin: 1rem 0;
  border-radius: 4px;
}
```

### 2. Image Processing Pipeline

#### Automated Image Optimization
```bash
#!/bin/bash
# optimize-images.sh
# Script to optimize all images in documentation

echo "Starting image optimization..."

# Create optimized images directory
mkdir -p ./static/img/optimized

# Process all images
find ./docs -name "*.png" -o -name "*.jpg" -o -name "*.jpeg" | while read image; do
  filename=$(basename "$image")
  dir=$(dirname "$image")
  optimized_path="./static/img/optimized/${filename%.*}_optimized.${filename##*.}"

  echo "Optimizing: $image"

  # Optimize based on image type
  if [[ "$image" == *.png ]]; then
    # Optimize PNG with pngquant
    pngquant --quality=65-80 --speed=1 --strip --force --output "$optimized_path" "$image"
    # Further optimize with optipng
    optipng -o7 "$optimized_path"
  else
    # Optimize JPEG with mozjpeg
    cjpeg -quality 80 -optimize -progressive "$image" > "$optimized_path.tmp"
    # Optimize with jpegtran
    jpegtran -copy none -optimize -perfect "$optimized_path.tmp" > "$optimized_path"
    rm "$optimized_path.tmp"
  fi

  # Create WebP version
  cwebp -q 80 -m 6 "$optimized_path" -o "${optimized_path%.*}.webp"

  echo "Optimized: $optimized_path"
done

echo "Image optimization completed!"
```

#### Responsive Image Implementation
```jsx
// src/components/ResponsiveImage.jsx
import React from 'react';
import useBaseUrl from '@docusaurus/useBaseUrl';

const ResponsiveImage = ({ src, alt, caption, className = '' }) => {
  return (
    <figure className={`responsive-image ${className}`}>
      <picture>
        <source
          srcSet={`${useBaseUrl(src.replace(/\.(png|jpe?g)$/, '.webp'))}`}
          type="image/webp"
        />
        <source
          srcSet={`${useBaseUrl(src.replace(/\.(png|jpe?g)$/, '.avif'))}`}
          type="image/avif"
        />
        <img
          src={useBaseUrl(src)}
          alt={alt}
          loading="lazy"
          decoding="async"
        />
      </picture>
      {caption && <figcaption>{caption}</figcaption>}
    </figure>
  );
};

export default ResponsiveImage;
```

## Code Splitting and Bundle Optimization

### 1. Docusaurus Configuration

#### Code Splitting Settings
```javascript
// docusaurus.config.js
module.exports = {
  // ... other config
  themes: [
    [
      '@docusaurus/theme-classic',
      {
        customCss: require.resolve('./src/css/custom.css'),
      },
    ],
  ],
  plugins: [
    // Enable code splitting
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'book',
        path: 'docs',
        routeBasePath: '/',
        sidebarPath: require.resolve('./sidebars.js'),
        editUrl: 'https://github.com/your-org/physical-ai-book/edit/main/',

        // Performance optimizations
        showLastUpdateTime: true,
        showLastUpdateAuthor: true,

        // Code splitting options
        remarkPlugins: [
          [require('@docusaurus/remark-plugin-npm2yarn'), {sync: true}],
        ],
      },
    ],
  ],

  // Performance optimizations
  webpack: {
    jsLoader: (isServer) => ({
      loader: require.resolve('swc-loader'),
      options: {
        jsc: {
          parser: {
            syntax: 'typescript',
            tsx: true,
          },
          transform: {
            react: {
              runtime: 'automatic',
            },
          },
        },
        module: {
          type: isServer ? 'commonjs' : 'es6',
        },
      },
    }),
  },

  // Preload optimization
  scripts: [
    {
      src: 'https://cdnjs.cloudflare.com/ajax/libs/prism/1.24.1/components/prism-python.min.js',
      async: true,
    },
  ],
};
```

### 2. Chunk Splitting Strategy

#### Optimized Chunk Configuration
```javascript
// webpack.config.js (for custom webpack config)
module.exports = {
  optimization: {
    splitChunks: {
      chunks: 'all',
      cacheGroups: {
        // Separate vendor libraries
        vendor: {
          test: /[\\/]node_modules[\\/]/,
          name: 'vendors',
          chunks: 'all',
          priority: 10,
          maxSize: 244000, // 244KB
        },
        // Separate common code
        common: {
          name: 'common',
          minChunks: 2,
          chunks: 'all',
          priority: 5,
          enforce: true,
          maxSize: 244000,
        },
        // Separate documentation-specific code
        docs: {
          test: /[\\/]docs[\\/]/,
          name: 'docs',
          chunks: 'all',
          priority: 7,
        },
      },
    },
    // Enable runtime chunk for better caching
    runtimeChunk: {
      name: 'runtime',
    },
  },
};
```

## Caching and CDN Optimization

### 1. Service Worker Configuration

#### Progressive Web App Features
```javascript
// sw.js (Service Worker)
const CACHE_NAME = 'physical-ai-book-v1';
const urlsToCache = [
  '/',
  '/offline.html',
  '/css/main.css',
  '/js/main.js',
];

self.addEventListener('install', event => {
  event.waitUntil(
    caches.open(CACHE_NAME)
      .then(cache => cache.addAll(urlsToCache))
  );
});

self.addEventListener('fetch', event => {
  event.respondWith(
    caches.match(event.request)
      .then(response => {
        // Return cached version or fetch from network
        return response || fetch(event.request);
      })
  );
});

// Cache first strategy for static assets
self.addEventListener('fetch', event => {
  if (event.request.destination === 'script' || event.request.destination === 'style') {
    event.respondWith(
      caches.match(event.request)
        .then(response => response || fetch(event.request))
    );
  }
});
```

### 2. Asset Optimization

#### CSS Optimization
```css
/* src/css/performance.css */
/* Critical CSS - Inline in HTML head */
.critical-layout {
  display: grid;
  grid-template-columns: minmax(0, 1fr) 300px;
  gap: 2rem;
  max-width: 1400px;
  margin: 0 auto;
  padding: 0 1rem;
}

/* Non-critical CSS - Load asynchronously */
@media print {
  .non-critical-styles {
    /* Styles for print */
  }
}

/* Optimize fonts loading */
@font-face {
  font-family: 'Inter';
  font-display: swap; /* Optimize font loading */
  src: url('/fonts/inter-var.woff2') format('woff2-variations');
  font-weight: 100 900;
  font-style: normal;
}

/* Font optimization */
body {
  font-family: 'Inter', system-ui, -apple-system, sans-serif;
  font-display: swap;
  line-height: 1.6;
}
```

## Third-Party Script Optimization

### 1. Analytics Optimization

#### Lazy-Loaded Analytics
```javascript
// src/utils/analytics.js
export const initializeAnalytics = () => {
  // Load analytics script only when needed
  if (typeof window !== 'undefined' && window.location.hostname !== 'localhost') {
    // Load Google Analytics asynchronously
    const script = document.createElement('script');
    script.async = true;
    script.src = `https://www.googletagmanager.com/gtag/js?id=G-XXXXXXXXXX`;
    document.head.appendChild(script);

    script.onload = () => {
      window.dataLayer = window.dataLayer || [];
      function gtag(){dataLayer.push(arguments);}
      gtag('js', new Date());
      gtag('config', 'G-XXXXXXXXXX');
    };
  }
};

// Defer analytics loading
if (typeof window !== 'undefined') {
  window.addEventListener('load', initializeAnalytics);
}
```

### 2. Search Optimization

#### Optimized Search Configuration
```javascript
// docusaurus.config.js
module.exports = {
  // ... other config
  themeConfig: {
    algolia: {
      // Optimized Algolia search
      appId: 'YOUR_APP_ID',
      apiKey: 'YOUR_SEARCH_KEY',
      indexName: 'physical-ai-humanoid-robotics',

      // Optimize search performance
      contextualSearch: true,
      searchParameters: {},
      searchPagePath: 'search',

      // Lazy load search
      externalUrlRegex: 'external\\.com|domain\\.com',
    },

    // Optimize navbar loading
    navbar: {
      hideOnScroll: true, // Hide navbar on scroll for more content space
    },
  },
};
```

## Build Optimization

### 1. GitHub Actions Optimization

#### Optimized Build Process
```yaml
# .github/workflows/deploy.yml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]
  workflow_dispatch:

jobs:
  deploy:
    name: Deploy to GitHub Pages
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: 18
          cache: yarn

      - name: Install dependencies
        run: yarn install --frozen-lockfile

      - name: Build website
        run: yarn build
        env:
          # Optimize build performance
          NODE_OPTIONS: --max-old-space-size=4096
          BUILD_ENV: production

      - name: Upload Build Artifact
        uses: actions/upload-pages-artifact@v1
        with:
          path: build

      - name: Deploy to GitHub Pages
        id: deployment
        uses: actions/deploy-pages@v1

      # Performance monitoring
      - name: Performance Audit
        run: |
          npm install -g lighthouse
          lighthouse https://${{ github.repository_owner }}.github.io/${{ github.event.repository.name }}/ --output html --output-path ./lighthouse-report.html
```

### 2. Asset Compression

#### Compression Configuration
```javascript
// post-build.js
const fs = require('fs');
const zlib = require('zlib');
const path = require('path');

// Compress assets after build
const compressAssets = (buildDir) => {
  const files = getAllFiles(buildDir);

  files.forEach(file => {
    if (isCompressible(file)) {
      const content = fs.readFileSync(file);
      const compressed = zlib.gzipSync(content);
      fs.writeFileSync(`${file}.gz`, compressed);

      // Set appropriate headers in _headers file
      updateHeadersFile(buildDir);
    }
  });
};

const isCompressible = (filePath) => {
  const compressibleTypes = ['.html', '.css', '.js', '.json', '.svg', '.xml'];
  return compressibleTypes.some(type => filePath.endsWith(type));
};

const updateHeadersFile = (buildDir) => {
  const headersPath = path.join(buildDir, '_headers');
  let headersContent = '';

  if (fs.existsSync(headersPath)) {
    headersContent = fs.readFileSync(headersPath, 'utf8');
  }

  // Add compression headers
  const compressionHeaders = `
/*
  Content-Encoding: gzip
  Vary: Accept-Encoding
`;

  if (!headersContent.includes('Content-Encoding')) {
    fs.writeFileSync(headersPath, headersContent + compressionHeaders);
  }
};

// Run compression after build
compressAssets('./build');
```

## Performance Monitoring

### 1. Lighthouse Configuration

#### Performance Testing Script
```javascript
// performance-test.js
const lighthouse = require('lighthouse');
const chromeLauncher = require('chrome-launcher');

async function runLighthouse(url) {
  const chrome = await chromeLauncher.launch({chromeFlags: ['--headless']});
  const options = {logLevel: 'info', output: 'html', onlyCategories: ['performance', 'accessibility', 'best-practices', 'seo']};
  const runnerResult = await lighthouse(url, options);

  // `.report` is the HTML report as a string
  const reportHtml = runnerResult.report;
  fs.writeFileSync('lhreport.html', reportHtml);

  // `.lhr` is the Lighthouse Result as a JS object
  console.log('Performance score:', runnerResult.lhr.categories.performance.score);
  console.log('Accessibility score:', runnerResult.lhr.categories.accessibility.score);

  await chrome.kill();
}

// Test local build
runLighthouse('http://localhost:3000');
```

### 2. Performance Budget Enforcement

#### Budget Configuration
```json
// performance-budget.json
{
  "extends": ["lighthouse:default"],
  "settings": {
    "maxArtifactsDownloadSize": 20000000
  },
  "audits": [
    {
      "path": "performance-budget",
      "options": {
        "budget": {
          "resourceSizes": [
            {
              "resourceType": "script",
              "budget": 300
            },
            {
              "resourceType": "stylesheet",
              "budget": 100
            },
            {
              "resourceType": "image",
              "budget": 500
            },
            {
              "resourceType": "total",
              "budget": 1500
            }
          ],
          "resourceCounts": [
            {
              "resourceType": "third-party",
              "budget": 10
            }
          ]
        }
      }
    }
  ]
}
```

## Mobile Performance Optimization

### 1. Responsive Design Optimization

#### Mobile-First Approach
```css
/* Mobile-optimized styles */
.mobile-optimized {
  /* Use minimal styles for mobile */
  font-size: 16px; /* Better for mobile readability */
  line-height: 1.6;
  padding: 1rem;
}

/* Optimize for touch targets */
.touch-target {
  min-height: 48px;
  min-width: 48px;
  padding: 12px;
}

/* Optimize for mobile viewport */
@media (max-width: 768px) {
  .desktop-only {
    display: none;
  }

  .mobile-optimized {
    padding: 0.5rem;
    font-size: 16px;
  }

  /* Optimize navigation for mobile */
  .mobile-nav {
    position: sticky;
    top: 0;
    z-index: 100;
  }
}

/* Optimize for tablets */
@media (min-width: 769px) and (max-width: 1024px) {
  .tablet-optimized {
    font-size: 17px;
    line-height: 1.65;
  }
}
```

### 2. Resource Prioritization

#### Critical Resource Loading
```html
<!-- In static/index.html or appropriate template -->
<!DOCTYPE html>
<html>
<head>
  <!-- Preload critical resources -->
  <link rel="preload" href="/css/main.css" as="style">
  <link rel="preload" href="/js/main.js" as="script">
  <link rel="prefetch" href="/docs/module1/" as="document">

  <!-- Critical CSS inline -->
  <style>
    /* Critical above-the-fold styles */
    body { margin: 0; font-family: system-ui, sans-serif; }
    .header { height: 60px; background: #25c2a0; }
  </style>

  <!-- Non-critical CSS async -->
  <link rel="stylesheet" href="/css/non-critical.css" media="print" onload="this.media='all'">
</head>
<body>
  <!-- Content -->

  <!-- Scripts at end for performance -->
  <script src="/js/main.js"></script>
</body>
</html>
```

## Server-Side Optimizations

### 1. GitHub Pages Headers

#### Optimized Headers Configuration
```
# In build/_headers file

/*
  Cache-Control: max-age=300
  X-Content-Type-Options: nosniff
  X-Frame-Options: DENY
  X-XSS-Protection: 1; mode=block

/static/*
  Cache-Control: max-age=31536000
  Content-Encoding: gzip

/build/*
  Cache-Control: max-age=31536000
  Content-Encoding: gzip

/*.css
  Cache-Control: max-age=31536000
  Content-Encoding: gzip

/*.js
  Cache-Control: max-age=31536000
  Content-Encoding: gzip

/*.png,/*.jpg,/*.jpeg,/*.gif,/*.webp
  Cache-Control: max-age=31536000
  Content-Encoding: gzip
```

### 2. Preconnect and DNS Prefetch

#### Resource Optimization
```html
<!-- In docusaurus.config.js head tags -->
module.exports = {
  // ... other config
  headTags: [
    {
      tagName: 'link',
      attributes: {
        rel: 'preconnect',
        href: 'https://fonts.googleapis.com',
      },
    },
    {
      tagName: 'link',
      attributes: {
        rel: 'preconnect',
        href: 'https://fonts.gstatic.com',
        crossorigin: '',
      },
    },
    {
      tagName: 'link',
      attributes: {
        rel: 'dns-prefetch',
        href: 'https://www.google-analytics.com',
      },
    },
    {
      tagName: 'link',
      attributes: {
        rel: 'preload',
        href: '/fonts/inter-var.woff2',
        as: 'font',
        type: 'font/woff2',
        crossorigin: '',
      },
    },
  ],
};
```

## Performance Testing and Validation

### 1. Automated Performance Testing

#### Performance Test Script
```javascript
// scripts/performance-test.js
const puppeteer = require('puppeteer');
const lighthouse = require('lighthouse');

(async () => {
  // Launch browser
  const browser = await puppeteer.launch();
  const page = await browser.newPage();

  // Enable performance metrics
  await page.tracing.start({path: 'trace.json'});

  // Navigate to page
  await page.goto('http://localhost:3000', {waitUntil: 'networkidle2'});

  // Stop tracing
  await page.tracing.stop();

  // Measure performance metrics
  const metrics = await page.metrics();
  console.log('Performance Metrics:', metrics);

  // Close browser
  await browser.close();

  // Run Lighthouse audit
  const runnerResult = await lighthouse('http://localhost:3000', {
    onlyCategories: ['performance'],
    output: 'json'
  });

  const perfScore = runnerResult.lhr.categories.performance.score;
  console.log(`Performance Score: ${(perfScore * 100).toFixed(2)}`);

  // Check if performance meets requirements
  if (perfScore < 0.9) { // 90% threshold
    console.error('Performance score below threshold!');
    process.exit(1);
  } else {
    console.log('Performance requirements met!');
  }
})();
```

### 2. Continuous Performance Monitoring

#### Performance Budget Script
```javascript
// scripts/check-performance.js
const fs = require('fs');
const path = require('path');

// Check build size
const buildDir = './build';
const totalSize = getDirectorySize(buildDir);
const maxSizeMB = 10; // 10MB budget

if (totalSize > maxSizeMB * 1024 * 1024) {
  console.error(`Build size (${(totalSize / 1024 / 1024).toFixed(2)}MB) exceeds budget (${maxSizeMB}MB)`);
  process.exit(1);
} else {
  console.log(`Build size OK: ${(totalSize / 1024 / 1024).toFixed(2)}MB`);
}

function getDirectorySize(dirPath) {
  const files = fs.readdirSync(dirPath);
  let totalSize = 0;

  for (const file of files) {
    const filePath = path.join(dirPath, file);
    const stat = fs.statSync(filePath);

    if (stat.isDirectory()) {
      totalSize += getDirectorySize(filePath);
    } else {
      totalSize += stat.size;
    }
  }

  return totalSize;
}
```

This performance optimization guide ensures that the Physical AI & Humanoid Robotics Book loads quickly and provides an excellent user experience across all devices and network conditions, meeting the GitHub Pages compatibility requirements while maintaining high performance standards.