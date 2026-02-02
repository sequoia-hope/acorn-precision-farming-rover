# Plan: GitHub Pages Documentation Site

## Objective

Serve the architecture documentation via GitHub Pages to provide a browsable, visual representation with rendered Mermaid diagrams.

---

## Option Comparison

| Option | Pros | Cons | Mermaid Support |
|--------|------|------|-----------------|
| **MkDocs + Material** | Python-based (matches codebase), excellent Mermaid plugin, professional look, easy nav | Requires build step | Native plugin |
| Jekyll | GitHub native, no build config needed | Limited Mermaid support, dated themes | Requires JS include |
| Docusaurus | Modern React-based, good for large sites | Overkill for this scope, Node.js dependency | Plugin available |
| VitePress | Fast, Vue-based | Node.js dependency, newer/less stable | Plugin available |
| Plain HTML | No dependencies | Manual work, no navigation | Manual JS include |

**Recommendation: MkDocs with Material theme**

- Aligns with Python codebase
- First-class Mermaid support via `mkdocs-mermaid2-plugin`
- Clean navigation structure
- Single `mkdocs.yml` configuration
- GitHub Actions workflow for automatic deployment

---

## Implementation Plan

### Phase 1: Setup MkDocs Configuration

**Files to create:**

```
docs/
├── mkdocs.yml              # Main configuration
├── docs/                   # MkDocs expects docs in subdirectory
│   ├── index.md            # Landing page
│   └── architecture/       # Move existing docs here
│       ├── 01-system-context.md
│       ├── 02-components.md
│       └── ...
└── requirements-docs.txt   # Documentation dependencies
```

**mkdocs.yml configuration:**

```yaml
site_name: Acorn Rover Documentation
site_description: Architecture documentation for Acorn Precision Farming Rover
repo_url: https://github.com/sequoia-hope/acorn-precision-farming-rover

theme:
  name: material
  palette:
    primary: green
    accent: light-green
  features:
    - navigation.sections
    - navigation.expand
    - toc.integrate

plugins:
  - search
  - mermaid2

markdown_extensions:
  - pymdownx.superfences:
      custom_fences:
        - name: mermaid
          class: mermaid
          format: !!python/name:mermaid2.fence_mermaid

nav:
  - Home: index.md
  - Architecture:
    - Overview: architecture/DOCUMENTATION_PLAN.md
    - System Context: architecture/01-system-context.md
    - Components: architecture/02-components.md
    - Data Flows: architecture/03-data-flows.md
    - Interfaces: architecture/04-interfaces.md
    - State Machines: architecture/05-state-machines.md
    - Configuration: architecture/06-configuration.md
```

**requirements-docs.txt:**

```
mkdocs>=1.5
mkdocs-material>=9.0
mkdocs-mermaid2-plugin>=1.0
```

### Phase 2: Adjust Markdown for MkDocs

Minor adjustments needed:

1. **Mermaid blocks**: Current format uses triple-backtick with `mermaid` - this is compatible
2. **Internal links**: Update any relative links between docs
3. **Standalone diagrams**: The `.mermaid` files in `diagrams/` can be embedded or linked
4. **Create index.md**: Landing page with project overview and quick links

### Phase 3: GitHub Actions Workflow

**Create `.github/workflows/docs.yml`:**

```yaml
name: Deploy Documentation

on:
  push:
    branches: [main]
    paths:
      - 'docs/**'
      - 'mkdocs.yml'
  workflow_dispatch:

permissions:
  contents: read
  pages: write
  id-token: write

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Setup Python
        uses: actions/setup-python@v5
        with:
          python-version: '3.11'

      - name: Install dependencies
        run: pip install -r requirements-docs.txt

      - name: Build documentation
        run: mkdocs build --site-dir _site

      - name: Upload artifact
        uses: actions/upload-pages-artifact@v3

  deploy:
    needs: build
    runs-on: ubuntu-latest
    environment:
      name: github-pages
      url: ${{ steps.deployment.outputs.page_url }}
    steps:
      - name: Deploy to GitHub Pages
        id: deployment
        uses: actions/deploy-pages@v4
```

### Phase 4: Enable GitHub Pages

1. Go to repository Settings → Pages
2. Set Source to "GitHub Actions"
3. The workflow will deploy on push to `docs/` or `mkdocs.yml`

---

## Directory Structure After Implementation

```
acorn-precision-farming-rover/
├── .github/
│   └── workflows/
│       └── docs.yml              # New: Pages deployment
├── docs/
│   ├── index.md                  # New: Landing page
│   └── architecture/
│       ├── DOCUMENTATION_PLAN.md
│       ├── 01-system-context.md
│       ├── 02-components.md
│       ├── 03-data-flows.md
│       ├── 04-interfaces.md
│       ├── 05-state-machines.md
│       ├── 06-configuration.md
│       └── diagrams/             # Existing .mermaid files
├── mkdocs.yml                    # New: Site configuration
└── requirements-docs.txt         # New: Doc dependencies
```

---

## Tasks Summary

| # | Task | Effort |
|---|------|--------|
| 1 | Create `mkdocs.yml` configuration | Small |
| 2 | Create `requirements-docs.txt` | Small |
| 3 | Create `docs/index.md` landing page | Small |
| 4 | Create GitHub Actions workflow | Small |
| 5 | Enable GitHub Pages in repo settings | Manual |
| 6 | Test local build with `mkdocs serve` | Small |
| 7 | Verify Mermaid rendering | Small |

---

## Local Development

```bash
# Install dependencies
pip install -r requirements-docs.txt

# Serve locally with hot reload
mkdocs serve

# Build static site
mkdocs build
```

---

## Expected Result

- **URL**: `https://sequoia-hope.github.io/acorn-precision-farming-rover/`
- **Features**:
  - Searchable documentation
  - Rendered Mermaid diagrams (interactive)
  - Mobile-responsive design
  - Dark/light mode toggle
  - Navigation sidebar
  - Table of contents per page

---

## Alternative: Minimal Approach

If MkDocs feels like too much tooling, a simpler option:

1. Add a single `_config.yml` for Jekyll
2. Include Mermaid JS via CDN in a layout
3. Use GitHub's built-in Jekyll processing

This requires less setup but produces a less polished result.

---

## Decision Needed

Proceed with MkDocs + Material theme implementation?
