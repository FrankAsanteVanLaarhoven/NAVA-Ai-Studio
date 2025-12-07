# 📚 Documentation Sites Integration

## 🌟 Overview

NAVΛ Studio IDE now includes **two complete documentation systems**:

1. **Docusaurus Site** (`docs-site/`) - Modern documentation with versioning
2. **MkDocs Suite** (`docs-mkdocs/`) - Alternative documentation system

---

## 📖 Docusaurus Site

**Location**: `docs-site/`  
**Framework**: Docusaurus (React-based)  
**Port**: 3000

### What's Included

**NAVΛ Language Documentation**:
- `NAVLA-LANGUAGE/SPEC.md` - Complete language specification
- `NAVLA-LANGUAGE/CODEGEN.md` - Code generation guide
- `NAVLA-LANGUAGE/DSL-TO-CONSTRAINTS.md` - DSL to constraints mapping

**NAVΛ Manual**:
- `NAVLA-MANUAL/INTRO.md` - Introduction
- `NAVLA-MANUAL/RUNTIME.md` - Runtime system
- `NAVLA-MANUAL/SDK.md` - SDK documentation
- `NAVLA-MANUAL/SYMBOLS-ACRONYMS.md` - Symbol reference
- `NAVLA-MANUAL/TUTORIALS/` - Step-by-step tutorials

**NAVΛ Mathematics**:
- `NAVLA-MATHS/FOUNDATIONS.md` - Mathematical foundations
- `NAVLA-MATHS/IK-JACOBIAN-DLS.md` - IK and Jacobian methods
- `NAVLA-MATHS/INTENT-MARKOV.md` - Intent and Markov processes
- `NAVLA-MATHS/SAFETY-THEOREMS.md` - Safety theorem proofs
- `NAVLA-MATHS/SPATIAL4D-CALCULUS.md` - 4D spatial calculus
- `NAVLA-MATHS/TIMING-CONTRACTS.md` - Timing contracts

**Code Templates**:
- `TEMPLATES/CODEGEN-PREFIXES/` - Rust, C++, Python, TypeScript templates
- `TEMPLATES/EXPERIMENT-TEMPLATE.md` - Experiment template
- `TEMPLATES/THEOREM-TEMPLATE.md` - Theorem proof template

**Validation & Verification**:
- `VALIDATION/BENCHMARKS.md` - Performance benchmarks
- `VALIDATION/CLAIMS-TRACEABILITY.md` - Claims traceability matrix
- `VALIDATION/EVIDENCE-PACKS.md` - Evidence pack generation
- `VALIDATION/VV-PLAN.md` - V&V plan

**Appendix**:
- `APPENDIX/LICENSES.md` - License information
- `APPENDIX/REFERENCES.md` - Academic references
- `APPENDIX/THEOREMS-REFERENCE.md` - Theorem index

### Features

- ✅ **Versioning** - Pre-seeded with v1.0.0
- ✅ **Local Search** - No Algolia keys needed
- ✅ **Auto-generated Sidebar** - From docs structure
- ✅ **Dark Mode** - Professional theming
- ✅ **GitHub Pages Ready** - CI/CD workflow included
- ✅ **Netlify/Vercel** - One-click deployment configs

---

## 🚀 Quick Start

### Start Documentation Site

```bash
cd ~/Desktop/"NAVA Studio IDE"
./start-docs.sh
```

**Opens at**: http://localhost:3000

### Manual Start

```bash
cd docs-site
npm install
npm run start
```

---

## 🛠️ Available Commands

### Development

```bash
cd docs-site

# Start dev server (hot reload)
npm run start

# Build static site
npm run build

# Serve production build
npm run serve

# Deploy to GitHub Pages
npm run deploy
```

### Versioning

```bash
cd docs-site

# Create new version
npm run version:make

# This creates:
# - versioned_docs/version-X.X.X/
# - versioned_sidebars/version-X.X.X-sidebars.json
```

---

## 📂 Directory Structure

```
docs-site/
├── docs/                          # Current documentation
│   ├── NAVLA-LANGUAGE/           # Language spec & codegen
│   ├── NAVLA-MANUAL/             # User manual & tutorials
│   ├── NAVLA-MATHS/              # Mathematical foundations
│   ├── TEMPLATES/                # Code templates
│   ├── VALIDATION/               # V&V documentation
│   └── APPENDIX/                 # References & theorems
│
├── versioned_docs/               # Version snapshots
│   └── version-1.0.0/           # v1.0.0 documentation
│
├── src/
│   ├── pages/index.md           # Landing page
│   └── css/custom.css           # Custom styling
│
├── static/                       # Static assets
│   └── img/                     # Images & logos
│
├── .github/workflows/           # CI/CD
│   └── deploy-gh-pages.yml     # Auto-deploy to GitHub Pages
│
├── docusaurus.config.js         # Main configuration
├── sidebars.js                  # Sidebar structure
├── netlify.toml                 # Netlify config
├── vercel.json                  # Vercel config
└── package.json                 # Dependencies & scripts
```

---

## 🌐 Deployment Options

### GitHub Pages (Automated)

**Already configured!** Just push to GitHub:

```bash
git push origin main
```

The GitHub Action (`.github/workflows/deploy-gh-pages.yml`) will:
1. Build the site
2. Deploy to GitHub Pages
3. Available at: `https://<username>.github.io/NAVA-Ai-Studio/`

### Netlify

```bash
# 1. Connect GitHub repo to Netlify
# 2. Set build command: npm run build
# 3. Set publish directory: build
# 4. Deploy!
```

Or use `netlify.toml` for automatic configuration.

### Vercel

```bash
# 1. Import GitHub repo in Vercel
# 2. Framework: Docusaurus
# 3. Build command: npm run build
# 4. Output: build
# 5. Deploy!
```

Or use `vercel.json` for automatic configuration.

---

## 📚 Documentation Categories

### For End Users
- **Overview** - Introduction to NAVΛ
- **Manual** - How to use NAVΛ Studio IDE
- **Tutorials** - Step-by-step guides
- **SDK** - SDK installation and usage

### For Developers
- **Language Spec** - Complete language reference
- **Codegen** - Code generation system
- **Templates** - Code templates for all languages
- **Runtime** - Runtime system architecture

### For Researchers
- **Mathematics** - Mathematical foundations
- **Foundations** - Theoretical basis
- **Safety Theorems** - Formal proofs
- **Spatial Calculus** - 4D navigation math

### For Validation
- **Benchmarks** - Performance data
- **Evidence Packs** - Verification artifacts
- **V&V Plan** - Validation & verification plan
- **Traceability** - Claims to evidence mapping

---

## 🔧 Customization

### Update Site Configuration

Edit `docs-site/docusaurus.config.js`:

```javascript
module.exports = {
  title: 'NAVΛ Studio',
  tagline: 'Van Laarhoven Navigation Calculus',
  url: 'https://your-domain.com',
  baseUrl: '/',
  organizationName: 'FrankAsanteVanLaarhoven',
  projectName: 'NAVA-Ai-Studio',
  // ... more config
};
```

### Customize Sidebar

Edit `docs-site/sidebars.js`:

```javascript
module.exports = {
  docs: [
    'OVERVIEW',
    {
      type: 'category',
      label: 'NAVΛ Language',
      items: ['NAVLA-LANGUAGE/SPEC', 'NAVLA-LANGUAGE/CODEGEN'],
    },
    // ... your structure
  ],
};
```

### Add New Documentation

```bash
cd docs-site/docs

# Add new file
echo "# My New Page" > MY-CATEGORY/my-page.md

# Update sidebar in sidebars.js
# Restart dev server to see changes
```

---

## 🎨 MkDocs Suite

**Location**: `docs-mkdocs/`  
**Framework**: MkDocs (Python-based)

### Same Content, Different Format

Contains the same documentation as Docusaurus but configured for MkDocs.

### Run MkDocs

```bash
cd docs-mkdocs

# Install MkDocs (if not installed)
pip install mkdocs mkdocs-material

# Start dev server
mkdocs serve

# Build static site
mkdocs build
```

**Opens at**: http://localhost:8000

---

## 🔍 Integration with IDE

### Access Documentation from IDE

**Option 1: External Link**
```typescript
// In OSDesktop.tsx or menu items
{
  label: 'Documentation',
  route: 'http://localhost:3000',
  action: () => window.open('http://localhost:3000', '_blank')
}
```

**Option 2: Embedded iframe**
```html
<!-- In workspace.html or app component -->
<iframe src="http://localhost:3000" 
        style="width:100%; height:100%; border:0">
</iframe>
```

**Option 3: Proxy through Vite**

Add to `vite.config.ts`:
```typescript
proxy: {
  '/docs': {
    target: 'http://localhost:3000',
    changeOrigin: true,
    rewrite: (p) => p.replace(/^\/docs/, ''),
  }
}
```

---

## 📋 Quick Commands

```bash
# Start documentation site
./start-docs.sh

# Or manually
cd docs-site
npm install
npm run start

# Build for production
npm run build

# Create new version
npm run version:make

# Deploy to GitHub Pages
npm run deploy
```

---

## 🌟 Features

### Docusaurus Features
- ✅ **React-based** - Modern, fast, responsive
- ✅ **Versioning** - Multiple doc versions
- ✅ **Search** - Built-in local search
- ✅ **Dark Mode** - Automatic theme switching
- ✅ **MDX Support** - Interactive components
- ✅ **i18n Ready** - Internationalization support
- ✅ **SEO Optimized** - Meta tags, sitemap
- ✅ **Fast** - Static site generation

### Documentation Content
- ✅ **Complete Language Spec** - Full NAVΛ reference
- ✅ **Mathematics** - Theoretical foundations
- ✅ **Tutorials** - Step-by-step learning
- ✅ **Code Templates** - All supported languages
- ✅ **Validation** - V&V documentation
- ✅ **Theorems** - Formal proofs

---

## 🎯 Use Cases

### For Users
- **Learn NAVΛ** - Tutorials and manual
- **Reference** - Language spec and symbols
- **SDK Guide** - Installation and setup

### For Developers
- **API Docs** - Complete API reference
- **Codegen** - Code generation guide
- **Templates** - Starting code for all languages

### For Researchers
- **Mathematics** - Theoretical foundations
- **Theorems** - Formal safety proofs
- **Validation** - Evidence and benchmarks

---

## ✅ Integration Checklist

- [x] ✅ Docusaurus site copied to `docs-site/`
- [x] ✅ MkDocs suite copied to `docs-mkdocs/`
- [x] ✅ Start script created (`start-docs.sh`)
- [x] ✅ Documentation integration guide created
- [x] ✅ Versioning pre-configured (v1.0.0)
- [x] ✅ GitHub Pages workflow included
- [x] ✅ Netlify/Vercel configs included

---

## 🚀 Next Steps

1. **Start docs site**: `./start-docs.sh`
2. **Customize content**: Edit files in `docs-site/docs/`
3. **Deploy**: Push to GitHub for auto-deployment
4. **Version**: Run `npm run version:make` when ready

---

**📚 Complete documentation system integrated! Start with `./start-docs.sh`** 🚀

