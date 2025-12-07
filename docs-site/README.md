# NAVA Docs — Docusaurus Site

## Local dev
```bash
npm i
npm run start
# open http://localhost:3000
```

## Build
```bash
npm run build
npm run serve
```

## Versioning
```bash
# creates versioned_docs/version-1.0.0 and versioned sidebars
npm run version:make
git add .
git commit -m "docs: cut 1.0.0"
```

## Deploy (GitHub Pages)
- Set `organizationName` and `projectName` in `docusaurus.config.js`.
- Push to `main`; GH Action builds and publishes `build/`.

## Deploy (Netlify/Vercel)
- Use the provided `netlify.toml` or `vercel.json` and point to `npm run build`.
