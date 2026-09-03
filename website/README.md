# SCV2 website

The public SCV2 website is built with Astro and Starlight. The custom homepage lives in `src/pages` and technical documentation lives in `src/content/docs` as Markdown.

## Local development

Use Node.js 24 or newer from this directory:

```powershell
npm install
npm run dev
```

Astro serves the project at `http://localhost:4321/scv2/` because the production site is a GitHub project page.

## Verification

```powershell
npm run build
```

The build performs Astro type and content checks, generates the static site in `dist`, and builds Starlight's search index.

## Publishing

The `pages.yml` workflow builds this directory and deploys its output. In the repository's **Settings → Pages → Build and deployment**, select **GitHub Actions** as the source.

The pre-migration website copy is preserved in [`../knowledge-base/legacy-website.md`](../knowledge-base/legacy-website.md).
