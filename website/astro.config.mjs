import { defineConfig } from 'astro/config';
import starlight from '@astrojs/starlight';

export default defineConfig({
  site: 'https://weiliangng.github.io',
  base: '/scv2',
  integrations: [
    starlight({
      title: 'SCV2',
      description: 'SCV2 supercapacitor controller documentation',
      customCss: ['./src/styles/docs.css'],
      favicon: '/favicon.svg',
      lastUpdated: true,
      editLink: {
        baseUrl: 'https://github.com/weiliangng/scv2/edit/main/website/',
      },
      social: [
        { icon: 'github', label: 'GitHub', href: 'https://github.com/weiliangng/scv2' },
      ],
      sidebar: [
        {
          label: 'Start here',
          items: [
            { label: 'Introduction', slug: 'docs/introduction' },
            { label: 'Installation', slug: 'docs/installation' },
          ],
        },
        {
          label: 'Hardware',
          items: [
            { label: 'Technical specifications', slug: 'docs/specifications' },
            { label: 'Operating envelope', slug: 'docs/operating-envelope' },
          ],
        },
        {
          label: 'API reference',
          items: [
            { label: 'Interfaces overview', slug: 'api/overview' },
            { label: 'CAN telemetry', slug: 'api/can-telemetry' },
            { label: 'USB CLI', slug: 'api/usb-cli' },
            { label: 'Telemetry transports', slug: 'api/telemetry' },
          ],
        },
        {
          label: 'Operations',
          items: [
            { label: 'Debugging & testing', slug: 'operations/debugging' },
            { label: 'FAQ', slug: 'reference/faq' },
          ],
        },
      ],
    }),
  ],
});
