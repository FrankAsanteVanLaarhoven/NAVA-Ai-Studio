/** @type {import('tailwindcss').Config} */
export default {
  content: [
    "./index.html",
    "./src/**/*.{js,ts,jsx,tsx}",
  ],
  theme: {
    extend: {
      colors: {
        'vnc-primary': '#00ff00',
        'vnc-secondary': '#00ccff',
        'vnc-accent': '#ff0099',
        'vnc-warning': '#ffff00',
        'vnc-bg-dark': '#1a1a1a',
        'vnc-bg-darker': '#0d0d0d',
        'vnc-text': '#ffffff',
      },
      fontFamily: {
        'mono': ['JetBrains Mono', 'Consolas', 'monospace'],
        'math': ['Latin Modern Math', 'STIX Two Math', 'serif'],
      },
    },
  },
  plugins: [],
}

