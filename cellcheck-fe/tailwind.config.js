/** @type {import('tailwindcss').Config} */
export default {
  content: [
    "./index.html",
    "./src/**/*.{js,ts,jsx,tsx}"
  ],
  theme: {
    extend: {
      colors: {
        'admin-red': '#ef4444',
        'admin-purple': '#9333ea',
        'user-blue': '#3b82f6',
        'user-indigo': '#6366f1'
      },
      backgroundImage: {
        'gradient-radial': 'radial-gradient(var(--tw-gradient-stops))',
        'gradient-to-r-user': 'linear-gradient(to right, var(--tw-gradient-stops))',
        'gradient-to-r-admin': 'linear-gradient(to right, var(--tw-gradient-stops))'
      }
    },
  },
  plugins: [],
}
