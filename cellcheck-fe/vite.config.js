import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// https://vite.dev/config/
export default defineConfig({
  plugins: [react()],
  server: {
    historyApiFallback: true,
    proxy: {
      '/api': {
        target: 'https://k12d106.p.ssafy.io',
        changeOrigin: true,
        secure: false,
        rewrite: (path) => path
      }
    },
    cors: false // Vite의 CORS 처리를 비활성화하여 프록시의 CORS 처리를 우선 적용
  },
})
