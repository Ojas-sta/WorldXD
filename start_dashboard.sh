#!/bin/bash

echo "================================================================="
echo "  Starting WorldXD Apple-Design Telemetry & Control Dashboard    "
echo "================================================================="

# Kill existing processes on ports 4002 (Backend) and 5173 (Frontend)
lsof -ti:4002 | xargs kill -9 2>/dev/null
lsof -ti:5173 | xargs kill -9 2>/dev/null

echo "1. Starting Backend Telemetry Server (Port 4002)..."
cd /Users/roopalisingh/WorldXD/dashboard/backend
node server.js &
BACKEND_PID=$!

echo "2. Starting Vite React Frontend (Port 5173)..."
cd /Users/roopalisingh/WorldXD/dashboard/frontend
npm run dev -- --host &
FRONTEND_PID=$!

echo ""
echo "✨ Dashboard is running!"
echo "- Web Dashboard UI: http://localhost:5173"
echo "- Telemetry Socket: ws://localhost:4002"
echo ""
echo "Press Ctrl+C to stop both servers."

trap "kill $BACKEND_PID $FRONTEND_PID 2>/dev/null" EXIT
wait $BACKEND_PID $FRONTEND_PID
