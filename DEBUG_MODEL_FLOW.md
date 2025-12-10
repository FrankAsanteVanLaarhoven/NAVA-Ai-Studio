# 🔍 Debugging Model Flow - Why Generic Responses?

## Issue
User selects "Gemini 2.5 Pro" but gets generic/predefined responses instead of real AI.

## Investigation Needed

1. Check if backend is being called correctly
2. Check if API keys are being passed
3. Check if there's a fallback to generic responses
4. Check browser console for errors

## Expected Flow
1. User selects model → `selectedModel` state
2. `handleSend()` called
3. `getBackendIdFromModel(selectedModel, provider)` → backend ID
4. `backendRegistry.getBackend(backendId)` → backend instance
5. `backendRegistry.chat()` → real API call
6. Response displayed

## Potential Issues
- Backend not found → falls back to default
- Default backend not available → falls back to generic
- API key not passed → request fails → generic response
- Error handling shows generic message instead of real error
