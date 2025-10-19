# Workspace Widgets Integration Guide

## How to Add Compact Weather and World Clock to workspace.html

### Step 1: Add CSS Link
Add this line in the `<head>` section of workspace.html:
```html
<link rel="stylesheet" href="/workspace-widgets.css">
```

### Step 2: Update Language Button
Find this line in workspace.html (around line 889):
```html
<div class="info-icon" onclick="showLanguageModal()" title="Language" style="cursor: pointer;">🇬🇧 EN</div>
```

Replace it with:
```html
<div class="info-icon" id="languageBtn" onclick="showLanguageModal()" title="Language" style="cursor: pointer;">🇬🇧 EN</div>
```

### Step 3: Add Widget HTML
Before the closing `</body>` tag, add this:
```html
<!-- Compact Widgets - Top Right Corner -->
<div class="top-right-widgets">
    <!-- Weather Widget -->
    <div class="compact-widget">
        <div class="widget-header">
            <div class="widget-title">🌤️ Weather</div>
        </div>
        <div class="weather-content">
            <div class="weather-icon-compact" id="weatherIconCompact">⛅</div>
            <div class="weather-info">
                <div class="weather-temp-compact" id="weatherTempCompact">--°C</div>
                <div class="weather-condition-compact" id="weatherConditionCompact">Loading...</div>
                <div class="weather-details-compact">
                    <span>💧 <span id="weatherHumidityCompact">--%</span></span>
                    <span>💨 <span id="weatherWindCompact">-- km/h</span></span>
                </div>
            </div>
        </div>
    </div>
    
    <!-- World Clock Widget -->
    <div class="compact-widget">
        <div class="widget-header">
            <div class="widget-title">⏰ Local Time</div>
        </div>
        <div class="world-clock-content">
            <div class="clock-time" id="clockTime">--:--:--</div>
            <div class="clock-location" id="clockLocation">Detecting...</div>
            <div class="clock-timezone" id="clockTimezone">GMT+00:00</div>
        </div>
    </div>
</div>

<!-- Language Selector Modal -->
<div class="language-modal" id="languageModal" onclick="if(event.target === this) closeLanguageModal()">
    <div class="language-panel">
        <div class="language-panel-header">
            <div class="language-panel-title">🌍 Select Language</div>
            <button class="language-panel-close" onclick="closeLanguageModal()">✕</button>
        </div>
        
        <div class="language-list">
            <div class="language-item active" onclick="selectLanguage('en')">
                <div class="language-flag">🇬🇧</div>
                <div class="language-name">English</div>
                <div class="language-check">✓</div>
            </div>
            <div class="language-item" onclick="selectLanguage('es')">
                <div class="language-flag">🇪🇸</div>
                <div class="language-name">Español</div>
                <div class="language-check">✓</div>
            </div>
            <div class="language-item" onclick="selectLanguage('fr')">
                <div class="language-flag">🇫🇷</div>
                <div class="language-name">Français</div>
                <div class="language-check">✓</div>
            </div>
            <div class="language-item" onclick="selectLanguage('de')">
                <div class="language-flag">🇩🇪</div>
                <div class="language-name">Deutsch</div>
                <div class="language-check">✓</div>
            </div>
            <div class="language-item" onclick="selectLanguage('zh')">
                <div class="language-flag">🇨🇳</div>
                <div class="language-name">中文 (Chinese)</div>
                <div class="language-check">✓</div>
            </div>
            <div class="language-item" onclick="selectLanguage('ja')">
                <div class="language-flag">🇯🇵</div>
                <div class="language-name">日本語 (Japanese)</div>
                <div class="language-check">✓</div>
            </div>
            <div class="language-item" onclick="selectLanguage('ar')">
                <div class="language-flag">🇸🇦</div>
                <div class="language-name">العربية (Arabic)</div>
                <div class="language-check">✓</div>
            </div>
            <div class="language-item" onclick="selectLanguage('pt')">
                <div class="language-flag">🇵🇹</div>
                <div class="language-name">Português</div>
                <div class="language-check">✓</div>
            </div>
            <div class="language-item" onclick="selectLanguage('ru')">
                <div class="language-flag">🇷🇺</div>
                <div class="language-name">Русский (Russian)</div>
                <div class="language-check">✓</div>
            </div>
        </div>
    </div>
</div>
```

### Step 4: Add JavaScript
Before the closing `</body>` tag, add this:
```html
<script src="/workspace-widgets.js"></script>
```

### Step 5: Restart the Server
Restart the dev server to see the changes:
```bash
npm run dev
```

### Features Added:
1. **Compact Weather Widget** - Shows current weather with temperature, conditions, humidity, and wind speed
2. **World Clock Widget** - Shows precise time with seconds, location, and timezone
3. **Language Selector Modal** - Beautiful modal with 9 language options
4. **Auto-updating** - Time updates every second, weather updates every 30 minutes
5. **Responsive Design** - Works on all screen sizes
6. **Persistent Language** - Remembers selected language in localStorage

### Customization:
- Edit `public/workspace-widgets.css` to change colors, sizes, positioning
- Edit `public/workspace-widgets.js` to connect to real weather APIs
- Modify the HTML to add/remove languages or change layout