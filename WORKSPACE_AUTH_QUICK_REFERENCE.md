# 🎯 Workspace Authentication - Quick Reference Card

## 🚀 Quick Start

### Access the Workspace
```
http://localhost:5176/workspace.html
```

### Test Credentials
```
Username: admin
Password: admin123
PIN: 1234
Guest: Click "Continue as Guest"
```

---

## ⏻ Power Button States

| Color | Status | Action |
|-------|--------|--------|
| 🔴 Red | Signed Out | Click to sign in |
| ⚪ White | Standby | Click to resume |
| 🟢 Green | Signed In | Click to sign out |

---

## 🔐 Authentication Methods

### 1. Username/Password
```
1. Click power button (⏻)
2. Enter: admin / admin123
3. Click "Sign In"
4. ✅ Done!
```

### 2. PIN
```
1. Click power button (⏻)
2. Click "🔢 Sign In with PIN"
3. Enter: 1234
4. Click "Sign In"
5. ✅ Done!
```

### 3. Guest Mode
```
1. Click power button (⏻)
2. Click "Continue as Guest"
3. ✅ Done! (Limited access)
```

### 4. Sign Up
```
1. Click power button (⏻)
2. Click "Create Account"
3. Fill in details
4. Click "Create Account"
5. ✅ Done!
```

---

## 🔓 Access Control

### Always Available (No Sign-In)
- ✅ 📚 Documentation
- ✅ 🌐 Browser
- ✅ 🇬🇧 Language Selector
- ✅ ❓ Help
- ✅ ⚙️ Settings

### Requires Sign-In
- 🔒 🦾 ROS Learning Center
- 🔒 ⋋ Full IDE
- 🔒 📥 Downloads
- 🔒 💻 Terminal
- 🔒 🤖 AI Assistant
- 🔒 ⋋ Applications
- 🔒 🗑️ Trash
- 🔒 📁 Desktop Folders

---

## 🎨 Visual Indicators

### Power Button
- **Red** = Not signed in
- **Green** = Signed in
- **White** = Standby mode

### Restricted Features
- **Faded** (30% opacity)
- **Not-allowed cursor**
- **🔒 in tooltip**

### Notifications
- **Green** = Success ✅
- **Red** = Error ❌
- **Yellow** = Warning ⚠️
- **Blue** = Info ℹ️

---

## 🔧 Troubleshooting

### Power button not working?
```javascript
localStorage.clear();
location.reload();
```

### Modal not appearing?
```javascript
document.getElementById('authModal').style.display = 'flex';
```

### Check auth state:
```javascript
console.log(JSON.parse(localStorage.getItem('navlambda_auth')));
```

### Force sign in:
```javascript
authState.isAuthenticated = true;
authState.authStatus = 'signed-in';
saveAuthState();
updateAuthUI();
applyAccessRestrictions();
```

---

## 📱 Features

### ✅ Implemented
- Username/password authentication
- PIN authentication
- Guest mode
- Sign up/account creation
- Sign out with confirmation
- Session persistence
- Access restrictions
- Visual status indicators
- Toast notifications
- Smooth animations

### 🔜 Coming Soon
- Backend API integration
- Email verification
- Password reset
- 2FA support
- OAuth providers
- Role-based access control

---

## 📊 Key Stats

- **Authentication Methods**: 4
- **Restricted Features**: 8 + folders
- **Always Available**: 5 features
- **Status States**: 3
- **Lines of Code**: 671
- **Test Credentials**: 3 sets

---

## 🎯 Testing Checklist

- [ ] Power button shows correct colors
- [ ] Modal appears on click
- [ ] Sign in works (admin/admin123)
- [ ] PIN works (1234)
- [ ] Guest mode works
- [ ] Sign up works
- [ ] Sign out works
- [ ] Access restrictions work
- [ ] Session persists on refresh
- [ ] Notifications appear

---

## 📞 Quick Help

### Documentation
- `WORKSPACE_AUTHENTICATION_GUIDE.md` - Full guide
- `WORKSPACE_AUTH_COMPLETE.md` - Summary
- `WORKSPACE_AUTH_TEST_GUIDE.md` - Testing

### Support
- Check browser console (F12)
- Clear localStorage if issues
- Refresh page
- Try different browser

---

## 💡 Pro Tips

1. **Quick Sign In**: Use PIN (1234) for fastest access
2. **Guest Mode**: Perfect for demos and quick exploration
3. **Session Persistence**: Your session is saved automatically
4. **Keyboard Shortcuts**: Tab through forms, Enter to submit
5. **Multiple Accounts**: Create as many accounts as you need

---

## 🎉 Success Indicators

### You're Signed In When:
- ✅ Power button is GREEN 🟢
- ✅ All dock icons are fully visible
- ✅ Desktop folders are unlocked
- ✅ Notification says "Signed in successfully"
- ✅ Status shows your username

### You're Signed Out When:
- ❌ Power button is RED 🔴
- ❌ Most dock icons are faded
- ❌ Desktop folders show 🔒
- ❌ Status shows "Signed Out"

---

## 🚀 Next Steps

1. **Test It**: Open workspace and try signing in
2. **Explore**: Check all the locked features
3. **Customize**: Modify allowed features in code
4. **Integrate**: Connect to your backend API
5. **Deploy**: Push to production

---

**Quick Access**: http://localhost:5176/workspace.html

**Test Login**: admin / admin123

**Test PIN**: 1234

---

*Built with ❤️ for NAVΛ Studio*
*Version: 1.0.0*
*Last Updated: 2024*

---

## 📋 Print This Card

Print this reference card and keep it handy for quick access to all authentication features!

**Workspace URL**: ___________________________

**Your Username**: ___________________________

**Your PIN**: ___________________________

**Notes**: ___________________________
         ___________________________
         ___________________________
