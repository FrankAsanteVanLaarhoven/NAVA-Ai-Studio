# 🚀 Starting ROBOTIS Server

## ✅ **Overlay is Working!**

The ROBOTIS overlay is working correctly. The connection error you're seeing means the ROBOTIS server needs to be started.

## 🔧 **How to Start ROBOTIS Server**

### **Option 1: Using Start Script (Recommended)**
```bash
cd /path/to/robotis-project
./start-robotis-system.sh
```

### **Option 2: Manual Start**
```bash
# Navigate to ROBOTIS project directory
cd /path/to/robotis-project

# Start the server (adjust command based on your setup)
npm start
# or
python -m http.server 3000
# or
node server.js
# or
./start.sh
```

### **Option 3: Check if Server is Already Running**
```bash
# Check if port 3000 is in use
lsof -ti:3000

# If a process is found, the server might already be running
# Try accessing: http://localhost:3000 in your browser
```

## 🎯 **After Starting the Server**

1. **Wait a few seconds** for the server to start
2. **Click the Retry button** in the overlay
3. **Or close and reopen** the ROBOTIS overlay
4. The ROBOTIS platform should load in the iframe

## 📋 **Troubleshooting**

### **Port 3000 Already in Use**
```bash
# Find what's using port 3000
lsof -ti:3000

# Kill the process if needed
kill -9 $(lsof -ti:3000)

# Then start ROBOTIS server
```

### **Server Starts But Still Can't Connect**
- Check firewall settings
- Verify server is listening on `0.0.0.0:3000` (not just `127.0.0.1`)
- Check server logs for errors
- Try accessing `http://localhost:3000` directly in browser

### **Different Port**
If ROBOTIS runs on a different port, update the overlay:
- Edit: `src/components/Workspace/OSDesktop.tsx`
- Find: `src="http://localhost:3000"`
- Change to your port: `src="http://localhost:YOUR_PORT"`

## ✅ **Verification**

Once the server is running:
- ✅ Overlay opens when clicking ROBOTIS icon
- ✅ Iframe loads `http://localhost:3000`
- ✅ ROBOTIS platform displays correctly
- ✅ No connection errors

---

**The overlay integration is complete and working!** Just start the ROBOTIS server and it will load automatically. 🎉

