// NAVΛ Studio System Operations Module
// Provides cross-platform system control functionality

use std::process::Command;

#[derive(Debug)]
pub enum SystemError {
    CommandFailed(String),
    UnsupportedPlatform,
    PermissionDenied,
}

impl std::fmt::Display for SystemError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SystemError::CommandFailed(msg) => write!(f, "System command failed: {}", msg),
            SystemError::UnsupportedPlatform => write!(f, "Operation not supported on this platform"),
            SystemError::PermissionDenied => write!(f, "Permission denied for system operation"),
        }
    }
}

impl std::error::Error for SystemError {}

pub struct SystemController;

impl SystemController {
    pub fn new() -> Self {
        SystemController
    }

    /// Put the system to sleep
    pub fn sleep_system() -> Result<String, SystemError> {
        #[cfg(target_os = "macos")]
        {
            match Command::new("pmset")
                .arg("sleepnow")
                .output()
            {
                Ok(output) => {
                    if output.status.success() {
                        Ok("System going to sleep...".to_string())
                    } else {
                        Err(SystemError::CommandFailed(
                            String::from_utf8_lossy(&output.stderr).to_string()
                        ))
                    }
                }
                Err(e) => Err(SystemError::CommandFailed(e.to_string())),
            }
        }

        #[cfg(target_os = "linux")]
        {
            match Command::new("systemctl")
                .arg("suspend")
                .output()
            {
                Ok(output) => {
                    if output.status.success() {
                        Ok("System going to sleep...".to_string())
                    } else {
                        Err(SystemError::CommandFailed(
                            String::from_utf8_lossy(&output.stderr).to_string()
                        ))
                    }
                }
                Err(e) => Err(SystemError::CommandFailed(e.to_string())),
            }
        }

        #[cfg(target_os = "windows")]
        {
            match Command::new("rundll32.exe")
                .args(&["powrprof.dll", "SetSuspendState", "0", "1", "0"])
                .output()
            {
                Ok(output) => {
                    if output.status.success() {
                        Ok("System going to sleep...".to_string())
                    } else {
                        Err(SystemError::CommandFailed(
                            String::from_utf8_lossy(&output.stderr).to_string()
                        ))
                    }
                }
                Err(e) => Err(SystemError::CommandFailed(e.to_string())),
            }
        }

        #[cfg(not(any(target_os = "macos", target_os = "linux", target_os = "windows")))]
        {
            Err(SystemError::UnsupportedPlatform)
        }
    }

    /// Restart the system
    pub fn restart_system() -> Result<String, SystemError> {
        #[cfg(target_os = "macos")]
        {
            match Command::new("osascript")
                .args(&["-e", "tell app \"System Events\" to restart"])
                .output()
            {
                Ok(output) => {
                    if output.status.success() {
                        Ok("System restarting...".to_string())
                    } else {
                        Err(SystemError::CommandFailed(
                            String::from_utf8_lossy(&output.stderr).to_string()
                        ))
                    }
                }
                Err(e) => Err(SystemError::CommandFailed(e.to_string())),
            }
        }

        #[cfg(target_os = "linux")]
        {
            match Command::new("systemctl")
                .arg("reboot")
                .output()
            {
                Ok(output) => {
                    if output.status.success() {
                        Ok("System restarting...".to_string())
                    } else {
                        Err(SystemError::CommandFailed(
                            String::from_utf8_lossy(&output.stderr).to_string()
                        ))
                    }
                }
                Err(e) => Err(SystemError::CommandFailed(e.to_string())),
            }
        }

        #[cfg(target_os = "windows")]
        {
            match Command::new("shutdown")
                .args(&["/r", "/t", "0"])
                .output()
            {
                Ok(output) => {
                    if output.status.success() {
                        Ok("System restarting...".to_string())
                    } else {
                        Err(SystemError::CommandFailed(
                            String::from_utf8_lossy(&output.stderr).to_string()
                        ))
                    }
                }
                Err(e) => Err(SystemError::CommandFailed(e.to_string())),
            }
        }

        #[cfg(not(any(target_os = "macos", target_os = "linux", target_os = "windows")))]
        {
            Err(SystemError::UnsupportedPlatform)
        }
    }

    /// Shutdown the system
    pub fn shutdown_system() -> Result<String, SystemError> {
        #[cfg(target_os = "macos")]
        {
            match Command::new("osascript")
                .args(&["-e", "tell app \"System Events\" to shut down"])
                .output()
            {
                Ok(output) => {
                    if output.status.success() {
                        Ok("System shutting down...".to_string())
                    } else {
                        Err(SystemError::CommandFailed(
                            String::from_utf8_lossy(&output.stderr).to_string()
                        ))
                    }
                }
                Err(e) => Err(SystemError::CommandFailed(e.to_string())),
            }
        }

        #[cfg(target_os = "linux")]
        {
            match Command::new("systemctl")
                .arg("poweroff")
                .output()
            {
                Ok(output) => {
                    if output.status.success() {
                        Ok("System shutting down...".to_string())
                    } else {
                        Err(SystemError::CommandFailed(
                            String::from_utf8_lossy(&output.stderr).to_string()
                        ))
                    }
                }
                Err(e) => Err(SystemError::CommandFailed(e.to_string())),
            }
        }

        #[cfg(target_os = "windows")]
        {
            match Command::new("shutdown")
                .args(&["/s", "/t", "0"])
                .output()
            {
                Ok(output) => {
                    if output.status.success() {
                        Ok("System shutting down...".to_string())
                    } else {
                        Err(SystemError::CommandFailed(
                            String::from_utf8_lossy(&output.stderr).to_string()
                        ))
                    }
                }
                Err(e) => Err(SystemError::CommandFailed(e.to_string())),
            }
        }

        #[cfg(not(any(target_os = "macos", target_os = "linux", target_os = "windows")))]
        {
            Err(SystemError::UnsupportedPlatform)
        }
    }

    /// Lock the screen
    pub fn lock_screen() -> Result<String, SystemError> {
        #[cfg(target_os = "macos")]
        {
            match Command::new("pmset")
                .arg("displaysleepnow")
                .output()
            {
                Ok(output) => {
                    if output.status.success() {
                        Ok("Screen locked".to_string())
                    } else {
                        Err(SystemError::CommandFailed(
                            String::from_utf8_lossy(&output.stderr).to_string()
                        ))
                    }
                }
                Err(e) => Err(SystemError::CommandFailed(e.to_string())),
            }
        }

        #[cfg(target_os = "linux")]
        {
            // Try multiple lock commands (different desktop environments)
            for cmd in &["loginctl", "xdg-screensaver", "gnome-screensaver-command"] {
                let args = match *cmd {
                    "loginctl" => vec!["lock-session"],
                    "xdg-screensaver" => vec!["lock"],
                    "gnome-screensaver-command" => vec!["-l"],
                    _ => continue,
                };

                if let Ok(output) = Command::new(cmd).args(&args).output() {
                    if output.status.success() {
                        return Ok("Screen locked".to_string());
                    }
                }
            }
            Err(SystemError::CommandFailed("No lock command available".to_string()))
        }

        #[cfg(target_os = "windows")]
        {
            match Command::new("rundll32.exe")
                .args(&["user32.dll", "LockWorkStation"])
                .output()
            {
                Ok(output) => {
                    if output.status.success() {
                        Ok("Screen locked".to_string())
                    } else {
                        Err(SystemError::CommandFailed(
                            String::from_utf8_lossy(&output.stderr).to_string()
                        ))
                    }
                }
                Err(e) => Err(SystemError::CommandFailed(e.to_string())),
            }
        }

        #[cfg(not(any(target_os = "macos", target_os = "linux", target_os = "windows")))]
        {
            Err(SystemError::UnsupportedPlatform)
        }
    }

    /// Log out current user
    pub fn logout_user() -> Result<String, SystemError> {
        #[cfg(target_os = "macos")]
        {
            match Command::new("osascript")
                .args(&["-e", "tell app \"System Events\" to log out"])
                .output()
            {
                Ok(output) => {
                    if output.status.success() {
                        Ok("Logging out...".to_string())
                    } else {
                        Err(SystemError::CommandFailed(
                            String::from_utf8_lossy(&output.stderr).to_string()
                        ))
                    }
                }
                Err(e) => Err(SystemError::CommandFailed(e.to_string())),
            }
        }

        #[cfg(target_os = "linux")]
        {
            // Try different logout methods
            for (cmd, args) in &[
                ("loginctl", vec!["terminate-user", ""]),
                ("gnome-session-quit", vec!["--logout", "--no-prompt"]),
                ("qdbus", vec!["org.kde.ksmserver", "/KSMServer", "logout", "0", "0", "0"]),
            ] {
                if let Ok(output) = Command::new(cmd).args(args).output() {
                    if output.status.success() {
                        return Ok("Logging out...".to_string());
                    }
                }
            }
            Err(SystemError::CommandFailed("No logout command available".to_string()))
        }

        #[cfg(target_os = "windows")]
        {
            match Command::new("shutdown")
                .args(&["/l"])
                .output()
            {
                Ok(output) => {
                    if output.status.success() {
                        Ok("Logging out...".to_string())
                    } else {
                        Err(SystemError::CommandFailed(
                            String::from_utf8_lossy(&output.stderr).to_string()
                        ))
                    }
                }
                Err(e) => Err(SystemError::CommandFailed(e.to_string())),
            }
        }

        #[cfg(not(any(target_os = "macos", target_os = "linux", target_os = "windows")))]
        {
            Err(SystemError::UnsupportedPlatform)
        }
    }

    /// Get system information
    pub fn get_system_info() -> String {
        let os = std::env::consts::OS;
        let arch = std::env::consts::ARCH;
        let family = std::env::consts::FAMILY;
        
        format!(
            "NAVΛ Studio IDE\nOS: {}\nArchitecture: {}\nFamily: {}\nRust-powered system engine",
            os, arch, family
        )
    }
}

