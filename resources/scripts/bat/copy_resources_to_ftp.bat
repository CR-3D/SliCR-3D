@echo off
REM Change to the script directory
cd /d "%~dp0"

REM Run the script using Git Bash with relative path
"C:\Program Files\Git\bin\bash.exe" -c "../shell/copy_resource_to_ftp.sh"

pause