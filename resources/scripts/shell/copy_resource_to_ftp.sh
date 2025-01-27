#!/bin/bash

# Define the source folder
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_FOLDER="$(cd "$SCRIPT_DIR/../../profiles" && pwd)"

# Load FTP configuration
CONFIG_FILE="$SCRIPT_DIR/ftp_config"

if [ ! -f "$CONFIG_FILE" ]; then
  echo "Error: Configuration file 'ftp_config' not found in $SCRIPT_DIR."
  exit 1
fi

source "$CONFIG_FILE"

# Check if all required FTP variables are set
if [ -z "$FTP_SERVER" ] || [ -z "$FTP_PORT" ] || [ -z "$FTP_USER" ] || [ -z "$FTP_PASSWORD" ] || [ -z "$PROFILES_PATH" ]; then
  echo "Error: Missing FTP configuration details in 'ftp_config'."
  exit 1
fi

# Check if the source folder exists
if [ ! -d "$SOURCE_FOLDER" ]; then
  echo "Error: Source folder '$SOURCE_FOLDER' does not exist."
  exit 1
fi

# Determine files to upload based on git status
echo "Checking for modified or new files in $SOURCE_FOLDER..."
cd "$SOURCE_FOLDER" || { echo "Failed to navigate to $SOURCE_FOLDER"; exit 1; }

# Get the list of modified or new files
CHANGED_FILES=$(git ls-files -m -o --exclude-standard)

if [ -z "$CHANGED_FILES" ]; then
  echo "No modified or new files to upload."
  exit 0
fi

# Upload each modified or new file to the FTP server
for file in $CHANGED_FILES; do
  REMOTE_FILE="$PROFILES_PATH/$file"
  echo "Uploading $file to $REMOTE_FILE..."
  curl --ftp-create-dirs -T "$file" "$FTP_SERVER/$REMOTE_FILE" --user "$FTP_USER:$FTP_PASSWORD"

  if [ $? -eq 0 ]; then
    echo "$file uploaded successfully!"
  else
    echo "Failed to upload $file."
    exit 1
  fi
done

echo "All modified files uploaded successfully!"
exit 0
