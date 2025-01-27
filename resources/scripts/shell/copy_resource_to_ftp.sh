#!/bin/bash

# Define the source folder to compress
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_FOLDER="$(cd "$SCRIPT_DIR/../../profiles" && pwd)"
ARCHIVE_FILE="$SCRIPT_DIR/profiles.zip"           # Path for the .zip archive

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

# Compress the contents of the source folder into a .zip archive using 7z
echo "Creating zip archive $ARCHIVE_FILE from the contents of $SOURCE_FOLDER using 7z..."
rm -f "$ARCHIVE_FILE" # Remove existing archive if it exists
cd "$SOURCE_FOLDER" || { echo "Failed to navigate to $SOURCE_FOLDER"; exit 1; }
if ! 7z a -tzip "$ARCHIVE_FILE" *; then
  echo "Error: Failed to create zip archive with 7z."
  exit 1
fi
echo "Zip archive created successfully: $ARCHIVE_FILE"

# Upload the .zip file to the FTP server
REMOTE_ARCHIVE="$PROFILES_PATH/profiles.zip"
echo "Uploading $ARCHIVE_FILE to $FTP_SERVER:$REMOTE_ARCHIVE..."
curl --ftp-create-dirs -T "$ARCHIVE_FILE" "$FTP_SERVER/$REMOTE_ARCHIVE" --user "$FTP_USER:$FTP_PASSWORD"

# Check if the upload was successful
if [ $? -eq 0 ]; then
  echo "File uploaded successfully to $REMOTE_ARCHIVE!"
else
  echo "Failed to upload $ARCHIVE_FILE to $REMOTE_ARCHIVE."
  exit 1
fi

exit 0
s